"""Retarget UE-skeleton poselib .npy clips (Reallusion drunken set) to a robot
with GMR, headless. Saves one .npz per clip (same format as retarget_headless).

Why this works with the soma_bvh IK config: poselib stores LOCAL quats whose
identity = the rig's T-pose with ALL bone world rotations = identity — the
same convention the SOMA BVH loader produces (BVH T-pose => identity world
rotations). So the soma_bvh_to_atlas rotation offsets apply directly; we just
map UE bone names onto the SOMA joint names the config expects.

Usage (GMR venv):
    .venv/bin/python scripts/retarget_ue_npy.py \
        --npy_dir ~/sparkpack/reallusion_fbx/drunken --out_dir /tmp/gmr_drunken \
        --robot atlas_fists --names attack_03_clip,combo_05_4
"""
import argparse
import os
import pathlib

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR

# UE bone -> SOMA joint name (inverse of the ProtoMotions MANNY_MAP).
UE_TO_SOMA = {
    "pelvis": "Hips",
    "spine_01": "Spine1",
    "spine_02": "Spine2",
    "spine_03": "Chest",
    "neck_01": "Neck1",
    "head": "Head",
    "clavicle_r": "RightShoulder",
    "upperarm_r": "RightArm",
    "lowerarm_r": "RightForeArm",
    "hand_r": "RightHand",
    "clavicle_l": "LeftShoulder",
    "upperarm_l": "LeftArm",
    "lowerarm_l": "LeftForeArm",
    "hand_l": "LeftHand",
    "thigh_r": "RightLeg",
    "calf_r": "RightShin",
    "foot_r": "RightFoot",
    "ball_r": "RightToeBase",
    "thigh_l": "LeftLeg",
    "calf_l": "LeftShin",
    "foot_l": "LeftFoot",
    "ball_l": "LeftToeBase",
}


def _unwrap(v):
    if hasattr(v, "keys") and "arr" in v:
        return np.asarray(v["arr"])
    return np.asarray(v)


def _quat_to_mat(q):
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    n = (q * q).sum(-1)
    s = 2.0 / np.clip(n, 1e-12, None)
    m = np.empty(q.shape[:-1] + (3, 3))
    m[..., 0, 0] = 1 - s * (y * y + z * z); m[..., 0, 1] = s * (x * y - w * z); m[..., 0, 2] = s * (x * z + w * y)
    m[..., 1, 0] = s * (x * y + w * z); m[..., 1, 1] = 1 - s * (x * x + z * z); m[..., 1, 2] = s * (y * z - w * x)
    m[..., 2, 0] = s * (x * z - w * y); m[..., 2, 1] = s * (y * z + w * x); m[..., 2, 2] = 1 - s * (x * x + y * y)
    return m


def _mat_to_quat_wxyz(m):
    w = np.sqrt(np.clip(1 + m[..., 0, 0] + m[..., 1, 1] + m[..., 2, 2], 1e-12, None)) / 2
    x = (m[..., 2, 1] - m[..., 1, 2]) / (4 * w)
    y = (m[..., 0, 2] - m[..., 2, 0]) / (4 * w)
    z = (m[..., 1, 0] - m[..., 0, 1]) / (4 * w)
    return np.stack([w, x, y, z], axis=-1)


def rig_rest_height(names, parents, offsets):
    """Rig height from the REST pose (identity rotations): head-to-lowest-foot
    span of the offset chain — constant per skeleton, immune to clip content
    (jumps inflate per-clip maxima; lying starts deflate percentiles)."""
    J = len(names)
    rest = np.zeros((J, 3))
    for j in range(J):
        if parents[j] >= 0:
            rest[j] = rest[parents[j]] + offsets[j]
    head = rest[names.index("head"), 2]
    feet = min(rest[names.index("foot_l"), 2], rest[names.index("foot_r"), 2])
    return float(head - feet) * 0.01 + 0.15


def load_ue_npy(path):
    """poselib npy -> (world_pos [T,J,3] m z-up, world_rot_wxyz [T,J,4], names, fps)."""
    d = np.load(path, allow_pickle=True).item()
    st = d["skeleton_tree"]
    names = list(st["node_names"])
    parents = np.asarray(_unwrap(st["parent_indices"]), dtype=np.int64)
    offsets = _unwrap(st["local_translation"]).astype(np.float64)  # cm
    quats = _unwrap(d["rotation"]).astype(np.float64)              # local xyzw
    root_t = _unwrap(d["root_translation"]).astype(np.float64)     # cm
    fps = int(d.get("fps", 30))

    local = _quat_to_mat(quats)          # [T,J,3,3]
    T, J = local.shape[:2]
    wrot = np.empty_like(local)
    wpos = np.empty((T, J, 3))
    for j in range(J):
        p = parents[j]
        if p < 0:
            wrot[:, j] = local[:, j]
            wpos[:, j] = root_t
        else:
            wrot[:, j] = wrot[:, p] @ local[:, j]
            wpos[:, j] = wpos[:, p] + np.einsum("tab,b->ta", wrot[:, p], offsets[j])
    return wpos * 0.01, _mat_to_quat_wxyz(wrot), names, fps


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--npy_dir", required=True)
    p.add_argument("--out_dir", required=True)
    p.add_argument("--robot", choices=["atlas", "atlas_fists"], default="atlas_fists")
    p.add_argument("--bind_npz", default="/tmp/ue_bind_raw.npz",
                   help="Raw T-pose bind world rotations (from MM_T_Pose.FBX)")
    p.add_argument("--human_height", type=float, default=2.41,
                   help="Rig standing height (m). This Reallusion drunken rig "
                   "measures 2.41 (median head z of the standing idle + head "
                   "cap); poselib rest pose is degenerate so it cannot be "
                   "derived per clip.")
    p.add_argument("--names", default=None,
                   help="comma-separated clip stems to convert (default: all)")
    args = p.parse_args()

    files = sorted(pathlib.Path(args.npy_dir).glob("*.npy"))
    if args.names:
        want = set(args.names.split(","))
        files = [f for f in files if f.stem in want]
    os.makedirs(args.out_dir, exist_ok=True)
    print(f"[GMR-ue] {len(files)} clips")

    for ci, f in enumerate(files):
        wpos, wrot, names, fps = load_ue_npy(f)
        idx = {n: i for i, n in enumerate(names)}
        human_height = args.human_height
        # Convert world rotations to the BVH convention (identity at T-pose):
        # poselib identity is a degenerate star-pose, so at the rig's REAL
        # T-pose the world rotations equal the bind orientations — divide
        # them out, exactly like the FBX->SOMA world-delta retarget.
        bind = np.load(args.bind_npz)
        wrot_m = _quat_to_mat(np.stack([wrot[..., 1], wrot[..., 2], wrot[..., 3], wrot[..., 0]], axis=-1))
        for ue in UE_TO_SOMA:
            j = idx[ue]
            wrot_m[:, j] = wrot_m[:, j] @ bind[ue].T
        wrot = _mat_to_quat_wxyz(wrot_m)

        retarget = GMR(
            actual_human_height=human_height,
            src_human="soma_bvh",
            tgt_robot=args.robot,
            verbose=False,
        )
        qpos_list = []
        for t in range(wpos.shape[0]):
            frame = {soma: [wpos[t, idx[ue]], wrot[t, idx[ue]]]
                     for ue, soma in UE_TO_SOMA.items()}
            # Neck2 aliases Neck1 (UE single neck)
            frame["Neck2"] = frame["Neck1"]
            qpos_list.append(retarget.retarget(frame))

        out = os.path.join(args.out_dir, f"{f.stem}.npz")
        np.savez(
            out, fps=float(fps),
            root_pos=np.array([q[:3] for q in qpos_list]),
            root_rot=np.array([q[3:7][[1, 2, 3, 0]] for q in qpos_list]),
            dof_pos=np.array([q[7:] for q in qpos_list]),
            robot_type=args.robot,
        )
        print(f"  ({ci+1}/{len(files)}) {f.stem}: {wpos.shape[0]} frames @ {fps} fps "
              f"(h={human_height:.2f}m)")


if __name__ == "__main__":
    main()
