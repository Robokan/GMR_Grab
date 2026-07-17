"""Retarget motions from a packed ProtoMotions SOMA23 MotionLib (.pt) to a
robot, headless, saving one .npz per motion (same format as
retarget_headless.py).

Feeds GMR the same frame format as the SOMA BVH loader — dicts of
body name -> [position (m, z-up), wxyz quaternion] — but sourced from the
lib's precomputed global transforms (gts/grs), so anything already converted
to SOMA (Reallusion drunken/combat clips, SEED, ...) can be retargeted
without a BVH intermediate.

Usage (GMR venv):
    .venv/bin/python scripts/retarget_soma_motionlib.py \
        --lib ~/sparkpack/ProtoMotions/data/soma_drunken_combat.pt \
        --out_dir /tmp/gmr_drunken --robot atlas_fists
"""
import argparse
import os

import numpy as np
import torch

from general_motion_retargeting import GeneralMotionRetargeting as GMR

SOMA23_BODY_NAMES = [
    "Hips", "Spine1", "Spine2", "Chest", "Neck1", "Neck2", "Head",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "RightLeg", "RightShin", "RightFoot", "RightToeBase",
    "LeftLeg", "LeftShin", "LeftFoot", "LeftToeBase",
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--lib", required=True)
    p.add_argument("--out_dir", required=True)
    p.add_argument("--robot", choices=["atlas", "atlas_fists"], default="atlas_fists")
    p.add_argument("--limit", type=int, default=0)
    args = p.parse_args()

    d = torch.load(args.lib, map_location="cpu", weights_only=False)
    gts = d["gts"].numpy()          # [F, 23, 3] z-up meters
    grs = d["grs"].numpy()          # [F, 23, 4] xyzw
    starts = d["length_starts"].numpy()
    nframes = d["motion_num_frames"].numpy()
    dts = d["motion_dt"].numpy() if hasattr(d["motion_dt"], "numpy") else np.asarray(d["motion_dt"])
    names = [str(f).split("/")[-1].replace(".motion", "") for f in d["motion_files"]]

    os.makedirs(args.out_dir, exist_ok=True)
    n = len(names) if not args.limit else min(args.limit, len(names))
    print(f"[GMR-lib] {n} motions from {args.lib}")

    retarget = None
    for mi in range(n):
        s, e = int(starts[mi]), int(starts[mi]) + int(nframes[mi])
        pos = gts[s:e]
        quat_wxyz = grs[s:e][..., [3, 0, 1, 2]]
        fps = 1.0 / float(dts[mi])

        # Height from the HEAD body (max-z catches raised hands): head center
        # at frame 0 + half-head, minus lowest foot.
        # Max over the CLIP, not frame 0: get-up clips start lying down.
        head_z = float(pos[:, SOMA23_BODY_NAMES.index("Head"), 2].max())
        human_height = head_z + 0.12

        # Per-clip retargeter: heights differ per clip and GMR bakes the
        # scale at construction.
        retarget = GMR(
            actual_human_height=human_height,
            src_human="soma_bvh",
            tgt_robot=args.robot,
            verbose=False,
        )

        qpos_list = []
        for t in range(pos.shape[0]):
            frame = {
                bn: [pos[t, i], quat_wxyz[t, i]]
                for i, bn in enumerate(SOMA23_BODY_NAMES)
            }
            qpos_list.append(retarget.retarget(frame))

        out = os.path.join(args.out_dir, f"{names[mi]}.npz")
        np.savez(
            out,
            fps=fps,
            root_pos=np.array([q[:3] for q in qpos_list]),
            root_rot=np.array([q[3:7][[1, 2, 3, 0]] for q in qpos_list]),
            dof_pos=np.array([q[7:] for q in qpos_list]),
            robot_type=args.robot,
        )
        print(f"  ({mi+1}/{n}) {names[mi]}: {pos.shape[0]} frames @ {fps:.0f} fps "
              f"(h={human_height:.2f}m)")


if __name__ == "__main__":
    main()
