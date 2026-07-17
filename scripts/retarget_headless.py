"""Headless batch SOMA-BVH -> Atlas retarget (no viewer, saves .pkl per clip).

Same math/path as soma_bvh_to_robot.py minus the GUI viewer, so it runs over
SSH / in scripts. Output pkl format is identical to that script's --save_path.

Usage (GMR venv):
    .venv/bin/python scripts/retarget_headless.py \
        --bvh_file /path/clip.bvh --out_dir /path/out --robot atlas_fists
    .venv/bin/python scripts/retarget_headless.py \
        --bvh_dir /path/bvhs --out_dir /path/out --robot atlas_fists
"""
import argparse
import os
import pathlib
import pickle

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.soma_bvh import load_soma_bvh_file


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bvh_file", default=None)
    p.add_argument("--bvh_dir", default=None)
    p.add_argument("--bvh_list", default=None, help="file with one BVH path per line")
    p.add_argument("--shard", default=None, help="i/N: process shard i of N")
    p.add_argument("--out_dir", required=True)
    p.add_argument("--robot", choices=["atlas", "atlas_fists"], default="atlas_fists")
    p.add_argument("--tgt_fps", type=int, default=30)
    p.add_argument("--limit", type=int, default=0, help="max clips (0 = all)")
    args = p.parse_args()

    if args.bvh_file:
        bvh_files = [args.bvh_file]
    elif args.bvh_list:
        bvh_files = [l.strip() for l in open(args.bvh_list) if l.strip()]
    else:
        bvh_files = sorted(str(x) for x in pathlib.Path(args.bvh_dir).rglob("*.bvh"))
    if args.shard:
        i, n = map(int, args.shard.split("/"))
        bvh_files = bvh_files[i::n]
    if args.limit:
        bvh_files = bvh_files[: args.limit]
    os.makedirs(args.out_dir, exist_ok=True)
    print(f"[GMR-headless] {len(bvh_files)} clips -> {args.out_dir}")

    retarget = None
    for ci, bvh in enumerate(bvh_files):
        frames, human_height, src_fps = load_soma_bvh_file(bvh)
        step = max(1, round(src_fps / args.tgt_fps))
        frames = frames[::step]
        fps = src_fps / step

        if retarget is None:
            retarget = GMR(
                actual_human_height=human_height,
                src_human="soma_bvh",
                tgt_robot=args.robot,
                verbose=False,
            )
        else:
            retarget.configuration.update(retarget.model.qpos0)

        qpos_list = [retarget.retarget(f) for f in frames]
        motion_data = {
            "fps": fps,
            "root_pos": np.array([q[:3] for q in qpos_list]),
            "root_rot": np.array([q[3:7][[1, 2, 3, 0]] for q in qpos_list]),  # wxyz->xyzw
            "dof_pos": np.array([q[7:] for q in qpos_list]),
            "local_body_pos": None,
            "link_body_list": None,
            "actual_human_height": retarget.actual_human_height,
            "human_height_assumption": retarget.human_height_assumption,
            "height_ratio": retarget.height_ratio,
            "robot_type": args.robot,
        }
        # .npz, not pickle: the ProtoMotions container runs numpy 1.26 and
        # cannot unpickle numpy-2 arrays; the npy format is version-stable.
        out = os.path.join(
            args.out_dir, os.path.basename(bvh).replace(".bvh", ".npz")
        )
        if os.path.exists(out):
            continue
        np.savez(
            out,
            fps=motion_data["fps"],
            root_pos=motion_data["root_pos"],
            root_rot=motion_data["root_rot"],
            dof_pos=motion_data["dof_pos"],
            robot_type=motion_data["robot_type"],
        )
        print(f"  ({ci+1}/{len(bvh_files)}) {os.path.basename(out)}: "
              f"{len(frames)} frames @ {fps:.0f} fps")


if __name__ == "__main__":
    main()
