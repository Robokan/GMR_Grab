"""Retarget SOMA-skeleton BVH motions (BONES-SEED dataset) to a robot.

Play a single clip:
    python scripts/soma_bvh_to_robot.py --bvh_file <clip.bvh> --robot atlas --rate_limit --loop

Browse a directory of clips (plays each once, in order):
    python scripts/soma_bvh_to_robot.py --bvh_dir <dir> --robot atlas --rate_limit
"""

import argparse
import os
import pathlib
import random

import numpy as np
from rich import print

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.soma_bvh import load_soma_bvh_file

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bvh_file", type=str, default=None, help="Single SOMA BVH file to play.")
    parser.add_argument("--bvh_dir", type=str, default=None, help="Directory of SOMA BVH files (searched recursively, each played once).")
    parser.add_argument("--robot", choices=["atlas", "atlas_fists"], default="atlas")
    parser.add_argument("--tgt_fps", type=int, default=30, help="Playback/retarget fps (source clips are subsampled).")
    parser.add_argument("--shuffle", action="store_true", default=False, help="Shuffle directory playback order.")
    parser.add_argument("--loop", action="store_true", default=False, help="Loop a single file forever.")
    parser.add_argument("--rate_limit", action="store_true", default=False)
    parser.add_argument("--record_video", action="store_true", default=False)
    parser.add_argument("--video_path", type=str, default="videos/soma_bvh.mp4")
    parser.add_argument("--save_path", type=str, default=None, help="Save retargeted motion (.pkl, single file only).")
    args = parser.parse_args()

    if (args.bvh_file is None) == (args.bvh_dir is None):
        parser.error("Provide exactly one of --bvh_file or --bvh_dir")

    if args.bvh_file:
        bvh_files = [args.bvh_file]
    else:
        bvh_files = sorted(str(p) for p in pathlib.Path(args.bvh_dir).rglob("*.bvh"))
        if not bvh_files:
            parser.error(f"No .bvh files found under {args.bvh_dir}")
        if args.shuffle:
            random.shuffle(bvh_files)
        print(f"[GMR] {len(bvh_files)} clips queued")

    retarget = None
    viewer = None
    for clip_idx, bvh_file in enumerate(bvh_files):
        frames, human_height, src_fps = load_soma_bvh_file(bvh_file)
        step = max(1, round(src_fps / args.tgt_fps))
        frames = frames[::step]
        fps = src_fps / step
        print(f"[GMR] ({clip_idx + 1}/{len(bvh_files)}) {os.path.basename(bvh_file)}: "
              f"{len(frames)} frames @ {fps:.0f} fps")

        if retarget is None:
            retarget = GMR(
                actual_human_height=human_height,
                src_human="soma_bvh",
                tgt_robot=args.robot,
                verbose=False,
            )
        else:
            # reset IK state so each clip starts from the rest pose instead of
            # the previous clip's final configuration
            retarget.configuration.update(retarget.model.qpos0)
            viewer = RobotMotionViewer(
                robot_type=args.robot,
                motion_fps=fps,
                record_video=args.record_video,
                video_path=args.video_path,
            )

        qpos_list = []
        i = 0
        while True:
            if args.loop and args.bvh_file:
                i = (i + 1) % len(frames)
            else:
                i += 1
                if i >= len(frames):
                    break
            qpos = retarget.retarget(frames[i])
            viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retarget.scaled_human_data,
                rate_limit=args.rate_limit,
                follow_camera=False,
            )
            if args.save_path is not None:
                qpos_list.append(qpos)

        if args.save_path is not None:
            import pickle
            motion_data = {
                "fps": fps,
                "root_pos": np.array([q[:3] for q in qpos_list]),
                "root_rot": np.array([q[3:7][[1, 2, 3, 0]] for q in qpos_list]),  # wxyz -> xyzw
                "dof_pos": np.array([q[7:] for q in qpos_list]),
                "local_body_pos": None,
                "link_body_list": None,
                "actual_human_height": retarget.actual_human_height,
                "human_height_assumption": retarget.human_height_assumption,
                "height_ratio": retarget.height_ratio,
                "robot_type": args.robot,
            }
            save_dir = os.path.dirname(args.save_path)
            if save_dir:
                os.makedirs(save_dir, exist_ok=True)
            with open(args.save_path, "wb") as f:
                pickle.dump(motion_data, f)
            print(f"Saved to {args.save_path}")

    viewer.close()
