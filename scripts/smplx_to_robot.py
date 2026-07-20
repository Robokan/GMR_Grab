import argparse
import pathlib
import os
import time

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.smpl import load_smplx_file, get_smplx_data_offline_fast

from rich import print

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--smplx_file",
        help="SMPLX motion file to load.",
        type=str,
        default=None,
    )

    parser.add_argument(
        "--smplx_dir",
        help="Directory of SMPL-X .npz files (searched recursively, each played once).",
        type=str,
        default=None,
    )

    parser.add_argument(
        "--shuffle",
        default=False,
        action="store_true",
        help="Shuffle directory playback order.",
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2",
                 "booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1", 
                "engineai_pm01", "t800", "t800_transparent", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1",
                "pnd_adam_lite", "openloong", "tienkung", "fourier_gr3", "atlas", "atlas_fists", "unitree_g1_revo2"],
        default="unitree_g1",
    )
    
    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )

    parser.add_argument(
        "--record_video",
        default=False,
        action="store_true",
        help="Record the video.",
    )

    parser.add_argument(
        "--rate_limit",
        default=False,
        action="store_true",
        help="Limit the rate of the retargeted robot motion to keep the same as the human motion.",
    )

    parser.add_argument(
        "--load_hands",
        default=False,
        action="store_true",
        help="Load hand poses from SMPL-X file (for GRAB dataset and similar).",
    )

    args = parser.parse_args()

    if (args.smplx_file is None) == (args.smplx_dir is None):
        parser.error("Provide exactly one of --smplx_file or --smplx_dir")

    SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"

    if args.smplx_file:
        smplx_files = [args.smplx_file]
    else:
        smplx_files = sorted(str(p) for p in pathlib.Path(args.smplx_dir).rglob("*.npz"))
        if not smplx_files:
            parser.error(f"No .npz files found under {args.smplx_dir}")
        if args.shuffle:
            import random
            random.shuffle(smplx_files)
        print(f"[GMR] {len(smplx_files)} clips queued")

    robot_motion_viewer = None
    for clip_idx, smplx_file in enumerate(smplx_files):
        # Load SMPLX trajectory (human height varies per subject, so the
        # retargeter is rebuilt per clip)
        smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(
            smplx_file, SMPLX_FOLDER, load_hands=args.load_hands
        )

        # align fps
        tgt_fps = 30
        smplx_data_frames, aligned_fps = get_smplx_data_offline_fast(smplx_data, body_model, smplx_output, tgt_fps=tgt_fps)

        print(f"[GMR] ({clip_idx + 1}/{len(smplx_files)}) {os.path.basename(smplx_file)}: "
              f"{len(smplx_data_frames)} frames, height {actual_human_height:.2f}m")

        retarget = GMR(
            actual_human_height=actual_human_height,
            src_human="smplx",
            tgt_robot=args.robot,
            verbose=(clip_idx == 0),
        )

        if robot_motion_viewer is None:
            robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                                    motion_fps=aligned_fps,
                                                    transparent_robot=0,
                                                    record_video=args.record_video,
                                                    video_path=f"videos/{args.robot}_{smplx_file.split('/')[-1].split('.')[0]}.mp4",)

        # FPS measurement variables
        fps_counter = 0
        fps_start_time = time.time()
        fps_display_interval = 2.0  # Display FPS every 2 seconds

        if args.save_path is not None:
            save_dir = os.path.dirname(args.save_path)
            if save_dir:  # Only create directory if it's not empty
                os.makedirs(save_dir, exist_ok=True)
            qpos_list = []

        i = 0
        while True:
            if args.loop and args.smplx_file:
                i = (i + 1) % len(smplx_data_frames)
            else:
                i += 1
                if i >= len(smplx_data_frames):
                    break

            # FPS measurement
            fps_counter += 1
            current_time = time.time()
            if current_time - fps_start_time >= fps_display_interval:
                actual_fps = fps_counter / (current_time - fps_start_time)
                print(f"Actual rendering FPS: {actual_fps:.2f}")
                fps_counter = 0
                fps_start_time = current_time

            # Update task targets.
            smplx_data = smplx_data_frames[i]

            # retarget
            qpos = retarget.retarget(smplx_data)

            # visualize
            robot_motion_viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retarget.scaled_human_data,
                human_pos_offset=np.array([0.0, 0.0, 0.0]),
                show_human_body_name=False,
                rate_limit=args.rate_limit,
                follow_camera=False,
            )
            if args.save_path is not None:
                qpos_list.append(qpos)

        if args.save_path is not None:
            import pickle
            root_pos = np.array([qpos[:3] for qpos in qpos_list])
            # save from wxyz to xyzw
            root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
            dof_pos = np.array([qpos[7:] for qpos in qpos_list])
            local_body_pos = None
            body_names = None

            motion_data = {
                "fps": aligned_fps,
                "root_pos": root_pos,
                "root_rot": root_rot,
                "dof_pos": dof_pos,
                "local_body_pos": local_body_pos,
                "link_body_list": body_names,
                # Scaling info for scene matching in SparkProtoMotions
                "actual_human_height": retarget.actual_human_height,
                "human_height_assumption": retarget.human_height_assumption,
                "height_ratio": retarget.height_ratio,  # actual / assumed
                "robot_type": args.robot,
            }
            with open(args.save_path, "wb") as f:
                pickle.dump(motion_data, f)
            print(f"Saved to {args.save_path}")

    robot_motion_viewer.close()
