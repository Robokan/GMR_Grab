"""
GRAB dataset to robot motion retargeting script.

GRAB dataset format:
- body.params contains: transl, global_orient, body_pose, left_hand_pose (24 PCA), right_hand_pose (24 PCA)
- Hand poses use 24 PCA components
- framerate is typically 120 Hz

Usage:
    python scripts/grab_to_robot.py \
        --grab_file /path/to/GRAB_data/sequences/s1/apple_eat_1.npz \
        --robot unitree_g1_with_hands \
        --smplx_model_path /path/to/smplx_models
"""
import argparse
import pathlib
import os
import time

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.smpl import load_grab_file, get_smplx_data_offline_fast

from rich import print

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser(description="Retarget GRAB motion data to robot")
    parser.add_argument(
        "--grab_file",
        help="GRAB motion file to load.",
        type=str,
        required=True,
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2", "unitree_h1_2_with_hands",
                 "booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1", 
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1",
                "pnd_adam_lite", "pnd_adam_inspire", "openloong", "tienkung", "fourier_gr3"],
        default="unitree_g1_with_hands",
    )
    
    parser.add_argument(
        "--smplx_model_path",
        type=str,
        default=None,
        help="Path to SMPL-X body models folder. If not provided, uses assets/body_models",
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
        "--preserve_hand_positions",
        default=False,
        action="store_true",
        help="Preserve absolute hand positions (no scaling for wrists). Use for manipulation tasks.",
    )
    
    parser.add_argument(
        "--no_viewer",
        default=False,
        action="store_true",
        help="Disable viewer (for batch processing). Requires --save_path.",
    )

    args = parser.parse_args()
    
    if args.no_viewer and args.save_path is None:
        parser.error("--no_viewer requires --save_path")

    # SMPL-X body models path
    if args.smplx_model_path:
        SMPLX_FOLDER = pathlib.Path(args.smplx_model_path)
    else:
        SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"
    
    print(f"[bold]Loading GRAB motion from:[/bold] {args.grab_file}")
    
    # Load GRAB trajectory
    grab_data, body_model, smplx_output, actual_human_height = load_grab_file(
        args.grab_file, SMPLX_FOLDER
    )
    
    print(f"[bold]Loaded motion with {grab_data['pose_body'].shape[0]} frames at {grab_data['mocap_frame_rate'].item()} Hz[/bold]")
    
    # align fps
    tgt_fps = 30
    smplx_data_frames, aligned_fps = get_smplx_data_offline_fast(grab_data, body_model, smplx_output, tgt_fps=tgt_fps)
    
    print(f"[bold]Resampled to {len(smplx_data_frames)} frames at {aligned_fps:.1f} Hz[/bold]")
   
    # Choose source type based on whether we want absolute hand positions
    if args.preserve_hand_positions:
        src_human = "smplx_hands_absolute"
        print("[bold yellow]Using absolute hand positioning (no wrist scaling)[/bold yellow]")
    else:
        src_human = "smplx"  # Default - uses smplx_to_g1_with_hands.json for g1_with_hands
        
    # Initialize the retargeting system
    retarget = GMR(
        actual_human_height=actual_human_height,
        src_human=src_human,
        tgt_robot=args.robot,
    )
    
    # Initialize viewer only if not in no_viewer mode
    robot_motion_viewer = None
    if not args.no_viewer:
        robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                                motion_fps=aligned_fps,
                                                transparent_robot=0,
                                                record_video=args.record_video,
                                                video_path=f"videos/{args.robot}_{pathlib.Path(args.grab_file).stem}.mp4",)


    curr_frame = 0
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []
        target_positions_list = []  # Store target positions for each frame
    
    # Start the viewer
    i = 0

    while True:
        if args.loop:
            i = (i + 1) % len(smplx_data_frames)
        else:
            i += 1
            if i >= len(smplx_data_frames):
                break
        
        # FPS measurement (only when viewer is active)
        if not args.no_viewer:
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

        # visualize (only if viewer is active)
        if robot_motion_viewer is not None:
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
            # Save target positions (the IK targets from scaled human data)
            frame_targets = {}
            for body_name, (pos, rot) in retarget.scaled_human_data.items():
                frame_targets[body_name] = (pos.copy(), rot.copy())
            target_positions_list.append(frame_targets)
            
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
            "source_file": args.grab_file,
            "target_positions": target_positions_list,  # IK target positions per frame
            # Scaling info for scene matching in SparkProtoMotions
            "actual_human_height": retarget.actual_human_height,
            "human_height_assumption": retarget.human_height_assumption,
            "height_ratio": retarget.height_ratio,  # actual / assumed
            "robot_type": args.robot,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"[bold green]Saved to {args.save_path}[/bold green]")
            
    if robot_motion_viewer is not None:
        robot_motion_viewer.close()
