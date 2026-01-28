from general_motion_retargeting import RobotMotionViewer, load_robot_motion
import argparse
import os
from tqdm import tqdm

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="unitree_g1")
                        
    parser.add_argument("--robot_motion_path", type=str, required=True)

    parser.add_argument("--record_video", action="store_true")
    parser.add_argument("--video_path", type=str, 
                        default="videos/example.mp4")
    
    parser.add_argument("--show_targets", action="store_true",
                        help="Show IK target markers (source mocap positions)")
                        
    args = parser.parse_args()
    
    robot_type = args.robot
    robot_motion_path = args.robot_motion_path
    
    if not os.path.exists(robot_motion_path):
        raise FileNotFoundError(f"Motion file {robot_motion_path} not found")
    
    motion_data, motion_fps, motion_root_pos, motion_root_rot, motion_dof_pos, motion_local_body_pos, motion_link_body_list = load_robot_motion(robot_motion_path)
    
    # Load target positions if available and requested
    target_positions = motion_data.get("target_positions", None)
    if args.show_targets and target_positions is None:
        print("Warning: --show_targets requested but no target_positions in motion file.")
        print("Re-run grab_to_robot.py with --save_path to include target positions.")
    
    env = RobotMotionViewer(robot_type=robot_type,
                            motion_fps=motion_fps,
                            camera_follow=False,
                            record_video=args.record_video, video_path=args.video_path)
    
    # Filter to show finger base and fingertip markers for all fingers
    # G1 middle finger maps to SMPL-X ring finger
    HAND_MARKERS = {
        "left_thumb3", "right_thumb3",
        "left_index3", "right_index3",
        "left_ring3", "right_ring3",
    }
    
    frame_idx = 0
    while True:
        # Get target positions for this frame if available
        frame_targets = None
        if args.show_targets and target_positions is not None:
            # Filter to only hand-related markers
            all_targets = target_positions[frame_idx]
            frame_targets = {k: v for k, v in all_targets.items() if k in HAND_MARKERS}
        
        env.step(motion_root_pos[frame_idx], 
                motion_root_rot[frame_idx], 
                motion_dof_pos[frame_idx], 
                human_motion_data=frame_targets,
                show_human_body_name=True,
                rate_limit=True)
        frame_idx += 1
        if frame_idx >= len(motion_root_pos):
            frame_idx = 0
    env.close()