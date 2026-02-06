#!/usr/bin/env python3
"""
Convert GMR pickle files to SparkProtoMotions .motion format.

This script converts the output of GMR (grab_to_robot.py, smplx_to_robot.py, etc.)
to the ProtoMotions format that can be used for training in SparkProtoMotions.

Usage:
    python scripts/gmr_pkl_to_proto_motion.py \
        --input /tmp/cubelarge_lift_adam_inspire.pkl \
        --output /tmp/cubelarge_lift_adam_inspire.motion \
        --robot_xml /path/to/adam_inspire.xml
"""

import argparse
import pickle
from pathlib import Path

import numpy as np
import torch

from general_motion_retargeting.kinematics_model import KinematicsModel
from general_motion_retargeting.params import ROBOT_XML_DICT


def compute_velocity_finite_diff(positions: torch.Tensor, fps: int) -> torch.Tensor:
    """
    Compute velocities using finite differences.
    
    Args:
        positions: (T, ...) tensor of positions
        fps: frames per second
        
    Returns:
        velocities: (T, ...) tensor of velocities
    """
    dt = 1.0 / fps
    velocities = torch.zeros_like(positions)
    
    if positions.shape[0] > 2:
        # Central difference for middle frames
        velocities[1:-1] = (positions[2:] - positions[:-2]) / (2 * dt)
        # Forward difference for first frame
        velocities[0] = (positions[1] - positions[0]) / dt
        # Backward difference for last frame
        velocities[-1] = (positions[-1] - positions[-2]) / dt
    elif positions.shape[0] == 2:
        velocities[0] = (positions[1] - positions[0]) / dt
        velocities[1] = velocities[0]
    
    return velocities


def compute_angular_velocity(rot_mats: torch.Tensor, fps: int) -> torch.Tensor:
    """
    Compute angular velocities from rotation matrices using finite differences.
    
    Args:
        rot_mats: (T, N, 3, 3) rotation matrices
        fps: frames per second
        
    Returns:
        ang_vel: (T, N, 3) angular velocities
    """
    dt = 1.0 / fps
    T, N, _, _ = rot_mats.shape
    ang_vel = torch.zeros(T, N, 3, device=rot_mats.device, dtype=rot_mats.dtype)
    
    if T > 1:
        # Compute relative rotation: R_rel = R_{t+1} @ R_t^T
        for t in range(T - 1):
            R_rel = rot_mats[t + 1] @ rot_mats[t].transpose(-1, -2)
            # Convert to axis-angle (approximate for small rotations)
            # Using Rodrigues formula inverse
            trace = R_rel[:, 0, 0] + R_rel[:, 1, 1] + R_rel[:, 2, 2]
            cos_angle = (trace - 1) / 2
            cos_angle = torch.clamp(cos_angle, -1, 1)
            angle = torch.acos(cos_angle)
            
            # Axis from skew-symmetric part
            axis = torch.stack([
                R_rel[:, 2, 1] - R_rel[:, 1, 2],
                R_rel[:, 0, 2] - R_rel[:, 2, 0],
                R_rel[:, 1, 0] - R_rel[:, 0, 1],
            ], dim=-1)
            
            # Normalize
            norm = torch.norm(axis, dim=-1, keepdim=True)
            norm = torch.where(norm < 1e-8, torch.ones_like(norm), norm)
            axis = axis / norm
            
            ang_vel[t] = axis * angle.unsqueeze(-1) / dt
        
        ang_vel[-1] = ang_vel[-2]  # Last frame
    
    return ang_vel


def quat_to_rot_mat(quat: torch.Tensor) -> torch.Tensor:
    """
    Convert quaternion (xyzw) to rotation matrix.
    
    Args:
        quat: (..., 4) quaternion in xyzw format
        
    Returns:
        rot_mat: (..., 3, 3) rotation matrix
    """
    x, y, z, w = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
    
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    
    rot_mat = torch.stack([
        1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
        2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
        2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy),
    ], dim=-1).reshape(*quat.shape[:-1], 3, 3)
    
    return rot_mat


def compute_contact_labels(
    positions: torch.Tensor,
    velocities: torch.Tensor,
    height_thresh: float = 0.05,
    vel_thresh: float = 0.15,
) -> torch.Tensor:
    """
    Compute contact labels based on position height and velocity.
    
    Args:
        positions: (T, N, 3) body positions
        velocities: (T, N, 3) body velocities
        height_thresh: Height threshold for contact
        vel_thresh: Velocity threshold for contact
        
    Returns:
        contacts: (T, N) boolean contact labels
    """
    heights = positions[..., 2]
    speeds = torch.norm(velocities, dim=-1)
    
    contacts = (heights < height_thresh) & (speeds < vel_thresh)
    return contacts


def convert_gmr_to_proto(
    gmr_pkl_path: str,
    robot_xml_path: str,
    output_path: str,
    device: str = "cuda:0",
):
    """
    Convert GMR pickle file to ProtoMotions .motion format.
    
    Args:
        gmr_pkl_path: Path to GMR pickle file
        robot_xml_path: Path to robot MuJoCo XML file
        output_path: Path for output .motion file
        device: Torch device
    """
    # Load GMR motion data
    with open(gmr_pkl_path, "rb") as f:
        gmr_data = pickle.load(f)
    
    print(f"Loaded GMR motion: {gmr_pkl_path}")
    print(f"  FPS: {gmr_data['fps']}")
    print(f"  Frames: {gmr_data['root_pos'].shape[0]}")
    print(f"  DOFs: {gmr_data['dof_pos'].shape[1]}")
    
    # Extract data
    fps = int(gmr_data['fps'])
    root_pos = torch.from_numpy(gmr_data['root_pos']).float().to(device)  # (T, 3)
    root_rot = torch.from_numpy(gmr_data['root_rot']).float().to(device)  # (T, 4) xyzw
    dof_pos = torch.from_numpy(gmr_data['dof_pos']).float().to(device)   # (T, num_dof)
    
    T = root_pos.shape[0]
    num_dof = dof_pos.shape[1]
    
    # Initialize kinematics model
    kinematics_model = KinematicsModel(robot_xml_path, device=device)
    num_bodies = len(kinematics_model.body_names)
    
    print(f"  Bodies: {num_bodies}")
    
    # Compute forward kinematics to get rigid body positions and rotations
    body_pos, body_rot = kinematics_model.forward_kinematics(
        root_pos, root_rot, dof_pos
    )  # (T, num_bodies, 3), (T, num_bodies, 4)
    
    # Compute velocities
    dof_vel = compute_velocity_finite_diff(dof_pos, fps)
    rigid_body_vel = compute_velocity_finite_diff(body_pos, fps)
    
    # Compute angular velocities
    body_rot_mats = quat_to_rot_mat(body_rot)
    rigid_body_ang_vel = compute_angular_velocity(body_rot_mats, fps)
    
    # Compute contact labels
    rigid_body_contacts = compute_contact_labels(body_pos, rigid_body_vel)
    
    # Create motion dictionary in ProtoMotions format
    motion_data = {
        "fps": fps,
        "dof_pos": dof_pos.cpu(),
        "dof_vel": dof_vel.cpu(),
        "rigid_body_pos": body_pos.cpu(),
        "rigid_body_rot": body_rot.cpu(),  # xyzw format
        "rigid_body_vel": rigid_body_vel.cpu(),
        "rigid_body_ang_vel": rigid_body_ang_vel.cpu(),
        "rigid_body_contacts": rigid_body_contacts.cpu(),
    }
    
    # Add scaling info if present
    if "height_ratio" in gmr_data:
        motion_data["height_ratio"] = gmr_data["height_ratio"]
    if "actual_human_height" in gmr_data:
        motion_data["actual_human_height"] = gmr_data["actual_human_height"]
    if "robot_type" in gmr_data:
        motion_data["robot_type"] = gmr_data["robot_type"]
    
    # Save
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(motion_data, str(output_path))
    
    print(f"Saved ProtoMotions format to: {output_path}")
    print(f"  rigid_body_pos shape: {motion_data['rigid_body_pos'].shape}")
    print(f"  rigid_body_rot shape: {motion_data['rigid_body_rot'].shape}")
    print(f"  dof_pos shape: {motion_data['dof_pos'].shape}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert GMR pickle files to SparkProtoMotions .motion format"
    )
    parser.add_argument(
        "--input", "-i",
        type=str,
        required=True,
        help="Path to GMR pickle file",
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="Output path for .motion file (default: same as input with .motion extension)",
    )
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Robot type (e.g., pnd_adam_inspire). If not specified, uses robot_type from pkl file.",
    )
    parser.add_argument(
        "--robot_xml",
        type=str,
        default=None,
        help="Path to robot MuJoCo XML file. If not specified, looks up from robot type.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Torch device (default: cuda:0)",
    )
    
    args = parser.parse_args()
    
    # Determine output path
    if args.output is None:
        args.output = str(Path(args.input).with_suffix(".motion"))
    
    # Determine robot XML path
    if args.robot_xml is not None:
        robot_xml = args.robot_xml
    else:
        # Try to get from pkl file or argument
        if args.robot is not None:
            robot_type = args.robot
        else:
            # Load pkl to get robot type
            with open(args.input, "rb") as f:
                data = pickle.load(f)
            robot_type = data.get("robot_type")
            if robot_type is None:
                raise ValueError("Robot type not found in pkl file. Specify --robot or --robot_xml")
        
        if robot_type not in ROBOT_XML_DICT:
            raise ValueError(f"Unknown robot type: {robot_type}. Available: {list(ROBOT_XML_DICT.keys())}")
        
        robot_xml = str(ROBOT_XML_DICT[robot_type])
    
    print(f"Using robot XML: {robot_xml}")
    
    convert_gmr_to_proto(
        gmr_pkl_path=args.input,
        robot_xml_path=robot_xml,
        output_path=args.output,
        device=args.device,
    )


if __name__ == "__main__":
    main()
