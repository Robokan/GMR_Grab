#!/usr/bin/env python3
"""
Batch process all GRAB motions to Adam Inspire robot and convert to ProtoMotions format.

Usage:
    python scripts/batch_grab_to_adam_inspire.py \
        --grab_dir /path/to/GRAB_data/sequences \
        --output_dir /path/to/output \
        --smplx_model_path /path/to/smplx_models
"""

import argparse
import os
import pickle
import subprocess
import sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
from tqdm import tqdm

import numpy as np
import torch


def process_single_motion(args_tuple):
    """Process a single GRAB motion file."""
    grab_file, pkl_output, motion_output, smplx_model_path, robot, robot_xml = args_tuple
    
    try:
        # Step 1: Retarget GRAB to robot (GMR)
        cmd = [
            sys.executable,
            "scripts/grab_to_robot.py",
            "--grab_file", str(grab_file),
            "--smplx_model_path", str(smplx_model_path),
            "--robot", robot,
            "--save_path", str(pkl_output),
            "--no_viewer",
        ]
        
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300,  # 5 minute timeout
        )
        
        if result.returncode != 0:
            return grab_file.name, False, f"GMR failed: {result.stderr[:200]}"
        
        # Step 2: Convert to ProtoMotions format
        cmd = [
            sys.executable,
            "scripts/gmr_pkl_to_proto_motion.py",
            "--input", str(pkl_output),
            "--output", str(motion_output),
            "--robot_xml", str(robot_xml),
            "--device", "cpu",
        ]
        
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60,
        )
        
        if result.returncode != 0:
            return grab_file.name, False, f"Conversion failed: {result.stderr[:200]}"
        
        return grab_file.name, True, None
        
    except subprocess.TimeoutExpired:
        return grab_file.name, False, "Timeout"
    except Exception as e:
        return grab_file.name, False, str(e)


def main():
    parser = argparse.ArgumentParser(
        description="Batch process GRAB motions to Adam Inspire and convert to ProtoMotions format"
    )
    parser.add_argument(
        "--grab_dir",
        type=str,
        default="/home/evaughan/sparkpack/GRAB_data/sequences",
        help="Directory containing GRAB sequence folders",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="/home/evaughan/sparkpack/SparkProtoMotions/data/motions/adam_inspire/GRAB",
        help="Output directory for motion files",
    )
    parser.add_argument(
        "--smplx_model_path",
        type=str,
        default="/home/evaughan/sparkpack/GRAB_data/smplx_models",
        help="Path to SMPL-X body models",
    )
    parser.add_argument(
        "--robot",
        type=str,
        default="pnd_adam_inspire",
        help="Robot type",
    )
    parser.add_argument(
        "--robot_xml",
        type=str,
        default="/home/evaughan/sparkpack/pnd_models/adam_inspire/adam_inspire.xml",
        help="Robot XML file for FK computation",
    )
    parser.add_argument(
        "--num_workers",
        type=int,
        default=4,
        help="Number of parallel workers",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing files",
    )
    
    args = parser.parse_args()
    
    grab_dir = Path(args.grab_dir)
    output_dir = Path(args.output_dir)
    
    # Create output directories
    pkl_dir = output_dir / "pkl"
    motion_dir = output_dir / "motion"
    pkl_dir.mkdir(parents=True, exist_ok=True)
    motion_dir.mkdir(parents=True, exist_ok=True)
    
    # Find all GRAB motion files
    grab_files = list(grab_dir.glob("**/*.npz"))
    print(f"Found {len(grab_files)} GRAB motion files")
    
    # Build task list
    tasks = []
    for grab_file in grab_files:
        # Preserve directory structure (e.g., s1/motion_name)
        relative_path = grab_file.relative_to(grab_dir)
        stem = relative_path.stem
        parent = relative_path.parent
        
        pkl_output = pkl_dir / parent / f"{stem}.pkl"
        motion_output = motion_dir / parent / f"{stem}.motion"
        
        # Skip if already exists and not forcing
        if motion_output.exists() and not args.force:
            continue
        
        # Create parent directories
        pkl_output.parent.mkdir(parents=True, exist_ok=True)
        motion_output.parent.mkdir(parents=True, exist_ok=True)
        
        tasks.append((
            grab_file,
            pkl_output,
            motion_output,
            args.smplx_model_path,
            args.robot,
            args.robot_xml,
        ))
    
    print(f"Processing {len(tasks)} motions (skipping {len(grab_files) - len(tasks)} existing)")
    
    if len(tasks) == 0:
        print("Nothing to process. Use --force to overwrite existing files.")
        return
    
    # Process with progress bar
    success_count = 0
    fail_count = 0
    failures = []
    
    # Run sequentially for better error handling (can use parallel with ProcessPoolExecutor)
    for task in tqdm(tasks, desc="Processing GRAB motions"):
        name, success, error = process_single_motion(task)
        if success:
            success_count += 1
        else:
            fail_count += 1
            failures.append((name, error))
            tqdm.write(f"Failed: {name} - {error}")
    
    print(f"\n=== Summary ===")
    print(f"Success: {success_count}")
    print(f"Failed: {fail_count}")
    
    if failures:
        print(f"\nFailed files:")
        for name, error in failures[:10]:
            print(f"  {name}: {error}")
        if len(failures) > 10:
            print(f"  ... and {len(failures) - 10} more")


if __name__ == "__main__":
    main()
