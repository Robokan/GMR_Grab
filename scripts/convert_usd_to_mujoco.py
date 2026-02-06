#!/usr/bin/env python3
"""
Convert a USD robot to MuJoCo XML format.

This script requires pxr (OpenUSD). Since pxr may only be available in
specific environments (e.g., Isaac Sim's Python 3.11), this script provides
a helper to run the conversion inside a Docker container.

Usage (direct):
    python convert_usd_to_mujoco.py <usd_file> [--output_dir <dir>]

Usage (via Isaac Lab container):
    python convert_usd_to_mujoco.py <usd_file> --docker <container_name> --mount <host_path>:<container_path>
"""

import argparse
import os
import subprocess
import sys


def run_in_docker(usd_path, output_dir, container_name, model_name, angle_unit, no_mesh_collision):
    """Run the conversion inside a Docker container that has pxr."""
    # Build the command to run inside the container
    script_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'general_motion_retargeting', 'usd_to_mujoco.py'))

    # Find Isaac Sim python inside the container
    isaac_python = "/isaac-sim/kit/python/bin/python3.11"
    usd_libs = "/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.la64.r.cp311"

    cmd_parts = [
        f"LD_LIBRARY_PATH=/isaac-sim/kit/python/lib:{usd_libs}/bin",
        f"PYTHONPATH={usd_libs}",
        isaac_python,
        script_path,
        usd_path,
    ]
    if output_dir:
        cmd_parts.extend(["--output_dir", output_dir])
    if model_name:
        cmd_parts.extend(["--model_name", model_name])
    if angle_unit:
        cmd_parts.extend(["--angle_unit", angle_unit])
    if no_mesh_collision:
        cmd_parts.append("--no_mesh_collision")

    cmd = " ".join(cmd_parts)
    result = subprocess.run(
        ["docker", "exec", container_name, "bash", "-c", cmd],
        capture_output=False
    )
    return result.returncode


def run_direct(usd_path, output_dir, model_name, angle_unit, no_mesh_collision):
    """Run the conversion directly (requires pxr to be importable)."""
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from general_motion_retargeting.usd_to_mujoco import UsdToMujoco

    converter = UsdToMujoco(
        usd_path=usd_path,
        output_dir=output_dir,
        model_name=model_name,
        use_mesh_collision=not no_mesh_collision,
        angle_unit=angle_unit,
    )
    converter.convert()


def main():
    parser = argparse.ArgumentParser(description="Convert USD robot to MuJoCo XML")
    parser.add_argument("usd_path", help="Path to USD/USDA file")
    parser.add_argument("--output_dir", "-o", default=None,
                        help="Output directory (default: <usd_dir>/mujoco_output)")
    parser.add_argument("--model_name", "-n", default=None,
                        help="Model name (default: derived from filename)")
    parser.add_argument("--no_mesh_collision", action="store_true",
                        help="Don't generate collision geometry")
    parser.add_argument("--angle_unit", choices=["radian", "degree"], default="radian",
                        help="Angle unit for joint limits (default: radian)")
    parser.add_argument("--docker", default=None,
                        help="Docker container name to run conversion in (e.g., spark-protomotions)")
    args = parser.parse_args()

    if args.docker:
        sys.exit(run_in_docker(
            args.usd_path, args.output_dir, args.docker,
            args.model_name, args.angle_unit, args.no_mesh_collision
        ))
    else:
        run_direct(
            args.usd_path, args.output_dir,
            args.model_name, args.angle_unit, args.no_mesh_collision
        )


if __name__ == "__main__":
    main()
