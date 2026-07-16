"""Loader for SOMA-skeleton BVH files (BONES-SEED dataset).

The SOMA rig differs from LAFAN1: it has a Root joint above Hips, both with
6 channels (translation + rotation), full finger chains, and cm units in a
y-up frame. The lafan_vendor parser assumes only the first joint carries
translation channels, so this module has its own minimal BVH parser.
"""

import re

import numpy as np
from scipy.spatial.transform import Rotation as R

# y-up (BVH) -> z-up (GMR world)
_YUP_TO_ZUP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


class _Joint:
    __slots__ = ("name", "parent", "offset", "channels")

    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.offset = np.zeros(3)
        self.channels = []


def _parse_bvh(bvh_file):
    joints = []
    stack = []
    frames = []
    frame_time = 1.0 / 30.0
    in_motion = False
    end_site = False

    with open(bvh_file) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if in_motion:
                m = re.match(r"Frames:\s*(\d+)", line)
                if m:
                    continue
                m = re.match(r"Frame Time:\s*([\d.eE+-]+)", line)
                if m:
                    frame_time = float(m.group(1))
                    continue
                frames.append(np.fromstring(line, sep=" "))
                continue
            if line.startswith("MOTION"):
                in_motion = True
                continue
            m = re.match(r"(ROOT|JOINT)\s+(\w+)", line)
            if m:
                parent = stack[-1] if stack else -1
                joints.append(_Joint(m.group(2), parent))
                continue
            if line.startswith("End Site"):
                end_site = True
                continue
            if line.startswith("{"):
                if not end_site:
                    stack.append(len(joints) - 1)
                continue
            if line.startswith("}"):
                if end_site:
                    end_site = False
                else:
                    stack.pop()
                continue
            m = re.match(r"OFFSET\s+(.+)", line)
            if m and not end_site:
                joints[-1].offset = np.fromstring(m.group(1), sep=" ")
                continue
            m = re.match(r"CHANNELS\s+\d+\s+(.+)", line)
            if m:
                joints[-1].channels = m.group(1).split()
                continue

    return joints, np.array(frames), frame_time


def _fk(joints, frame_values):
    """Forward kinematics for one frame. Returns (positions, rotations) in
    the BVH (y-up, cm) frame; rotations are scipy Rotation objects."""
    n = len(joints)
    pos = np.zeros((n, 3))
    rot = [None] * n
    c = 0
    for i, j in enumerate(joints):
        local_t = j.offset.copy()
        angles = {}
        for ch in j.channels:
            v = frame_values[c]
            c += 1
            if ch.endswith("position"):
                # position channels carry the full local translation
                # (the static OFFSET is already baked in)
                local_t["XYZ".index(ch[0])] = v
            else:
                angles[ch[0]] = v
        if angles:
            order = "".join(ch[0] for ch in j.channels if ch.endswith("rotation"))
            local_r = R.from_euler(order.upper(), [angles[a] for a in order], degrees=True)
        else:
            local_r = R.identity()
        if j.parent < 0:
            pos[i] = local_t
            rot[i] = local_r
        else:
            pos[i] = pos[j.parent] + rot[j.parent].apply(local_t)
            rot[i] = rot[j.parent] * local_r
    return pos, rot


def load_soma_bvh_file(bvh_file):
    """Load a SOMA-skeleton BVH file.

    Returns (frames, human_height, fps) where frames is a list of dicts
    mapping joint name -> [position (m, z-up), wxyz quaternion].
    """
    joints, frame_data, frame_time = _parse_bvh(bvh_file)
    conv = R.from_matrix(_YUP_TO_ZUP)

    # standing height from the first frame (the zero pose of this rig is
    # bone-aligned, not a standing pose, so it can't be used)
    names = [j.name for j in joints]
    p0, _ = _fk(joints, frame_data[0])
    top = p0[names.index("HeadEnd")][1] if "HeadEnd" in names else p0[:, 1].max()
    bottom = min(p0[names.index(n)][1] for n in ("LeftToeEnd", "RightToeEnd") if n in names)
    human_height = (top - bottom) / 100.0

    frames = []
    for fv in frame_data:
        pos, rot = _fk(joints, fv)
        result = {}
        for i, j in enumerate(joints):
            p = _YUP_TO_ZUP @ pos[i] / 100.0
            q = (conv * rot[i]).as_quat(scalar_first=True)
            result[j.name] = [p, q]
        frames.append(result)

    return frames, human_height, 1.0 / frame_time
