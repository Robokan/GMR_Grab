"""Summarize the physics skeleton of each USD in a directory (debug helper)."""
import glob
import os
import sys

from pxr import Usd, UsdGeom, UsdPhysics

for path in sorted(glob.glob(os.path.join(sys.argv[1], "*.usd*"))):
    if path.endswith((".usda", ".usd", ".usdc")):
        try:
            stage = Usd.Stage.Open(path)
        except Exception as e:
            print(f"{os.path.basename(path)}: OPEN FAILED {e}")
            continue
        if not stage:
            print(f"{os.path.basename(path)}: open failed")
            continue
        n_rev = n_d6 = n_fix = n_body = n_finger_joint = 0
        joint_names = []
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                n_rev += 1
                joint_names.append(prim.GetName())
                if "Fing" in prim.GetName() or "Hand_2" in prim.GetName():
                    n_finger_joint += 1
            elif prim.IsA(UsdPhysics.FixedJoint):
                n_fix += 1
            elif prim.IsA(UsdPhysics.Joint) and not prim.IsA(UsdPhysics.PrismaticJoint):
                if prim.GetTypeName() == "PhysicsJoint":
                    n_d6 += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                n_body += 1
        arm_joints = sorted(n for n in joint_names if n.startswith("Arm") and n.endswith("_L_Joint"))
        print(f"{os.path.basename(path):45s} bodies={n_body:3d} revolute={n_rev:3d} "
              f"d6={n_d6} fixed={n_fix} finger_joints={n_finger_joint:2d} armL={len(arm_joints)}")
