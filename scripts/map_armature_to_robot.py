import omni.usd
from pxr import Usd, UsdSkel, UsdGeom, Sdf, Gf, UsdPhysics
import omni.kit.commands
import re
from collections import deque

# Dictionary for joint overrides
joint_info = {
    "Head": {"from": "Neck_1", "to": "Head", "revolute": {"axis":"Y"}},
    "Neck_2": {"from": "Chest", "to": "Neck_1", "revolute": {"axis":"X"}},
    "Backbone": {"from": "Waist", "to": "Chest", "revolute": {"axis": "X"}},
    "Twist": {"from": "Hip", "to": "Waist", "revolute": {"axis": "Y"}},
    "Arm_1_L": {"from": "Chest", "to": "Arm1_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}},
    "Arm_3_L": {"from": "Arm1_L", "to": "Arm2_L", "revolute": {"axis": "Z"}, "rotate": {"X": 180}},
    "Arm_4_L": {"from": "Arm2_L", "to": "Arm3_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}},
    "Arm_6_L": {"from": "Arm3_L", "to": "Arm4_L", "revolute": {"axis": "X"}},
    "Arm_7_L": {"from": "Arm4_L", "to": "Arm7_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}},
    "Arm_8_L": {"from": "Arm7_L", "to": "Arm5_L", "revolute": {"axis": "X"}},
    "Arm_9_L": {"from": "Arm5_L", "to": "Arm6_L", "revolute": {"axis": "Z"},"rotate": {"X": 180}},
    "Hand_L": {"from": "Arm6_L", "to": "Hand1_L", "fixed": {}},
    "Arm_1_R": {"from": "Chest", "to": "Arm1_R", "revolute": {"axis": "Y"}},
    "Arm_3_R": {"from": "Arm1_R", "to": "Arm2_R", "revolute": {"axis": "Z"}},
    "Arm_4_R": {"from": "Arm2_R", "to": "Arm3_R", "revolute": {"axis": "Y"}},
    "Arm_6_R": {"from": "Arm3_R", "to": "Arm4_R", "revolute": {"axis": "X"}},
    "Arm_7_R": {"from": "Arm4_R", "to": "Arm7_R", "revolute": {"axis": "Y"}},
    "Arm_8_R": {"from": "Arm7_R", "to": "Arm5_R", "revolute": {"axis": "X"}},
    "Arm_9_R": {"from": "Arm5_R", "to": "Arm6_R", "revolute": {"axis": "Z"}},
    "Hand_R": {"from": "Arm6_R", "to": "Hand1_R", "fixed": {}},
    "Finger_Index1_R": {"from": "Hand1_R", "to": "Finguer1_R", {"axis": "Y"}},
    "Finger_Index2_R": {"from": "Finguer1_R", "to": "Finguer2_R", {"axis": "Y"}},
    "Finger_Middle1_R": {"from": "Hand1_R", "to": "Finguer3_R", {"axis": "Y"}},
    "Finger_Middle2_R": {"from": "Finguer3_R", "to": "Finguer4_R", {"axis": "Y"}},
    "Hand2_R": {"from": "Hand1_R", "to": "Hand2_R", {"axis": "X"}},
    "Finger_Thumb1_R": {"from": "Hand2_R", "to": "Finguer5_R", {"axis": "Y"}},
    "Finger_Thumb2_R": {"from": "Finguer5_R", "to": "Finguer6_R", {"axis": "Y"}},
    "Leg_1_L": {"from": "Hip", "to": "Leg1_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}},
    "Leg_3_L": {"from": "Leg1_L", "to": "Leg2_L", "revolute": {"axis": "Z"},"rotate": {"X": 180}},
    "Leg_5_L": {"from": "Leg2_L", "to": "Leg3_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}},
    "Leg_8_L": {"from": "Leg3_L", "to": "Leg4_L", "revolute": {"axis": "Z"}, "rotate": {"X": 180}},
    "Foot_L": {"from": "Leg4_L", "to": "Foot_L", "d6": {"X": True, "Y":True, "Z":True}, "rotate": {"X": 180}},
    "Leg_1_R": {"from": "Hip", "to": "Leg1_R", "revolute": {"axis": "Y"}},
    "Leg_3_R": {"from": "Leg1_R", "to": "Leg2_R", "revolute": {"axis": "Z"}},
    "Leg_5_R": {"from": "Leg2_R", "to": "Leg3_R", "revolute": {"axis": "Y"}},
    "Leg_8_R": {"from": "Leg3_R", "to": "Leg4_R", "revolute": {"axis": "Z"}},
    "Foot_R": {"from": "Leg4_R", "to": "Foot_R", "d6": {"X": True, "Y":True,  "Z":True}},
}

# List of regular expressions to filter joints (leaf names, lowercased)
filter_patterns = [".*arm_[1,3,4,6,7,8,9]_[l,r].*", ".*hand.*_[l,r]", "head", "backbone", "twist", "neck_2","leg_.*", "foot.*"]

# Get the current stage and selected root Xform path
stage = omni.usd.get_context().get_stage()
selected_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
if not selected_paths:
    print("Please select the root Xform prim (containing Skeleton and meshes).")
else:
    root_path = Sdf.Path(selected_paths[0])
    root_prim = stage.GetPrimAtPath(root_path)
    print(f"Selected root: {root_path}")
    
    # Find the Skeleton prim under root Xform
    skeleton_prim = None
    for child in root_prim.GetChildren():
        if child.IsA(UsdSkel.Skeleton):
            skeleton_prim = child
            break
    
    if not skeleton_prim:
        print("No Skeleton prim found under the selected Xform.")
    else:
        print(f"Skeleton found: {skeleton_prim.GetPath()}")
        skel = UsdSkel.Skeleton(skeleton_prim)
        
        # Get the local transform of the skeleton prim
        skel_xform = UsdGeom.Xformable(skeleton_prim)
        skel_matrix = skel_xform.GetLocalTransformation(Usd.TimeCode.Default())
        
        # Get joints and rest transforms
        joints = skel.GetJointsAttr().Get()
        print("Skeleton joints:", joints)  # Added to debug joint names
        
        rest_xforms = skel.GetRestTransformsAttr().Get()
        
        if len(joints) != len(rest_xforms):
            print("Mismatch between joints and rest transforms.")
        else:
            # Find all Mesh prims under root (recursive)
            meshes = [prim for prim in Usd.PrimRange(root_prim) if prim.IsA(UsdGeom.Mesh)]
            
            print(f"Found {len(meshes)} Mesh prims.")
            
            # Collect intended body names from joint_info
            intended_body_names = set()
            for info in joint_info.values():
                if "from" in info:
                    intended_body_names.add(info["from"].lower())
                if "to" in info:
                    intended_body_names.add(info["to"].lower())
            
            # Get Xforms around meshes, add rigid bodies and colliders only to immediate children of root that are intended bodies
            body_prims = set()
            body_paths = []
            for mesh_prim in meshes:
                # Add collider to mesh
                UsdPhysics.CollisionAPI.Apply(mesh_prim)
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
                mesh_collision_api.CreateApproximationAttr().Set("convexHull")
                
                parent_prim = mesh_prim.GetParent()
                if parent_prim and parent_prim.IsA(UsdGeom.Xform) and parent_prim.GetParent() == root_prim and parent_prim.GetName().lower() in intended_body_names and parent_prim not in body_prims:
                    UsdPhysics.RigidBodyAPI.Apply(parent_prim)
                    body_prims.add(parent_prim)
                    body_paths.append(str(parent_prim.GetPath()))
            
            print("Added rigid bodies and colliders.")
            
            # Add centered pivots using Pivot Tool API
            try:
                omni.kit.commands.execute("PivotToolAddPivot", prim_paths=body_paths, center_pivot=True)
                print("Added centered pivots to Xforms containing meshes.")
            except Exception as e:
                print(f"Failed to execute PivotToolAddPivot: {e}")
                # Fallback to manual pivot centering
                time = Usd.TimeCode.Default()
                bbox_cache = UsdGeom.BBoxCache(time, ['default'])
                for parent_prim in body_prims:
                    parent_xform = UsdGeom.Xformable(parent_prim)
                    bounds = bbox_cache.ComputeUntransformedBound(parent_prim)
                    center = bounds.ComputeCentroid()
                    vec_center = Gf.Vec3f(center)
                    vec_neg_center = Gf.Vec3f(-center)
                    
                    ops = parent_xform.GetOrderedXformOps()
                    pivot_op = parent_xform.AddTranslateOp(opSuffix="pivot", precision=UsdGeom.XformOp.PrecisionFloat)
                    pivot_op.Set(vec_center)
                    inverse_op = parent_xform.AddTranslateOp(opSuffix="pivotInverse", precision=UsdGeom.XformOp.PrecisionFloat)
                    inverse_op.Set(vec_neg_center)
                    new_order = [pivot_op] + ops + [inverse_op]
                    parent_xform.SetXformOpOrder(new_order)
                print("Added centered pivots manually.")
            
            # Clear selection to avoid UI errors on removed prims
            omni.usd.get_context().get_selection().clear_selected_prim_paths()
            
            # Compute world matrices for joints
            joint_to_world = {}
            joint_index = {joint: i for i, joint in enumerate(joints)}
            for i, joint in enumerate(joints):
                joint_sdf = Sdf.Path(joint)
                parent_sdf = joint_sdf.GetParentPath()
                parent_joint = str(parent_sdf) if str(parent_sdf) else None
                matrix = Gf.Matrix4d(rest_xforms[i])
                if parent_sdf == Sdf.Path():
                    matrix = skel_matrix * matrix
                    joint_to_world[joint] = matrix
                else:
                    if parent_joint in joint_to_world:
                        joint_to_world[joint] = joint_to_world[parent_joint] * matrix
                    else:
                        print(f"Parent {parent_joint} not computed for {joint}")
                        continue
            
            # Create cubes Xform
            cubes_path = root_path.AppendChild("Cubes")
            if not stage.GetPrimAtPath(cubes_path):
                UsdGeom.Xform.Define(stage, cubes_path)
            UsdGeom.Imageable(stage.GetPrimAtPath(cubes_path)).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            
            time = Usd.TimeCode.Default()
            root_world = UsdGeom.Xformable(root_prim).ComputeLocalToWorldTransform(time)
            inv_root = root_world.GetInverse()
            
            # Dictionary to store cube transforms for joints
            joint_to_cube_transform = {}
            
            # Rebuild hierarchy for debug with spheres at joint locations
            markers_hierarchy_path = root_path.AppendChild("MarkersHierarchy")
            if not stage.GetPrimAtPath(markers_hierarchy_path):
                UsdGeom.Xform.Define(stage, markers_hierarchy_path)
            UsdGeom.Imageable(stage.GetPrimAtPath(markers_hierarchy_path)).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            
            for i, joint in enumerate(joints):
                joint_sdf = Sdf.Path(joint)
                joint_rel_path = joint.lstrip('/')
                full_path = markers_hierarchy_path.AppendPath(joint_rel_path)
                
                # Create Xform
                xform_prim = UsdGeom.Xform.Define(stage, full_path)
                xform = UsdGeom.Xformable(xform_prim)
                
                matrix = Gf.Matrix4d(rest_xforms[i])
                parent_sdf = joint_sdf.GetParentPath()
                parent_joint = str(parent_sdf) if str(parent_sdf) else None
                if parent_joint is None or parent_joint == '':
                    matrix = skel_matrix * matrix
                    flip_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 180)
                    flip_matrix = Gf.Matrix4d(1.0).SetRotate(flip_rot)
                    matrix = matrix * flip_matrix
                
                transform_op = xform.AddTransformOp()
                xform.SetXformOpOrder([transform_op])
                transform_op.Set(matrix)
                
                # Add sphere child if in filter
                leaf_lower = Sdf.Path(joint).name.lower()
                if any(re.match(pattern, leaf_lower) for pattern in filter_patterns):
                    sphere_path = full_path.AppendChild("Sphere")
                    sphere = UsdGeom.Sphere.Define(stage, sphere_path)
                    sphere.CreateRadiusAttr().Set(0.1)
                    
                    # Create corresponding cube
                    cube_joint_name = Sdf.Path(joint).name
                    cube_path = cubes_path.AppendChild(cube_joint_name)
                    if stage.GetPrimAtPath(cube_path):
                        stage.RemovePrim(cube_path)
                    
                    cube = UsdGeom.Cube.Define(stage, cube_path)
                    cube.CreateSizeAttr().Set(0.2)
                    
                    sphere_world = UsdGeom.Xformable(sphere.GetPrim()).ComputeLocalToWorldTransform(time)
                    relative_matrix = inv_root * sphere_world
                    
                    cube_xform = UsdGeom.Xformable(cube.GetPrim())
                    cube_xform.ClearXformOpOrder()
                    transform_op = cube_xform.AddTransformOp()
                    cube_xform.SetXformOpOrder([transform_op])
                    transform_op.Set(relative_matrix)
                    
                    # Store the cube's transform for this joint
                    joint_to_cube_transform[joint] = relative_matrix
            
            print("Rebuilt hierarchy under 'MarkersHierarchy' with spheres and cubes at joint locations.")
            
            # Create a mapping from body names to paths (case-insensitive)
            name_to_path = {prim.GetName().lower(): prim.GetPath() for prim in body_prims}
            
            # Create mapping from leaf joint name to full joint path
            leaf_to_full_joint = {Sdf.Path(j).name.lower(): j for j in joints}
            
            # Create top-level joints Xform
            joints_path = root_path.AppendChild("Joints")
            if not stage.GetPrimAtPath(joints_path):
                UsdGeom.Xform.Define(stage, joints_path)
            
            # Add articulation roots to top-level bodies specified in joint_info
            for joint_name, info in joint_info.items():
                if "from" in info:
                    parent_name = info["from"].lower()
                    if parent_name in name_to_path:
                        top_path = name_to_path[parent_name]
                        top_prim = stage.GetPrimAtPath(top_path)
                        UsdPhysics.ArticulationRootAPI.Apply(top_prim)
                    else:
                        print(f"Parent body {info['from']} not found for joint {joint_name}, skipping articulation root.")
            
            # Add joints under /root/Joints with positions and rotations from cube transforms
            time = Usd.TimeCode.Default()
            for joint_name, info in joint_info.items():
                joint_name_key = joint_name
                
                # Skip if joint doesn't match filter patterns
                child_leaf = joint_name_key.lower()
                if not any(re.match(pattern, child_leaf) for pattern in filter_patterns):
                    continue
                
                # Get full joint path for cube transform lookup
                if joint_name_key.lower() not in leaf_to_full_joint:
                    print(f"No full joint path found for {joint_name_key}, skipping.")
                    continue
                joint_full_path = leaf_to_full_joint[joint_name_key.lower()]
                
                # Get parent and child paths from joint_info
                if "from" not in info or "to" not in info:
                    print(f"Missing 'from' or 'to' in joint_info for {joint_name_key}, skipping.")
                    continue
                
                parent_name = info["from"].lower()
                child_name = info["to"].lower()
                
                if parent_name not in name_to_path or child_name not in name_to_path:
                    print(f"Parent {info['from']} or child {info['to']} not found for {joint_name_key}, skipping.")
                    continue
                
                parent_path = name_to_path[parent_name]
                child_path = name_to_path[child_name]
                
                # Determine joint type and axis
                joint_type = None
                axis = "X"  # Default axis for revolute joints
                if "revolute" in info:
                    joint_type = "revolute"
                    if "axis" in info["revolute"]:
                        axis = info["revolute"]["axis"]
                elif "fixed" in info:
                    joint_type = "fixed"
                elif "d6" in info:
                    joint_type = "d6"
                else:
                    print(f"Unknown joint type for {joint_name_key}, skipping.")
                    continue
                
                # Create joint prim under /root/Joints
                joint_path = joints_path.AppendChild(joint_name_key + "_Joint")
                if stage.GetPrimAtPath(joint_path):
                    stage.RemovePrim(joint_path)
                
                # Create joint based on type
                if joint_type == "revolute":
                    joint_prim = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
                    joint_prim.CreateAxisAttr(axis)
                elif joint_type == "fixed":
                    joint_prim = UsdPhysics.FixedJoint.Define(stage, joint_path)
                elif joint_type == "d6":
                    joint_prim = UsdPhysics.Joint.Define(stage, joint_path)
                    d6_info = info["d6"]
                    # Lock all translations
                    # for trans_axis in ["X", "Y", "Z"]:
                    #     trans_dof = "trans" + trans_axis
                    #     limit_api = UsdPhysics.LimitAPI.Apply(joint_prim.GetPrim(), trans_dof)
                    #     limit_api.CreateLowAttr().Set(1.0)
                    #     limit_api.CreateHighAttr().Set(-1.0)
                    # # Configure rotations based on d6_info
                    # for rot_axis in ["X", "Y", "Z"]:
                    #     rot_dof = "rot" + rot_axis
                    #     if rot_axis in d6_info and d6_info[rot_axis]:
                    #         # free: no limit
                    #         pass
                    #     else:
                    #         # locked
                    #         limit_api = UsdPhysics.LimitAPI.Apply(joint_prim.GetPrim(), rot_dof)
                    #         limit_api.CreateLowAttr().Set(1.0)
                    #         limit_api.CreateHighAttr().Set(-1.0)
                    #Add drives for free rotations
                    for rot_axis in ["X", "Y", "Z"]:
                        if rot_axis in d6_info and d6_info[rot_axis]:
                            drive_dof = "rot" + rot_axis
                            drive = UsdPhysics.DriveAPI.Apply(joint_prim.GetPrim(), drive_dof)
                            drive.CreateTypeAttr("force")
                            drive.CreateDampingAttr(10)
                            drive.CreateStiffnessAttr(100.0)
                else:
                    continue
                
                joint_prim.CreateBody0Rel().SetTargets([parent_path])
                joint_prim.CreateBody1Rel().SetTargets([child_path])
                
                # Set local positions and rotations using the cube's transform
                if joint_full_path in joint_to_cube_transform:
                    joint_matrix = joint_to_cube_transform[joint_full_path]
                    joint_trans = joint_matrix.ExtractTranslation()
                    joint_rot = joint_matrix.ExtractRotation().GetQuat()
                    
                    # Convert joint_rot to Gf.Quatf
                    joint_rot = Gf.Quatf(float(joint_rot.GetReal()), Gf.Vec3f(joint_rot.GetImaginary()))
                    
                    print(f"Original joint_rot for {joint_name_key}: {joint_rot}")
                    
                    # Apply additional rotation from 'rotate' dictionary if present
                    if "rotate" in info:
                        rot_dict = info["rotate"]
                        # Initialize identity quaternion
                        extra_rot = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
                        # Apply rotations in Z, Y, X order (to apply X first, then Y, then Z when multiplying extra_rot = rot_quat * extra_rot)
                        for axis in ["Z", "Y", "X"]:
                            if axis in rot_dict:
                                angle_deg = float(rot_dict[axis])
                                axis_vec = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[axis]
                                rot = Gf.Rotation(axis_vec, angle_deg)
                                rot_quat = Gf.Quatf(float(rot.GetQuaternion().GetReal()), Gf.Vec3f(rot.GetQuaternion().GetImaginary()))
                                # Compose rotations (rot_quat applied first)
                                extra_rot = rot_quat * extra_rot
                        # Apply extra rotation after the original joint_rot
                        joint_rot = joint_rot * extra_rot
                        print(f"Modified joint_rot for {joint_name_key}: {joint_rot}")
                    
                    parent_prim = stage.GetPrimAtPath(parent_path)
                    child_prim = stage.GetPrimAtPath(child_path)
                    parent_world = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(time)
                    child_world = UsdGeom.Xformable(child_prim).ComputeLocalToWorldTransform(time)
                    inv_parent = parent_world.GetInverse()
                    inv_child = child_world.GetInverse()
                    
                    # Transform position to local spaces
                    local_pos0 = inv_parent.Transform(Gf.Vec3d(joint_trans))
                    local_pos1 = inv_child.Transform(Gf.Vec3d(joint_trans))
                    joint_prim.CreateLocalPos0Attr().Set(Gf.Vec3f(local_pos0))
                    joint_prim.CreateLocalPos1Attr().Set(Gf.Vec3f(local_pos1))
                    
                    # Transform rotation to local spaces
                    parent_rot = parent_world.ExtractRotation().GetQuat()
                    child_rot = child_world.ExtractRotation().GetQuat()
                    # Convert parent and child rotations to Gf.Quatf
                    parent_rot = Gf.Quatf(float(parent_rot.GetReal()), Gf.Vec3f(parent_rot.GetImaginary()))
                    child_rot = Gf.Quatf(float(child_rot.GetReal()), Gf.Vec3f(child_rot.GetImaginary()))
                    inv_parent_rot = parent_rot.GetInverse()
                    inv_child_rot = child_rot.GetInverse()
                    local_rot0 = inv_parent_rot * joint_rot
                    local_rot1 = inv_child_rot * joint_rot
                    
                    # Convert Vec3d to Vec3f for Gf.Quatf
                    local_rot0_imag = Gf.Vec3f(local_rot0.GetImaginary())
                    local_rot1_imag = Gf.Vec3f(local_rot1.GetImaginary())
                    joint_prim.CreateLocalRot0Attr().Set(Gf.Quatf(local_rot0.GetReal(), local_rot0_imag))
                    joint_prim.CreateLocalRot1Attr().Set(Gf.Quatf(local_rot1.GetReal(), local_rot1_imag))
                    print(f"LocalRot0 for {joint_name_key}: {joint_prim.GetLocalRot0Attr().Get()}")
                    print(f"LocalRot1 for {joint_name_key}: {joint_prim.GetLocalRot1Attr().Get()}")
                else:
                    print(f"No cube transform found for {joint_full_path}, using default position and rotation.")
                    joint_prim.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
                    joint_prim.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
                    joint_prim.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
                    joint_prim.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
                
                if joint_type == "revolute":
                    # Add drive for revolute joints
                    drive = UsdPhysics.DriveAPI.Apply(joint_prim.GetPrim(), "angular")
                    drive.CreateTypeAttr("force")
                    drive.CreateDampingAttr(10)
                    drive.CreateStiffnessAttr(100.0)
            
            print("Added joints and drives under top-level 'Joints' Xform with positions and rotations from cube transforms.")
