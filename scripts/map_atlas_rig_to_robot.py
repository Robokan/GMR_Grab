import omni.usd
from pxr import Usd, UsdSkel, UsdGeom, Sdf, Gf, UsdPhysics
import omni.kit.commands
import re
from collections import deque

# Dictionary for joint overrides
# Each joint can have:
#   "drive": {"damping": N, "stiffness": N} - drive parameters (defaults: damping=10, stiffness=100)
#   "limits": {"low": N, "high": N} - joint limits in degrees (revolute angular)
#   For d6 joints, per-axis drive/limits:
#     "d6": {"X": {...}, "Y": {...}, "Z": {...}}
#     where each axis dict can have: true (use defaults), or
#       {"drive": {"damping": N, "stiffness": N}, "limits": {"low": N, "high": N}}
joint_info = {
    "Head": {"from": "Neck_1", "to": "Head", "revolute": {"axis":"Y"}, "drive": {"damping": 10, "stiffness": 100}},
    "Neck_2": {"from": "Chest", "to": "Neck_1", "revolute": {"axis":"X"}, "drive": {"damping": 10, "stiffness": 100}},
    "Backbone": {"from": "Waist", "to": "Chest", "revolute": {"axis": "X"}, "drive": {"damping": 10, "stiffness": 100}},
    "Twist": {"from": "Hip", "to": "Waist", "revolute": {"axis": "Y"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_1_L": {"from": "Chest", "to": "Arm1_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_3_L": {"from": "Arm1_L", "to": "Arm2_L", "revolute": {"axis": "Z"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_4_L": {"from": "Arm2_L", "to": "Arm3_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_6_L": {"from": "Arm3_L", "to": "Arm4_L", "revolute": {"axis": "X"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_7_L": {"from": "Arm4_L", "to": "Arm7_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_8_L": {"from": "Arm7_L", "to": "Arm5_L", "revolute": {"axis": "X"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_9_L": {"from": "Arm5_L", "to": "Arm6_L", "revolute": {"axis": "Z"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Hand_L": {"from": "Arm6_L", "to": "Hand1_L", "fixed": {}},
    "Arm_1_R": {"from": "Chest", "to": "Arm1_R", "revolute": {"axis": "Y"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_3_R": {"from": "Arm1_R", "to": "Arm2_R", "revolute": {"axis": "Z"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_4_R": {"from": "Arm2_R", "to": "Arm3_R", "revolute": {"axis": "Y"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_6_R": {"from": "Arm3_R", "to": "Arm4_R", "revolute": {"axis": "X"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_7_R": {"from": "Arm4_R", "to": "Arm7_R", "revolute": {"axis": "Y"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_8_R": {"from": "Arm7_R", "to": "Arm5_R", "revolute": {"axis": "X"}, "drive": {"damping": 10, "stiffness": 100}},
    "Arm_9_R": {"from": "Arm5_R", "to": "Arm6_R", "revolute": {"axis": "Z"}, "drive": {"damping": 10, "stiffness": 100}},
    "Hand_R": {"from": "Arm6_R", "to": "Hand1_R", "fixed": {}},
    "Leg_1_L": {"from": "Hip", "to": "Leg1_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Leg_3_L": {"from": "Leg1_L", "to": "Leg2_L", "revolute": {"axis": "Z"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Leg_5_L": {"from": "Leg2_L", "to": "Leg3_L", "revolute": {"axis": "Y"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    "Leg_8_L": {"from": "Leg3_L", "to": "Leg4_L", "revolute": {"axis": "Z"}, "rotate": {"X": 180}, "drive": {"damping": 10, "stiffness": 100}},
    # Foot L/R swap: the original rig has Foot_L body on the right side and vice versa
    "Foot_L": {"from": "Leg4_L", "to": "Foot_R", "rotate": {"X": 180},
               "d6": {
                   "X": {"drive": {"damping": 10, "stiffness": 100}, "limits": {"low": -45, "high": 45}},
                   "Y": {"drive": {"damping": 20, "stiffness": 200}, "limits": {"low": -90, "high": 45}},
                   "Z": {"drive": {"damping": 10, "stiffness": 100}, "limits": {"low": -45, "high": 45}},
               }},
    "Leg_1_R": {"from": "Hip", "to": "Leg1_R", "revolute": {"axis": "Y"}, "drive": {"damping": 10, "stiffness": 100}},
    "Leg_3_R": {"from": "Leg1_R", "to": "Leg2_R", "revolute": {"axis": "Z"}, "drive": {"damping": 10, "stiffness": 100}},
    "Leg_5_R": {"from": "Leg2_R", "to": "Leg3_R", "revolute": {"axis": "Y"}, "drive": {"damping": 10, "stiffness": 100}},
    "Leg_8_R": {"from": "Leg3_R", "to": "Leg4_R", "revolute": {"axis": "Z"}, "drive": {"damping": 10, "stiffness": 100}},
    # Foot L/R swap: the original rig has Foot_R body on the left side and vice versa
    "Foot_R": {"from": "Leg4_R", "to": "Foot_L",
               "d6": {
                   "X": {"drive": {"damping": 10, "stiffness": 100}, "limits": {"low": -45, "high": 45}},
                   "Y": {"drive": {"damping": 20, "stiffness": 200}, "limits": {"low": -90, "high": 45}},
                   "Z": {"drive": {"damping": 10, "stiffness": 100}, "limits": {"low": -45, "high": 45}},
               }},
}

# List of regular expressions to filter joints (leaf names, lowercased)
filter_patterns = [".*arm_[1,3,4,6,7,8,9]_[l,r].*", ".*hand.*_[l,r]", "head", "backbone", "twist", "neck_2","leg_.*", "foot.*"]

# Xforms to completely remove from the stage (hydraulics and other non-essential
# geometry near joints that can cause collider interference)
exclude_xforms = [
    "Hidraulicfoot1_L", "Hidraulicfoot1_R",
    "Hidraulicfoot2_L", "Hidraulicfoot2_R",
    "Hidraulicleg1_L", "Hidraulicleg1_R",
    "Hidraulicleg2_L", "Hidraulicleg2_R",
    "Hidraulicwaist_L", "Hidraulicwaist_R",
]

# Get the current stage and auto-detect root from skeleton location
stage = omni.usd.get_context().get_stage()

# Find the skeleton anywhere in the stage, use its parent Xform as root
skeleton_prim = None
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    if prim.IsA(UsdSkel.Skeleton):
        skeleton_prim = prim
        break

if not skeleton_prim:
    print("No Skeleton prim found in the stage.")
else:
    root_prim = skeleton_prim.GetParent()
    root_path = root_prim.GetPath()
    print(f"Found skeleton at: {skeleton_prim.GetPath()}")
    print(f"Using parent as root: {root_path}")

    # ---- Remove excluded Xforms (hydraulics etc.) before any processing ----
    excluded_count = 0
    for xf_name in exclude_xforms:
        xf_path = root_path.AppendChild(xf_name)
        if stage.GetPrimAtPath(xf_path):
            stage.RemovePrim(xf_path)
            excluded_count += 1
    if excluded_count:
        print(f"Removed {excluded_count} excluded Xforms: {', '.join(exclude_xforms[:5])}...")

    skel = UsdSkel.Skeleton(skeleton_prim)

    # Get the local transform of the skeleton prim
    skel_xform = UsdGeom.Xformable(skeleton_prim)
    skel_matrix = skel_xform.GetLocalTransformation(Usd.TimeCode.Default())

    # Get joints and rest transforms
    joints = skel.GetJointsAttr().Get()
    print(f"Skeleton has {len(joints)} joints")

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

        # ---- Build dynamic reparent mapping from skeleton hierarchy ----
        # For each non-body Xform (has mesh but no rigid body), use the skeleton
        # hierarchy to find the nearest ancestor that IS a body, then reparent into it.

        # Collect all Armature Xform children that have meshes
        armature_xform_names = set()
        body_xform_names = set()
        non_body_xform_names = set()
        for child in root_prim.GetChildren():
            if child.IsA(UsdGeom.Xform) and not child.IsA(UsdSkel.Skeleton):
                has_mesh = any(c.IsA(UsdGeom.Mesh) for c in child.GetChildren())
                if has_mesh:
                    name = child.GetName()
                    armature_xform_names.add(name)
                    if name.lower() in intended_body_names:
                        body_xform_names.add(name)
                    else:
                        non_body_xform_names.add(name)

        # Build name lookup between skeleton joint leaf names and Armature Xform names
        def normalize_name(name):
            """Remove underscores and lowercase for fuzzy matching."""
            return name.replace("_", "").lower()

        # Map: normalized skeleton leaf name -> full skeleton joint path
        skel_norm_to_path = {}
        for j in joints:
            leaf = Sdf.Path(j).name
            skel_norm_to_path[normalize_name(leaf)] = j

        # Map: normalized Armature Xform name -> actual Armature Xform name
        xform_norm_to_name = {}
        for name in armature_xform_names:
            xform_norm_to_name[normalize_name(name)] = name

        # Bridge: use joint_info to map skeleton joint names -> Armature body Xform names
        for jkey, jval in joint_info.items():
            jnorm = normalize_name(jkey)
            if "to" in jval and jval["to"] in body_xform_names:
                xform_norm_to_name[jnorm] = jval["to"]

        # Map skeleton intermediate joints to body Xforms
        for j in joints:
            leaf = Sdf.Path(j).name
            leaf_norm = normalize_name(leaf)
            if leaf_norm not in xform_norm_to_name:
                for jkey, jval in joint_info.items():
                    if "to" in jval:
                        to_norm = normalize_name(jval["to"])
                        to_skel = skel_norm_to_path.get(to_norm)
                        if to_skel:
                            to_parent = Sdf.Path(to_skel).GetParentPath()
                            if to_parent != Sdf.Path() and normalize_name(to_parent.name) == leaf_norm:
                                if "from" in jval and jval["from"] in body_xform_names:
                                    xform_norm_to_name[leaf_norm] = jval["from"]

        # For each non-body Xform, find it in the skeleton and walk up to nearest body ancestor
        CHILD_TO_RIGIDBODY = {}

        for nb_name in non_body_xform_names:
            nb_norm = normalize_name(nb_name)

            if nb_norm not in skel_norm_to_path:
                # Try to infer from name
                found = False
                for body_name in body_xform_names:
                    if body_name.lower() in nb_name.lower():
                        CHILD_TO_RIGIDBODY[nb_name] = body_name
                        found = True
                        break
                if not found:
                    print(f"  WARNING: Cannot find parent body for '{nb_name}'")
                continue

            skel_path = skel_norm_to_path[nb_norm]
            current = Sdf.Path(skel_path)
            found_body = None
            while True:
                current = current.GetParentPath()
                if current == Sdf.Path() or str(current) == ".":
                    break
                ancestor_leaf = current.name
                ancestor_norm = normalize_name(ancestor_leaf)
                if ancestor_norm in xform_norm_to_name:
                    ancestor_xform = xform_norm_to_name[ancestor_norm]
                    if ancestor_xform in body_xform_names:
                        found_body = ancestor_xform
                        break

            if found_body:
                CHILD_TO_RIGIDBODY[nb_name] = found_body
            else:
                for body_name in body_xform_names:
                    if body_name.lower() in nb_name.lower():
                        CHILD_TO_RIGIDBODY[nb_name] = body_name
                        break
                else:
                    print(f"  WARNING: Cannot find parent body for '{nb_name}'")

        print(f"Built reparent mapping ({len(CHILD_TO_RIGIDBODY)} non-body -> body):")
        for child, parent in sorted(CHILD_TO_RIGIDBODY.items()):
            print(f"  {child} -> {parent}")

        # ---- Add rigid bodies and colliders ----
        body_prims = set()
        body_paths = []
        collider_count = 0
        sub_collider_count = 0
        for mesh_prim in meshes:
            parent_prim = mesh_prim.GetParent()
            if not (parent_prim and parent_prim.IsA(UsdGeom.Xform) and parent_prim.GetParent() == root_prim):
                continue

            parent_name = parent_prim.GetName()

            # Case 1: Direct body part
            if parent_name.lower() in intended_body_names:
                UsdPhysics.CollisionAPI.Apply(mesh_prim)
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
                mesh_collision_api.CreateApproximationAttr().Set("convexHull")
                collider_count += 1
                if parent_prim not in body_prims:
                    UsdPhysics.RigidBodyAPI.Apply(parent_prim)
                    body_prims.add(parent_prim)
                    body_paths.append(str(parent_prim.GetPath()))
            # Case 2: Sub-component that will be reparented under a rigid body
            elif parent_name in CHILD_TO_RIGIDBODY:
                UsdPhysics.CollisionAPI.Apply(mesh_prim)
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
                mesh_collision_api.CreateApproximationAttr().Set("convexHull")
                sub_collider_count += 1

        print(f"Added rigid bodies to {len(body_prims)} body prims, colliders to {collider_count} body meshes + {sub_collider_count} sub-component meshes.")

        # ---- Reparent non-body Xforms into their parent rigid bodies ----
        layer = stage.GetRootLayer()
        time = Usd.TimeCode.Default()
        reparented = 0
        for child_name, parent_body in CHILD_TO_RIGIDBODY.items():
            src = root_path.AppendChild(child_name)
            dst = root_path.AppendChild(parent_body).AppendChild(child_name)
            child_prim = stage.GetPrimAtPath(src)
            new_parent_prim = stage.GetPrimAtPath(root_path.AppendChild(parent_body))
            if not child_prim or not new_parent_prim:
                continue

            # Compute transforms to preserve world position
            child_world = UsdGeom.Xformable(child_prim).ComputeLocalToWorldTransform(time)
            new_parent_world = UsdGeom.Xformable(new_parent_prim).ComputeLocalToWorldTransform(time)
            new_local = child_world * new_parent_world.GetInverse()

            # Move the prim
            Sdf.CopySpec(layer, src, layer, dst)
            stage.RemovePrim(src)

            # Overwrite transform to preserve world position
            moved_prim = stage.GetPrimAtPath(dst)
            if moved_prim:
                xf = UsdGeom.Xformable(moved_prim)
                xf.ClearXformOpOrder()
                mat_op = xf.AddTransformOp()
                mat_op.Set(new_local)

            reparented += 1
        print(f"Reparented {reparented} non-body Xforms into their parent rigid bodies.")

        # ---- Add centered pivots using Pivot Tool API ----
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

        print(f"Created {len(joint_to_cube_transform)} joint markers.")

        # Create a mapping from body names to paths (case-insensitive)
        name_to_path = {prim.GetName().lower(): prim.GetPath() for prim in body_prims}

        # Create mapping from leaf joint name to full joint path
        leaf_to_full_joint = {Sdf.Path(j).name.lower(): j for j in joints}

        # Create top-level joints Xform
        joints_path = root_path.AppendChild("Joints")
        if not stage.GetPrimAtPath(joints_path):
            UsdGeom.Xform.Define(stage, joints_path)

        # Add ArticulationRootAPI only to the root body (Hip).
        # PhysX requires exactly ONE articulation root for the entire kinematic chain.
        hip_key = "hip"
        if hip_key in name_to_path:
            hip_prim = stage.GetPrimAtPath(name_to_path[hip_key])
            UsdPhysics.ArticulationRootAPI.Apply(hip_prim)
            print(f"Applied ArticulationRootAPI to Hip: {name_to_path[hip_key]}")
        else:
            print("WARNING: Hip body not found, cannot apply ArticulationRootAPI")

        # Add joints under /root/Joints with positions and rotations from cube transforms
        time = Usd.TimeCode.Default()
        created_joints = 0
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
                jp = joint_prim.GetPrim()
                FLT_MAX = 3.4028234663852886e+38

                # Explicit joint properties for stability
                joint_prim.CreateCollisionEnabledAttr().Set(False)
                joint_prim.CreateBreakForceAttr().Set(FLT_MAX)
                joint_prim.CreateBreakTorqueAttr().Set(FLT_MAX)
                joint_prim.CreateExcludeFromArticulationAttr().Set(False)
                jp.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)

                # PhysX-specific joint properties
                jp.CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(0.02)
                jp.CreateAttribute("physxJoint:jointFriction", Sdf.ValueTypeNames.Float).Set(0.0)
                jp.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(1000000.0)

                # Lock translational DOFs: low > high means LOCKED
                for trans_dof in ["transX", "transY", "transZ"]:
                    limit_api = UsdPhysics.LimitAPI.Apply(jp, trans_dof)
                    limit_api.CreateLowAttr().Set(1.0)
                    limit_api.CreateHighAttr().Set(-1.0)

                # Configure rotational DOFs from per-axis config
                for rot_axis in ["X", "Y", "Z"]:
                    if rot_axis not in d6_info or not d6_info[rot_axis]:
                        continue
                    axis_cfg = d6_info[rot_axis]
                    drive_dof = "rot" + rot_axis

                    # Drive params
                    drv_damping = 10.0
                    drv_stiffness = 100.0
                    if isinstance(axis_cfg, dict) and "drive" in axis_cfg:
                        drv_damping = float(axis_cfg["drive"].get("damping", 10))
                        drv_stiffness = float(axis_cfg["drive"].get("stiffness", 100))

                    drive = UsdPhysics.DriveAPI.Apply(jp, drive_dof)
                    drive.CreateTypeAttr("force")
                    drive.CreateDampingAttr(drv_damping)
                    drive.CreateStiffnessAttr(drv_stiffness)
                    drive.CreateMaxForceAttr(FLT_MAX)
                    drive.CreateTargetPositionAttr(0.0)
                    drive.CreateTargetVelocityAttr(0.0)

                    # Limits: from per-axis config
                    if isinstance(axis_cfg, dict) and "limits" in axis_cfg:
                        limit_api = UsdPhysics.LimitAPI.Apply(jp, drive_dof)
                        limit_api.CreateLowAttr().Set(float(axis_cfg["limits"]["low"]))
                        limit_api.CreateHighAttr().Set(float(axis_cfg["limits"]["high"]))

                        # PhysX-specific limit enforcement properties
                        jp.CreateAttribute(f"physxLimit:{drive_dof}:stiffness", Sdf.ValueTypeNames.Float).Set(drv_stiffness)
                        jp.CreateAttribute(f"physxLimit:{drive_dof}:damping", Sdf.ValueTypeNames.Float).Set(drv_damping)
                        jp.CreateAttribute(f"physxLimit:{drive_dof}:restitution", Sdf.ValueTypeNames.Float).Set(0.0)
                        jp.CreateAttribute(f"physxLimit:{drive_dof}:bounceThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
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
                    extra_rot = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
                    for ax in ["Z", "Y", "X"]:
                        if ax in rot_dict:
                            angle_deg = float(rot_dict[ax])
                            axis_vec = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[ax]
                            rot = Gf.Rotation(axis_vec, angle_deg)
                            rot_quat = Gf.Quatf(float(rot.GetQuaternion().GetReal()), Gf.Vec3f(rot.GetQuaternion().GetImaginary()))
                            extra_rot = rot_quat * extra_rot
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
                parent_rot = Gf.Quatf(float(parent_rot.GetReal()), Gf.Vec3f(parent_rot.GetImaginary()))
                child_rot = Gf.Quatf(float(child_rot.GetReal()), Gf.Vec3f(child_rot.GetImaginary()))
                inv_parent_rot = parent_rot.GetInverse()
                inv_child_rot = child_rot.GetInverse()
                local_rot0 = inv_parent_rot * joint_rot
                local_rot1 = inv_child_rot * joint_rot

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
                # Add drive with configurable params from joint_info
                drive_cfg = info.get("drive", {})
                drive = UsdPhysics.DriveAPI.Apply(joint_prim.GetPrim(), "angular")
                drive.CreateTypeAttr("force")
                drive.CreateDampingAttr(float(drive_cfg.get("damping", 10)))
                drive.CreateStiffnessAttr(float(drive_cfg.get("stiffness", 100)))
                # Joint limits for revolute
                if "limits" in info:
                    limit_api = UsdPhysics.LimitAPI.Apply(joint_prim.GetPrim(), "angular")
                    limit_api.CreateLowAttr().Set(float(info["limits"]["low"]))
                    limit_api.CreateHighAttr().Set(float(info["limits"]["high"]))

            created_joints += 1
            print(f"  Created {joint_type} joint: {joint_name_key}")

        print(f"\nCreated {created_joints} physics joints.")
        print("Done. Save the stage to preserve changes.")
