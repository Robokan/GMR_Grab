#!/usr/bin/env python3
"""
USD Robot to MuJoCo XML Converter

Converts a USD file containing a physics-articulated robot (with PhysicsRevoluteJoint,
PhysicsFixedJoint, PhysicsPrismaticJoint, etc.) into a MuJoCo MJCF XML file.

Requirements:
    - pxr (OpenUSD Python bindings)

Usage:
    python usd_to_mujoco.py input.usda --output_dir ./output
"""

import argparse
import math
import os
import sys
from collections import defaultdict, deque
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

try:
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
except ImportError:
    print("Error: pxr (OpenUSD) is required. Install via 'pip install usd-core' or use Isaac Sim's Python.")
    sys.exit(1)


# ─── Pure-Python quaternion / vector math ─────────────────────────────────────

def vec3(x=0.0, y=0.0, z=0.0):
    return (float(x), float(y), float(z))

def vec3_add(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])

def vec3_sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def vec3_scale(v, s):
    return (v[0]*s, v[1]*s, v[2]*s)

def vec3_norm(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

def vec3_normalize(v):
    n = vec3_norm(v)
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return (v[0]/n, v[1]/n, v[2]/n)

def quat(w=1.0, x=0.0, y=0.0, z=0.0):
    return (float(w), float(x), float(y), float(z))

def quat_from_gf(q):
    if q is None:
        return (1.0, 0.0, 0.0, 0.0)
    r = float(q.GetReal())
    im = q.GetImaginary()
    return (r, float(im[0]), float(im[1]), float(im[2]))

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_inv(q):
    return (q[0], -q[1], -q[2], -q[3])

def quat_normalize(q):
    n = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)

def quat_rotate(q, v):
    """Rotate vector v=(x,y,z) by quaternion q=(w,x,y,z)."""
    qv = (0.0, v[0], v[1], v[2])
    r = quat_mul(quat_mul(q, qv), quat_inv(q))
    return (r[1], r[2], r[3])

def quat_is_identity(q, tol=1e-4):
    return abs(q[0] - 1.0) < tol and abs(q[1]) < tol and abs(q[2]) < tol and abs(q[3]) < tol

def mat4_to_pos_quat(matrix):
    """Extract position and quaternion from a Gf.Matrix4d.
    Handles scaled matrices by extracting rotation properly."""
    t = matrix.ExtractTranslation()
    pos = vec3(t[0], t[1], t[2])
    rot = matrix.ExtractRotation()
    q = rot.GetQuat()
    qu = quat_from_gf(q)
    return pos, quat_normalize(qu)


# ─── Formatting helpers ──────────────────────────────────────────────────────

def fmt(v, p=6):
    return " ".join(f"{x:.{p}g}" for x in v)


# ─── Data classes ────────────────────────────────────────────────────────────

class UsdBody:
    def __init__(self, prim, name):
        self.prim = prim
        self.name = name
        self.path = str(prim.GetPath())
        # World pose for MuJoCo
        self.world_pos = vec3()     # body center in world
        self.world_quat = quat()    # body orientation in world
        # USD world transform (from ComputeLocalToWorldTransform)
        self.usd_world_matrix = None
        self.usd_world_pos = vec3()
        self.usd_world_quat = quat()
        # Tree
        self.meshes = []
        self.children = []
        self.parent = None
        self.joint = None


class UsdJoint:
    def __init__(self, prim, name, joint_type, axis, body0_path, body1_path,
                 local_pos0, local_pos1, local_rot0, local_rot1,
                 lower_limit, upper_limit):
        self.prim = prim
        self.name = name
        self.joint_type = joint_type
        self.axis = axis
        self.body0_path = body0_path
        self.body1_path = body1_path
        self.local_pos0 = local_pos0
        self.local_pos1 = local_pos1
        self.local_rot0 = local_rot0
        self.local_rot1 = local_rot1
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit


# ─── Main converter ──────────────────────────────────────────────────────────

class UsdToMujoco:
    """Converts a USD robot file to MuJoCo MJCF XML format."""

    def __init__(self, usd_path, output_dir=None, model_name=None,
                 use_mesh_collision=True, angle_unit="radian"):
        self.usd_path = os.path.abspath(usd_path)
        self.stage = Usd.Stage.Open(self.usd_path)
        if not self.stage:
            raise ValueError(f"Failed to open USD file: {usd_path}")

        if output_dir is None:
            output_dir = os.path.join(os.path.dirname(self.usd_path), "mujoco_output")
        self.output_dir = os.path.abspath(output_dir)
        self.mesh_dir = os.path.join(self.output_dir, "meshes")

        if model_name is None:
            model_name = os.path.splitext(os.path.basename(usd_path))[0]
        self.model_name = model_name
        self.use_mesh_collision = use_mesh_collision
        self.angle_unit = angle_unit

        self.bodies = {}
        self.joints = []
        self.root_body = None
        self.up_axis = UsdGeom.GetStageUpAxis(self.stage)
        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(self.stage)

        self.root_prim = self.stage.GetDefaultPrim()
        if not self.root_prim:
            for p in self.stage.GetPseudoRoot().GetChildren():
                if p.IsA(UsdGeom.Xform):
                    self.root_prim = p
                    break
        if not self.root_prim:
            raise ValueError("No root Xform prim found")

        print(f"USD file  : {usd_path}")
        print(f"Root prim : {self.root_prim.GetPath()}")
        print(f"Up axis   : {self.up_axis}")

    # ── pipeline ──────────────────────────────────────────────────────────

    def convert(self):
        print("\n--- Parsing bodies ---")
        self._parse_bodies()
        print(f"Found {len(self.bodies)} rigid bodies")

        print("\n--- Parsing joints ---")
        self._parse_joints()
        print(f"Found {len(self.joints)} joints")

        print("\n--- Building kinematic tree ---")
        self._build_tree()

        print("\n--- Computing body world positions from joints ---")
        self._compute_world_positions_from_joints()

        print("\n--- Collecting meshes ---")
        self._collect_meshes()

        print("\n--- Exporting meshes ---")
        os.makedirs(self.mesh_dir, exist_ok=True)
        self._export_meshes()

        print("\n--- Generating MuJoCo XML ---")
        xml_path = self._generate_xml()
        print(f"\nDone!  XML -> {xml_path}")
        print(f"       Meshes -> {self.mesh_dir}")
        return xml_path

    # ── parse bodies ──────────────────────────────────────────────────────

    def _parse_bodies(self):
        t = Usd.TimeCode.Default()
        for prim in Usd.PrimRange(self.root_prim):
            if not (prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.IsA(UsdGeom.Xform)):
                continue

            body = UsdBody(prim, prim.GetName())

            # Store USD world transform
            xf = UsdGeom.Xformable(prim)
            body.usd_world_matrix = xf.ComputeLocalToWorldTransform(t)
            body.usd_world_pos, body.usd_world_quat = mat4_to_pos_quat(body.usd_world_matrix)

            self.bodies[body.path] = body
            print(f"  {body.name:20s}  usd_pos=({fmt(body.usd_world_pos)})")

    # ── parse joints ──────────────────────────────────────────────────────

    def _parse_joints(self):
        for prim in Usd.PrimRange(self.root_prim):
            is_rev = prim.IsA(UsdPhysics.RevoluteJoint)
            is_pris = prim.IsA(UsdPhysics.PrismaticJoint)
            is_fix = prim.IsA(UsdPhysics.FixedJoint)
            is_jnt = prim.IsA(UsdPhysics.Joint)
            if not (is_jnt or is_rev or is_pris or is_fix):
                continue

            jtype = "d6"
            if is_rev: jtype = "revolute"
            elif is_pris: jtype = "prismatic"
            elif is_fix: jtype = "fixed"

            api = UsdPhysics.Joint(prim)
            b0 = api.GetBody0Rel().GetTargets()
            b1 = api.GetBody1Rel().GetTargets()
            b0p = str(b0[0]) if b0 else None
            b1p = str(b1[0]) if b1 else None
            if not b0p or not b1p:
                continue

            axis = "X"
            if is_rev:
                a = UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()
                if a: axis = str(a)
            elif is_pris:
                a = UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get()
                if a: axis = str(a)

            lp0 = api.GetLocalPos0Attr().Get()
            lp1 = api.GetLocalPos1Attr().Get()
            lr0 = api.GetLocalRot0Attr().Get()
            lr1 = api.GetLocalRot1Attr().Get()

            lp0 = vec3(lp0[0], lp0[1], lp0[2]) if lp0 else vec3()
            lp1 = vec3(lp1[0], lp1[1], lp1[2]) if lp1 else vec3()
            lr0 = quat_from_gf(lr0) if lr0 else quat()
            lr1 = quat_from_gf(lr1) if lr1 else quat()

            lo = hi = None
            if is_rev:
                lo = UsdPhysics.RevoluteJoint(prim).GetLowerLimitAttr().Get()
                hi = UsdPhysics.RevoluteJoint(prim).GetUpperLimitAttr().Get()
            elif is_pris:
                lo = UsdPhysics.PrismaticJoint(prim).GetLowerLimitAttr().Get()
                hi = UsdPhysics.PrismaticJoint(prim).GetUpperLimitAttr().Get()

            j = UsdJoint(prim, prim.GetName(), jtype, axis,
                         b0p, b1p, lp0, lp1, lr0, lr1, lo, hi)
            self.joints.append(j)
            print(f"  {j.name:24s}  {jtype:10s}  {b0p.split('/')[-1]:>12s} -> {b1p.split('/')[-1]:<12s}  axis={axis}")

    # ── tree ──────────────────────────────────────────────────────────────

    def _build_tree(self):
        root_path = None
        for prim in Usd.PrimRange(self.root_prim):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                p = str(prim.GetPath())
                if p in self.bodies:
                    root_path = p
                    break
        if root_path is None:
            b1s = {j.body1_path for j in self.joints}
            b0s = {j.body0_path for j in self.joints}
            cands = b0s - b1s
            root_path = next(iter(cands)) if cands else next(iter(self.bodies))

        self.root_body = self.bodies[root_path]
        print(f"  Root: {self.root_body.name}")

        adj = defaultdict(list)
        for j in self.joints:
            adj[j.body0_path].append((j, j.body1_path, True))
            adj[j.body1_path].append((j, j.body0_path, False))

        visited = {root_path}
        q = deque([root_path])
        while q:
            cp = q.popleft()
            cb = self.bodies.get(cp)
            if cb is None:
                continue
            for jnt, np_, is_par in adj[cp]:
                if np_ in visited or np_ not in self.bodies:
                    continue
                visited.add(np_)
                child = self.bodies[np_]
                child.parent = cb
                cb.children.append(child)
                if is_par:
                    child.joint = jnt
                else:
                    child.joint = UsdJoint(jnt.prim, jnt.name, jnt.joint_type, jnt.axis,
                                           cp, np_, jnt.local_pos1, jnt.local_pos0,
                                           jnt.local_rot1, jnt.local_rot0,
                                           jnt.lower_limit, jnt.upper_limit)
                q.append(np_)

        self._print_tree(self.root_body)

    def _print_tree(self, b, d=0):
        js = f" [{b.joint.name}]" if b.joint else ""
        print(f"  {'  '*d}{b.name}{js}")
        for c in b.children:
            self._print_tree(c, d+1)

    # ── compute world positions ──────────────────────────────────────────

    def _compute_world_positions_from_joints(self):
        """Compute body world positions by propagating through the kinematic tree
        using joint localPos values and the USD world transforms.
        
        Strategy:
        - The joint world position is computed reliably from:
          joint_world = parent_usd_world_transform * localPos0
        - Place each body at the joint that connects it to its parent.
        - The body orientation is taken from the parent's USD world rotation
          (since the joint frame defines the body's local coordinate system).
        - For the root body, use the mesh centroid.
        """
        t = Usd.TimeCode.Default()
        
        # First compute mesh centroids in world for all bodies (as fallback)
        body_centroids = {}
        for bp, body in self.bodies.items():
            centroids = []
            bw = body.usd_world_matrix
            child_paths = {c.path for c in body.children}
            for prim in Usd.PrimRange(body.prim):
                if prim == body.prim:
                    continue
                pp = str(prim.GetPath())
                if any(pp.startswith(cp) for cp in child_paths):
                    continue
                if not prim.IsA(UsdGeom.Mesh):
                    continue
                mesh = UsdGeom.Mesh(prim)
                pts = mesh.GetPointsAttr().Get()
                if not pts:
                    continue
                mw = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(t)
                for p in pts:
                    wp = mw.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
                    centroids.append(vec3(wp[0], wp[1], wp[2]))
            
            if centroids:
                cx = sum(c[0] for c in centroids) / len(centroids)
                cy = sum(c[1] for c in centroids) / len(centroids)
                cz = sum(c[2] for c in centroids) / len(centroids)
                body_centroids[bp] = vec3(cx, cy, cz)
            else:
                body_centroids[bp] = body.usd_world_pos

        # For the root body, use mesh centroid as position
        root_centroid = body_centroids.get(self.root_body.path, vec3())
        self.root_body.world_pos = root_centroid
        self.root_body.world_quat = self.root_body.usd_world_quat
        print(f"  {self.root_body.name:20s}  pos=({fmt(self.root_body.world_pos)})  [root, from mesh centroid]")

        # Propagate through tree using BFS
        queue = deque()
        for child in self.root_body.children:
            queue.append(child)

        while queue:
            body = queue.popleft()
            j = body.joint
            parent = body.parent

            if j is not None:
                # Compute joint world position from parent's USD frame
                parent_mat = parent.usd_world_matrix
                jp_gf = parent_mat.Transform(
                    Gf.Vec3d(j.local_pos0[0], j.local_pos0[1], j.local_pos0[2]))
                joint_world_pos = vec3(jp_gf[0], jp_gf[1], jp_gf[2])

                # Place body at joint position
                body.world_pos = joint_world_pos

                # Body orientation: use the body's USD world rotation
                # (This preserves the body's natural frame from the USD)
                body.world_quat = body.usd_world_quat

                print(f"  {body.name:20s}  pos=({fmt(body.world_pos)})  [from joint {j.name}]")
            else:
                # No joint (shouldn't happen in tree traversal, but just in case)
                body.world_pos = body_centroids.get(body.path, body.usd_world_pos)
                body.world_quat = body.usd_world_quat
                print(f"  {body.name:20s}  pos=({fmt(body.world_pos)})  [from centroid]")

            for child in body.children:
                queue.append(child)

    # ── meshes ────────────────────────────────────────────────────────────

    def _collect_meshes(self):
        """Collect mesh data, transforming vertices to MuJoCo body-local frame."""
        t = Usd.TimeCode.Default()
        total = 0

        for bp, body in self.bodies.items():
            child_paths = {c.path for c in body.children}
            inv_body_quat = quat_inv(body.world_quat)

            for prim in Usd.PrimRange(body.prim):
                if prim == body.prim:
                    continue
                pp = str(prim.GetPath())
                if any(pp.startswith(cp) for cp in child_paths):
                    continue
                if not prim.IsA(UsdGeom.Mesh):
                    continue

                mesh = UsdGeom.Mesh(prim)
                pts = mesh.GetPointsAttr().Get()
                fcounts = mesh.GetFaceVertexCountsAttr().Get()
                findices = mesh.GetFaceVertexIndicesAttr().Get()
                if not pts or not fcounts or not findices:
                    continue

                mw = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(t)

                # Transform: mesh -> world -> body-local
                verts = []
                for p in pts:
                    wp = mw.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
                    world_pt = vec3(wp[0], wp[1], wp[2])
                    delta = vec3_sub(world_pt, body.world_pos)
                    local_pt = quat_rotate(inv_body_quat, delta)
                    verts.append(local_pt)

                tris = []
                idx = 0
                for cnt in fcounts:
                    if cnt == 3:
                        tris.append((findices[idx], findices[idx+1], findices[idx+2]))
                    elif cnt == 4:
                        tris.append((findices[idx], findices[idx+1], findices[idx+2]))
                        tris.append((findices[idx], findices[idx+2], findices[idx+3]))
                    else:
                        for i in range(1, cnt-1):
                            tris.append((findices[idx], findices[idx+i], findices[idx+i+1]))
                    idx += cnt

                body.meshes.append({'name': prim.GetName(), 'vertices': verts, 'triangles': tris})
                total += 1

        print(f"  {total} meshes across {len(self.bodies)} bodies")

    def _export_meshes(self):
        for body in self.bodies.values():
            if not body.meshes:
                continue
            all_v, all_t, off = [], [], 0
            for md in body.meshes:
                all_v.extend(md['vertices'])
                for tri in md['triangles']:
                    all_t.append((tri[0]+off, tri[1]+off, tri[2]+off))
                off += len(md['vertices'])
            if not all_v:
                continue
            fn = f"{body.name}.obj"
            path = os.path.join(self.mesh_dir, fn)
            with open(path, 'w') as f:
                f.write(f"# {body.name}: {len(all_v)} verts, {len(all_t)} tris\n")
                for v in all_v:
                    f.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")
                for t_ in all_t:
                    f.write(f"f {t_[0]+1} {t_[1]+1} {t_[2]+1}\n")
            print(f"  {fn:24s}  {len(all_v):6d} verts  {len(all_t):6d} tris")

    # ── MuJoCo XML generation ─────────────────────────────────────────────

    def _body_rel_pos_quat(self, body):
        if body.parent is None:
            return body.world_pos, body.world_quat
        par = body.parent
        delta = vec3_sub(body.world_pos, par.world_pos)
        pqi = quat_inv(par.world_quat)
        rel_pos = quat_rotate(pqi, delta)
        rel_quat = quat_normalize(quat_mul(pqi, body.world_quat))
        return rel_pos, rel_quat

    def _joint_pos_axis(self, body):
        """Joint position and axis in the MuJoCo child body frame.
        
        The child body origin is placed at the joint world position.
        So joint pos in body frame = (0, 0, 0).
        The axis is computed from the joint frame rotation.
        """
        j = body.joint
        if j is None:
            return vec3(), vec3(0, 0, 1)

        # Joint is at body origin (since we placed body at joint position)
        joint_pos = vec3(0, 0, 0)

        # Compute joint axis in world frame
        # axis_world = parent_usd_rot * localRot0 * axis_unit
        parent = body.parent
        if parent:
            parent_usd_quat = parent.usd_world_quat
        else:
            parent_usd_quat = quat()

        amap = {'X': vec3(1,0,0), 'Y': vec3(0,1,0), 'Z': vec3(0,0,1)}
        ax_joint_frame = amap.get(j.axis, vec3(0,0,1))

        joint_frame_world_quat = quat_normalize(quat_mul(parent_usd_quat, j.local_rot0))
        axis_world = quat_rotate(joint_frame_world_quat, ax_joint_frame)

        # Transform to child body frame
        inv_child_quat = quat_inv(body.world_quat)
        axis_child = quat_rotate(inv_child_quat, axis_world)
        axis_child = vec3_normalize(axis_child)

        return joint_pos, axis_child

    def _generate_xml(self):
        root = Element("mujoco", model=self.model_name)

        SubElement(root, "compiler", angle=self.angle_unit, meshdir="meshes")
        SubElement(root, "option", gravity="0 0 -9.81")

        visual = SubElement(root, "visual")
        SubElement(visual, "global", offwidth="1280", offheight="960")

        default = SubElement(root, "default")
        SubElement(default, "joint", type="hinge", damping="1.0", armature="0.01")
        SubElement(default, "geom", contype="0", conaffinity="0", group="1", density="0")
        SubElement(default, "motor", ctrllimited="true", ctrlrange="-1 1")
        col_def = SubElement(default, "default", **{"class": "collision"})
        SubElement(col_def, "geom", contype="1", conaffinity="1", group="3",
                   density="1000", rgba="0.5 0.5 0.5 0.5")

        asset = SubElement(root, "asset")
        self._add_mesh_assets(self.root_body, asset, set())

        wb = SubElement(root, "worldbody")
        SubElement(wb, "light", pos="0 0 3", dir="0 0 -1",
                   diffuse="0.8 0.8 0.8", specular="0.3 0.3 0.3")
        SubElement(wb, "geom", name="floor", type="plane", size="5 5 0.1",
                   rgba="0.8 0.9 0.8 1", contype="1", conaffinity="1", group="2")
        self._body_xml(self.root_body, wb, is_root=True)

        act = SubElement(root, "actuator")
        self._add_actuators(self.root_body, act)

        raw = tostring(root, encoding='unicode')
        pretty = minidom.parseString(raw).toprettyxml(indent="  ")
        lines = pretty.split('\n')
        if lines[0].startswith('<?xml'):
            lines = lines[1:]
        xml_str = '\n'.join(lines)

        os.makedirs(self.output_dir, exist_ok=True)
        xml_path = os.path.join(self.output_dir, f"{self.model_name}.xml")
        with open(xml_path, 'w') as f:
            f.write(xml_str)
        return xml_path

    def _add_mesh_assets(self, body, asset_el, seen):
        if body.meshes and body.name not in seen:
            seen.add(body.name)
            SubElement(asset_el, "mesh", name=body.name, file=f"{body.name}.obj")
        for c in body.children:
            self._add_mesh_assets(c, asset_el, seen)

    def _body_xml(self, body, parent_el, is_root=False):
        pos, qu = self._body_rel_pos_quat(body)
        attr = {"name": body.name, "pos": fmt(pos)}
        if not quat_is_identity(qu):
            attr["quat"] = fmt(qu)
        bel = SubElement(parent_el, "body", **attr)

        if is_root:
            SubElement(bel, "freejoint", name=body.name)

        if body.joint and not is_root:
            j = body.joint
            if j.joint_type == "revolute":
                jp, ja = self._joint_pos_axis(body)
                ja_attr = {"name": j.name, "pos": fmt(jp), "axis": fmt(ja), "type": "hinge"}
                if j.lower_limit is not None and j.upper_limit is not None:
                    if self.angle_unit == "radian":
                        lo = math.radians(float(j.lower_limit))
                        hi = math.radians(float(j.upper_limit))
                    else:
                        lo, hi = float(j.lower_limit), float(j.upper_limit)
                    ja_attr["range"] = f"{lo:.6g} {hi:.6g}"
                    ja_attr["limited"] = "true"
                SubElement(bel, "joint", **ja_attr)
            elif j.joint_type == "prismatic":
                jp, ja = self._joint_pos_axis(body)
                ja_attr = {"name": j.name, "pos": fmt(jp), "axis": fmt(ja), "type": "slide"}
                if j.lower_limit is not None and j.upper_limit is not None:
                    ja_attr["range"] = f"{float(j.lower_limit):.6g} {float(j.upper_limit):.6g}"
                    ja_attr["limited"] = "true"
                SubElement(bel, "joint", **ja_attr)
            elif j.joint_type == "d6":
                jp, _ = self._joint_pos_axis(body)
                SubElement(bel, "joint", name=j.name, type="ball", pos=fmt(jp))

        if body.meshes:
            SubElement(bel, "geom", type="mesh", mesh=body.name, rgba="0.7 0.7 0.7 1")
            if self.use_mesh_collision:
                SubElement(bel, "geom", type="mesh", mesh=body.name, **{"class": "collision"})

        for c in body.children:
            self._body_xml(c, bel)

    def _add_actuators(self, body, act_el):
        if body.joint and body.joint.joint_type in ("revolute", "prismatic"):
            SubElement(act_el, "motor", name=f"motor_{body.joint.name}", joint=body.joint.name)
        for c in body.children:
            self._add_actuators(c, act_el)


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Convert USD robot to MuJoCo XML")
    parser.add_argument("usd_path", help="Path to USD/USDA file")
    parser.add_argument("--output_dir", "-o", default=None)
    parser.add_argument("--model_name", "-n", default=None)
    parser.add_argument("--no_mesh_collision", action="store_true")
    parser.add_argument("--angle_unit", choices=["radian", "degree"], default="radian")
    args = parser.parse_args()

    converter = UsdToMujoco(
        usd_path=args.usd_path,
        output_dir=args.output_dir,
        model_name=args.model_name,
        use_mesh_collision=not args.no_mesh_collision,
        angle_unit=args.angle_unit,
    )
    converter.convert()


if __name__ == "__main__":
    main()
