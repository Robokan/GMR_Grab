"""Inspect material bindings, UVs, and GeomSubsets in a USD file (debug helper)."""
import sys
from collections import Counter

from pxr import Usd, UsdGeom, UsdShade

stage = Usd.Stage.Open(sys.argv[1])
mat_counter = Counter()
n_mesh = n_uv = n_subset = 0
examples = []
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    n_mesh += 1
    mesh = UsdGeom.Mesh(prim)
    st = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
    has_uv = bool(st and st.HasValue())
    if has_uv:
        n_uv += 1
    binding = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    mat = binding.GetPrim().GetName() if binding else None
    mat_counter[mat] += 1
    subsets = UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets()
    if subsets:
        n_subset += 1
    if len(examples) < 6:
        interp = st.GetInterpolation() if has_uv else '-'
        examples.append(f"{prim.GetPath()} uv={has_uv}({interp}) mat={mat} subsets={len(subsets)}")

print(f"meshes={n_mesh} with_uv={n_uv} with_subsets={n_subset}")
print("materials:", dict(mat_counter))
for e in examples:
    print(" ", e)

# texture file per material
print("\nmaterial -> diffuse texture:")
for prim in stage.Traverse():
    if not prim.IsA(UsdShade.Material):
        continue
    mat = UsdShade.Material(prim)
    tex = None
    for shader_prim in Usd.PrimRange(prim):
        sh = UsdShade.Shader(shader_prim)
        if not sh:
            continue
        fin = sh.GetInput("file")
        if fin and fin.Get():
            tex = fin.Get()
            break
    print(f"  {prim.GetName()}: {tex}")
