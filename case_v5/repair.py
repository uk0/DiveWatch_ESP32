"""Repair STL watertightness + regenerate 3MF."""
import trimesh
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent /
                        ".claude/skills/bambu-3mf/scripts"))

for name in ("top", "bottom"):
    stl = Path(__file__).parent / f"{name}.stl"
    m = trimesh.load(str(stl), force="mesh")
    print(f"{name}: watertight={m.is_watertight}  v={len(m.vertices)} f={len(m.faces)}")
    if not m.is_watertight:
        m.fill_holes()
        m.merge_vertices()
        m.fix_normals()
        m.update_faces(m.unique_faces())
        m.update_faces(m.nondegenerate_faces())
        m.remove_unreferenced_vertices()
        print(f"   after repair: watertight={m.is_watertight}  v={len(m.vertices)} f={len(m.faces)}")
    m.export(str(stl))
print("Done.")
