"""STL -> Bambu Studio 兼容 3MF 转换器.

使用 lib3mf (官方 3MF Consortium 库), 生成的 3MF 含 Production extension.
拓竹 Bambu Studio / OrcaSlicer / 其他主流切片器都能正确识别.
"""
import sys
from pathlib import Path

import lib3mf
import trimesh


def stl_to_3mf(stl_path: Path, mf3_path: Path, label: str = None):
    label = label or stl_path.stem
    # 读取 STL
    mesh = trimesh.load(str(stl_path))
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError(f"{stl_path} 不是单体 Trimesh")
    print(f"[{label}] vertices={len(mesh.vertices)} faces={len(mesh.faces)}")

    # 建 3MF 模型
    wrapper = lib3mf.Wrapper()
    model = wrapper.CreateModel()
    model.SetUnit(lib3mf.ModelUnit.MilliMeter)

    mesh_obj = model.AddMeshObject()
    mesh_obj.SetName(label)

    # vertices
    pos_array = []
    for v in mesh.vertices:
        p = lib3mf.Position()
        p.Coordinates[0] = float(v[0])
        p.Coordinates[1] = float(v[1])
        p.Coordinates[2] = float(v[2])
        pos_array.append(p)

    # triangles
    tri_array = []
    for f in mesh.faces:
        t = lib3mf.Triangle()
        t.Indices[0] = int(f[0])
        t.Indices[1] = int(f[1])
        t.Indices[2] = int(f[2])
        tri_array.append(t)

    mesh_obj.SetGeometry(pos_array, tri_array)

    # 加到 build plate
    identity = wrapper.GetIdentityTransform()
    model.AddBuildItem(mesh_obj, identity)

    # 写 3MF
    writer = model.QueryWriter("3mf")
    writer.WriteToFile(str(mf3_path))
    print(f"[{label}] wrote {mf3_path} ({mf3_path.stat().st_size} bytes)")


if __name__ == "__main__":
    case_dir = Path(__file__).parent
    for name in ("top_cover", "bottom_case"):
        stl = case_dir / f"{name}.stl"
        mf3 = case_dir / f"{name}.3mf"
        stl_to_3mf(stl, mf3, name)
