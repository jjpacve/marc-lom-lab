"""Export the M01 concept as named STEP solids and a metre-scale chassis mesh."""
import json
from math import floor
from pathlib import Path

from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorGen
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

PROJECT = Path(__file__).resolve().parents[1]
ROBOT = json.loads((PROJECT / "model.json").read_text())["robot"]
MATERIALS = {
    "alloy": (0.64, 0.72, 0.75),
    "dark": (0.14, 0.19, 0.21),
    "orange": (0.88, 0.55, 0.19),
    "accent": (0.57, 0.81, 0.32),
    "tire": (0.095, 0.10, 0.10),
}


def solid_box(center, size):
    start = [amount - width / 2 for amount, width in zip(center, size)]
    return BRepPrimAPI_MakeBox(gp_Pnt(*start), *size).Shape()


def cylinder(start, axis, radius, height):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*start), gp_Dir(*axis)), radius, height).Shape()


def compound(shapes):
    root = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(root)
    for item in shapes:
        builder.Add(root, item)
    return root


def perforated_member(part, center, size):
    x, y, z = center
    dx, dy, dz = size
    wall = 1.8
    body = solid_box(center, size)
    profile = part["shape"]
    if profile == "channel":
        void = solid_box((x, y, z + wall), (dx + 0.4, dy - 2 * wall, dz))
        body = BRepAlgoAPI_Cut(body, void).Shape()
        holes = [cylinder((x + i * 8, y - dy / 2 - 1, z + 1), (0, 1, 0), 2.1, dy + 2)
                 for i in range(-floor((dx - 9) / 16), floor((dx - 9) / 16) + 1)]
    elif profile == "cross":
        holes = [cylinder((x - dx / 2 - 1, y + i * 8, z), (1, 0, 0), 2.1, dx + 2)
                 for i in range(-floor((dy - 9) / 16), floor((dy - 9) / 16) + 1)]
    else:
        holes = [cylinder((x - dx / 2 - 1, y, z + i * 8), (1, 0, 0), 2.1, dx + 2)
                 for i in range(-floor((dz - 9) / 16), floor((dz - 9) / 16) + 1)]
    return BRepAlgoAPI_Cut(body, compound(holes)).Shape()


def make_part(part, lift=0):
    center = [1000 * n for n in part["at"]]
    center[2] += lift
    size = [1000 * n for n in part["size"]]
    body = perforated_member(part, center, size) if "shape" in part else solid_box(center, size)
    return part["name"], part["material"], body


def robot_parts():
    body = [make_part(part) for part in ROBOT["body"]]
    r = ROBOT["wheelRadius"] * 1000
    w = ROBOT["wheelWidth"] * 1000
    for side, name in ((1, "Left"), (-1, "Right")):
        y = side * ROBOT["trackWidth"] * 500
        start = (ROBOT["axleX"] * 1000, y - w / 2, r)
        axle = (0, 1, 0)
        bore = cylinder((start[0], start[1] - 1, start[2]), axle, 18, w + 2)
        tire = BRepAlgoAPI_Cut(cylinder(start, axle, r, w), bore).Shape()
        body.extend([
            (f"{name} tire", "tire", tire),
            (f"{name} hub", "accent", cylinder(start, axle, 18, w)),
        ])
    caster = ROBOT["casterRadius"] * 1000
    body.append(("Rear caster", "dark",
                 BRepPrimAPI_MakeSphere(gp_Pnt(ROBOT["casterX"] * 1000, 0, caster), caster).Shape()))
    for side, name in ((1, "Left"), (-1, "Right")):
        body.append((f"{name} glide", "dark",
                     BRepPrimAPI_MakeSphere(gp_Pnt(86, 53 * side, 34), 11).Shape()))
    lift = 1000 * ROBOT["liftMin"]
    body.extend(make_part(part, lift) for part in ROBOT["carriage"])
    for side, name in ((1, "Left"), (-1, "Right")):
        center = (ROBOT["jawX"] * 1000, side * ROBOT["jawOpen"] * 1000, lift)
        body.append((f"{name} gripper pad", "accent", solid_box(center, [1000 * n for n in ROBOT["jawSize"]])))
    return body


def write_step(parts, target):
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    root = shapes.NewShape()
    TDataStd_Name.Set_s(root, TCollection_ExtendedString("M01 - LOM 2027 concept"))
    for name, material, solid in parts:
        if not BRepCheck_Analyzer(solid).IsValid():
            raise ValueError(f"Invalid CAD solid: {name}")
        item = shapes.AddShape(solid, False)
        TDataStd_Name.Set_s(item, TCollection_ExtendedString(name))
        colors.SetColor(item, Quantity_Color(*MATERIALS[material], Quantity_TOC_RGB), XCAFDoc_ColorGen)
        instance = shapes.AddComponent(root, item, TopLoc_Location())
        TDataStd_Name.Set_s(instance, TCollection_ExtendedString(name))
    shapes.UpdateAssemblies()
    writer = STEPCAFControl_Writer()
    writer.SetColorMode(True)
    writer.SetNameMode(True)
    if not writer.Transfer(doc, STEPControl_AsIs) or writer.Write(str(target)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP export failed for {target}")


def write_chassis_mesh(parts, target):
    mesh = ["# Exported from the M01 STEP chassis geometry, in metres.", f"mtllib {target.stem}.mtl"]
    offset = 0
    for name, material, solid in parts:
        BRepMesh_IncrementalMesh(solid, 0.28, False, 0.40)
        mesh.extend((f"o {name.replace(' ', '_')}", f"usemtl {material}"))
        faces = TopExp_Explorer(solid, TopAbs_FACE)
        while faces.More():
            face = TopoDS.Face_s(faces.Current())
            location = TopLoc_Location()
            triangles = BRep_Tool.Triangulation_s(face, location)
            if triangles is None:
                raise ValueError(f"Cannot mesh {name}")
            transform = location.Transformation()
            for index in range(1, triangles.NbNodes() + 1):
                point = triangles.Node(index).Transformed(transform)
                mesh.append("v " + " ".join(f"{v / 1000:.7f}" for v in (point.X(), point.Y(), point.Z())))
            for index in range(1, triangles.NbTriangles() + 1):
                a, b, c = triangles.Triangle(index).Get()
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                mesh.append(f"f {offset + a} {offset + b} {offset + c}")
            offset += triangles.NbNodes()
            faces.Next()
    target.write_text("\n".join(mesh) + "\n")
    target.with_suffix(".mtl").write_text("\n".join(
        f"newmtl {name}\nKd {r} {g} {b}\nKa 0.10 0.10 0.10\nKs 0.10 0.10 0.10\nNs 10\n"
        for name, (r, g, b) in MATERIALS.items()
    ))


def main():
    parts = robot_parts()
    write_step(parts, PROJECT / "cad/M01.step")
    target = PROJECT / "webots/meshes/M01-chassis.obj"
    target.parent.mkdir(exist_ok=True)
    write_chassis_mesh(parts[:len(ROBOT["body"])], target)
    print(f"Exported {len(parts)} named parts")


if __name__ == "__main__":
    main()
