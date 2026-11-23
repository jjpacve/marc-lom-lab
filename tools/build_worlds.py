"""Build the two Webots worlds and the offline viewer's geometry from model.json."""
import json
from math import acos, cos, pi, radians, sin, sqrt, tan
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
MODEL = json.loads((PROJECT / "model.json").read_text())
F = MODEL["field"]
R = MODEL["robot"]
LAB = MODEL["lab"]
PAD_MASS = 0.018
HALF = F["size"] / 2
INNER = HALF - F["baseInside"]
COLORS = {
    "alloy": (0.57, 0.66, 0.69), "dark": (0.12, 0.16, 0.19),
    "orange": (0.82, 0.53, 0.17), "accent": (0.49, 0.78, 0.30),
    "tire": (0.065, 0.075, 0.08), "blue": (0.18, 0.50, 0.95),
    "red": (0.92, 0.25, 0.27), "metal": (0.49, 0.57, 0.61),
    "core": (0.12, 0.81, 0.69), "mat": (0.14, 0.18, 0.20),
    "tape": (0.87, 0.89, 0.85), "ramp": (0.38, 0.53, 0.40),
}


def field_layout():
    starts, bases, lines, props, pillars, hang_slots = [], [], [], [], [], []
    lines.append({"a": [-HALF, HALF], "b": [HALF, -HALF], "color": "tape"})
    for side, sign in (("blue", -1), ("red", 1)):
        x, y = (value * sign for value in F["startCenter"])
        row_width, row_depth = F["startSize"]
        offset = (R["frontX"] + R["rearX"]) / 2
        starts.extend([
            {"side": side, "x": x, "y": y, "w": row_width, "d": row_depth,
             "spawnX": x + sign * offset, "spawnY": y,
             "yaw": pi if sign > 0 else 0},
            {"side": side, "x": y, "y": x, "w": row_depth, "d": row_width,
             "spawnX": y, "spawnY": x + sign * offset,
             "yaw": -pi / 2 if sign > 0 else pi / 2},
        ])
        midpoint = (HALF + INNER) * sign / 2
        bases.append({"side": side, "x": midpoint, "y": midpoint, "inner": INNER})
        yaw = pi / 4 if sign < 0 else -3 * pi / 4
        for index in range(2):
            lateral = sign * (1 - 2 * index) * R["hang"]["slotOffset"]
            bar_x = sign * F["tridentCrossAt"] + lateral / sqrt(2)
            bar_y = sign * F["tridentCrossAt"] - lateral / sqrt(2)
            hang_slots.append({
                "id": f"{side}_{index}", "side": side, "slot": index,
                "barX": bar_x, "barY": bar_y, "barZ": F["tridentHeight"] + 0.017,
                "x": bar_x - cos(yaw) * R["hang"]["reach"],
                "y": bar_y - sin(yaw) * R["hang"]["reach"],
                "yaw": yaw,
            })
        track_end = F["markedCube"] * sign
        inner = INNER * sign
        lines.extend([
            {"a": [inner, inner], "b": [track_end, track_end], "color": "tape"},
            {"a": [x - sign * row_width / 2, y + sign * row_depth / 2],
             "b": [y + sign * row_depth / 2, x - sign * row_width / 2], "color": "tape"},
            {"a": [track_end, track_end], "b": [0.073 * sign, 0.073 * sign], "color": side},
        ])
        props.append({
            "id": f"{side}_1", "side": side, "tag": 1, "kind": "cube", "origin": "floor",
            "x": track_end, "y": track_end, "z": F["cubeSide"] / 2,
        })
        for tag, key, z in zip((2, 3), ("tridentLongAt", "tridentShortAt"), reversed(F["hookCubeZ"])):
            d = F[key] * sign
            props.append({
                "id": f"{side}_{tag}", "side": side, "tag": tag, "kind": "cube",
                "origin": "hook", "x": d, "y": d, "z": z,
            })
    for index, sign in enumerate((-1, 1)):
        x, y = F["pillar"] * sign, -F["pillar"] * sign
        pillars.append({"x": x, "y": y})
        for tier, z in enumerate(F["pillarLevels"]):
            offset = (-1 if tier == 0 else 1) * F["pillarOffset"]
            props.append({
                "id": f"pillar_{index}_{tier}", "kind": "cube", "origin": "pillar",
                "x": x + offset, "y": y - offset, "z": z + F["cubeSide"] / 2,
            })
    props.append({
        "id": "core", "kind": "core", "origin": "core", "x": 0, "y": 0,
        "z": F["tridentHeight"] + 0.005 + F["coreSize"][2] / 2,
    })
    return {"starts": starts, "bases": bases, "lines": lines, "props": props,
            "pillars": pillars, "hangSlots": hang_slots}


def numbers(values):
    return " ".join(f"{value:.7g}" for value in values)


def shape(geometry, material):
    return (
        f"Shape {{ appearance PBRAppearance {{ baseColor {numbers(COLORS[material])} "
        f"roughness 0.73 metalness 0 }} geometry {geometry} }}"
    )


def box(size):
    return f"Box {{ size {numbers(size)} }}"


def pose(at, child, rotation=None):
    turn = f" rotation {numbers(rotation)}" if rotation else ""
    return f"Pose {{ translation {numbers(at)}{turn} children [ {child} ] }}"


def visual_box(at, size, material, rotation=None):
    return pose(at, shape(box(size), material), rotation)


def static_box(at, size, material, name, contact="default", rotation=None, extra_children=""):
    turn = f"rotation {numbers(rotation)}" if rotation else ""
    return (
        f'Solid {{ translation {numbers(at)} {turn} name "{name}" contactMaterial "{contact}" '
        f"children [ {shape(box(size), material)} {extra_children} ] boundingObject {box(size)} }}"
    )


def floor_line(a, b, width, color):
    from math import atan2, hypot

    dx, dy = b[0] - a[0], b[1] - a[1]
    return visual_box(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0.0005),
                      (hypot(dx, dy), width, 0.001), color, (0, 0, 1, atan2(dy, dx)))


def base_parts(base):
    sign = -1 if base["side"] == "blue" else 1
    rail_at = sign * (INNER - F["baseWidth"] / 2)
    w, h = F["baseWidth"], F["baseHeight"]
    color = base["side"]
    for axis in range(2):
        at = (rail_at, base["y"], h - 0.013)
        dims = (w, F["baseInside"] + w, 0.026)
        if axis:
            at = (base["x"], rail_at, h - 0.013)
            dims = (F["baseInside"] + w, w, 0.026)
        yield static_box(at, dims, color, f"{color} base rail {axis + 1}")
        for index, amount in enumerate((INNER + 0.008, HALF - 0.024)):
            leg = (rail_at, sign * amount, (h - 0.026) / 2)
            if axis:
                leg = (sign * amount, rail_at, (h - 0.026) / 2)
            yield static_box(leg, (w, w, h - 0.026), "metal", f"{color} base leg {axis}-{index}")


def trident_parts(slots):
    h, w = F["tridentHeight"], F["tridentBeamWidth"]
    diagonal = (0, 0, 1, pi / 4)
    yield static_box((0, 0, 0.012), (0.107, 0.074, 0.024), "metal", "Trident pedestal", rotation=diagonal)
    for side in (-1, 1):
        d = 0.024 * side
        yield static_box((d, d, (h - 0.028) / 2), (0.016, 0.035, h - 0.028),
                         "metal", f"Trident post {side}", rotation=diagonal)
    yield static_box((0, 0, h - 0.009), (F["tridentSpan"], w, 0.018),
                     "metal", "Trident rail", rotation=diagonal)
    yield static_box((0, 0, h - 0.002), (0.066, 0.066, 0.014), "metal", "Core saddle", rotation=diagonal)
    for sign, color in ((-1, "blue"), (1, "red")):
        d = F["tridentCrossAt"] * sign
        sockets = []
        for slot in (item for item in slots if item["side"] == color):
            along_beam = (slot["barX"] - slot["barY"]) / sqrt(2)
            sockets.append(
                f'Connector {{ translation {along_beam:.7g} 0 0.009 rotation 0 1 0 {-pi / 2:.7g} '
                f'name "{slot["id"]} rail seat" model "LOM {color} crossbeam" type "passive" }}'
            )
        yield static_box((d, d, h + 0.008), (F["tridentCrossSpan"], w, 0.018),
                         color, f"{color} crossbeam", rotation=(0, 0, 1, -pi / 4),
                         extra_children=" ".join(sockets))


def payload(prop):
    core = prop["kind"] == "core"
    cube = F["cubeSide"]
    dims = F["coreSize"] if core else (cube, cube, cube)
    material = "core" if core else prop.get("side", "metal")
    drawing = [shape(box(dims), material)]
    collision = [box(dims)]
    if prop["origin"] == "hook":
        # The hook slides in the direction across the rail until the top lip clears it.
        bottom = cube / 2
        lip = F["tridentHeight"] - prop["z"] + 0.004
        upright = (0, 0.018, (bottom + lip) / 2)
        upright_dims = (0.016, 0.008, lip - bottom)
        cap = (0, 0.007, lip)
        cap_dims = (0.017, 0.031, 0.008)
        drawing += [visual_box(upright, upright_dims, "metal"), visual_box(cap, cap_dims, "metal")]
        collision += [pose(upright, box(upright_dims)), pose(cap, box(cap_dims))]
    x, y, z = (prop[k] for k in ("x", "y", "z"))
    yaw = pi / 4 if prop["origin"] in ("hook", "core") else 0
    mass = F["coreMass"] if core else F["cubeMass"]
    return (
        f'DEF {prop["id"].upper()} Solid {{ translation {numbers((x,y,z))} rotation 0 0 1 {yaw:.7g} '
        f'name "{prop["id"]}" contactMaterial "payload" children [ {" ".join(drawing)} ] '
        f'boundingObject Group {{ children [ {" ".join(collision)} ] }} '
        f'physics Physics {{ density -1 mass {mass} }} }}'
    )


def wheel(side):
    r, w = R["wheelRadius"], R["wheelWidth"]
    at = (R["axleX"], side * R["trackWidth"] / 2, r)
    tire = f"Cylinder {{ height {w} radius {r} subdivision 36 }}"
    hub = f"Cylinder {{ height {w + 0.001} radius 0.018 subdivision 24 }}"
    name = "left" if side > 0 else "right"
    # Webots Cylinder's native axis is +Z.
    turn = (1, 0, 0, -pi / 2)
    return f'''
      HingeJoint {{
        jointParameters HingeJointParameters {{ axis 0 1 0 anchor {numbers(at)} }}
        device [
          RotationalMotor {{ name "{name} wheel" maxVelocity 30 maxTorque {R["wheelTorque"]} sound "" }}
          PositionSensor {{ name "{name} encoder" }}
        ]
        endPoint Solid {{
          translation {numbers(at)} name "{name} drive tire" contactMaterial "tire"
          children [ {pose((0, 0, 0), shape(tire, "tire"), turn)} {pose((0, 0, 0), shape(hub, "accent"), turn)} ]
          boundingObject {pose((0, 0, 0), tire, turn)}
          physics Physics {{ density -1 mass 0.06 }}
        }}
      }}'''


def jaw(side):
    name = "left" if side > 0 else "right"
    open_y = side * R["jawOpen"]
    at = (R["jawX"], open_y, 0)
    min_stop = R["jawClosed"] - R["jawOpen"]
    return f'''
          SliderJoint {{
            jointParameters JointParameters {{ axis 0 {side} 0 minStop {min_stop:.6g} maxStop 0 dampingConstant 0.42 }}
            device [
              LinearMotor {{ name "{name} jaw" maxVelocity {R["jawSpeed"]} maxForce 4.8 minPosition {min_stop:.6g} maxPosition 0 sound "" }}
              PositionSensor {{ name "{name} jaw encoder" }}
            ]
            endPoint Solid {{
              translation {numbers(at)} name "{name} pad" contactMaterial "jaw"
              children [ {shape(box(R["jawSize"]), "accent")} ]
              boundingObject {box(R["jawSize"])}
              physics Physics {{ density -1 mass {PAD_MASS} }}
            }}
          }}'''


def robot_proto():
    body = 'CadShape { url [ "../meshes/M01-chassis.obj" ] }'
    carriage = "\n".join(visual_box(item["at"], item["size"], item["material"]) for item in R["carriage"])
    carriage_collision = " ".join(pose(item["at"], box(item["size"])) for item in R["carriage"]
                                  if item["name"] != "Grip servo envelope")
    hook = R["hang"]
    collision = " ".join(pose(item["at"], box(item["size"])) for item in R["body"]
                         if item["name"] in ("Left chassis channel", "Right chassis channel",
                                             "Front cross member", "Rear cross member",
                                             "Left lift guide", "Right lift guide", "Electronics deck"))
    lift_stop = R["liftMax"] - R["liftMin"]
    caster = f'Sphere {{ radius {R["casterRadius"]} subdivision 2 }}'
    front_glide = "Sphere { radius 0.011 subdivision 2 }"
    glides = " ".join(pose((0.086, side * 0.053, 0.034), front_glide) for side in (-1, 1))
    carriage_mass = 0.12
    base_mass = R["mass"] - 0.12 - 2 * PAD_MASS - carriage_mass - 0.016
    return f'''#VRML_SIM R2025a utf8
PROTO M01 [
  field SFVec3f translation 0 0 0
  field SFRotation rotation 0 0 1 0
  field SFString railModel "LOM blue crossbeam"
  field MFString controllerArgs []
] {{
  Robot {{
    translation IS translation
    rotation IS rotation
    name "M01"
    controller "lom_driver"
    controllerArgs IS controllerArgs
    supervisor TRUE
    children [
      {body}
      {wheel(1)}
      {wheel(-1)}
      BallJoint {{
        jointParameters BallJointParameters {{ anchor {R["casterX"]} 0 {R["casterRadius"]} }}
        endPoint Solid {{
          translation {R["casterX"]} 0 {R["casterRadius"]}
          name "rear caster" contactMaterial "caster"
          children [ {shape(caster, "dark")} ]
          boundingObject {caster}
          physics Physics {{ density -1 mass 0.016 }}
        }}
      }}
      {pose((0.111, 0, 0.247), 'Camera { name "front camera" width 320 height 240 fieldOfView 1.1 }', (0, 1, 0, 0.43))}
      GPS {{ name "gps" }}
      InertialUnit {{ name "imu" }}
      Gyro {{ name "gyro" }}
      SliderJoint {{
        jointParameters JointParameters {{ axis 0 0 1 minStop 0 maxStop {lift_stop:.6g} dampingConstant 0.85 }}
        device [
          LinearMotor {{ name "lift" maxVelocity {R["liftSpeed"]} maxForce 18 minPosition 0 maxPosition {lift_stop:.6g} sound "" }}
          PositionSensor {{ name "lift encoder" }}
        ]
        endPoint Solid {{
          translation 0 0 {R["liftMin"]}
          name "lift carriage"
          children [
            {carriage}
            Connector {{
              translation {hook["reach"]} 0 {hook["heightAboveLift"]}
              rotation 0 1 0 {pi / 2:.7g}
              name "rail hook"
              model IS railModel
              type "active"
              numberOfRotations 0
              distanceTolerance 0.046
            }}
            {jaw(1)}
            {jaw(-1)}
          ]
          boundingObject Group {{ children [ {carriage_collision} ] }}
          physics Physics {{ density -1 mass {carriage_mass} }}
        }}
      }}
    ]
    boundingObject Group {{ children [ {collision} {glides} ] }}
    physics Physics {{ density -1 mass {base_mass:.6g} centerOfMass [ -0.032 0 {R["centerOfGravity"]} ] }}
  }}
}}
'''


def viewpoint():
    position = (-2.34, -2.83, 3.48)
    length = sqrt(sum(n * n for n in position))
    z = tuple(n / length for n in position)
    horizontal = sqrt(z[0] ** 2 + z[1] ** 2)
    x = (-z[1] / horizontal, z[0] / horizontal, 0)
    y = (-z[2] * x[1], z[2] * x[0], horizontal)
    angle = acos((x[0] + y[1] + z[2] - 1) / 2)
    factor = 2 * sin(angle)
    axis = ((y[2] - z[1]) / factor, (z[0] - x[2]) / factor, (x[1] - y[0]) / factor)
    return f"Viewpoint {{ position {numbers(position)} orientation {numbers((*axis, angle))} fieldOfView 0.62 }}"


def world_header():
    return f'''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/M01.proto"

WorldInfo {{
  basicTimeStep 4
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties {{ material1 "mat" material2 "tire" coulombFriction [ {R["friction"]} ] bumpSound "" rollSound "" slideSound "" }}
    ContactProperties {{ material1 "mat" material2 "caster" coulombFriction [ 0.055 ] bumpSound "" rollSound "" slideSound "" }}
    ContactProperties {{ material1 "jaw" material2 "payload" coulombFriction [ 1.6 ] bumpSound "" rollSound "" slideSound "" }}
    ContactProperties {{ material1 "mat" material2 "payload" coulombFriction [ 0.48 ] bumpSound "" rollSound "" slideSound "" }}
  ]
}}
{viewpoint()}
Background {{ skyColor [ 0.090 0.13 0.16 ] }}
DirectionalLight {{ direction -1 0.3 -2 ambientIntensity 0.62 intensity 2.1 castShadows TRUE }}
{static_box((0, 0, -F["matHeight"] / 2), (F["size"], F["size"], F["matHeight"]), "mat", "EVA mat", "mat")}
{static_box((0, 0, -0.40), (4.6, 4.6, 0.04), "dark", "Catch surface")}
'''


def mat_tiles():
    tile = F["tile"]
    for k in range(-2, 3):
        offset = (k + 0.5) * tile
        for start, end in (((offset, -HALF), (offset, HALF)), ((-HALF, offset), (HALF, offset))):
            yield floor_line(start, end, 0.0015, "dark")


def arena_world(layout):
    nodes = [world_header(), *mat_tiles(), *trident_parts(layout["hangSlots"])]
    for line in layout["lines"]:
        nodes.append(floor_line(line["a"], line["b"], F["lineWidth"], line["color"]))
    for slot in layout["hangSlots"]:
        x, y, yaw = slot["x"], slot["y"], slot["yaw"]
        start = (x - 0.032 * cos(yaw), y - 0.032 * sin(yaw))
        tip = (x + 0.032 * cos(yaw), y + 0.032 * sin(yaw))
        nodes.append(floor_line(start, tip, 0.012, slot["side"]))
        for sign in (-1, 1):
            tail = (x - sign * 0.017 * sin(yaw), y + sign * 0.017 * cos(yaw))
            nodes.append(floor_line(tail, tip, 0.009, slot["side"]))
    for start in layout["starts"]:
        x, y = start["x"], start["y"]
        w, d = start["w"] / 2, start["d"] / 2
        corners = [(x - w, y - d), (x + w, y - d), (x + w, y + d), (x - w, y + d)]
        for a, b in zip(corners, corners[1:] + corners[:1]):
            nodes.append(floor_line(a, b, 0.010, "tape"))
        if w > d:
            edge = (abs(x) + w - 0.034) * (-1 if x < 0 else 1)
            a, b = (edge, y - d), (edge, y + d)
        else:
            edge = (abs(y) + d - 0.034) * (-1 if y < 0 else 1)
            a, b = (x - w, edge), (x + w, edge)
        nodes.append(floor_line(a, b, F["lineWidth"], start["side"]))
    for base in layout["bases"]:
        nodes.extend(base_parts(base))
    for index, pillar in enumerate(layout["pillars"]):
        x, y = pillar["x"], pillar["y"]
        plate = F["pillarPlate"]
        for tier, z in enumerate(F["pillarLevels"]):
            offset = (-1 if tier == 0 else 1) * F["pillarOffset"]
            nodes.append(static_box((x + offset, y - offset, z - 0.003),
                                    (plate, plate, 0.006), "metal", f"Pillar {index} plate {tier}"))
        nodes.append(static_box((x, y, 0.047), (0.022, 0.022, 0.094), "metal", f"Pillar {index} upright"))
    nodes.extend(payload(prop) for prop in layout["props"])
    start = layout["starts"][0]
    nodes.append(f'DEF M01 M01 {{ translation {numbers((start["spawnX"], start["spawnY"], 0.001))} }}')
    return "\n".join(nodes) + "\n"


def lab_world():
    nodes = [world_header(), *mat_tiles()]
    start, y = LAB["rampStart"], LAB["rampY"]
    length, width = LAB["rampLength"], LAB["rampWidth"]
    angle = radians(LAB["rampAngle"])
    thickness = 0.018
    height = length * tan(angle)
    nodes.append(static_box(
        (start + length / 2 + sin(angle) * thickness / 2, y, height / 2 - cos(angle) * thickness / 2),
        (length / cos(angle), width, thickness), "ramp", "12 degree ramp", "mat", (0, 1, 0, -angle)
    ))
    platform = LAB["platformLength"]
    nodes.append(static_box((start + length + platform / 2, y, height - 0.01),
                            (platform, width, 0.02), "ramp", "Ramp landing", "mat"))
    step = LAB["stepHeight"]
    nodes.append(static_box((LAB["stepX"], LAB["stepY"], step / 2),
                            (0.105, 0.30, step), "metal", "12 mm threshold", "mat"))
    nodes.append(payload({"id": "lab_cube", "kind": "cube", "origin": "floor",
                          "x": LAB["cube"][0], "y": LAB["cube"][1], "z": F["cubeSide"] / 2}))
    nodes.append(f'DEF M01 M01 {{ translation {numbers((*LAB["start"], 0.001))} controllerArgs [ "--lab" ] }}')
    return "\n".join(nodes) + "\n"


def main():
    layout = field_layout()
    (PROJECT / "web/model.js").write_text(
        "// Generated by tools/build_worlds.py from model.json.\n"
        f"window.LOM_MODEL = {json.dumps({**MODEL, 'layout': layout}, ensure_ascii=False, indent=2)};\n",
        encoding="utf-8",
    )
    (PROJECT / "webots/protos/M01.proto").write_text(robot_proto(), encoding="utf-8")
    (PROJECT / "webots/worlds/lom_2027.wbt").write_text(arena_world(layout), encoding="utf-8")
    (PROJECT / "webots/worlds/drive_lab.wbt").write_text(lab_world(), encoding="utf-8")


if __name__ == "__main__":
    main()
