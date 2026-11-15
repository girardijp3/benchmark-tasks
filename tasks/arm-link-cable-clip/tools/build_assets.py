"""Writes assets/*.usda (boxes, cylinders and convex meshes, plain text). Run: python build_assets.py"""
import math
from pathlib import Path


def f(v):
    return f"{v:.7g}"


def v3(p):
    return f"({f(p[0])}, {f(p[1])}, {f(p[2])})"


def mat(name, rgb, rough=0.6, metal=0.0, root="ROOT"):
    return f'''        def Material "{name}"
        {{
            token outputs:surface.connect = </{root}/Looks/{name}/PreviewSurface.outputs:surface>

            def Shader "PreviewSurface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = ({f(rgb[0])}, {f(rgb[1])}, {f(rgb[2])})
                float inputs:metallic = {f(metal)}
                float inputs:roughness = {f(rough)}
                token outputs:surface
            }}
        }}
'''


def phys_mat(name, static, dynamic):
    return f'''        def Material "{name}" (
            prepend apiSchemas = ["PhysicsMaterialAPI"]
        )
        {{
            float physics:dynamicFriction = {f(dynamic)}
            float physics:restitution = 0
            float physics:staticFriction = {f(static)}
        }}
'''


def _xf(centre, rot_y_deg=0.0, rot_xyz=None, scale=None):
    ops, order = [f"        double3 xformOp:translate = {v3(centre)}"], ['"xformOp:translate"']
    r = rot_xyz if rot_xyz is not None else ((0.0, rot_y_deg, 0.0) if rot_y_deg else None)
    if r is not None:
        ops.append(f"        float3 xformOp:rotateXYZ = {v3(r)}")
        order.append('"xformOp:rotateXYZ"')
    if scale is not None:
        ops.append(f"        float3 xformOp:scale = {v3(scale)}")
        order.append('"xformOp:scale"')
    ops.append(f"        uniform token[] xformOpOrder = [{', '.join(order)}]")
    return "\n".join(ops)


def vis_box(name, m, size, centre, rot_y_deg=0.0, rot_xyz=None, root="ROOT"):
    return f'''    def Cube "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        rel material:binding = </{root}/Looks/{m}>
        double size = 1
{_xf(centre, rot_y_deg, rot_xyz, size)}
    }}
'''


def col_box(name, pm, size, centre, rot_y_deg=0.0, rot_xyz=None, root="ROOT"):
    return f'''        def Cube "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = </{root}/Looks/{pm}> (
                bindMaterialAs = "weakerThanDescendants"
            )
            custom float physxCollision:contactOffset = 0.002
            custom float physxCollision:restOffset = 0
            uniform token purpose = "guide"
            double size = 1
{_xf(centre, rot_y_deg, rot_xyz, size)}
        }}
'''


def _cyl_rot(axis):
    return {"x": (0.0, 90.0, 0.0), "y": (-90.0, 0.0, 0.0), "z": None}[axis]


def vis_cyl(name, m, radius, length, centre, axis="z", root="ROOT"):
    r = _cyl_rot(axis)
    return f'''    def Cylinder "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        rel material:binding = </{root}/Looks/{m}>
        uniform token axis = "Z"
        double height = {f(length)}
        double radius = {f(radius)}
{_xf(centre, rot_xyz=r)}
    }}
'''


def col_cyl(name, pm, radius, length, centre, axis="z", root="ROOT"):
    r = _cyl_rot(axis)
    return f'''        def Cylinder "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = </{root}/Looks/{pm}> (
                bindMaterialAs = "weakerThanDescendants"
            )
            custom float physxCollision:contactOffset = 0.002
            custom float physxCollision:restOffset = 0
            uniform token purpose = "guide"
            uniform token axis = "Z"
            double height = {f(length)}
            double radius = {f(radius)}
{_xf(centre, rot_xyz=r)}
        }}
'''


def hull_prism(name, pm, poly_xz, y0, y1, m=None, collide=True, visual=True, root="ROOT", indent=2):
    """A convex polygon (x, z points, counter-clockwise seen from -y... order as given) extruded from y0 to y1, as
    a convex mesh. Returns (visual text, collider text)."""
    pts = [(x, y0, z) for x, z in poly_xz] + [(x, y1, z) for x, z in poly_xz]
    n = len(poly_xz)
    faces, counts = [], []
    # side quads
    for i in range(n):
        j = (i + 1) % n
        faces += [i, j, n + j, n + i]
        counts.append(4)
    faces += list(range(n))[::-1]
    counts.append(n)
    faces += [n + i for i in range(n)]
    counts.append(n)
    # triangulate (Isaac splits n-gons from one corner, which is fine for convex faces, but do it ourselves)
    tri, tc = [], []
    pos = 0
    for c in counts:
        face = faces[pos:pos + c]
        pos += c
        for k in range(1, c - 1):
            tri += [face[0], face[k], face[k + 1]]
            tc.append(3)
    body = f'''            int[] faceVertexCounts = [{", ".join(map(str, tc))}]
            int[] faceVertexIndices = [{", ".join(map(str, tri))}]
            point3f[] points = [{", ".join(v3(p) for p in pts)}]
            uniform token subdivisionScheme = "none"
'''
    vis = col = ""
    if visual:
        vis = f'''    def Mesh "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        rel material:binding = </{root}/Looks/{m}>
{body.replace("            ", "        ")}    }}
'''
    if collide:
        col = f'''        def Mesh "{name}_col" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = </{root}/Looks/{pm}> (
                bindMaterialAs = "weakerThanDescendants"
            )
            uniform token physics:approximation = "convexHull"
            custom float physxCollision:contactOffset = 0.002
            custom float physxCollision:restOffset = 0
            uniform token purpose = "guide"
{body}        }}
'''
    return vis, col


def arc_boxes(prefix, pm, r_in, thick, th0, th1, step, width, y_c=0.0, r_in_end=None, centre=(0.0, 0.0)):
    """Tangent boxes round the axis (x, z plane), from polar angle th0 to th1 (deg, from +x toward +z), inner radius
    r_in (to r_in_end at th1, linearly), radial thickness ``thick``, ``width`` along y. Returns [(name, size, centre,
    rot_y_deg)]."""
    out = []
    n = max(1, int(round(abs(th1 - th0) / step)))
    for i in range(n):
        a0 = th0 + (th1 - th0) * i / n
        a1 = th0 + (th1 - th0) * (i + 1) / n
        am = 0.5 * (a0 + a1)
        d = math.radians(abs(a1 - a0))
        ri = r_in if r_in_end is None else r_in + (r_in_end - r_in) * (i + 0.5) / n
        rm = ri + thick / 2
        length = 2 * (ri + thick) * math.tan(d / 2) * 1.02
        t = math.radians(am)
        c = (centre[0] + rm * math.cos(t), y_c, centre[1] + rm * math.sin(t))
        out.append((f"{prefix}{i}", (length, width, thick), c, -(am + 90.0)))
    return out


def body_prim(name, doc, mass, translate, children, apis=("PhysicsRigidBodyAPI", "PhysicsMassAPI"), kin=False,
              com=None, extra="", colliders="", root_name=None):
    api = ", ".join(f'"{a}"' for a in apis)
    k = "\n        bool physics:kinematicEnabled = true" if kin else ""
    ms = f"\n        float physics:mass = {f(mass)}" if mass else ""
    cm = f"\n        point3f physics:centerOfMass = {v3(com)}" if com else ""
    return f'''    def Xform "{name}" (
        prepend apiSchemas = [{api}]
        doc = "{doc}"
    )
    {{{k}{ms}{cm}
        custom float physxRigidBody:maxDepenetrationVelocity = 0.2
        custom int physxRigidBody:solverPositionIterationCount = 32
        custom int physxRigidBody:solverVelocityIterationCount = 4
{extra}        double3 xformOp:translate = {v3(translate)}
        uniform token[] xformOpOrder = ["xformOp:translate"]

{children}
        def Scope "Colliders"
        {{
{colliders}        }}
    }}
'''


def revolute(name, doc, b0, b1, pos0, pos1, lo, hi, stiffness_per_deg, damping_per_deg, axis="Y", root="ROOT",
             target=0.0):
    return f'''        def PhysicsRevoluteJoint "{name}" (
            prepend apiSchemas = ["PhysicsDriveAPI:angular"]
            doc = "{doc}"
        )
        {{
            float drive:angular:physics:damping = {f(damping_per_deg)}
            float drive:angular:physics:stiffness = {f(stiffness_per_deg)}
            float drive:angular:physics:targetPosition = {f(target)}
            uniform token drive:angular:physics:type = "force"
            uniform token physics:axis = "{axis}"
            rel physics:body0 = </{root}/{b0}>
            rel physics:body1 = </{root}/{b1}>
            point3f physics:localPos0 = {v3(pos0)}
            point3f physics:localPos1 = {v3(pos1)}
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
            float physics:lowerLimit = {f(lo)}
            float physics:upperLimit = {f(hi)}
        }}
'''


def prismatic(name, doc, b0, b1, pos0, pos1, lo, hi, stiffness, damping, axis="Z", root="ROOT", target=0.0):
    return f'''        def PhysicsPrismaticJoint "{name}" (
            prepend apiSchemas = ["PhysicsDriveAPI:linear"]
            doc = "{doc}"
        )
        {{
            float drive:linear:physics:damping = {f(damping)}
            float drive:linear:physics:stiffness = {f(stiffness)}
            float drive:linear:physics:targetPosition = {f(target)}
            uniform token drive:linear:physics:type = "force"
            uniform token physics:axis = "{axis}"
            rel physics:body0 = </{root}/{b0}>
            rel physics:body1 = </{root}/{b1}>
            point3f physics:localPos0 = {v3(pos0)}
            point3f physics:localPos1 = {v3(pos1)}
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
            float physics:lowerLimit = {f(lo)}
            float physics:upperLimit = {f(hi)}
        }}
'''


def header(root, doc):
    return f'''#usda 1.0
(
    defaultPrim = "{root}"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}" (
    doc = "{doc}"
)
{{
'''

# ============================ the cable clip and the link jig ============================
MM = 0.001
OUT = Path(__file__).resolve().parent.parent / "assets"

# the link: a 25 mm carbon-fibre tube (typical arm link), held by its two joint housings in a repair jig
TUBE_R, TUBE_LEN, ZC = 12.5 * MM, 0.30, 0.060
HOUSING_R, HOUSING_LEN = 26 * MM, 40 * MM

# the clip (frame: origin the bore's centre, y the bore's axis, z up toward the loop; the opening faces -z)
W = 24 * MM                    # width along the tube
R_IN, THICK = 13.5, 5.5        # mm: the bore (tube 12.5 + 1.0) and the wall
PIVOT_TH = 25.0                # deg: the jaws hang from the ring at this polar angle (above the equator)
TIP_TH, R_TIP = -43.0, 11.0    # deg, mm: the lip tip: 0.8 mm inside the tube when it sits on the ring top (axis 1 mm above the bore centre)
JAW_STIFF = 0.10               # N m/rad: each jaw's spring
JAW_DAMP = 0.0015              # N m s/rad
JAW_LIMIT = 25.0               # deg: how far a jaw can swing open


def clip_boxes():
    ring = arc_boxes("Ring", "", R_IN, THICK, PIVOT_TH, 180 - PIVOT_TH, 10.0, W)
    jaw = arc_boxes("JawA", "", 13.8, THICK, PIVOT_TH, -8.0, 8.0, W) + \
        arc_boxes("JawB", "", 13.8, THICK, -8.0, TIP_TH, 8.0, W, r_in_end=R_TIP)
    return ring, jaw


def mm3(size, ctr):
    return tuple(s * MM for s in size), tuple(c * MM for c in ctr)


def write_clip():
    R = "ClipAsm"
    looks = '    def Scope "Looks"\n    {\n' + \
        mat("Plastic", (0.95, 0.45, 0.10), 0.5, root=R) + mat("JawPlastic", (0.95, 0.62, 0.10), 0.5, root=R) + \
        mat("Slot", (0.05, 0.05, 0.05), 0.8, root=R) + phys_mat("Phys_Plastic", 0.30, 0.25) + "    }\n\n"
    ring, jaw = clip_boxes()
    # all box sizes here are (length, width, thickness) in the arc helper's units: length and thickness mm, width m
    def conv(size, ctr):
        return (size[0] * MM, size[1], size[2] * MM), (ctr[0] * MM, ctr[1], ctr[2] * MM)
    vis, col = "", ""
    for name, size, ctr, a in ring:
        s, c = conv(size, ctr)
        vis += vis_box(name, "Plastic", s, c, a, root=R)
        col += col_box(name, "Phys_Plastic", s, c, a, root=R)
    loop_size, loop_c = (14 * MM, W, 17 * MM), (0.0, 0.0, 26.5 * MM)
    vis += vis_box("Loop", "Plastic", loop_size, loop_c, root=R)
    col += col_box("Loop", "Phys_Plastic", loop_size, loop_c, root=R)
    vis += vis_box("Slot", "Slot", (16 * MM, 6 * MM, 5 * MM), (0.0, 0.0, 27.0 * MM), root=R)
    rm = R_IN + THICK / 2
    P = {"R": (rm * math.cos(math.radians(PIVOT_TH)) * MM, 0.0, rm * math.sin(math.radians(PIVOT_TH)) * MM)}
    P["L"] = (-P["R"][0], 0.0, P["R"][2])
    bodies = body_prim("Clip", "The clip's ring and cable-tie loop: orange nylon, 24 mm wide, 38 mm across. Root link.",
                       0.024, (0, 0, 0), vis, apis=("PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI",
                                                    "PhysicsArticulationRootAPI", "PhysxArticulationAPI"),
                       com=(0.0, 0.0, 0.004),
                       extra="        custom bool physxArticulation:enabledSelfCollisions = 0\n", colliders=col)
    low = 1e9
    for side, sgn in (("R", 1.0), ("L", -1.0)):
        pv = P[side]
        vis = col = ""
        for name, size, ctr, a in jaw:
            s, c = conv(size, ctr)
            aa = a if sgn > 0 else -a
            cx = sgn * c[0]
            t = math.radians(aa)
            for dx in (-0.5, 0.5):
                for dz in (-0.5, 0.5):
                    z = c[2] - dx * s[0] * math.sin(t) + dz * s[2] * math.cos(t)
                    low = min(low, z)
            c = (cx - pv[0], c[1], c[2] - pv[2])
            vis += vis_box(name, "JawPlastic", s, c, aa, root=R)
            col += col_box(name, "Phys_Plastic", s, c, aa, root=R)
        bodies += body_prim(f"Jaw{side}", f"The {'right' if side == 'R' else 'left'} jaw: a spring arm ending in a lip. Origin on its hinge.",
                            0.008, pv, vis, colliders=col)
    kdeg = JAW_STIFF * math.pi / 180.0
    cdeg = JAW_DAMP * math.pi / 180.0
    joints = '    def Scope "Joints"\n    {\n'
    joints += revolute("HingeR", "The right jaw's hinge: 0 is rest (lip 0.8 mm inside the tube); it opens by turning negative, against a 0.1 N m/rad spring.",
                       "Clip", "JawR", P["R"], (0, 0, 0), -JAW_LIMIT, 0.0, kdeg, cdeg, root=R)
    joints += revolute("HingeL", "The left jaw's hinge: 0 is rest; it opens by turning positive, against a 0.1 N m/rad spring.",
                       "Clip", "JawL", P["L"], (0, 0, 0), 0.0, JAW_LIMIT, kdeg, cdeg, root=R)
    joints += "    }\n"
    text = header(R, "Nylon cable clip for a 25 mm tube: a 24 mm wide ring with two spring jaws and a cable-tie loop on top.") + looks + bodies + "\n" + joints + "}\n"
    (OUT / "cable_clip.usda").write_text(text)
    print("clip lowest point z (mm, clip frame):", round(low / MM, 2), "-> stand height (m)", round(-low, 5))


def write_jig():
    R = "ROOT"
    looks = '    def Scope "Looks"\n    {\n' + mat("Carbon", (0.10, 0.10, 0.11), 0.35, 0.1) + \
        mat("Alu", (0.72, 0.74, 0.77), 0.35, 0.8) + mat("Band", (0.95, 0.85, 0.10), 0.7) + \
        mat("Jig", (0.20, 0.30, 0.55), 0.7) + phys_mat("Phys_Carbon", 0.30, 0.25) + "    }\n\n"
    vis = col = ""
    vis += vis_cyl("Tube", "Carbon", TUBE_R, TUBE_LEN, (0, 0, ZC), "y")
    col += col_cyl("Tube", "Phys_Carbon", TUBE_R, TUBE_LEN, (0, 0, ZC), "y")
    for s, name in ((1, "A"), (-1, "B")):
        y = s * (TUBE_LEN / 2 + HOUSING_LEN / 2)
        vis += vis_cyl("Housing" + name, "Alu", HOUSING_R, HOUSING_LEN, (0, y, ZC), "y")
        col += col_cyl("Housing" + name, "Phys_Carbon", HOUSING_R, HOUSING_LEN, (0, y, ZC), "y")
        h = ZC - HOUSING_R * 0.7
        vis += vis_box("Saddle" + name, "Jig", (0.05, HOUSING_LEN, h), (0, y, h / 2))
        col += col_box("Saddle" + name, "Phys_Carbon", (0.05, HOUSING_LEN, h), (0, y, h / 2))
    vis += vis_cyl("PaintedBand", "Band", TUBE_R + 0.0004, 0.030, (0, 0, ZC), "y")
    text = f'''#usda 1.0
(
    defaultPrim = "ROOT"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "ROOT" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    doc = "A robot arm link in a repair jig: a 25 mm carbon-fibre tube 300 mm long with an aluminium joint housing at each end, held in two saddles; a 30 mm band is painted at the clip's place. Fixed to the table."
)
{{
    bool physics:kinematicEnabled = true
    float physics:mass = 2.0

{looks}{vis}
    def Scope "Colliders"
    {{
{col}    }}
}}
'''
    (OUT / "link_jig.usda").write_text(text)


write_clip()
write_jig()
