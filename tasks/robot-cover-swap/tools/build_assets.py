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

# ============================ the robot's electronics-bay deck, two covers and the scrap tray ============================
MM = 0.001
OUT = Path(__file__).resolve().parent.parent / "assets"

# cover (frame: origin the middle of the skirt's lower edge, x forward, y left, z up), mm
COV_L, COV_W = 150.0, 100.0
SKIRT_H, WALL = 20.0, 3.0
PLATE_T = 3.0                       # the top plate, z SKIRT_H .. SKIRT_H + PLATE_T
RIB = (40.0, 12.0, 14.0)            # the finger rib on the plate: x, y, z
PEGS = [(-50.0, -30.0), (-50.0, 30.0), (50.0, 0.0)]   # peg centres in the cover's locked pose
SHAFT_R, HEAD_R, HEAD_T = 2.5, 6.0, 2.5
SLIDE = 10.0                        # the slot's length: the cover slides this far (toward the robot, -x) to lock
HOLE_X, HOLE_Y = 22.0, 18.0         # the hole the peg's head goes through (head 12 mm across)
SLOT_W = 8.0
SLOT_END = SHAFT_R + 0.5            # slot end past the peg's axis when locked
DECK_L, DECK_W, DECK_T = 190.0, 140.0, 12.0
CHASSIS_H = 60.0                    # the deck top above the table (the robot body below it)
RISER_H = 30.0
POST_H = SKIRT_H + PLATE_T + 1.0   # a peg's shaft above the deck top: the head leaves the plate 1 mm of play
TRAY_IN = (174.0, 124.0)
TRAY_H = 35.0
MASS = 0.10


def plate_boxes():
    """The top plate minus the three keyholes (a square hole and a slot each), as merged boxes (x0, x1, y0, y1)."""
    holes = []
    for px, py in PEGS:
        holes.append((px - SLIDE - HOLE_X / 2, px - SLIDE + HOLE_X / 2, py - HOLE_Y / 2, py + HOLE_Y / 2))
        holes.append((px - SLIDE, px + SLOT_END, py - SLOT_W / 2, py + SLOT_W / 2))
    xs = sorted({-COV_L / 2, COV_L / 2} | {h[0] for h in holes} | {h[1] for h in holes})
    ys = sorted({-COV_W / 2, COV_W / 2} | {h[2] for h in holes} | {h[3] for h in holes})
    rows = []
    for j in range(len(ys) - 1):
        yc = (ys[j] + ys[j + 1]) / 2
        runs, start = [], None
        for i in range(len(xs) - 1):
            xc = (xs[i] + xs[i + 1]) / 2
            inside = any(h[0] < xc < h[1] and h[2] < yc < h[3] for h in holes)
            if not inside and start is None:
                start = xs[i]
            if (inside or i == len(xs) - 2) and start is not None:
                end = xs[i] if inside else xs[i + 1]
                runs.append((start, end))
                start = None
        rows.append((ys[j], ys[j + 1], runs))
    # merge identical runs in consecutive rows
    boxes, open_ = [], {}
    for y0, y1, runs in rows:
        cur = {}
        for r in runs:
            if r in open_:
                b = open_.pop(r)
                b[3] = y1
                cur[r] = b
            else:
                cur[r] = [r[0], r[1], y0, y1]
        boxes += list(open_.values())
        open_ = cur
    boxes += list(open_.values())
    return [tuple(b) for b in boxes]


def cover_text(name, root, body_rgb, doc_text, cracked):
    looks = '    def Scope "Looks"\n    {\n' + mat("Shell", body_rgb, 0.5, root=root) + \
        mat("Rib", tuple(c * 0.8 for c in body_rgb), 0.5, root=root) + mat("Crack", (0.45, 0.02, 0.02), 0.8, root=root) + \
        mat("Label", (0.1, 0.55, 0.25), 0.7, root=root) + phys_mat("Phys_Abs", 0.45, 0.35) + "    }\n\n"
    vis, col = "", ""
    zc = SKIRT_H + PLATE_T / 2
    for k, (x0, x1, y0, y1) in enumerate(plate_boxes()):
        s = ((x1 - x0) * MM, (y1 - y0) * MM, PLATE_T * MM)
        c = ((x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, zc * MM)
        vis += vis_box(f"Plate{k}", "Shell", s, c, root=root)
        col += col_box(f"Plate{k}", "Phys_Abs", s, c, root=root)
    sk = SKIRT_H / 2 * MM
    for nm, s, c in (("SkirtF", (WALL, COV_W, SKIRT_H), (COV_L / 2 - WALL / 2, 0.0)),
                     ("SkirtB", (WALL, COV_W, SKIRT_H), (-COV_L / 2 + WALL / 2, 0.0)),
                     ("SkirtL", (COV_L - 2 * WALL, WALL, SKIRT_H), (0.0, COV_W / 2 - WALL / 2)),
                     ("SkirtR", (COV_L - 2 * WALL, WALL, SKIRT_H), (0.0, -COV_W / 2 + WALL / 2))):
        s = tuple(v * MM for v in s)
        vis += vis_box(nm, "Shell", s, (c[0] * MM, c[1] * MM, sk), root=root)
        col += col_box(nm, "Phys_Abs", s, (c[0] * MM, c[1] * MM, sk), root=root)
    rs = tuple(v * MM for v in RIB)
    rc = (0.0, 0.0, (SKIRT_H + PLATE_T + RIB[2] / 2) * MM)
    vis += vis_box("Rib", "Rib", rs, rc, root=root)
    col += col_box("Rib", "Phys_Abs", rs, rc, root=root)
    ztop = (SKIRT_H + PLATE_T + 0.3) * MM
    if cracked:  # crack lines: thin dark boxes on the plate (visual only)
        for k, (cx, cy, ln, ang) in enumerate(((30, 35, 70, 25), (-35, 25, 50, -40), (55, -20, 40, 70), (-20, -45, 60, 10))):
            vis += vis_box(f"Crack{k}", "Crack", (ln * MM, 1.2 * MM, 0.5 * MM), (cx * MM, cy * MM, ztop), rot_xyz=(0, 0, ang), root=root)
    else:
        vis += vis_box("Label", "Label", (60 * MM, 14 * MM, 0.5 * MM), (-45 * MM, -35 * MM, ztop), root=root)
    body = body_prim("Cover", doc_text, MASS, (0, 0, 0), vis, colliders=col, com=(0.0, 0.0, 0.012))
    return header(root, doc_text) + looks + body + "}\n"


def write_deck():
    R = "ROOT"
    looks = '    def Scope "Looks"\n    {\n' + mat("Deck", (0.25, 0.27, 0.30), 0.6, 0.3) + mat("Body", (0.12, 0.13, 0.15), 0.7) + mat("Peg", (0.78, 0.78, 0.80), 0.35, 0.9) + \
        phys_mat("Phys_Metal", 0.40, 0.30) + "    }\n\n"
    vis = col = ""
    s = (DECK_L * MM, DECK_W * MM, DECK_T * MM)
    c = (0.0, 0.0, -DECK_T / 2 * MM)    # the deck's top face is the fixture's z = 0
    vis += vis_box("Deck", "Deck", s, c)
    col += col_box("Deck", "Phys_Metal", s, c)
    bh = CHASSIS_H - DECK_T
    bs = ((DECK_L - 30) * MM, (DECK_W - 30) * MM, bh * MM)
    bc = (0.0, 0.0, -(DECK_T + bh / 2) * MM)
    vis += vis_box("Body", "Body", bs, bc)
    col += col_box("Body", "Phys_Metal", bs, bc)
    for k, (px, py) in enumerate(PEGS):
        vis += vis_cyl(f"Post{k}", "Peg", SHAFT_R * MM, POST_H * MM, (px * MM, py * MM, POST_H / 2 * MM), "z")
        col += col_cyl(f"Post{k}", "Phys_Metal", SHAFT_R * MM, POST_H * MM, (px * MM, py * MM, POST_H / 2 * MM), "z")
        hz = (POST_H + HEAD_T / 2) * MM
        vis += vis_cyl(f"Head{k}", "Peg", HEAD_R * MM, HEAD_T * MM, (px * MM, py * MM, hz), "z")
        col += col_cyl(f"Head{k}", "Phys_Metal", HEAD_R * MM, HEAD_T * MM, (px * MM, py * MM, hz), "z")
    text = f'''#usda 1.0
(
    defaultPrim = "ROOT"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "ROOT" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    doc = "The top deck of a robot's electronics bay: a 190 x 140 x 12 mm plate with three headed posts (5 mm shaft, 12 mm head) that carry the cover's keyholes. Fixed to the table. Origin: the middle of the deck's top face, 60 mm above the table (the robot body below it is a box)."
)
{{
    bool physics:kinematicEnabled = true
    float physics:mass = 3.0

{looks}{vis}
    def Scope "Colliders"
    {{
{col}    }}
}}
'''
    (OUT / "bay_deck.usda").write_text(text)


def write_tray():
    R = "ROOT"
    looks = '    def Scope "Looks"\n    {\n' + mat("Tray", (0.15, 0.35, 0.65), 0.6) + phys_mat("Phys_Tray", 0.45, 0.35) + "    }\n\n"
    vis = col = ""
    ix, iy = TRAY_IN[0] * MM, TRAY_IN[1] * MM
    t = WALL * MM
    parts = [("Floor", (ix + 2 * t, iy + 2 * t, t), (0, 0, t / 2)),
             ("WallF", (t, iy + 2 * t, TRAY_H * MM), (ix / 2 + t / 2, 0, TRAY_H * MM / 2 + t)),
             ("WallB", (t, iy + 2 * t, TRAY_H * MM), (-ix / 2 - t / 2, 0, TRAY_H * MM / 2 + t)),
             ("WallL", (ix, t, TRAY_H * MM), (0, iy / 2 + t / 2, TRAY_H * MM / 2 + t)),
             ("WallR", (ix, t, TRAY_H * MM), (0, -iy / 2 - t / 2, TRAY_H * MM / 2 + t))]
    parts.append(("Riser", ((TRAY_IN[0] + 2 * WALL - 20) * MM, (TRAY_IN[1] + 2 * WALL - 20) * MM, RISER_H * MM),
                  (0, 0, -RISER_H / 2 * MM)))
    for nm, s, c in parts:
        vis += vis_box(nm, "Tray", s, c)
        col += col_box(nm, "Phys_Tray", s, c)
    text = f'''#usda 1.0
(
    defaultPrim = "ROOT"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "ROOT" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    doc = "A scrap tray: 174 x 124 mm inside, 35 mm walls, 3 mm floor. On a 30 mm riser, fixed to the table. Origin: the middle of the floor's underside, 30 mm above the table."
)
{{
    bool physics:kinematicEnabled = true
    float physics:mass = 1.0

{looks}{vis}
    def Scope "Colliders"
    {{
{col}    }}
}}
'''
    (OUT / "scrap_tray.usda").write_text(text)


(OUT / "cover_cracked.usda").write_text(cover_text("Cracked", "CoverCracked", (0.55, 0.57, 0.60),
    "The bay's original top cover: grey ABS, 150 x 100 mm, 20 mm skirt, 3 mm plate with three keyholes, finger rib on top, a cracked plate (crack lines drawn only). 100 g. Origin: the middle of the skirt's lower edge.", True))
(OUT / "cover_new.usda").write_text(cover_text("New", "CoverNew", (0.93, 0.93, 0.90),
    "The new top cover: white ABS, 150 x 100 mm, 20 mm skirt, 3 mm plate with three keyholes, finger rib on top, a green label. 100 g. Origin: the middle of the skirt's lower edge.", False))
def write_rack():
    looks = '    def Scope "Looks"\n    {\n' + mat("Rack", (0.35, 0.38, 0.42), 0.7) + phys_mat("Phys_Rack", 0.45, 0.35) + "    }\n\n"
    s = (170 * MM, 120 * MM, RISER_H * MM)
    c = (0, 0, -RISER_H / 2 * MM)
    vis = vis_box("Rack", "Rack", s, c)
    col = col_box("Rack", "Phys_Rack", s, c)
    text = f'''#usda 1.0
(
    defaultPrim = "ROOT"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "ROOT" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    doc = "A parts rack: a 170 x 120 mm block 30 mm tall that the new cover lies on. Fixed to the table. Origin: the middle of its top face."
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
    (OUT / "parts_rack.usda").write_text(text)


write_deck()
write_tray()
write_rack()
print("plate boxes:", len(plate_boxes()))
