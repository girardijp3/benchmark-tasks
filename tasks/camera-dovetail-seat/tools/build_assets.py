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


def hull_points(name, pm, m, pts, root="ROOT"):
    """A convex mesh from a point cloud (scipy ConvexHull, triangulated here). Returns (visual text, collider text)."""
    import numpy as np
    from scipy.spatial import ConvexHull

    P = np.asarray(pts, float)
    hull = ConvexHull(P)
    tri = []
    ctr = P.mean(axis=0)
    for simplex in hull.simplices:
        a, b, c = P[simplex]
        n = np.cross(b - a, c - a)
        if np.dot(n, a - ctr) < 0:
            simplex = simplex[[0, 2, 1]]
        tri += [int(i) for i in simplex]
    used = sorted(set(tri))
    remap = {v: i for i, v in enumerate(used)}
    pl = [P[i] for i in used]
    tri = [remap[i] for i in tri]
    body = f"""int[] faceVertexCounts = [{", ".join(["3"] * (len(tri) // 3))}]
            int[] faceVertexIndices = [{", ".join(map(str, tri))}]
            point3f[] points = [{", ".join(v3(p) for p in pl)}]
            uniform token subdivisionScheme = "none"
"""
    vis = f'''    def Mesh "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        rel material:binding = </{root}/Looks/{m}>
        {body.replace(chr(10) + "            ", chr(10) + "        ")}    }}
'''
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


# ===== a robot head's dovetail camera mount, the camera module, and the parts rack the module lies on =====
MM = 0.001
OUT = Path(__file__).resolve().parent.parent / "assets"

# mount (frame: origin the middle of the plate's top face under the rail's axis, y toward the stop), mm
RAIL_Y0, RAIL_Y1 = -30.0, 36.0          # the male dovetail rail's ends; the stop block starts at RAIL_Y1
RAIL_BASE, RAIL_TOP, RAIL_H = 7.0, 11.5, 6.0   # half-widths at the plate and at the rail's top, height
STOP_Y1 = 44.0
PLATE_X, PLATE_Y0, PLATE_Y1, PLATE_T = 55.0, -95.0, 52.0, 8.0
HEAD_H = 50.0                           # the plate's top above the table
# module (frame: origin the middle of its underside, y along the slide), mm
MOD_W, MOD_L, MOD_H = 36.0, 40.0, 22.0
SLOT_OPEN, SLOT_TOP_H = 9.5, 7.5        # half-width of the slot at the underside (2.5 mm round the rail's base), height of the slot
SLOT_SLOPE = (RAIL_TOP - RAIL_BASE) / RAIL_H   # the slot's walls lean as the rail's do
MASS = 0.08
RISER_H = 30.0


def prism(name, mat_name, pm, poly, y0, y1, root="ROOT"):
    v, c = hull_prism(name, pm, [(x * MM, z * MM) for x, z in poly], y0 * MM, y1 * MM, m=mat_name, root=root)
    return v, c


def mount_text():
    looks = '    def Scope "Looks"\n    {\n' + mat("Plate", (0.20, 0.22, 0.26), 0.6, 0.3) + mat("Rail", (0.72, 0.74, 0.77), 0.35, 0.8) + \
        mat("Head", (0.10, 0.11, 0.13), 0.7) + mat("Stop", (0.85, 0.35, 0.10), 0.5) + phys_mat("Phys_Metal", 0.40, 0.30) + "    }\n\n"
    vis = col = ""
    ps = (2 * PLATE_X * MM, (PLATE_Y1 - PLATE_Y0) * MM, PLATE_T * MM)
    pc = (0, (PLATE_Y0 + PLATE_Y1) / 2 * MM, -PLATE_T / 2 * MM)
    vis += vis_box("Plate", "Plate", ps, pc)
    col += col_box("Plate", "Phys_Metal", ps, pc)
    hs = (0.09, 0.12, (HEAD_H - PLATE_T) * MM)
    hc = (0, -0.01, -(PLATE_T + (HEAD_H - PLATE_T) / 2) * MM)
    vis += vis_box("Head", "Head", hs, hc)
    col += col_box("Head", "Phys_Metal", hs, hc)
    v, c = prism("Rail", "Rail", "Phys_Metal", [(-RAIL_BASE, 0), (RAIL_BASE, 0), (RAIL_TOP, RAIL_H), (-RAIL_TOP, RAIL_H)], RAIL_Y0, RAIL_Y1)
    vis += v
    col += c
    ss = (50 * MM, (STOP_Y1 - RAIL_Y1) * MM, 16 * MM)
    sc = (0, (RAIL_Y1 + STOP_Y1) / 2 * MM, 8 * MM)
    vis += vis_box("Stop", "Stop", ss, sc)
    col += col_box("Stop", "Phys_Metal", ss, sc)
    return f'''#usda 1.0
(
    defaultPrim = "ROOT"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "ROOT" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    doc = "A robot head's camera mount: a plate with a male dovetail rail (14 mm wide at the plate, 23 mm at the top, 6 mm tall, 66 mm long) open at its front end and closed by a stop block at the other. Fixed to the table. Origin: the middle of the plate's top face under the rail's axis, 50 mm above the table, y toward the stop."
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


def module_text():
    R = "CameraModule"
    looks = '    def Scope "Looks"\n    {\n' + mat("Body", (0.12, 0.13, 0.15), 0.5, root=R) + mat("Lens", (0.02, 0.03, 0.06), 0.15, 0.2, root=R) + \
        mat("Ring", (0.70, 0.72, 0.75), 0.3, 0.8, root=R) + mat("Label", (0.9, 0.9, 0.9), 0.7, root=R) + phys_mat("Phys_Plastic", 0.40, 0.30) + "    }\n\n"
    vis = col = ""
    hw = MOD_W / 2
    L0, L1 = -MOD_L / 2, MOD_L / 2
    for side, sg in (("L", -1), ("R", 1)):
        poly = [(sg * SLOT_OPEN, 0), (sg * hw, 0), (sg * hw, SLOT_TOP_H), (sg * (SLOT_OPEN + SLOT_SLOPE * SLOT_TOP_H), SLOT_TOP_H)]
        poly = poly if sg > 0 else poly[::-1]
        v, c = prism(f"Cheek{side}", "Body", "Phys_Plastic", poly, L0, L1, root=R)
        vis += v
        col += c
    rs = (MOD_W * MM, MOD_L * MM, (MOD_H - SLOT_TOP_H) * MM)
    rc = (0, 0, (SLOT_TOP_H + (MOD_H - SLOT_TOP_H) / 2) * MM)
    vis += vis_box("Roof", "Body", rs, rc, root=R)
    col += col_box("Roof", "Phys_Plastic", rs, rc, root=R)
    # lens barrel on the +x side face (visual only), a label on top
    vis += vis_cyl("Barrel", "Ring", 7 * MM, 6 * MM, ((hw + 3) * MM, 0, 15 * MM), "x", root=R)
    vis += vis_cyl("Glass", "Lens", 5 * MM, 6.5 * MM, ((hw + 3.2) * MM, 0, 15 * MM), "x", root=R)
    vis += vis_box("Label", "Label", (20 * MM, 12 * MM, 0.5 * MM), (0, 0, (MOD_H + 0.3) * MM), root=R)
    body = body_prim("Module", "Camera module: 36 x 40 x 22 mm, 80 g, a dovetail slot in its underside. Origin: the middle of the underside.",
                     MASS, (0, 0, 0), vis, colliders=col, com=(0.0, 0.0, 0.01))
    return header(R, "Robot camera module with a dovetail slot (slides on the head's rail).") + looks + body + "}\n"


def rack_text():
    looks = '    def Scope "Looks"\n    {\n' + mat("Block", (0.35, 0.38, 0.42), 0.7) + mat("Top", (0.85, 0.85, 0.85), 0.7) + phys_mat("Phys_Block", 0.45, 0.35) + "    }\n\n"
    s = (70 * MM, 70 * MM, RISER_H * MM)
    vis = vis_box("Block", "Block", s, (0, 0, -RISER_H / 2 * MM)) + vis_box("TopMark", "Top", (0.06, 0.06, 0.0006), (0, 0, 0.0003))
    col = col_box("Block", "Phys_Block", s, (0, 0, -RISER_H / 2 * MM))
    return f'''#usda 1.0
(
    defaultPrim = "ROOT"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "ROOT" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    doc = "A parts rack: a 70 x 70 mm block 30 mm tall that the camera module lies on, a white square marked on top. Fixed to the table. Origin: the middle of its top face."
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


(OUT / "head_mount.usda").write_text(mount_text())
(OUT / "camera_module.usda").write_text(module_text())
(OUT / "parts_rack.usda").write_text(rack_text())
