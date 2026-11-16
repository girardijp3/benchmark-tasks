"""Writes this task's USD assets with pxr (rounded prisms and lathes; no meshes from outside).

Run:  python build_assets.py        (writes ./assets/*.usda)

Frames: metres, kilograms, Z up. The balance asset's origin is the middle of the base's underside; the root link
``Base`` is free. The two front feet are child links on vertical prismatic joints; each carries a knurled wheel on a
revolute joint. The fixed rear foot is part of ``Base``. The bubble is a separate gravity-free visual body.

Real product: Shimadzu AUX220 analytical balance, 220 g x 0.1 mg, 217 W x 356 D x 338 H mm, 7.0 kg (Shimadzu AU
series specification page and the AUW/AUX instruction manual).  The manual's levelling section: two level screws at
the front, a fixed foot at the centre rear, the level indicator at the front, and "the screws grow longer when turned
clockwise from above".  Values marked EST are estimates (the manual does not publish the screw thread, wheel size,
travel or torque); every number is listed in REFERENCE.md.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001


class Asset:
    def __init__(self, fname, name, doc, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True, no_gravity=False):
        self.path = OUT / fname
        self.stage = Usd.Stage.CreateInMemory()
        s = self.stage
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(s, 1.0)
        self.root = UsdGeom.Xform.Define(s, f"/{name}")
        s.SetDefaultPrim(self.root.GetPrim())
        p = self.root.GetPrim()
        p.SetMetadata("documentation", doc)
        self.name = name
        if body:
            UsdPhysics.RigidBodyAPI.Apply(p)
            m = UsdPhysics.MassAPI.Apply(p)
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
            if no_gravity:
                p.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        self.mats = {}
        self.n = 0
        self.cur = f"/{name}"

    def link(self, lname, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), pos=(0, 0, 0)):
        path = f"/{self.name}/{lname}"
        x = UsdGeom.Xform.Define(self.stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        p = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(p)
        m = UsdPhysics.MassAPI.Apply(p)
        m.CreateMassAttr(mass)
        m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        self.cur = path
        return p

    def color(self, key, rgb, rough=0.6, metal=0.0, opacity=1.0):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        if opacity < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def phys_mat(self, key, static, dynamic):
        k = "Phys_" + key
        if k in self.mats:
            return self.mats[k]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{k}")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        self.mats[k] = mat
        return mat

    def _xf(self, g, center, rot_x=0.0, rot_y=0.0, rot_z=0.0):
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        if rot_x:
            g.AddRotateXOp().Set(rot_x)
        if rot_y:
            g.AddRotateYOp().Set(rot_y)
        if rot_z:
            g.AddRotateZOp().Set(rot_z)

    def box(self, center, size, rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0)):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center, *rot)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key)

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

    def sphere(self, center, radius, rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Sphere.Define(self.stage, f"{self.cur}/{name or 'Sphere'}{self.n}")
        g.CreateRadiusAttr(radius)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

    def prism_z(self, poly_xy, z0, z1, rgb, collide=None, key="c", name="Prism", poly_top=None):
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        pts = [(x, y, z0) for (x, y) in poly_xy] + [(x, y, z1) for (x, y) in (poly_top or poly_xy)]
        k = len(poly_xy)
        faces, counts = [], []
        for i in range(1, k - 1):
            faces += [0, i + 1, i]
            faces += [k, k + i, k + i + 1]
            counts += [3, 3]
        for i in range(k):
            j = (i + 1) % k
            faces += [i, j, k + j, i, k + j, k + i]
            counts += [3, 3]
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        g.CreateFaceVertexCountsAttr(counts)
        g.CreateFaceVertexIndicesAttr(faces)
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        self._finish(g, rgb, collide, key, mesh=True)

    def _finish(self, g, rgb, collide, key, mesh=False):
        prim = g.GetPrim()
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(prim)
            if mesh:
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.001)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
            pm = self.phys_mat(key, *collide)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
            if rgb is None:
                g.CreatePurposeAttr("guide")
        if rgb is not None:
            if key == "glass":
                mat = self.color("glass", rgb, rough=0.08, metal=0.0, opacity=0.24)
            elif key == "steel":
                mat = self.color("steel", rgb, rough=0.28, metal=0.85)
            else:
                mat = self.color("m%02d%02d%02d" % tuple(int(c * 99) for c in rgb), rgb)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def articulation_root(self):
        p = self.root.GetPrim()
        m = UsdPhysics.MassAPI(p)
        mass, com, inertia = m.GetMassAttr().Get(), m.GetCenterOfMassAttr().Get(), m.GetDiagonalInertiaAttr().Get()
        p.RemoveAPI(UsdPhysics.MassAPI)
        p.RemoveAPI(UsdPhysics.RigidBodyAPI)
        self.base_prim = self.link("Base", mass, com=tuple(com), inertia=tuple(inertia))
        UsdPhysics.ArticulationRootAPI.Apply(self.base_prim)

    def prismatic(self, name, b0, b1, axis="Z", lo=-0.04, hi=0.0, stiffness=0.0, damping=5.0, target=0.0,
                  p0=(0, 0, 0), p1=(0, 0, 0), max_force=None):
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.PrismaticJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(stiffness)
        d.CreateDampingAttr(damping)
        d.CreateTargetPositionAttr(target)
        if max_force is not None:
            d.CreateMaxForceAttr(max_force)
        return j

    def revolute(self, name, b0, b1, axis="Z", lo=-1440.0, hi=1440.0, stiffness=0.0, damping=0.0, target=0.0,
                 p0=(0, 0, 0), p1=(0, 0, 0), armature=1e-6, max_force=None):
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.RevoluteJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(stiffness)
        d.CreateDampingAttr(damping)
        d.CreateTargetPositionAttr(target)
        if max_force is not None:
            d.CreateMaxForceAttr(max_force)
        j.GetPrim().AddAppliedSchema("PhysxJointAPI")
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(armature)
        return j

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path, self.path.stat().st_size, "bytes")


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


def rounded_rect(w, d, r, n=6):
    """A rounded rectangle in x-y (clockwise), ``n`` points per corner."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, d / 2 - r, 0), (-w / 2 + r, d / 2 - r, 90),
                       (-w / 2 + r, -d / 2 + r, 180), (w / 2 - r, -d / 2 + r, 270)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def lathe(profile, n=96):
    """Revolve a closed (r, z) profile about z: (points, triangles), outward wound."""
    k = len(profile)
    pts = [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z) for i in range(n) for (r, z) in profile]
    tris = []
    for i in range(n):
        i2 = (i + 1) % n
        for j in range(k):
            j2 = (j + 1) % k
            a_, b_, c_, d_ = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            tris += [(a_, b_, c_), (a_, c_, d_)]
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[x], np.cross(p[y], p[z])) for x, y, z in tris) / 6.0
    return pts, (tris if vol >= 0 else [(x, z, y) for x, y, z in tris])


def vmesh(a, pts_tris, rgb, rough=0.5, metal=0.0, name="Vis", center=(0, 0, 0)):
    pts, tris = pts_tris
    a.n += 1
    g = UsdGeom.Mesh.Define(a.stage, f"{a.cur}/{name}{a.n}")
    P = [(p[0] + center[0], p[1] + center[1], p[2] + center[2]) for p in pts]
    g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in p]) for p in P]))
    g.CreateFaceVertexCountsAttr([3] * len(tris))
    g.CreateFaceVertexIndicesAttr([int(v) for t in tris for v in t])
    g.CreateSubdivisionSchemeAttr("none")
    arr = np.asarray(P, float)
    g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in arr.min(axis=0)]),
                                      Gf.Vec3f(*[float(v) for v in arr.max(axis=0)])]))
    key = "v%02d%02d%02d_%d_%d" % (int(rgb[0] * 99), int(rgb[1] * 99), int(rgb[2] * 99),
                                   int(rough * 100), int(metal * 100))
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(a.color(key, rgb, rough, metal))


# ======================================================================================================================
# Dimensions (m).  Published: 217 W x 356 D x 338 H, 7.0 kg, pan 80 mm.  EST: base 60 mm, draft shield 238 mm, foot
# extension 40 mm, wheel 34 mm dia x 26 mm, thread 1.75 mm/rev, front feet at x -168, y +-85 mm, rear foot at x +150.
# ======================================================================================================================
BODY_W, BODY_D = 0.217, 0.356
BASE_H = 0.060
BODY_H = 0.298  # base 60 + draft shield 238 (EST)
FOOT_NOM = 0.040
FRONT_X, FRONT_Y = -0.168, 0.085
REAR_X = 0.150
WHEEL_R, WHEEL_H = 0.017, 0.026
WHEEL_Z = 0.024  # wheel centre above the foot origin (the pad's bottom)
PAD_R, PAD_H = 0.012, 0.008
SHAFT_R, SHAFT_TOP = 0.005, 0.100
M_BODY, M_FOOT, M_WHEEL = 6.70, 0.12, 0.03  # total 7.00 kg (published net weight); split EST

# colours: die-cast aluminium base painted dark graphite, ABS housing light grey, glass pale blue, stainless steel
GRAPHITE = (0.16, 0.17, 0.18)
HOUSING = (0.80, 0.80, 0.78)
GLASS = (0.72, 0.82, 0.86)
STEEL = (0.72, 0.73, 0.75)
DARK = (0.05, 0.05, 0.055)
RUBBER = (0.03, 0.03, 0.03)
LCD = (0.05, 0.16, 0.11)
LCD_ON = (0.35, 0.95, 0.55)
RED = (0.62, 0.05, 0.04)
WHITE = (0.92, 0.92, 0.90)


def balance(fname):
    inertia = box_inertia(M_BODY, BODY_W, BODY_D, BODY_H)
    a = Asset(fname, "Balance",
              "Shimadzu AUX220 analytical balance: 220 g x 0.1 mg, 217 x 356 x 338 mm, 7.0 kg, 80 mm pan. "
              "Two large front level screws and one fixed centre-rear foot (three-point support). Free articulation "
              "root Base; FootL/FootR slide vertically on prismatic joints and carry the knurled wheels WheelL/WheelR "
              "on revolute joints. Origin: middle of the base's underside.",
              M_BODY, com=(0.0, 0.0, 0.095), inertia=inertia)
    a.articulation_root()
    base = a.stage.GetPrimAtPath(f"/Balance/Base")
    a.cur = "/Balance/Base"

    # -- base: rounded die-cast housing, 60 mm tall, with a 2 mm top edge chamfer --------------------------------
    rp = rounded_rect(BODY_W, BODY_D, 0.010, n=7)
    rp_top = rounded_rect(BODY_W - 0.006, BODY_D - 0.006, 0.009, n=7)
    # the front 40 mm is relieved to expose the two level screws: the lower 40 mm of the base is cut back
    # 28 mm, leaving an 18 mm front lip above the 26 mm wheels (as the drawing's rounded front base does)
    rp_low = [(x + 0.014, y) for (x, y) in rounded_rect(0.328, BODY_D, 0.010, n=7)]
    a.prism_z(rp_low, 0.0, 0.020, GRAPHITE, (0.7, 0.6), key="base", name="BaseLow")
    a.prism_z(rp_low, 0.020, 0.040, GRAPHITE, (0.7, 0.6), key="base", name="BaseMid")
    a.prism_z(rp, 0.040, BASE_H - 0.002, GRAPHITE, (0.7, 0.6), key="base", name="BaseUp")
    a.prism_z(rp_top, BASE_H - 0.002, BASE_H, GRAPHITE, (0.7, 0.6), key="base", name="BaseTop")
    # rubber feet pads (the rear foot is part of the body; the front feet are separate links)
    a.cur = "/Balance/Base"
    # the rear foot: a 30 mm rubber pad with a spherical underside (collider: sphere r 15 at z -25 mm)
    a.cyl((REAR_X, 0, -0.031), 0.015, 0.018, rgb=RUBBER, collide=None, key="rubber", name="RearFoot")
    a.sphere((REAR_X, 0, -0.025), 0.015, rgb=RUBBER, collide=(0.95, 0.9), key="rubber", name="RearPad")

    # -- key panel: a shallow wedge across the front, with the LCD, six keys and the circular level -----------------
    # wedge x from the front face (-178 mm) to -90 mm, z 60 -> 75 mm
    panel_poly = [(-0.178, 0.060), (-0.178, 0.072), (-0.140, 0.077), (-0.140, 0.060)]
    a.cur = "/Balance/Base"
    # a y-prism: poly in x-z, along y +-108.5 mm
    pts = [(x, y, z) for y in (-0.1085, 0.1085) for (x, z) in panel_poly]
    a.n += 1
    g = UsdGeom.Mesh.Define(a.stage, f"/Balance/Base/KeyPanel{a.n}")
    # build a convex prism along y
    g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    k = 4
    faces, counts = [], []
    for i in range(1, k - 1):
        faces += [0, i, i + 1]
        faces += [k, k + i + 1, k + i]
        counts += [3, 3]
    for i in range(k):
        j = (i + 1) % k
        faces += [i, k + j, j, i, k + i, k + j]
        counts += [3, 3]
    g.CreateFaceVertexCountsAttr(counts)
    g.CreateFaceVertexIndicesAttr(faces)
    g.CreateSubdivisionSchemeAttr("none")
    g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-0.178, -0.1085, 0.060), Gf.Vec3f(-0.140, 0.1085, 0.077)]))
    a._finish(g, HOUSING, (0.7, 0.6), "housing", mesh=True)

    # display recess (dark) on the sloped face, 50 x 17 mm, with four pale-green digit groups
    a.box((-0.1595, 0.040, 0.0752), (0.017, 0.050, 0.0015), LCD, None, name="LCD")
    for i in range(4):
        a.box((-0.1595, 0.020 + i * 0.014, 0.0761), (0.014, 0.010, 0.0012), LCD_ON, None, name="Digit")
    # six keys, two rows of three, in front of the display
    for i, (dx, dy) in enumerate(((-0.171, -0.030), (-0.159, -0.030), (-0.147, -0.030),
                                  (-0.171, -0.045), (-0.159, -0.045), (-0.147, -0.045))):
        a.box((dx, dy, 0.0753), (0.010, 0.008, 0.0018), (0.62, 0.63, 0.62), None, name="Key")
    # a small dark legend plate on the front face
    a.box((-0.1795, 0.045, 0.045), (0.001, 0.060, 0.020), (0.35, 0.36, 0.36), None, name="Plate")
    for i in range(5):
        a.box((-0.1802, 0.022 + i * 0.012, 0.045), (0.0008, 0.050, 0.0016), (0.10, 0.10, 0.10), None, name="Label")

    # -- circular level: 30 mm black bezel, glass well, red centre circle, a green bubble body is separate ----------
    VIAL = np.array([-0.160, -0.073, 0.0784])
    a.cyl((VIAL[0], VIAL[1], 0.0758), 0.017, 0.003, rgb=DARK, collide=None, key="dark", name="VialBezel")
    a.cyl((VIAL[0], VIAL[1], 0.0772), 0.0148, 0.0012, rgb=GLASS, collide=None, key="glass", name="VialGlass")
    a.cyl((VIAL[0], VIAL[1], 0.0779), 0.0025, 0.0004, rgb=RED, collide=None, key="red", name="VialCentre")
    a.cyl((VIAL[0], VIAL[1], 0.0778), 0.0090, 0.0003, rgb=(0.55, 0.55, 0.55), collide=None, name="VialRing")

    # -- draft shield: base plate, 3 glass sides + top, dark frame and handles -------------------------------------
    SH_X0, SH_X1 = -0.135, 0.160
    SH_Y = 0.095
    SH_Z0, SH_Z1 = 0.070, 0.296
    a.box(((SH_X0 + SH_X1) / 2, 0, 0.065), (SH_X1 - SH_X0, 2 * SH_Y + 0.010, 0.010), GRAPHITE, (0.7, 0.6),
          key="base", name="ShieldFloor")
    # glass: left, right, front, top (3 mm thick, pale blue, 20% opaque)
    a.box((SH_X0 + 0.0015, 0, (SH_Z0 + SH_Z1) / 2), (0.003, 2 * SH_Y, SH_Z1 - SH_Z0), GLASS, None, key="glass", name="GlassFront")
    a.box((SH_X1 - 0.0015, 0, (SH_Z0 + SH_Z1) / 2), (0.003, 2 * SH_Y, SH_Z1 - SH_Z0), GLASS, None, key="glass", name="GlassBack")
    for s in (-1, 1):
        a.box(((SH_X0 + SH_X1) / 2, s * (SH_Y - 0.0015), (SH_Z0 + SH_Z1) / 2),
              (SH_X1 - SH_X0, 0.003, SH_Z1 - SH_Z0), GLASS, None, key="glass", name="GlassSide")
    a.box(((SH_X0 + SH_X1) / 2, 0, SH_Z1 - 0.0015), (SH_X1 - SH_X0, 2 * SH_Y, 0.003), GLASS, None, key="glass", name="GlassTop")
    # corner posts and top rails
    for sx in (SH_X0, SH_X1):
        for sy in (-SH_Y, SH_Y):
            a.box((sx, sy, (SH_Z0 + SH_Z1) / 2), (0.010, 0.010, SH_Z1 - SH_Z0), GRAPHITE, None, name="Post")
    for sy in (-SH_Y, SH_Y):
        a.box(((SH_X0 + SH_X1) / 2, sy, SH_Z1 - 0.006), (SH_X1 - SH_X0, 0.010, 0.012), GRAPHITE, None, name="TopRail")
    for sx in (SH_X0, SH_X1):
        a.box((sx, 0, SH_Z1 - 0.006), (0.010, 2 * SH_Y, 0.012), GRAPHITE, None, name="TopEnd")
    # top handle/vent, a 54 mm oval-ish boss
    a.cyl(((SH_X0 + SH_X1) / 2, 0, SH_Z1 + 0.00075), 0.027, 0.0015, rgb=GRAPHITE, collide=None, name="TopHandle")
    a.cyl(((SH_X0 + SH_X1) / 2, 0, SH_Z1 + 0.0019), 0.019, 0.0008, rgb=(0.35, 0.36, 0.37), collide=None, name="TopKnob")
    # sliding-door handle on the front glass
    a.box((-0.138, 0, 0.250), (0.006, 0.044, 0.012), (0.40, 0.41, 0.42), None, name="DoorHandle")
    # pan support and 80 mm stainless pan, inside
    PAN = np.array([0.010, 0.0, 0.118])
    a.cyl((PAN[0], PAN[1], 0.094), 0.016, 0.050, rgb=STEEL, collide=None, key="steel", name="PanPost")
    a.cyl((PAN[0], PAN[1], PAN[2]), 0.040, 0.0015, rgb=STEEL, collide=None, key="steel", name="Pan")
    a.cyl((PAN[0], PAN[1], PAN[2] + 0.0009), 0.038, 0.0008, rgb=(0.82, 0.83, 0.84), collide=None, key="steel", name="PanFace")

    # -- back: power socket, label, and a power cord lying on the bench ---------------------------------------
    a.box((0.1785, -0.030, 0.085), (0.004, 0.030, 0.022), DARK, None, name="Socket")
    a.box((0.1785, 0.045, 0.085), (0.004, 0.060, 0.028), (0.55, 0.56, 0.56), None, name="BackPlate")

    # -- front feet: two links on prismatic joints, each with a knurled wheel on a revolute joint ----------------
    for s, nm, wnm in ((1, "FootL", "WheelL"), (-1, "FootR", "WheelR")):
        y = s * FRONT_Y
        a.cur = "/Balance/Base"
        # a small threaded boss on the underside, where the screw enters
        a.cyl((FRONT_X, y, 0.0395), 0.014, 0.003, rgb=GRAPHITE, collide=None, key="base", name="Boss")
        # foot link
        a.link(nm, M_FOOT, com=(0, 0, 0.045), inertia=box_inertia(M_FOOT, 0.024, 0.024, 0.100),
               pos=(FRONT_X, y, 0.0))
        a.cur = f"/Balance/{nm}"
        a.cyl((0, 0, 0.004), PAD_R, PAD_H, rgb=RUBBER, collide=None, key="rubber", name="Pad")
        a.sphere((0, 0, PAD_R), PAD_R, rgb=RUBBER, collide=(0.95, 0.9), key="rubber", name="PadSole")
        a.cyl((0, 0, 0.029), SHAFT_R, SHAFT_TOP - 0.008, rgb=STEEL, collide=None, key="steel", name="Shaft")
        a.cyl((0, 0, SHAFT_TOP - 0.002), 0.006, 0.004, rgb=(0.55, 0.56, 0.57), collide=None, key="steel", name="ShaftTop")
        # wheel link
        a.link(wnm, M_WHEEL, com=(0, 0, 0), inertia=(2.4e-6, 2.4e-6, 4.3e-6), pos=(FRONT_X, y, WHEEL_Z))
        a.cur = f"/Balance/{wnm}"
        # the knurled wheel: a 96-sided lathe with a chamfered rim, plus 24 visual knurl ridges
        prof = [(0.0052, -WHEEL_H / 2), (0.0150, -WHEEL_H / 2), (WHEEL_R, -WHEEL_H / 2 + 0.0015),
                (WHEEL_R, WHEEL_H / 2 - 0.0015), (0.0150, WHEEL_H / 2), (0.0052, WHEEL_H / 2)]
        vmesh(a, lathe(prof, 96), (0.06, 0.06, 0.065), rough=0.55, metal=0.15, name="Wheel")
        for i in range(24):
            th = 2 * math.pi * i / 24
            x, y = WHEEL_R * math.cos(th), WHEEL_R * math.sin(th)
            a.box((x, y, 0), (0.0016, 0.0032, WHEEL_H - 0.004), (0.10, 0.10, 0.11), None, name="Knurl",
                  rot=(0, 0, math.degrees(th)))
        # a red index mark on the top face
        a.cyl((0.010, 0, WHEEL_H / 2 + 0.0004), 0.0018, 0.0008, rgb=RED, collide=None, key="red", name="Index")
        # wheel collider: a 96-sided cylinder of the outer diameter (solid, the shaft passes inside)
        a.cyl((0, 0, 0), WHEEL_R, WHEEL_H, rgb=None, collide=(0.95, 0.9), key="wheel", name="WheelCol")

    # -- joints -------------------------------------------------------------------------------------------------
    base = a.stage.GetPrimAtPath("/Balance/Base")
    for s, nm, wnm in ((1, "FootL", "WheelL"), (-1, "FootR", "WheelR")):
        y = s * FRONT_Y
        a.cur = "/Balance/Base"
        foot = a.stage.GetPrimAtPath(f"/Balance/{nm}")
        wheel = a.stage.GetPrimAtPath(f"/Balance/{wnm}")
        a.prismatic(f"{nm}_Slide", base, foot, axis="Z", lo=-0.052, hi=0.0, stiffness=250000.0,
                    damping=4000.0, target=-FOOT_NOM, p0=(FRONT_X, y, 0.0), p1=(0, 0, 0), max_force=800.0)
        a.revolute(f"{wnm}_Spin", foot, wheel, axis="Z", lo=-1440.0, hi=1440.0, stiffness=2.0,
                   damping=0.06, target=0.0, p0=(0, 0, WHEEL_Z), p1=(0, 0, 0), armature=2.0e-6, max_force=1.5)
    a.save()


def bubble(fname):
    """The green spirit bubble: a separate gravity-free, collision-free visual body, moved by the task file."""
    a = Asset(fname, "Bubble", "The 7 mm green spirit bubble in the AUX220's circular level (visual indicator; "
              "gravity off, no collision). Origin: the bubble's centre.",
              0.0005, com=(0, 0, 0), inertia=(1e-9, 1e-9, 2e-9), no_gravity=True)
    a.cur = "/Bubble"
    a.cyl((0, 0, 0), 0.0035, 0.0012, rgb=(0.30, 0.95, 0.42), collide=None, key="bubble", name="Bubble")
    a.sphere((0, 0, 0.0003), 0.0032, rgb=(0.45, 1.0, 0.50), collide=None, key="bubble", name="BubbleTop")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    balance("balance_aux220.usda")
    bubble("level_bubble.usda")
