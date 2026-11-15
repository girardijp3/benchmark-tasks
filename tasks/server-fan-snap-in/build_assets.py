"""Writes the task's USD assets with pxr (boxes and small convex prisms; no meshes from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset origin is on the table unless its doc says otherwise; metres, kg, Z up."""
import math
from pathlib import Path

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
        self.cur = f"/{name}"  # the prim the next shapes go under (a link, in an articulation)

    def link(self, lname, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), pos=(0, 0, 0)):
        """Start a rigid link under the root (articulation): later shapes go in it."""
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

    def color(self, key, rgb, rough=0.6):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
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

    def _xf(self, g, center, rot_x=0.0):
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        if rot_x:
            g.AddRotateXOp().Set(rot_x)

    def box(self, center, size, rgb=None, collide=None, key="c", name=None):
        """A box (size = x, y, z edge lengths). rgb: visible, coloured; collide=(static, dynamic): a collider."""
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center)
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

    def capsule(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None):
        """A capsule: ``height`` is the straight part between the two hemispheres."""
        self.n += 1
        g = UsdGeom.Capsule.Define(self.stage, f"{self.cur}/{name or 'Capsule'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

    def prism_x(self, poly_yz, x0, x1, rgb, collide=None, key="c", name="Wall"):
        """A convex prism along x: cross-section ``poly_yz`` (list of (y, z), counter-clockwise seen from +x)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        pts = [(x, y, z) for x in (x0, x1) for (y, z) in poly_yz]
        k = len(poly_yz)
        faces, counts = [], []
        for i in range(1, k - 1):  # caps
            faces += [0, i + 1, i]
            faces += [k, k + i, k + i + 1]
            counts += [3, 3]
        for i in range(k):  # sides
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

    def prism_y(self, poly_xz, y0, y1, rgb, collide=None, key="c", name="Prism"):
        """A convex prism along y: cross-section ``poly_xz`` (list of (x, z), counter-clockwise seen from -y)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        pts = [(x, y, z) for y in (y0, y1) for (x, z) in poly_xz]
        k = len(poly_xz)
        faces, counts = [], []
        for i in range(1, k - 1):
            faces += [0, i, i + 1]
            faces += [k, k + i + 1, k + i]
            counts += [3, 3]
        for i in range(k):
            j = (i + 1) % k
            faces += [i, k + j, j, i, k + i, k + j]
            counts += [3, 3]
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        g.CreateFaceVertexCountsAttr(counts)
        g.CreateFaceVertexIndicesAttr(faces)
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        self._finish(g, rgb, collide, key, mesh=True)

    def prism_z(self, poly_xy, z0, z1, rgb, collide=None, key="c", name="Post"):
        """A convex prism along z: cross-section ``poly_xy`` (list of (x, y), counter-clockwise seen from +z)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        pts = [(x, y, z) for z in (z0, z1) for (x, y) in poly_xy]
        k = len(poly_xy)
        faces, counts = [], []
        for i in range(1, k - 1):
            faces += [0, i, i + 1]
            faces += [k, k + i + 1, k + i]
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
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.color("m%02d%02d%02d" % tuple(int(c * 99) for c in rgb), rgb))

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path, self.path.stat().st_size, "bytes")


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))




# -- parts (a 120 x 120 x 25 mm case fan, typical; a tool-less bracket with two spring latches) ---------------------------
FAN_W, FAN_H, BAR = 120 * MM, 25 * MM, 8 * MM
FAN_MASS = 0.130
LEDGE = 6 * MM  # the bracket ring's top (the fan's lower face rests on it)
OPEN_HALF = 63.5 * MM  # half the corner posts' opening (3.5 mm a side around the 120 mm fan)
WALL_T, WALL_TOP, CORNER_ARM = 8 * MM, 46 * MM, 14 * MM  # corner post wall thickness, top above the table, arm length
HINGE_Z, HINGE_Y = 9 * MM, 65.5 * MM  # latch hinge axis (along x) above the table and its distance from the fan's axis
ARM_H, ARM_T, ARM_W = 34.1 * MM, 4 * MM, 20 * MM  # latch arm height above the hinge, thickness (y), width (x)
HOOK_UNDER = 22.6 * MM  # hook underside above the hinge (fan top is at 25 mm above the ledge: 31 mm above the table, 22 above the hinge)
HOOK_REACH = 7.5 * MM  # hook protrusion from the arm's inner face (4 mm over the fan's top edge, 3.5 mm off its side)
K_LATCH = 0.09  # N m/rad: each latch's spring (a drive, preloaded 2 degrees against its stop)
D_LATCH = 0.003  # N m s/rad
K_DEG, D_DEG = K_LATCH * math.pi / 180, D_LATCH * math.pi / 180
LATCH_MASS = 0.012


def bracket():
    a = Asset("fan_bracket.usda", "FanBracket",
              "A tool-less bracket for a 120 mm case fan, one articulation: a fixed base (a 147 mm square ring 6 mm high with four "
              "L-shaped corner posts, 127 mm opening, 40 mm walls with 6 mm lead-in bevels) and two spring latches (34 mm arms on "
              "hinges along x, 7.5 mm hooks 22.6 mm over their hinges) on the middle of the two sides along x. Each latch rests "
              "against its inner stop (angle 0) under a 0.09 N m/rad spring preloaded 2 degrees and swings outward 20 degrees at "
              "most: positive angle opens the -y latch, negative the +y latch. Origin on the table at the fan's axis.",
              mass=1.0, body=False)
    fr = (0.45, 0.35)
    col = (0.16, 0.17, 0.2)
    lat = (0.85, 0.45, 0.1)
    a.link("Base", 1.0, com=(0, 0, 0.01), inertia=(5e-3, 5e-3, 1e-2))
    UsdPhysics.ArticulationRootAPI.Apply(a.stage.GetPrimAtPath("/FanBracket/Base"))  # on the fixed base link: Isaac needs the root on a rigid body
    o, i, h = 73.5 * MM, 50 * MM, LEDGE
    a.box((0, (o + i) / 2, h / 2), (2 * o, o - i, h), rgb=col, collide=fr, key="plastic")
    a.box((0, -(o + i) / 2, h / 2), (2 * o, o - i, h), rgb=col, collide=fr, key="plastic")
    a.box(((o + i) / 2, 0, h / 2), (o - i, 2 * i, h), rgb=col, collide=fr, key="plastic")
    a.box((-(o + i) / 2, 0, h / 2), (o - i, 2 * i, h), rgb=col, collide=fr, key="plastic")
    xi, xo, zt = OPEN_HALF, OPEN_HALF + WALL_T, WALL_TOP
    for sx in (-1, 1):
        for sy in (-1, 1):
            # a wall along y at x = sx*[xi, xo], near the corner y from the opening's edge inward CORNER_ARM
            poly = [(sx * xi, h), (sx * xo, h), (sx * xo, zt), (sx * (xi + 6 * MM), zt), (sx * xi, zt - 6 * MM)]
            if sx < 0:
                poly = poly[::-1]
            y0, y1 = (xi + WALL_T - CORNER_ARM, xi + WALL_T) if sy > 0 else (-(xi + WALL_T), -(xi + WALL_T - CORNER_ARM))
            a.prism_y(poly, y0, y1, col, collide=fr, key="plastic")
            # a wall along x at y = sy*[xi, xo]
            polyx = [(sy * xi, h), (sy * xo, h), (sy * xo, zt), (sy * (xi + 6 * MM), zt), (sy * xi, zt - 6 * MM)]
            if sy < 0:
                polyx = polyx[::-1]
            x0, x1 = (xi - CORNER_ARM + 0 * MM, xi) if sx > 0 else (-xi, -(xi - CORNER_ARM))
            a.prism_x(polyx, x0, x1, col, collide=fr, key="plastic")
    # the latches: link origin on the hinge axis, arm above it
    for name, s in (("LatchP", 1), ("LatchN", -1)):
        a.link(name, LATCH_MASS, com=(0, 0, 0.012), inertia=(2e-6, 2e-6, 1e-6), pos=(0, s * HINGE_Y, HINGE_Z))
        a.box((0, 0, ARM_H / 2), (ARM_W, ARM_T, ARM_H), rgb=lat, collide=fr, key="plastic")
        yi = -s * ARM_T / 2  # the arm's inner face (toward the fan), in the link frame
        poly = [(yi, HOOK_UNDER), (yi - s * HOOK_REACH, HOOK_UNDER), (yi - s * HOOK_REACH, HOOK_UNDER + 1.2 * MM),
                (yi, ARM_H)]  # a 54 degree ramp (a shallower one self-locks against the friction)
        if s < 0:
            poly = poly[::-1]
        a.prism_x(poly, -ARM_W / 2, ARM_W / 2, lat, collide=(0.3, 0.25), key="hook")
    for name, s in (("HingeP", 1), ("HingeN", -1)):
        j = UsdPhysics.RevoluteJoint.Define(a.stage, f"/FanBracket/{name}")
        j.CreateBody0Rel().SetTargets(["/FanBracket/Base"])
        j.CreateBody1Rel().SetTargets([f"/FanBracket/Latch{'P' if s > 0 else 'N'}"])
        j.CreateAxisAttr("X")
        j.CreateLocalPos0Attr(Gf.Vec3f(0, s * HINGE_Y, HINGE_Z))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        # outward is negative for the +y latch (its top swings toward +y by a negative turn about +x), positive for -y
        j.CreateLowerLimitAttr(-20.0 if s > 0 else 0.0)
        j.CreateUpperLimitAttr(0.0 if s > 0 else 20.0)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(K_DEG)
        d.CreateDampingAttr(D_DEG)
        d.CreateTargetPositionAttr(2.0 * s)
        j.GetPrim().SetMetadata("documentation", "Latch hinge: 0 = hook over the fan's top edge (inner stop); the spring pushes 2 deg "
                                "past the stop.")
    a.save()


def fan():
    m = FAN_MASS
    a = Asset("case_fan.usda", "CaseFan",
              "A 120 x 120 x 25 mm case fan, 130 g, shown as a square frame (four 8 mm bars: a 104 mm square opening) with a hub "
              "and spokes painted on. Origin at the middle of its lower face; axis along z.",
              mass=m, com=(0, 0, FAN_H / 2), inertia=box_inertia(m, FAN_W, FAN_W, FAN_H))
    fr = (0.35, 0.3)
    col = (0.12, 0.12, 0.14)
    inner = FAN_W - 2 * BAR
    a.box((0, (FAN_W - BAR) / 2, FAN_H / 2), (FAN_W, BAR, FAN_H), rgb=col, collide=fr, key="plastic")
    a.box((0, -(FAN_W - BAR) / 2, FAN_H / 2), (FAN_W, BAR, FAN_H), rgb=col, collide=fr, key="plastic")
    a.box(((FAN_W - BAR) / 2, 0, FAN_H / 2), (BAR, inner, FAN_H), rgb=col, collide=fr, key="plastic")
    a.box((-(FAN_W - BAR) / 2, 0, FAN_H / 2), (BAR, inner, FAN_H), rgb=col, collide=fr, key="plastic")
    a.cyl((0, 0, FAN_H / 2), 21 * MM, 20 * MM, rgb=(0.05, 0.05, 0.06))
    blade = (0.3, 0.32, 0.36)
    for k in range(7):
        ang = k * 2 * math.pi / 7
        a.n += 1
        g = UsdGeom.Cube.Define(a.stage, f"/CaseFan/Blade{a.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(0.0345 * math.cos(ang), 0.0345 * math.sin(ang), FAN_H / 2))
        g.AddRotateZOp().Set(math.degrees(ang) + 25.0)
        g.AddScaleOp().Set(Gf.Vec3f(0.032, 0.014, 0.003))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(a.color("blade", blade))
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    bracket()
    fan()
