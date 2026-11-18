"""Writes this task's USD assets with pxr (boxes, capsules, cylinders; no meshes from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up."""
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

    def capsule(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0)):
        """A capsule: ``height`` is the straight part between the two hemispheres."""
        self.n += 1
        g = UsdGeom.Capsule.Define(self.stage, f"{self.cur}/{name or 'Capsule'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center, *rot)
        self._finish(g, rgb, collide, key)

    def articulation_root(self):
        UsdPhysics.ArticulationRootAPI.Apply(self.root.GetPrim())

    def revolute(self, name, b0, b1, axis="X", lo=-60.0, hi=0.0, stiffness=0.0, damping=0.0, target=0.0, p0=(0, 0, 0), p1=(0, 0, 0), armature=1e-6):
        """A revolute joint about ``axis`` (limits ``lo``..``hi`` degrees) from body ``b0`` to ``b1``, with an angular
        drive (stiffness in N m per degree, damping in N m s per degree, target in degrees)."""
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
        j.GetPrim().AddAppliedSchema("PhysxJointAPI")
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(armature)
        return j

    def sphere(self, center, radius, rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Sphere.Define(self.stage, f"{self.cur}/{name or 'Sphere'}{self.n}")
        g.CreateRadiusAttr(radius)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

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


# ======================================================================================================================
# Locking-carabiner task assets. Metres, kilograms, Z up.
#   carabiner.usda : a ~60 mm aluminium screw-lock (SCREW-LOCK) locking carabiner, ~50 g, an articulation: the body
#                    (free root) and a spring-closed gate hinged at one end. Around the gate, near the hinge, is the
#                    knurled screw-locking sleeve (visual); it is what the robot grips and turns to lock the gate.
#                    In the body's plane y-z the spine is on top (z = 0) along y, the loop and gate are below.
#                    Origin: the spine's middle.
#   anchor.usda    : a fixed anchor eye: a 200 x 200 x 8 mm base plate carrying one upright 8 mm eye-bolt post (40 mm
#                    up) with a ring on top. The carabiner is hooked over the post; the ring stops it sliding off.
# ======================================================================================================================
WIRE_R = 3.0 * MM                      # 6 mm cross-section (compact locking carabiner)
# Loop wire centre-line, mm (y, z): 54 x 54 centre-line => ~60 x 60 mm outline with the 6 mm wire.
LOOP_PTS = [(-17, 0), (17, 0), (27, -10), (27, -44), (17, -54), (-17, -54), (-27, -44), (-27, -10)]
HINGE = (0.0, 17 * MM, -54 * MM)       # the gate's hinge (x, y, z) in the body frame
NOSE = (-17 * MM, -54 * MM)
SLEEVE_LOCAL = (0.0, 8 * MM, -54 * MM)  # knurled sleeve centre on the gate, near the hinge (body frame)
POST_R = 3.0 * MM                       # 6 mm eye-bolt post (matches a real anchor eye's stem)
BOARD_T = 8 * MM
BOARD_SIZE = (0.24, 0.20)
POST_TOP = 0.040                        # post axis 40 mm above the table
POST_DX, POST_DY = 0.035, 0.0           # the post's x, y offset from the board's middle
K_GATE = 0.05                           # N m per radian (spring-closed)
D_GATE = 0.0005                         # N m s per radian
RAD = math.pi / 180.0


def carabiner(fname):
    mb, mg = 0.045, 0.005                # 45 g body + 5 g gate (+ sleeve included in gate) = 50 g total
    a = Asset(fname, "Carabiner", "A ~60 mm aluminium SCREW-LOCK locking carabiner: 6 mm wire, 54 x 54 mm centre-line "
              "(~60 mm outline), 50 g. Body: a spine along y on top (z = 0) and a loop below it; gate: a 34 mm bar hinged "
              "at its lower right end (y +17) that swings inward (up) 60 degrees, spring-closed (0.05 N m/rad) against the "
              "nose at y -17. A knurled screw-locking sleeve surrounds the gate near the hinge. Origin: spine middle; the "
              "loop's plane is y-z.", mb, com=(0, 0, -0.026),
              inertia=(mb / 12 * (0.054**2 + 0.056**2), 1e-6 * 3, 1e-6 * 3))
    a.articulation_root()
    alu = (0.78, 0.18, 0.20)             # red anodised body
    sleeve_col = (0.90, 0.72, 0.16)      # gold/yellow locking sleeve (Petzl's unlocked-warning colour family)
    f = (0.35, 0.30)
    pts = LOOP_PTS
    segs = [(pts[0], pts[1]), (pts[1], pts[2]), (pts[2], pts[3]), (pts[3], pts[4]),
            (pts[6], pts[5]), (pts[7], pts[6]), (pts[0], pts[7])]
    for (y0, z0), (y1, z1) in segs:
        dy, dz = (y1 - y0) * MM, (z1 - z0) * MM
        ang = math.degrees(math.atan2(dz, dy))
        a.capsule((0.0, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM), WIRE_R, max(math.hypot(dy, dz) - 2 * WIRE_R, 1e-4),
                  "Y", alu, f, key="alu", name="Wire", rot=(ang, 0, 0))
    root = a.root.GetPrim()
    gate = a.link("Gate", mg, com=(0, -0.017, 0), inertia=(mg / 12 * 0.034**2, 2e-8, mg / 12 * 0.034**2), pos=HINGE)
    a.capsule((0.0, -0.017, 0.0), WIRE_R, 0.034 - 2 * WIRE_R, "Y", (0.85, 0.85, 0.88), f, key="steel", name="GateBar")
    # The screw-locking sleeve: a knurled collar around the gate bar near the hinge (visual, part of the gate link).
    sy, sz = SLEEVE_LOCAL[1] - HINGE[1], SLEEVE_LOCAL[2] - HINGE[2]  # in the gate link's frame
    a.cyl((0.0, sy - 0.006, sz), 0.0075, 0.014, "Y", sleeve_col, f, key="sleeve", name="Sleeve")
    for k in range(10):
        ang = 2 * math.pi * k / 10
        a.capsule((0.0, sy - 0.006 + 0.005 * math.cos(ang), sz + 0.0075 * math.sin(ang)), 0.0009, 0.010, "Y",
                  (0.55, 0.42, 0.08), None, key="sleeve", name="Knurl")
    a.revolute("Gate", root, gate, "X", lo=-60.0, hi=0.0, stiffness=K_GATE * RAD, damping=D_GATE * RAD, target=0.0,
               p0=HINGE, p1=(0, 0, 0))
    a.save()


def anchor(fname):
    a = Asset(fname, "Anchor", "A fixed anchor eye: a 200 x 200 x 8 mm base plate with one upright 8 mm eye-bolt post, "
              "40 mm above the table, topped by a 24 mm-OD ring (the eye). Origin: table level, the base's middle.", 2.0,
              com=(0, 0, 0.004), inertia=(1e-2, 1e-2, 1e-2))
    steel = (0.55, 0.57, 0.60)
    f = (0.30, 0.25)
    a.box((0, 0, BOARD_T / 2), (BOARD_SIZE[0], BOARD_SIZE[1], BOARD_T), (0.55, 0.45, 0.30), (0.45, 0.40), key="wood", name="Base")
    a.cyl((POST_DX, POST_DY, (BOARD_T + POST_TOP) / 2), POST_R, POST_TOP - BOARD_T, "Z", steel, f, key="steel", name="Post")
    # A ring (the "eye") on top of the post: 12 small capsules on a 12 mm-radius circle in the horizontal plane.
    for k in range(12):
        t0 = 2 * math.pi * k / 12
        t1 = 2 * math.pi * (k + 1) / 12
        r = 0.012
        p0 = (POST_DX + r * math.cos(t0), POST_DY + r * math.sin(t0), POST_TOP)
        p1 = (POST_DX + r * math.cos(t1), POST_DY + r * math.sin(t1), POST_TOP)
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        ang = math.degrees(math.atan2(dy, dx))
        a.capsule(((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, POST_TOP), 0.0022,
                  max(math.hypot(dx, dy) - 0.0044, 1e-4), "Y", steel, f, key="steel", name="Ring", rot=(0, 0, ang))
    a.sphere((POST_DX, POST_DY, POST_TOP), POST_R + 0.001, steel, f, key="steel", name="PostCap")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    carabiner("carabiner.usda")
    anchor("anchor.usda")
