"""Writes this task's USD assets with pxr.  Run:  local_sim/.venv/bin/python build_assets.py

Frames: metres, kilograms, Z up.  The track's root is the centre of its cross-section, at (0.42, -0.10, 0.9975)
world (0.25 m above the table), slot facing DOWN (the real ceiling orientation, lowered to bench height).  The
head's articulation root is fixed at the track bottom (0.42, -0.10, 0.98875).  All product dimensions are the real
ones below; values marked EST are estimates.

  Halo L651P single-circuit H-track: 35 mm wide x 17.5 mm high, 24 mm slot, 120 V 20 A (Cooper Lighting L650/L651).
  Satco TH489 12 W forged H-track LED head: 69.85 mm dia x 76.2 mm, 0.29 kg, 159 mm extension, 120 V.
  H-style twist-lock adapter: 40 mm body, two radial contact tabs, 90 degree cam, 0.3-0.6 N m locking torque.
"""
import math
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets"
sys.path.insert(0, str(HERE.parents[2] / "submission_tools"))
import meshkit  # noqa: E402

MM = 0.001
DEG = math.pi / 180.0
TOP = 0.7475

# product
TRACK_Z = TOP + 0.250
TRACK_BOTTOM = TRACK_Z - 0.00875
SEAT_Z = TRACK_BOTTOM
TRACK_L, TRACK_W, TRACK_H = 1.000, 0.035, 0.0175
HEAD_R, HEAD_H = 0.034925, 0.0762
EXT = 0.159
BODY_H, BODY_R = 0.030, 0.020
STEM_R = 0.010
LEVER_R = 0.090
LEVER_Z = -0.03875
PADDLE_W, PADDLE_T, PADDLE_H = 0.020, 0.014, 0.030
TAB_R, TAB_T, TAB_W, TAB_H, TAB_Z = 0.0120, 0.005, 0.004, 0.004, 0.0045

BLACK = (0.045, 0.047, 0.050)
MATTE_BLACK = (0.060, 0.061, 0.064)
ALU = (0.46, 0.47, 0.49)
DARK = (0.10, 0.105, 0.11)
STEEL = (0.48, 0.49, 0.51)
COPPER = (0.72, 0.42, 0.22)
WHITE = (0.90, 0.90, 0.87)
GLASS = (0.16, 0.18, 0.20)
WARM = (0.82, 0.72, 0.48)
RED = (0.70, 0.09, 0.07)
YELLOW = (0.92, 0.75, 0.10)
RUBBER = (0.12, 0.12, 0.12)


class Asset:
    def __init__(self, fname, name, doc, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True, fixed_root=False):
        self.path = OUT / fname
        self.stage = Usd.Stage.CreateInMemory()
        s = self.stage
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(s, 1.0)
        self.root = UsdGeom.Xform.Define(s, f"/{name}")
        s.SetDefaultPrim(self.root.GetPrim())
        self.root.GetPrim().SetMetadata("documentation", doc)
        self.name = name
        self.mats = {}
        self.n = 0
        self.cur = f"/{name}"
        if body:
            UsdPhysics.RigidBodyAPI.Apply(self.root.GetPrim())
            m = UsdPhysics.MassAPI.Apply(self.root.GetPrim())
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        if fixed_root:
            UsdPhysics.ArticulationRootAPI.Apply(self.root.GetPrim())

    def color(self, key, rgb, rough=0.55, metallic=0.0, opacity=1.0):
        k = f"{key}_{int(rgb[0]*255):02x}{int(rgb[1]*255):02x}{int(rgb[2]*255):02x}_{int(rough*100):02d}_{int(opacity*100):03d}"
        if k in self.mats:
            return self.mats[k]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{k}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{k}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[k] = mat
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

    def _xf(self, g, center, rot=(0.0, 0.0, 0.0)):
        g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
        if rot[0]:
            g.AddRotateXOp().Set(float(rot[0]))
        if rot[1]:
            g.AddRotateYOp().Set(float(rot[1]))
        if rot[2]:
            g.AddRotateZOp().Set(float(rot[2]))

    def _finish(self, g, rgb, collide, key="c", opacity=1.0, rough=0.55, metal=0.0):
        prim = g.GetPrim()
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.001)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
            pm = self.phys_mat(key, *collide)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
            if rgb is None:
                g.CreatePurposeAttr("guide")
        if rgb is not None:
            mat = self.color(key, rgb, rough=rough, metallic=metal, opacity=opacity)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def box(self, center, size, rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0), opacity=1.0, rough=0.55,
            metal=0.0):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center, rot)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key, opacity, rough, metal)
        return g

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0),
            opacity=1.0, rough=0.55, metal=0.0):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center, rot)
        self._finish(g, rgb, collide, key, opacity, rough, metal)
        return g

    def mesh(self, pts, tris, rgb=None, collide=None, key="c", name="Mesh", opacity=1.0, rough=0.55, metal=0.0,
             approximation="convexHull"):
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        P = np.asarray(pts, float)
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in p]) for p in P]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for tr in tris for i in tr])
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*P.min(0)), Gf.Vec3f(*P.max(0))]))
        prim = g.GetPrim()
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(prim)
            if approximation:
                prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token, custom=False).Set(approximation)
            pm = self.phys_mat(key, *collide)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
        if rgb is not None:
            mat = self.color(key, rgb, rough=rough, metallic=metal, opacity=opacity)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        return g

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
        self.n = 0
        return p

    def revolute(self, name, b0, b1, axis="Z", lo=-180.0, hi=180.0, stiffness=0.0, damping=0.0, target=0.0,
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
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(stiffness))
        d.CreateDampingAttr(float(damping))
        d.CreateTargetPositionAttr(float(target))
        if max_force is not None:
            d.CreateMaxForceAttr(float(max_force))
        j.GetPrim().AddAppliedSchema("PhysxJointAPI")
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(float(armature))
        return j

    def prismatic(self, name, b0, b1, axis="Z", lo=-0.025, hi=0.003, stiffness=20000.0, damping=200.0, target=0.0,
                  p0=(0, 0, 0), p1=(0, 0, 0)):
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
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(stiffness))
        d.CreateDampingAttr(float(damping))
        d.CreateTargetPositionAttr(float(target))
        j.GetPrim().AddAppliedSchema("PhysxJointAPI")
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(1e-7)
        return j

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path, self.path.stat().st_size, "bytes")


def lathe_at(profile, center, n=96):
    pts, tris = meshkit.lathe(profile, n)
    c = np.asarray(center, float)
    return np.asarray(pts) + c, tris


def ring_at(r_in, r_out, z0, z1, center, n=96):
    pts, tris = meshkit.lathe([(r_in, z0), (r_out, z0), (r_out, z1), (r_in, z1)], n)
    c = np.asarray(center, float)
    return np.asarray(pts) + c, tris


def lighting_track(fname):
    a = Asset(fname, "LightingTrack",
              "A 1.000 m Halo L651P single-circuit H-track, 35 x 17.5 mm, 24 mm slot, 120 V 20 A, slot DOWN on a "
              "bench pedestal at 0.9975 m.  Fixed.  Local origin at the track cross-section centre; table -0.250 m.",
              3.2, com=(0, 0, -0.120), inertia=(0.32, 0.60, 0.90))
    a.cur = "/LightingTrack"
    for sx in (-1, 1):
        a.box((sx * 0.01675, 0.0, 0.0005), (0.0015, TRACK_L, 0.0135), ALU, (0.35, 0.30), key="alu",
              name="TrackSide", rough=0.38, metal=0.75)
        a.box((sx * 0.01475, 0.0, -0.0075), (0.0055, TRACK_L, 0.0025), ALU, (0.35, 0.30), key="alu",
              name="TrackLip", rough=0.38, metal=0.75)
        a.box((sx * 0.01205, 0.0, -0.00825), (0.0007, TRACK_L, 0.0004), (0.62, 0.63, 0.64), None, name="LipEdge")
    a.box((0, 0, 0.0080), (TRACK_W, TRACK_L, 0.0015), ALU, (0.35, 0.30), key="alu", name="TrackBack", rough=0.38,
          metal=0.75)
    a.box((0, 0, 0.00685), (0.030, TRACK_L - 0.010, 0.001), DARK, None, name="CavityCeiling")
    for sx in (-1, 1):
        a.box((sx * 0.007, 0, 0.00595), (0.004, TRACK_L - 0.020, 0.0010), COPPER, None, name="BusBar", metal=0.85)
    a.box((-0.01755, 0, 0), (0.0006, TRACK_L - 0.004, 0.0020), (0.02, 0.02, 0.022), None, name="PolarityGroove")
    a.box((-0.01745, 0, -0.0022), (0.0005, TRACK_L - 0.004, 0.0010), (0.85, 0.86, 0.84), None, name="PolarityLine")
    for sy in (-1, 1):
        a.box((0, sy * (TRACK_L / 2 - 0.004), 0), (0.036, 0.008, 0.019), MATTE_BLACK, None, name="EndCap")
    # bench pedestal, behind and clear of the hanging head
    px = 0.200
    a.box((px, 0, -0.246), (0.200, 0.200, 0.008), DARK, (0.9, 0.8), key="rubber", name="BasePlate")
    a.box((px, 0, -0.2485), (0.210, 0.210, 0.003), RUBBER, None, name="BasePad")
    a.box((px, 0, -0.1085), (0.080, 0.080, 0.277), (0.18, 0.19, 0.20), (0.6, 0.5), key="stand", name="Pedestal")
    a.box((0.100, 0, 0.0255), (0.225, 0.040, 0.007), (0.18, 0.19, 0.20), (0.6, 0.5), key="stand", name="TopArm")
    a.box((0, 0, 0.0245), (0.020, 0.050, 0.0255), (0.16, 0.17, 0.18), None, name="TrackClamp")
    lab = (-0.0177, -0.230, 0.0)
    a.box(lab, (0.0008, 0.075, 0.020), WHITE, None, name="TrackLabel")
    a.box((-0.0177, 0.360, 0), (0.0008, 0.040, 0.014), YELLOW, None, name="VoltageSticker")
    a.box((0.045, 0.470, -0.195), (0.002, 0.060, 0.085), RED, None, name="LockoutTag")
    a.box((0.0462, 0.470, -0.182), (0.0008, 0.050, 0.010), WHITE, None, name="LockoutStripe")
    a.save()


def track_head(fname):
    # fixed articulation root at the track bottom; two joints: Twist (lever) and Insert (tug load / cam)
    a = Asset(fname, "TrackHead",
              "A Satco TH489 12 W forged H-track LED head (69.85 mm x 76.2 mm, 0.29 kg, 159 mm extension, 120 V) "
              "on a 40 mm H-style twist-lock adapter, hanging below a Halo H-track.  Fixed-root articulation: "
              "Twist is the 90 deg lever/cam, Insert is the axial load path (0 seated, negative pulled out). "
              "The lever is a 20 x 8 x 30 mm paddle at 70 mm radius (EST handling size).  Root at the track bottom.",
              1.0, com=(0, 0, -0.02), inertia=(0.02, 0.02, 0.02), fixed_root=True)
    root = a.stage.GetPrimAtPath("/TrackHead")
    twist = a.link("TwistLink", 0.020, inertia=(1e-5, 1e-5, 1e-5))
    head = a.link("HeadLink", 0.350, com=(0, 0, -0.100), inertia=(0.00090, 0.00090, 0.00023))

    # --- visible/collider geometry on the moving head link (local origin = track bottom) ---------------------------
    a.cur = "/TrackHead/HeadLink"
    for sy in (-1, 1):
        a.box((0, sy * TAB_R, TAB_Z), (TAB_W, TAB_T, TAB_H), (0.55, 0.56, 0.58), (0.35, 0.30), key="steel",
              name="Tab")
        a.box((0, sy * (TAB_R + TAB_T / 2 - 0.0004), TAB_Z + 0.0012), (TAB_W + 0.0004, 0.0008, TAB_H - 0.0016),
              (0.66, 0.67, 0.69), None, name="TabFace")
    a.cyl((0, 0, 0.00125), 0.0110, 0.0025, rgb=None, collide=(0.35, 0.30), key="poly", name="NeckCollider")
    prof = [(0.0, 0), (0.0185, 0), (0.020, -0.0015), (0.020, -0.0285), (0.0185, -0.030), (0.0, -0.030)]
    a.mesh(*lathe_at(prof, (0, 0, 0)), rgb=MATTE_BLACK, collide=None, key="poly", name="AdapterBody", rough=0.52)
    a.cyl((0, 0, -0.015), 0.0198, 0.030, rgb=None, collide=(0.35, 0.30), key="poly", name="AdapterCollider")
    a.mesh(*ring_at(0.020, 0.021, -0.0115, -0.0090, (0, 0, 0)), rgb=(0.08, 0.08, 0.085), collide=None,
           key="poly", name="BodyRib")
    # lever paddle and its radial arm
    a.cyl((0, LEVER_R, LEVER_Z), 0.012, 0.030, rgb=None, collide=(0.55, 0.45), key="grip", name="KnobCollider")
    a.mesh(*lathe_at([(0.0, LEVER_Z - 0.015), (0.012, LEVER_Z - 0.015), (0.012, LEVER_Z + 0.015),
                      (0.0, LEVER_Z + 0.015)], (0.0, LEVER_R, 0.0), 64), rgb=(0.16, 0.17, 0.18), collide=None,
           key="grip", name="LeverKnob", rough=0.6)
    for k in range(24):
        ang = 2 * math.pi * k / 24
        a.box((0.0118 * math.cos(ang), LEVER_R + 0.0118 * math.sin(ang), LEVER_Z),
              (0.0012, 0.0012, 0.026), (0.28, 0.29, 0.30), None, name="Knurl", rot=(0, 0, math.degrees(ang)))
    # the arm and its mount sit BELOW the gripper palm, clear of the parallel jaws
    a.box((0, 0.035, -0.0475), (0.010, 0.010, 0.035), DARK, None, name="LeverPost")
    a.box((0, 0.0575, -0.065), (0.010, 0.055, 0.006), DARK, None, name="LeverArm")
    a.box((0, 0.085, -0.0590), (0.012, 0.010, 0.016), DARK, None, name="PaddleMount")
    a.mesh([(0, LEVER_R - 0.0041, LEVER_Z + 0.012), (-0.006, LEVER_R - 0.0041, LEVER_Z + 0.002),
            (0.006, LEVER_R - 0.0041, LEVER_Z + 0.002)], [(0, 1, 2)], WHITE, None, name="ArrowHead")
    a.box((0, 0.0202, LEVER_Z + 0.008), (0.002, 0.001, 0.006), WHITE, None, name="PolarityArrow")
    # swivel, stem and head
    a.mesh(*lathe_at([(0, -0.030), (0.014, -0.030), (0.014, -0.040), (0, -0.040)], (0, 0, 0)), rgb=DARK,
           collide=None, key="metal", name="SwivelCollar", metal=0.6)
    a.cyl((0, 0, -0.035), STEM_R, 0.0428, rgb=None, collide=(0.4, 0.35), key="metal", name="StemCollider")
    a.cyl((0, 0, -0.035), STEM_R, 0.0428, rgb=(0.08, 0.085, 0.09), collide=None, key="metal", name="Stem")
    hz0, hz1 = -EXT, -EXT + HEAD_H
    prof = [(0, hz1 - 0.002), (0.028, hz1 - 0.002), (0.0330, hz1 - 0.006), (0.0348, hz1 - 0.012),
            (0.0348, hz0 + 0.010), (0.0340, hz0 + 0.004), (0.0330, hz0), (0, hz0)]
    a.mesh(*lathe_at(prof, (0, 0, 0), 96), rgb=(0.055, 0.058, 0.062), collide=None, key="head", name="HeadBody",
           rough=0.36, metal=0.65)
    a.cyl((0, 0, (hz0 + hz1) / 2), 0.0346, HEAD_H, rgb=None, collide=(0.45, 0.38), key="head", name="HeadCollider")
    for k in range(12):
        ang = 2 * math.pi * k / 12
        a.box((0.0355 * math.cos(ang), 0.0355 * math.sin(ang), hz0 + 0.044), (0.004, 0.003, 0.042),
              (0.08, 0.085, 0.09), None, name="Fin", rot=(0, 0, math.degrees(ang)))
    a.mesh(*ring_at(0.0305, 0.0352, hz0, hz0 + 0.008, (0, 0, 0)), rgb=(0.14, 0.15, 0.16), collide=None,
           key="metal", name="Bezel", metal=0.8)
    a.mesh(*lathe_at([(0, hz0 - 0.0015), (0.030, hz0 - 0.0015), (0.030, hz0 + 0.001), (0, hz0 + 0.001)],
                     (0, 0, 0), 96), rgb=GLASS, collide=None, key="glass", name="Lens", rough=0.12)
    a.mesh(*ring_at(0.0347, 0.0350, hz0 + 0.016, hz0 + 0.021, (0, 0, 0)), rgb=WHITE, collide=None,
           name="ModelBand")
    a.box((0.0347, 0, hz0 + 0.058), (0.001, 0.030, 0.014), WHITE, None, name="HeadLabel")

    # --- joints ----------------------------------------------------------------------------------------------------
    a.revolute("Twist", root, twist, axis="Z", lo=-180, hi=180, stiffness=2.6, damping=0.05, target=0.0,
               p0=(0, 0, 0), p1=(0, 0, 0), armature=2e-4, max_force=0.9)
    a.prismatic("Insert", twist, head, axis="Z", lo=-0.025, hi=0.003, stiffness=20000.0, damping=200.0,
                target=0.0, p0=(0, 0, 0), p1=(0, 0, 0))
    a.save()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    lighting_track("lighting_track.usda")
    track_head("track_head.usda")


if __name__ == "__main__":
    main()
