"""Writes this task's USD assets with pxr (boxes, cylinders and prisms; no meshes from outside).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

Frames: metres, kilograms, Z up.  The toolbox asset's origin is the middle of the base's underside on the bench; the
root link ``Base`` is free on its four rubber feet.  ``ArmL``/``ArmR`` are single steel cantilever arms on revolute
joints at the base's side rims; each carries ``WingL``/``WingR`` on a second revolute joint that keeps the tray level
(the counter-rotation stands in for the second parallel arm of the real parallelogram).  ``Top`` is the lid on a
vertical prismatic joint, with the two draw latches ``LatchL``/``LatchR`` on the base's front corner posts, their
hooks under the lid's keepers.

Real product: STANLEY 1-94-738 18 in. metal cantilever tool box, 460 x 208 x 208 mm, 4.8 kg, five trays, two metal
latches, two carry handles, 35 kg load; published on the Stanley product page.  The latches, tray sizes, arm lengths,
masses beyond the published total and the handle height are estimates (EST); every number is listed in REFERENCE.md.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001

# -- published size and the model's mass split -------------------------------------------------------------------------
Y_LEN, X_DEP, Z_BOX = 0.460, 0.208, 0.208  # 460 x 208 x 208 mm (Stanley product page)
HALF_Y, HALF_X = Y_LEN / 2, X_DEP / 2
BASE_TOP = 0.105  # the black base's top rim (EST from the product photo: the black bottom third)
LID_BOT = 0.195  # the lid's underside (208 mm overall, 13 mm lid)
M_BASE, M_ARM, M_WING, M_TOP, M_LATCH = 3.20, 0.15, 0.40, 0.40, 0.02
M_HAMMER, M_WRENCH, M_SCREWDRIVER = 0.50, 0.35, 0.15

# -- the arm / tray geometry -------------------------------------------------------------------------------------------
ARM_PIVOT_Y, ARM_PIVOT_Z = 0.225, 0.107
ARM_LEN, ARM_DROP = 0.149, 0.007  # rest arm: inward 149 mm and 7 mm down
WING_REST_Y, WING_REST_Z = ARM_PIVOT_Y - ARM_LEN, ARM_PIVOT_Z - ARM_DROP  # 0.076, 0.094
TRAY_LEN, TRAY_DEP, TRAY_H, TRAY_WALL = 0.160, 0.150, 0.030, 0.002
LATCH_X, LATCH_Y, LATCH_Z = -0.114, 0.160, 0.195
LATCH_PLATE_T, LATCH_PLATE_W, LATCH_LEN = 0.004, 0.012, 0.035

# colours: Stanley yellow and black, clear zinc-plated steel, rubber feet
YELLOW = (0.96, 0.78, 0.10)
BLACK = (0.05, 0.05, 0.055)
STEEL = (0.62, 0.64, 0.66)
DARKSTEEL = (0.35, 0.36, 0.37)
RUBBER = (0.02, 0.02, 0.02)
WOOD = (0.45, 0.26, 0.10)


class Asset:
    def __init__(self, fname, name, doc, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True):
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

    def color(self, key, rgb, rough=0.6, metal=0.0):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
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

    def _xf(self, g, center, rot=(0.0, 0.0, 0.0)):
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        if rot[0]:
            g.AddRotateXOp().Set(rot[0])
        if rot[1]:
            g.AddRotateYOp().Set(rot[1])
        if rot[2]:
            g.AddRotateZOp().Set(rot[2])

    def box(self, center, size, rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0)):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center, rot)
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

    def _finish(self, g, rgb, collide, key):
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
            if key == "steel":
                mat = self.color("steel", rgb, rough=0.30, metal=0.80)
            elif key == "darksteel":
                mat = self.color("darksteel", rgb, rough=0.45, metal=0.55)
            elif key == "rubber":
                mat = self.color("rubber", rgb, rough=0.95)
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


def tray(a, y0, y1, x0, x1, z0, z1, rgb=YELLOW, name="Tray"):
    """An open steel tray shell: floor and four 2 mm walls, colliders on all five pieces.  ``y0``/``y1`` are its
    outer and inner ends; the floor's top is ``z0``."""
    w = TRAY_WALL
    fl = 0.002
    a.box(((x0 + x1) / 2, (y0 + y1) / 2, z0 + fl / 2), ((x1 - x0), abs(y1 - y0), fl), rgb, (0.55, 0.45),
          key="tray", name=name + "Floor")
    a.box(((x0 + x1) / 2, y0 + (w / 2) * (1 if y1 > y0 else -1), z0 + (z1 - z0) / 2),
          (x1 - x0, w, z1 - z0), rgb, (0.55, 0.45), key="tray", name=name + "Outer")
    a.box(((x0 + x1) / 2, y1 - (w / 2) * (1 if y1 > y0 else -1), z0 + (z1 - z0) / 2),
          (x1 - x0, w, z1 - z0), rgb, (0.55, 0.45), key="tray", name=name + "Inner")
    for xx, lbl in ((x0, "Front"), (x1, "Back")):
        a.box((xx + (w / 2 if xx == x0 else -w / 2), (y0 + y1) / 2, z0 + (z1 - z0) / 2),
              (w, abs(y1 - y0), z1 - z0), rgb, (0.55, 0.45), key="tray", name=name + lbl)


def toolbox(fname):
    inertia = box_inertia(M_BASE, X_DEP, Y_LEN, BASE_TOP)
    a = Asset(fname, "Toolbox",
              "STANLEY 1-94-738 18 in. metal cantilever tool box: 460 x 208 x 208 mm, 4.8 kg, five trays, two metal "
              "latches, two carry handles.  Free articulation root Base on four rubber feet; ArmL/ArmR on revolute "
              "joints at the base side rims, WingL/WingR level trays on the arm tips, Top the lid on a vertical "
              "prismatic joint, LatchL/LatchR draw latches on the front corner posts.  Origin: middle of the base's "
              "underside on the bench.", M_BASE, com=(0.0, 0.0, 0.048), inertia=inertia)
    a.articulation_root()
    base = a.stage.GetPrimAtPath("/Toolbox/Base")
    a.cur = "/Toolbox/Base"

    # -- four rubber feet, then the black base tray ----------------------------------------------------------------
    for sx in (-0.080, 0.080):
        for sy in (-0.200, 0.200):
            a.box((sx, sy, 0.006), (0.030, 0.040, 0.012), RUBBER, (0.95, 0.90), key="rubber", name="Foot")
    a.box((0, 0, 0.014), (X_DEP - 2 * 0.004, Y_LEN - 2 * 0.004, 0.004), BLACK, (0.55, 0.45), key="base", name="BaseFloor")
    for sx, lbl in ((-HALF_X + 0.002, "Front"), (HALF_X - 0.002, "Back")):
        a.box((sx, 0, (0.016 + BASE_TOP) / 2), (0.004, Y_LEN, BASE_TOP - 0.016), BLACK, (0.55, 0.45),
              key="base", name="Base" + lbl)
    for sy, lbl in ((-HALF_Y + 0.002, "Right"), (HALF_Y - 0.002, "Left")):
        a.box((0, sy, (0.016 + BASE_TOP) / 2), (X_DEP - 0.008, 0.004, BASE_TOP - 0.016), BLACK, (0.55, 0.45),
              key="base", name="BaseSide" + lbl)
    # a rolled top rim and the black name plate with the yellow Stanley label
    # the rolled rim is a ring of four strips outside the tray cavities (a solid plate would fill the box)
    for sx in (-HALF_X + 0.003, HALF_X - 0.003):
        a.box((sx, 0, BASE_TOP - 0.003), (0.006, Y_LEN + 0.002, 0.006), BLACK, (0.55, 0.45), key="base", name="RimX")
    for sy in (-HALF_Y + 0.003, HALF_Y - 0.003):
        a.box((0, sy, BASE_TOP - 0.003), (X_DEP + 0.002, 0.006, 0.006), BLACK, (0.55, 0.45), key="base", name="RimY")
    a.box((-HALF_X - 0.0005, 0, 0.050), (0.0015, 0.062, 0.027), (0.85, 0.68, 0.10), None, name="LabelPlate")
    a.box((-HALF_X - 0.0013, 0, 0.050), (0.0008, 0.052, 0.019), BLACK, None, name="LabelText")

    # -- two fixed trays nested inside the base (trays 1 and 2 of the five) ----------------------------------------
    tray(a, -0.210, 0.210, -0.090, 0.090, 0.016, 0.065, name="FixedLow")
    tray(a, -0.210, 0.210, -0.090, 0.090, 0.065, 0.090, name="FixedHigh")

    # (the latches hang on their own hinge knuckles on the front face; no static post crosses the lid's skirt)

    # -- the two carry bails folded down against the ends (visual only) --------------------------------------------
    for sy in (-1, 1):
        yy = sy * (HALF_Y + 0.006)
        a.cyl((0, yy, 0.060), 0.004, 0.065, axis="X", rgb=BLACK, collide=None, name="BailGrip")
        for xx in (-0.040, 0.040):
            a.cyl((xx, yy, 0.105), 0.004, 0.090, axis="Z", rgb=BLACK, collide=None, name="BailArm")
        a.box((0, yy, 0.148), (0.100, 0.010, 0.008), BLACK, None, name="BailTop")

    # -- arms: two zinc straps per arm, authored at the closed pose (inward and 13 mm down) -------------------------
    arm_mid_y = -ARM_LEN / 2
    arm_mid_z = -ARM_DROP / 2
    arm_angle = math.degrees(math.atan2(ARM_DROP, ARM_LEN))
    for side, s in (("L", 1), ("R", -1)):
        a.link("Arm" + side, M_ARM, com=(0, s * arm_mid_y, arm_mid_z),
               inertia=box_inertia(M_ARM, 0.120, ARM_LEN, 0.004), pos=(0, s * ARM_PIVOT_Y, ARM_PIVOT_Z))
        a.cur = f"/Toolbox/Arm{side}"
        for xx in (-0.060, 0.060):
            a.box((xx, s * arm_mid_y, arm_mid_z), (0.012, ARM_LEN, 0.003), STEEL, (0.55, 0.45),
                  key="steel", name="ArmStrap", rot=(s * arm_angle, 0, 0))
        a.cyl((0, 0, 0), 0.008, 0.016, axis="X", rgb=DARKSTEEL, collide=None, name="ArmPivot")
        a.cyl((0, s * ARM_LEN, -ARM_DROP), 0.008, 0.016, axis="X", rgb=DARKSTEEL, collide=None, name="ArmTip")

    # -- the moving trays (trays 3 and 4): left low, right high on two risers ----------------------------------------
    a.link("WingL", M_WING, com=(0, -TRAY_LEN / 2, 0.015),
           inertia=box_inertia(M_WING, TRAY_DEP, TRAY_LEN, TRAY_H), pos=(0, WING_REST_Y, WING_REST_Z))
    a.cur = "/Toolbox/WingL"
    tray(a, 0.0, -TRAY_LEN, -TRAY_DEP / 2, TRAY_DEP / 2, 0.0, TRAY_H, name="WingLTray")
    a.box((0, -0.002, 0.0015), (0.170, 0.004, 0.003), STEEL, None, name="WingLRim")

    a.link("WingR", M_WING, com=(0, TRAY_LEN / 2, 0.055),
           inertia=box_inertia(M_WING, TRAY_DEP, TRAY_LEN, TRAY_H), pos=(0, -WING_REST_Y, WING_REST_Z))
    a.cur = "/Toolbox/WingR"
    tray(a, 0.0, TRAY_LEN, -TRAY_DEP / 2, TRAY_DEP / 2, 0.040, 0.040 + TRAY_H, name="WingRTray")
    for xx in (-0.090, 0.090):
        a.box((xx, 0, 0.020), (0.012, 0.008, 0.040), STEEL, (0.55, 0.45), key="steel", name="WingRRiser")
    a.box((0, 0, 0.0015), (0.192, 0.010, 0.003), STEEL, None, name="WingRBase")

    # -- the lid: top plate, 25 mm skirt and the wire tray handle --------------------------------------------------
    a.link("Top", M_TOP, com=(0, 0, -0.004), inertia=box_inertia(M_TOP, X_DEP, Y_LEN, 0.038),
           pos=(0, 0, LID_BOT))
    a.cur = "/Toolbox/Top"
    a.box((0, 0, 0.0065), (X_DEP, Y_LEN, 0.013), YELLOW, (0.55, 0.45), key="tray", name="LidTop")
    for sx, lbl in ((-HALF_X + 0.0015, "Front"), (HALF_X - 0.0015, "Back")):
        a.box((sx, 0, -0.004), (0.003, Y_LEN, 0.008), YELLOW, (0.55, 0.45), key="tray", name="LidSkirt" + lbl)
    for sy, lbl in ((-HALF_Y + 0.0015, "Right"), (HALF_Y - 0.0015, "Left")):
        a.box((0, sy, -0.004), (X_DEP - 0.006, 0.003, 0.008), YELLOW, (0.55, 0.45), key="tray", name="LidSkirt" + lbl)
    # the wire tray handle: a U loop along x on the lid's front half, grip bar 60 mm at z +21 mm
    for xx in (-0.090, -0.030):
        a.cyl((xx, 0, 0.017), 0.004, 0.008, axis="Z", rgb=BLACK, collide=None, name="HandlePost")
    a.cyl((-0.060, 0, 0.021), 0.004, 0.060, axis="X", rgb=BLACK, collide=(0.60, 0.50), key="handle", name="GripBar")
    a.box((-0.060, 0, 0.016), (0.064, 0.010, 0.002), DARKSTEEL, None, name="HandlePlate")
    # the two latch keepers on the lid's front skirt, and the hinge/pad detail
    for sy in (-LATCH_Y, LATCH_Y):
        a.box((-HALF_X + 0.001, sy, -0.0285), (0.014, 0.012, 0.005), STEEL, (0.55, 0.45), key="steel", name="LidKeeper")
        a.box((-HALF_X + 0.004, sy, -0.018), (0.006, 0.010, 0.020), STEEL, (0.55, 0.45), key="steel", name="KeeperBracket")
    a.box((-0.020, 0, 0.0005), (0.030, 0.120, 0.001), DARKSTEEL, None, name="LidHingePlate")

    # -- the two draw latches: plate on a revolute joint at the top, hook at the bottom, grip knob outward --------
    for side, s in (("L", 1), ("R", -1)):
        a.link("Latch" + side, M_LATCH, com=(-0.004, 0, -0.018), inertia=(2.0e-5, 1.0e-5, 2.0e-5),
               pos=(LATCH_X, s * LATCH_Y, LATCH_Z))
        a.cur = f"/Toolbox/Latch{side}"
        a.box((0, 0, -LATCH_LEN / 2), (LATCH_PLATE_T, LATCH_PLATE_W, LATCH_LEN), STEEL, (0.55, 0.45),
              key="steel", name="LatchPlate")
        a.box((-0.001, 0, -LATCH_LEN + 0.003), (0.010, LATCH_PLATE_W, 0.006), STEEL, (0.55, 0.45),
              key="steel", name="LatchHook")
        a.cyl((-0.008, 0, -LATCH_LEN + 0.005), 0.005, 0.016, axis="Y", rgb=DARKSTEEL, collide=(0.70, 0.60),
              key="darksteel", name="LatchKnob")
        a.box((-0.004, 0, -LATCH_LEN + 0.005), (0.006, 0.008, 0.010), STEEL, None, name="LatchStem")
        a.cyl((0, 0, 0), 0.006, 0.016, axis="Y", rgb=DARKSTEEL, collide=None, name="LatchHinge")

    # -- joints ---------------------------------------------------------------------------------------------------
    base = a.stage.GetPrimAtPath("/Toolbox/Base")
    top = a.stage.GetPrimAtPath("/Toolbox/Top")
    a.prismatic("Top_Lift", base, top, axis="Z", lo=0.0, hi=0.115, stiffness=5000.0, damping=100.0,
                target=0.0, p0=(0, 0, LID_BOT), p1=(0, 0, 0), max_force=300.0)
    for side, s, lo, hi, k in (("L", 1, -135.0, 5.0, 800.0), ("R", -1, -5.0, 145.0, 800.0)):
        arm = a.stage.GetPrimAtPath(f"/Toolbox/Arm{side}")
        wing = a.stage.GetPrimAtPath(f"/Toolbox/Wing{side}")
        a.revolute(f"Arm{side}_Swing", base, arm, axis="X", lo=lo, hi=hi, stiffness=k, damping=80.0, target=0.0,
                   p0=(0, s * ARM_PIVOT_Y, ARM_PIVOT_Z), p1=(0, 0, 0), armature=2.0e-3, max_force=120.0)
        a.revolute(f"Wing{side}_Level", arm, wing, axis="X", lo=-140.0, hi=140.0, stiffness=k, damping=80.0,
                   target=0.0, p0=(0, -s * ARM_LEN, -ARM_DROP), p1=(0, 0, 0), armature=2.0e-3, max_force=120.0)
        latch = a.stage.GetPrimAtPath(f"/Toolbox/Latch{side}")
        a.revolute(f"Latch{side}_Swing", base, latch, axis="Y", lo=-30.0, hi=5.0, stiffness=0.25, damping=0.02,
                   target=0.0, p0=(LATCH_X, s * LATCH_Y, LATCH_Z), p1=(0, 0, 0), armature=1.0e-5, max_force=2.0)
    a.save()


def hammer(fname):
    a = Asset(fname, "Hammer", "A 300 mm claw hammer: 220 mm hickory handle, 90 mm steel head.  Rests in the "
              "toolbox's upper fixed tray.  Origin: the centre of the handle.", M_HAMMER, com=(0, 0.040, 0.0),
              inertia=box_inertia(M_HAMMER, 0.09, 0.30, 0.03))
    a.cur = "/Hammer"
    a.cyl((0, -0.005, 0), 0.007, 0.220, axis="Y", rgb=WOOD, collide=(0.55, 0.45), key="wood", name="Handle")
    a.box((0, 0.105, 0), (0.030, 0.090, 0.022), DARKSTEEL, (0.55, 0.45), key="steel", name="Head")
    a.box((0, 0.145, -0.006), (0.026, 0.030, 0.012), DARKSTEEL, None, name="Claw")
    a.cyl((0, -0.110, 0), 0.008, 0.010, axis="Y", rgb=BLACK, collide=None, name="Grip")
    a.save()


def wrench(fname):
    a = Asset(fname, "Wrench", "A 150 mm adjustable wrench: 120 mm steel handle, 30 mm jaw head.  Rests in the "
              "left moving tray.  Origin: the centre of the handle bar.", M_WRENCH, com=(0, 0.010, 0.0),
              inertia=box_inertia(M_WRENCH, 0.030, 0.150, 0.012))
    a.cur = "/Wrench"
    a.box((0, -0.010, 0), (0.018, 0.120, 0.008), STEEL, (0.55, 0.45), key="steel", name="Handle")
    a.box((0, 0.062, 0), (0.030, 0.040, 0.012), DARKSTEEL, (0.55, 0.45), key="steel", name="Head")
    a.box((0, 0.084, 0), (0.008, 0.014, 0.010), STEEL, None, name="Jaw")
    a.box((0.012, 0.052, 0), (0.006, 0.018, 0.008), DARKSTEEL, None, name="Worm")
    a.save()


def screwdriver(fname):
    a = Asset(fname, "Screwdriver", "A 120 mm screwdriver: 60 mm acetate handle, 60 mm steel blade.  Rests in the "
              "right moving tray.  Origin: the centre of the handle.", M_SCREWDRIVER, com=(0, 0.020, 0.0),
              inertia=box_inertia(M_SCREWDRIVER, 0.024, 0.120, 0.024))
    a.cur = "/Screwdriver"
    a.cyl((0, -0.030, 0), 0.012, 0.060, axis="Y", rgb=(0.75, 0.10, 0.05), collide=(0.55, 0.45), key="handle",
          name="Handle")
    a.cyl((0, 0.035, 0), 0.003, 0.070, axis="Y", rgb=STEEL, collide=(0.55, 0.45), key="steel", name="Blade")
    a.box((0, 0.072, 0), (0.006, 0.006, 0.004), STEEL, None, name="Tip")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    toolbox("toolbox.usda")
    hammer("hammer.usda")
    wrench("wrench.usda")
    screwdriver("screwdriver.usda")
