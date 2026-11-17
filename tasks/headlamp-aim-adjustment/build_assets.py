"""Writes this task's USD assets with pxr (boxes, cylinders, prisms, convex meshes and a flat beam patch;
nothing from outside). Run:  local_sim/.venv/bin/python build_assets.py

Frames: metres, kilograms, Z up. The headlamp rig's root is at the world origin; the table top is 0.7475 m.
All product dimensions are the real ones (see README.md / REFERENCE.md); values marked EST are estimates.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001
DEG = math.pi / 180.0
TOP = 0.7475

# -- the real product -------------------------------------------------------------------------------------------------
# Hella 1058 / 1L6 002 395-131 178 mm H4 headlamp insert: 178 mm dia, 88 mm deep, 0.55 kg (Hella outline drawing).
LAMP_R = 89 * MM
LAMP_DEPTH = 88 * MM
LAMP_MASS = 0.55
LAMP_PIVOT = np.array([0.420, -0.130, 0.860])       # gimbal centre at the rear face
LENS_X = LAMP_PIVOT[0] + LAMP_DEPTH                 # 0.620: the lens face
# ECE aim: the horizontal cut-off 36 mm below the lamp centre at 3.00 m (0.69 deg / 1.2 %; i-MiEV service manual)
SCREEN_X = LENS_X + 3.000                           # 3.508: the beam-screen face
TARGET_DROP = 0.036
LAMP_Z = LAMP_PIVOT[2]
SCREEN_W, SCREEN_H = 1.600, 1.200
SCREEN_Z = LAMP_Z
TARGET_H = LAMP_Z - TARGET_DROP                     # 0.824
BAND_V, BAND_H = 0.015, 0.026                       # real limits: +-15 mm vertical, +-26 mm horizontal
# Two aim screws, 8 mm hex socket, 14 mm hex head, 90 mm apart on the top rear of the housing (Hella adjuster screws)
SCREW_X = 0.366
SCREW_Y = {"V": -0.175, "H": -0.085}
SCREW_Z = 0.900
SCREW_TOP = SCREW_Z + 0.005
SCREW_R_ACROSS = 0.007                              # 14 mm across flats
SCREW_SOCKET_AF = 0.0082                            # 8.2 mm socket across flats
SCREW_SOCKET_DEPTH = 0.008
# T-handle 8 mm hex driver: 80 mm T-bar, 135 mm from the bar centre to the bit tip, 0.18 kg
DRIVER_BAR_R = 0.006
DRIVER_BAR_L = 0.080
DRIVER_BIT_AF = 0.0078
DRIVER_TIP_X = SCREW_X + 0.003                      # 0.369: bit tip 8 mm into the socket
DRIVER_GRIP_X = DRIVER_TIP_X - 0.150                # 0.219: the gripper grips the handle centre here
DRIVER_MASS = 0.180
# 12 V supply (bench equipment)
SUPPLY_XY = (0.190, -0.330)
SUPPLY_Z = TOP + 0.030

CHROME = (0.78, 0.79, 0.80)
STEEL = (0.55, 0.56, 0.58)
ZINC = (0.70, 0.71, 0.70)
BLACK = (0.055, 0.058, 0.062)
DARK = (0.12, 0.12, 0.13)
GLASS = (0.72, 0.82, 0.86)
CERAMIC = (0.88, 0.86, 0.80)
COPPER = (0.72, 0.42, 0.22)
BRASS = (0.67, 0.55, 0.28)
BLUE = (0.16, 0.32, 0.72)
RED = (0.72, 0.10, 0.08)
GREEN = (0.10, 0.55, 0.20)
WHITE = (0.92, 0.92, 0.90)
YELLOW = (0.95, 0.78, 0.10)
SCREEN_WHITE = (0.86, 0.87, 0.84)
SCREEN_BLACK = (0.06, 0.06, 0.065)
BEAM = (1.00, 0.92, 0.62)


class Asset:
    def __init__(self, fname, name, doc, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True,
                 fixed_root=False):
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
        self.mats = {}
        self.n = 0
        self.cur = f"/{name}"
        if body:
            UsdPhysics.RigidBodyAPI.Apply(p)
            m = UsdPhysics.MassAPI.Apply(p)
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        if fixed_root:
            UsdPhysics.ArticulationRootAPI.Apply(p)

    # -- looks -------------------------------------------------------------------------------------------------------
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

    # -- prims -------------------------------------------------------------------------------------------------------
    def _xf(self, g, center, rot=(0.0, 0.0, 0.0)):
        g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
        if rot[0]:
            g.AddRotateXOp().Set(float(rot[0]))
        if rot[1]:
            g.AddRotateYOp().Set(float(rot[1]))
        if rot[2]:
            g.AddRotateZOp().Set(float(rot[2]))

    def _finish(self, g, rgb, collide, key="c", name=None, opacity=1.0, rough=0.55, metal=0.0):
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

    def box(self, center, size, rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0), opacity=1.0, rough=0.55):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center, rot)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key, name, opacity, rough)
        return g

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0),
            opacity=1.0, rough=0.55, metal=0.0):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center, rot)
        self._finish(g, rgb, collide, key, name, opacity, rough, metal)
        return g

    def sphere(self, center, radius, rgb=None, collide=None, key="c", name=None, opacity=1.0, rough=0.55):
        self.n += 1
        g = UsdGeom.Sphere.Define(self.stage, f"{self.cur}/{name or 'Sphere'}{self.n}")
        g.CreateRadiusAttr(radius)
        self._xf(g, center)
        self._finish(g, rgb, collide, key, name, opacity, rough)
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

    def prismatic(self, name, b0, b1, axis="Z", lo=-0.002, hi=0.0, stiffness=0.0, damping=0.5, target=0.0,
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

    def fixed_joint(self, name, b0, b1, p0=(0, 0, 0), p1=(0, 0, 0)):
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.FixedJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        return j

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path, self.path.stat().st_size, "bytes")


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


def hex_prism(r, z0, z1, phase=0.0):
    """A closed 6-sided prism (points, triangles) about z; corners at phase + 60k."""
    pts = []
    for z in (z0, z1):
        for k in range(6):
            a = phase + k * math.pi / 3
            pts.append((r * math.cos(a), r * math.sin(a), z))
    tris = []
    for k in range(1, 5):
        tris += [(0, k + 1, k), (6, 6 + k, 6 + k + 1)]
    for k in range(6):
        j = (k + 1) % 6
        tris += [(k, j, 6 + j), (k, 6 + j, 6 + k)]
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[x], np.cross(p[y], p[z])) for x, y, z in tris) / 6.0
    return pts, (tris if vol >= 0 else [(x, z, y) for x, y, z in tris])


# ======================================================================================================================
# headlamp_rig.usda: the stand, the 178 mm H4 headlamp on its two-axis gimbal, and the two real aim screws.
# ======================================================================================================================
def headlamp_rig(fname):
    a = Asset(fname, "HeadlampRig",
              "A Hella 1058 / 1L6 002 395-131 178 mm H4 headlamp (88 mm deep, 0.55 kg) in a bench gimbal on a "
              "cast stand at 0.950 m. The fixed stand is the articulation root; the lamp has a vertical yaw joint "
              "and a horizontal pitch joint on the gimbal (AimYaw, AimPitch). Two 8 mm hex-socket aim screws stand "
              "on the stand at 90 mm centres; each has a visible head (ScrewHeadV/H) and a coaxial shaft "
              "(ScrewV/H) coupled through a 0.9 N m torque fuse. Turning a head drives its shaft and the lamp's "
              "aim drive. Origin: world; table top 0.7475 m.",
              2.4, com=(0.50, 0.0, 0.89), inertia=(0.05, 0.05, 0.03), fixed_root=True)
    rig = a.stage.GetPrimAtPath("/HeadlampRig")
    a.cur = "/HeadlampRig"

    # --- stand: base plate, post, clevis under the rear of the lamp ---------------------------------------
    bx, by = 0.360, -0.090
    a.box((bx, by, TOP + 0.004), (0.100, 0.080, 0.008), DARK, (0.9, 0.8), key="rubber", name="BasePlate")
    a.box((bx + 0.010, by, TOP + 0.054), (0.032, 0.032, 0.100), (0.40, 0.42, 0.46), (0.7, 0.6), key="stand",
          name="Post")
    a.box((bx + 0.010, by, TOP + 0.007), (0.040, 0.040, 0.006), STEEL, None, key="steel", name="PostFoot")
    # a short neck from the post top to the clevis
    a.box((0.400, by, 0.838), (0.060, 0.028, 0.020), (0.40, 0.42, 0.46), (0.7, 0.6), key="stand", name="Neck")
    for sy in (-1, 1):
        a.box((0.407, by + sy * 0.036, 0.860), (0.014, 0.008, 0.056), (0.40, 0.42, 0.46), (0.7, 0.6),
              key="stand", name="ClevisEar")
    a.cyl((0.420, by, 0.860), 0.009, 0.094, axis="Y", rgb=STEEL, collide=None, key="steel", name="GimbalPin")
    a.box((0.400, by, 0.790), (0.030, 0.030, 0.018), (0.40, 0.42, 0.46), (0.7, 0.6), key="stand", name="ClevisFoot")
    # a bubble level on the base (visual; the bench is level)
    a.cyl((0.300, by + 0.040, TOP + 0.011), 0.010, 0.004, axis="Z", rgb=(0.75, 0.82, 0.70), collide=None,
          name="LevelVial")
    # a small label plate on the front of the base
    a.box((bx + 0.075, by, TOP + 0.012), (0.001, 0.060, 0.020), (0.90, 0.90, 0.86), None, name="LabelPlate")
    for i in range(4):
        a.box((bx + 0.0755, by - 0.024 + i * 0.014, TOP + 0.012), (0.0008, 0.008, 0.002), (0.15, 0.15, 0.15), None,
              name="LabelLine")

    # --- 12 V supply and its leads (bench equipment) ---------------------------------------------------------
    a.box((SUPPLY_XY[0], SUPPLY_XY[1], SUPPLY_Z), (0.120, 0.080, 0.060), (0.10, 0.11, 0.13), None,
          name="SupplyBox")
    a.box((SUPPLY_XY[0], SUPPLY_XY[1] + 0.031, SUPPLY_Z + 0.010), (0.070, 0.002, 0.030), (0.90, 0.90, 0.86), None,
          name="SupplyLabel")
    a.box((SUPPLY_XY[0] + 0.035, SUPPLY_XY[1] + 0.031, SUPPLY_Z + 0.020), (0.024, 0.006, 0.012), (0.15, 0.15, 0.15),
          None, name="SupplySwitch")
    a.box((SUPPLY_XY[0] - 0.035, SUPPLY_XY[1] + 0.031, SUPPLY_Z + 0.020), (0.005, 0.004, 0.005), GREEN, None,
          name="SupplyLED")
    # a slack two-core lead: four short capsules from the supply to the lamp's rear cup (visual only)
    lead_pts = [(0.195, -0.300, 0.790), (0.250, -0.255, 0.800), (0.300, -0.205, 0.810),
                (0.350, -0.160, 0.825), (0.390, -0.138, 0.845)]
    for i in range(len(lead_pts) - 1):
        p0, p1 = np.array(lead_pts[i]), np.array(lead_pts[i + 1])
        mid, d = (p0 + p1) / 2, p1 - p0
        ln = float(np.linalg.norm(d))
        yaw = math.degrees(math.atan2(d[1], d[0]))
        pitch = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
        a.cyl(tuple(mid), 0.004, ln, axis="Z", rgb=BLACK, collide=None, name=f"Lead{i}", rot=(90 - pitch, 0, yaw))
    a.cyl((0.394, -0.132, 0.860), 0.012, 0.018, axis="X", rgb=BLACK, collide=None, name="RearBoot")

    # --- the yaw link (small collar on the gimbal pin) -------------------------------------------------------
    yaw = a.link("AimYaw", 0.060, com=(0, 0, 0), inertia=(2e-5, 2e-5, 2e-5), pos=(0, 0, 0))
    a.cur = "/HeadlampRig/AimYaw"
    a.cyl((0, 0, 0), 0.012, 0.024, axis="Y", rgb=STEEL, collide=None, key="steel", name="YawHub")
    a.box((0.006, 0.0, 0.0), (0.014, 0.030, 0.026), DARK, (0.6, 0.5), key="stand", name="YawYoke")

    # --- the lamp: a 178 mm round H4 insert on the pitch link ------------------------------------------------
    pitch = a.link("AimPitch", 0.580, com=(0.044, 0.0, 0.0), inertia=(0.0060, 0.0045, 0.0045), pos=(0, 0, 0))
    a.cur = "/HeadlampRig/AimPitch"
    # chromed reflector bowl and the black housing; the 178 mm lens faces +x
    a.cyl((0.044, 0.0, 0.0), LAMP_R, LAMP_DEPTH, axis="X", rgb=CHROME, collide=(0.5, 0.4), key="lamp",
          name="Housing")
    a.cyl((0.0885, 0.0, 0.0), 0.086, 0.004, axis="X", rgb=GLASS, collide=None, key="glass", name="Lens",
          opacity=0.55, rough=0.12)
    a.cyl((0.0895, 0.0, 0.0), LAMP_R, 0.004, axis="X", rgb=CHROME, collide=None, key="chrome", name="LensRim",
          rough=0.35, metal=0.9)
    a.cyl((-0.020, 0.0, 0.0), 0.030, 0.040, axis="X", rgb=BLACK, collide=(0.5, 0.4), key="lamp", name="RearCup")
    a.cyl((-0.040, 0.0, 0.0), 0.012, 0.016, axis="X", rgb=BLACK, collide=None, key="lamp", name="Socket")
    # a shallow reflector cone inside the lens (visual only)
    a.cyl((0.050, 0.0, 0.0), 0.070, 0.030, axis="X", rgb=CHROME, collide=None, key="chrome", name="Reflector")
    # the two adjuster gearboxes on the rear face, around the screw shafts
    for key, sy in SCREW_Y.items():
        a.cyl((-0.043, sy, 0.040), 0.014, 0.022, axis="X", rgb=BLACK, collide=None, key="lamp",
              name=f"Gearbox{key}")
    # the steady pad on the left side (the second hand braces here)
    a.box((0.038, 0.0885, 0.0), (0.030, 0.004, 0.030), (0.30, 0.31, 0.33), (0.9, 0.8), key="rubber",
          name="SteadyPad")
    a.box((0.038, 0.0860, 0.0), (0.034, 0.002, 0.034), STEEL, None, key="steel", name="SteadyPadPlate")
    # small aim markings: a blue ring for the vertical screw and a yellow ring for the horizontal one
    for key, col in (("V", BLUE), ("H", YELLOW)):
        a.cyl((SCREW_X - 0.012, SCREW_Y[key], SCREW_Z), 0.016, 0.001, axis="X", rgb=col, collide=None, name=f"Mark{key}")

    # --- the aim screws: heads face the robot (-X), shafts run into the rear housing, torque fuse between -----
    for key, sy in SCREW_Y.items():
        hp = a.link(f"ScrewHead{key}", 0.020, com=(0, 0, 0), inertia=(2e-6, 2e-6, 1e-6), pos=(0, 0, 0))
        a.cur = f"/HeadlampRig/ScrewHead{key}"
        # an open 14 mm hex socket head: six outer plates and six socket walls, so the bit enters the socket
        for k in range(6):
            ang = k * math.pi / 3 + math.pi / 6
            a.box((0.000, 6.0 * MM * math.cos(ang), 6.0 * MM * math.sin(ang)),
                  (0.008, 0.002, 0.0048), ZINC, (0.6, 0.5), key="steel", name=f"HeadOuter{k}",
                  rot=(math.degrees(ang), 0, 0))
            a.box((-0.001, 4.7 * MM * math.cos(ang), 4.7 * MM * math.sin(ang)),
                  (0.008, 0.0012, 0.0048), STEEL, (0.4, 0.3), key="steel", name=f"SocketWall{k}",
                  rot=(math.degrees(ang), 0, 0))
        a.cyl((0.006, 0, 0), 0.0105, 0.002, axis="X", rgb=STEEL, collide=None, key="steel", name="HeadFlange")
        a.cyl((0.0035, 0, 0), 0.0048, 0.0010, axis="X", rgb=(0.10, 0.10, 0.11), collide=None, name="SocketFloor")
        a.cyl((0.010, 0, 0), 0.0065, 0.008, axis="X", rgb=DARK, collide=None, name="Coupling")
        sp = a.link(f"ScrewShaft{key}", 0.028, com=(0.030, 0, 0), inertia=(2e-6, 2e-6, 1e-6), pos=(0, 0, 0))
        a.cur = f"/HeadlampRig/ScrewShaft{key}"
        # visual shaft and worm inside the rear housing; no collider (the head carries the socket)
        a.cyl((0.035, 0, 0), 0.0030, 0.060, axis="X", rgb=STEEL, collide=None, key="steel", name="Shaft")
        a.cyl((0.058, 0, 0), 0.0125, 0.008, axis="X", rgb=BRASS, collide=None, key="brass", name="WormGear",
              rough=0.4, metal=0.7)
        for k in range(10):
            ang = 2 * math.pi * k / 10
            a.box((0.054, 0.0115 * math.cos(ang), 0.0115 * math.sin(ang)),
                  (0.012, 0.0020, 0.0020), BRASS, None, key="brass", name="WormTooth",
                  rot=(math.degrees(ang), 0, 0))
        a.cyl((0.005, 0, 0), 0.0038, 0.008, axis="X", rgb=STEEL, collide=None, key="steel", name="ShaftTop")
        # a T-handle 8 mm hex key left in the socket: its shaft is keyed to the head; the robot grips the T-bar
        kp = a.link(f"Key{key}", 0.030, com=(0.0, 0.0, 0.0), inertia=(2e-5, 1e-5, 1e-5), pos=(0, 0, 0))
        a.cur = f"/HeadlampRig/Key{key}"
        pts, tris = hex_prism(DRIVER_BIT_AF / math.cos(math.pi / 6), -0.005, 0.003, phase=0.0)
        a.mesh(pts, tris, (0.60, 0.62, 0.66), (0.5, 0.4), key="steel", name="KeyBit")
        a.cyl((-0.020, 0, 0), 0.0032, 0.030, axis="X", rgb=STEEL, collide=(0.5, 0.4), key="steel", name="KeyShaft")
        a.cyl((-0.035, 0, 0), 0.0030, 0.040, axis="Z", rgb=(0.62, 0.63, 0.66), collide=(0.9, 0.8), key="grip",
              name="TBar")
        for sz in (-1, 1):
            a.cyl((-0.035, 0, sz * 0.021), 0.0035, 0.004, axis="Z", rgb=(0.16, 0.17, 0.19), collide=None,
                  key="grip", name="TBarCap")
        a.cur = "/HeadlampRig"

    # --- joints ---------------------------------------------------------------------------------------------
    yawp = a.stage.GetPrimAtPath("/HeadlampRig/AimYaw")
    pitchp = a.stage.GetPrimAtPath("/HeadlampRig/AimPitch")
    a.revolute("AimYaw", rig, yawp, axis="Z", lo=-6.0, hi=6.0, stiffness=5.2, damping=0.10, target=0.0,
               p0=tuple(LAMP_PIVOT), p1=(0, 0, 0), armature=2.0e-4, max_force=20.0)
    a.revolute("AimPitch", yawp, pitchp, axis="Y", lo=-6.0, hi=6.0, stiffness=5.2, damping=0.10, target=0.0,
               p0=(0, 0, 0), p1=(0, 0, 0), armature=2.0e-4, max_force=20.0)
    for key, sy in SCREW_Y.items():
        hp = a.stage.GetPrimAtPath(f"/HeadlampRig/ScrewHead{key}")
        sp = a.stage.GetPrimAtPath(f"/HeadlampRig/ScrewShaft{key}")
        kp = a.stage.GetPrimAtPath(f"/HeadlampRig/Key{key}")
        a.fixed_joint(f"KeyFix{key}", hp, kp, p0=(0, 0, 0), p1=(0, 0, 0))
        a.revolute(f"ScrewHead{key}", rig, hp, axis="X", lo=-720.0, hi=720.0, stiffness=0.0, damping=0.001,
                   target=0.0, p0=(SCREW_X, sy, SCREW_Z), p1=(0, 0, 0), armature=2.0e-7)
        # the shaft follows the head through a 0.9 N m torque fuse (the drive saturates at the strip torque)
        a.revolute(f"Screw{key}", rig, sp, axis="X", lo=-540.0, hi=540.0, stiffness=0.5, damping=0.02, target=0.0,
                   p0=(SCREW_X, sy, SCREW_Z), p1=(0, 0, 0), armature=2.0e-7, max_force=0.9)
    a.save()


# ======================================================================================================================
# beam_screen.usda: a 1.6 x 1.2 m white aiming screen on two legs, with the ECE target line and the real tolerance band.
# ======================================================================================================================
def beam_screen(fname):
    a = Asset(fname, "BeamScreen",
              "A 1.600 x 1.200 m matte white beam-aiming screen on two legs, its face 3.000 m from the H4 lamp lens. "
              "The red horizontal line is the ECE 1.2 % target 36 mm below the lamp centre (0.914 m); the two green "
              "lines are the +-15 mm vertical limit; the blue vertical line is the lamp centreline. The screen is "
              "fixed to the floor; its face is the x=0 plane of local coordinates (the task spawns it at x=3.620).",
              8.0, com=(0.0, 0.0, 0.0), inertia=(0.6, 0.6, 1.0), body=True)
    root = a.stage.GetPrimAtPath("/BeamScreen")
    a.cur = "/BeamScreen"
    # board: front face local x=0, 20 mm thick, 1.6 m wide, 1.2 m high, centre z=0
    a.box((0.010, 0.0, 0.0), (0.020, SCREEN_W, SCREEN_H), SCREEN_WHITE, (0.6, 0.5), key="screen", name="Board")
    # frame
    for sy in (-1, 1):
        a.box((0.012, sy * (SCREEN_W / 2 + 0.018), 0.0), (0.024, 0.036, SCREEN_H + 0.036), SCREEN_BLACK, None,
              name="SideFrame")
        a.box((0.030, sy * 0.58, -SCREEN_H / 2 - 0.030), (0.024, 0.040, 0.060), SCREEN_BLACK, None, name="Foot")
        a.box((0.030, sy * 0.58, -SCREEN_H / 2 - 0.30), (0.040, 0.040, 0.600), SCREEN_BLACK, (0.7, 0.6),
              key="stand", name="Leg")
        a.box((0.030, sy * 0.58, -0.890), (0.080, 0.080, 0.020), DARK, (0.9, 0.8), key="rubber", name="FootPad")
    a.box((0.012, 0.0, SCREEN_H / 2 + 0.018), (0.024, SCREEN_W + 0.072, 0.036), SCREEN_BLACK, None, name="TopFrame")
    a.box((0.012, 0.0, -SCREEN_H / 2 - 0.018), (0.024, SCREEN_W + 0.072, 0.036), SCREEN_BLACK, None, name="BottomFrame")
    # printed marks, 1 mm in front of the face: red target line, green tolerance lines, blue centreline
    zt = TARGET_H - SCREEN_Z
    a.box((-0.0010, 0.0, zt), (0.0010, 1.240, 0.012), RED, None, name="TargetLine")
    for dz in (-BAND_V, BAND_V):
        a.box((-0.0012, 0.0, zt + dz), (0.0010, 1.240, 0.008), GREEN, None, name="BandLine")
    a.box((-0.0012, 0.0, 0.0), (0.0010, 0.010, 1.000), BLUE, None, name="CentreLine")
    # 100 mm tick marks along the centreline, a 50 mm scale bar at the right
    for k in range(-5, 6):
        a.box((-0.0012, 0.0, k * 0.100), (0.0010, 0.024, 0.006), SCREEN_BLACK, None, name="Tick")
    for k in range(6):
        a.box((-0.0012, 0.66 + k * 0.040, -0.48), (0.0010, 0.020, 0.008), SCREEN_BLACK, None, name="ScaleTick")
    a.box((-0.0012, 0.70, -0.48), (0.0010, 0.002, 0.200), SCREEN_BLACK, None, name="ScaleRule")
    a.save()


# ======================================================================================================================
# beam_patch.usda: a flat, collision-free visual patch for the low-beam light on the screen, moved by the task from
# the lamp's real pose. Local: x=0 on the screen, y horizontal (kink at y=0), z from 0 (bottom) to 0.5 (cut-off).
# ======================================================================================================================
def beam_patch(fname):
    a = Asset(fname, "BeamPatch",
              "A collision-free low-beam light patch on the aiming screen. Local frame: x=0 at the screen face, +y to "
              "the lamp's left (the horizontal cut-off runs to +y), +z up; the horizontal cut-off edge is at z=0.500 "
              "and the 15 deg kick-up rises to the right (-y). The task moves this visual body from the lamp's real "
              "pose; the score never reads it.",
              0.002, com=(0, 0, 0.25), inertia=(1e-5, 1e-5, 1e-5), body=True)
    root = a.stage.GetPrimAtPath("/BeamPatch")
    a.cur = "/BeamPatch"
    # five-vertex beam polygon: bottom y +-0.62 at z=0; horizontal cut-off to +y at z=0.5; kick-up to -y at 15 deg
    ys = -0.62
    ztop = 0.5 + 0.62 * math.tan(15 * DEG)
    pts = [(-0.62, 0.0), (0.62, 0.0), (0.62, 0.5), (0.0, 0.5), (-0.62, ztop)]
    verts = [(0.0, y, z) for (y, z) in pts]
    tris = [(0, 1, 2), (0, 2, 3), (0, 3, 4)]
    a.mesh(verts, tris, BEAM, None, key="beam", name="Beam", opacity=0.50, rough=0.9)
    # bright cut-off bar (horizontal part and the 15 deg kick-up)
    a.box((0.0, 0.31, 0.503), (0.002, 0.620, 0.020), (1.0, 0.97, 0.82), None, name="CutoffBar", opacity=0.95)
    a.box((0.0, -0.31, 0.5 + 0.31 * math.tan(15 * DEG)), (0.002, 0.625, 0.020), (1.0, 0.97, 0.82), None,
          name="KickBar", opacity=0.95, rot=(-15.0, 0, 0))
    # the low-beam hot spot just below the cut-off
    a.cyl((-0.0015, 0.05, 0.405), 0.095, 0.003, axis="X", rgb=(1.0, 0.93, 0.70), collide=None, name="HotSpot",
          opacity=0.65, rough=0.9)
    a.save()


# ======================================================================================================================
# screwdriver.usda: a T-handle 8 mm hex driver, origin at the T-bar centre, +Z along the shaft to the bit.
# ======================================================================================================================
# ======================================================================================================================
# holster.usda: a bench holster that holds the driver upright by its shaft.
# ======================================================================================================================
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    headlamp_rig("headlamp_rig.usda")
    beam_screen("beam_screen.usda")
    beam_patch("beam_patch.usda")


if __name__ == "__main__":
    main()
