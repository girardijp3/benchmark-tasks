"""Writes the task's USD assets with pxr (boxes, cylinders, closed lathe meshes).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: metres, kilograms, Z up. Every asset was made for this task by its authors, CC-BY 4.0.

Real product: Dyson V6 Cord-free stick vacuum (0.4 L clear bin, red bin-release button).
The motor/body module is shown on the bench with its clear bin on the front; the wand and floor head
are fixed visual props. Bodies and colliders use the published module class sizes.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"


# ---- small mesh helpers (closed, outward-wound) ---------------------------------------------------------------
def _orient(pts, tris):
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[a], np.cross(p[b], p[c])) for a, b, c in tris) / 6.0
    return tris if vol >= 0 else [(a, c, b) for a, b, c in tris]


def lathe(profile, n=48):
    k = len(profile)
    pts = [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z)
           for i in range(n) for (r, z) in profile]
    tris = []
    for i in range(n):
        i2 = (i + 1) % n
        for j in range(k):
            j2 = (j + 1) % k
            a, b, c, d = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            tris += [(a, b, c), (a, c, d)]
    return pts, _orient(pts, tris)


def disc(r, z0, z1, n=48, axis="Z"):
    pts, tris = lathe([(0.0, z0), (r, z0), (r, z1), (0.0, z1)], n)
    if axis == "Y":
        pts = [(x, z, -y) for x, y, z in pts]
    elif axis == "X":
        pts = [(z, y, -x) for x, y, z in pts]
    return pts, tris


# ---- USD authoring ---------------------------------------------------------------------------------------------
class Asset:
    def __init__(self, fname, name, doc, mass=None, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True):
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
        self.n = 0
        self.cur = f"/{name}"
        if mass is not None and body:
            UsdPhysics.RigidBodyAPI.Apply(self.root.GetPrim())
            m = UsdPhysics.MassAPI.Apply(self.root.GetPrim())
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        self.mats = {}

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

    def color(self, key, rgb, rough=0.55, metallic=0.0):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def phys_mat(self, key, static, dynamic, restitution=0.0):
        k = "Phys_" + key
        if k in self.mats:
            return self.mats[k]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{k}")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(restitution)
        self.mats[k] = mat
        return mat

    def _mat(self, rgb):
        return self.color("m%02d%02d%02d" % tuple(int(max(0.0, min(1.0, c)) * 99) for c in rgb), rgb)

    def box(self, center, size, rgb=None, collide=None, key="plastic", name=None):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key)

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="plastic", name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        self._finish(g, rgb, collide, key)

    def mesh(self, pts, tris, rgb=None, collide=None, key="plastic", name=None, guide=False):
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name or 'Mesh'}{self.n}")
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([i for t in tris for i in t])
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr("convexHull")
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.phys_mat(key, *collide),
                                                                UsdShade.Tokens.weakerThanDescendants, "physics")
        if guide:
            g.CreatePurposeAttr("guide")
        elif rgb is not None:
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self._mat(rgb))
        return g

    def _finish(self, g, rgb, collide, key):
        prim = g.GetPrim()
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0008)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.phys_mat(key, *collide),
                                                         UsdShade.Tokens.weakerThanDescendants, "physics")
            if rgb is None:
                g.CreatePurposeAttr("guide")
        if rgb is not None:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self._mat(rgb))

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


# ---- dimensions (metres; mirror the task file) -------------------------------------------------------------------
BODY_L, BODY_W, BODY_H = 0.120, 0.150, 0.195   # motor/body module footprint and height
BIN_L, BIN_W, BIN_H, BIN_WALL = 0.070, 0.070, 0.085, 0.004   # 0.4 L clear bin (0.070^2 x 0.085 = 0.42 L)
BIN_MASS = 0.14                                 # empty plastic bin; 0.06 kg dust is separate debris
FLAP_T = 0.004
EJECT_R, EJECT_LEN, EJECT_TRAVEL = 0.007, 0.014, 0.010
EJECT_K, EJECT_D = 600.0, 1.0                   # 6 N at the 10 mm stop (brief 4-7 N)
WASTE_W, WASTE_H, WASTE_WALL = 0.240, 0.035, 0.006
FR = (0.45, 0.35)                               # plastic-on-plastic
GRIP_FR = (0.9, 0.8)                            # rubberised bin grip ribs / button
VAC_ORIGIN = (0.36, 0.0)                        # world x,y of the body footprint centre
BIN_SEAT = (0.290, 0.0)                         # world x,y of the seated bin centre
EJECT_W = (0.292, 0.0, 0.215)                   # world x,y,z of the button knob at rest


def vac():
    a = Asset("vac.usda", "StickVac",
              "Dyson V6-class cordless stick-vacuum motor/body module on the bench: a 120 x 150 x 195 mm housing with "
              "a front cradle for the 0.4 L clear bin, a red spring bin-release button (600 N/m, 10 mm, 6 N) on the "
              "front top, and a fixed wand/floor-head prop. The body is a free articulation; the button is a prismatic "
              "link. Origin at the middle of the body footprint on the bench.", body=False)
    a.link("Body", 1.90, com=(0.0, 0.0, 0.105), inertia=box_inertia(1.90, BODY_L, BODY_W, BODY_H))
    UsdPhysics.ArticulationRootAPI.Apply(a.stage.GetPrimAtPath("/StickVac/Body"))
    col = (0.82, 0.83, 0.85)
    dark = (0.16, 0.17, 0.19)
    red = (0.85, 0.10, 0.10)
    purple = (0.30, 0.16, 0.48)
    rubber = (0.10, 0.10, 0.11)
    steel = (0.62, 0.64, 0.66)
    # local frame: body origin at world (0.36,0,TOP). world = local + (0.36,0,0).
    for sx in (-1, 1):
        for sy in (-1, 1):
            a.box((sx * 0.045, sy * 0.058, 0.005), (0.024, 0.024, 0.010), rgb=rubber,
                  collide=(1.2, 1.0), key="rubber", name="Foot")
    a.box((0.015, 0.0, 0.1025), (0.090, BODY_W, 0.185), rgb=col, collide=FR, key="plastic", name="Shell")
    # front cradle (local x -0.09..0 == world 0.27..0.36): shelf and two side guides straddle the seated bin
    a.box((-0.045, 0.0, 0.013), (0.090, 0.098, 0.006), rgb=dark, collide=(0.8, 0.6), key="plastic", name="CradleShelf")
    for sy in (-1, 1):
        a.box((-0.045, sy * 0.055, 0.023), (0.090, 0.006, 0.014), rgb=dark, collide=FR,
              key="plastic", name="CradleSide")
    a.box((0.0, 0.0, 0.023), (0.008, 0.116, 0.014), rgb=dark, collide=FR, key="plastic", name="CradleBack")
    # motor / cyclone housing and accents
    a.cyl((0.010, 0.0, 0.155), 0.040, 0.080, axis="Z", rgb=dark, collide=FR, key="plastic", name="Motor")
    a.cyl((0.010, 0.0, 0.200), 0.036, 0.012, axis="Z", rgb=purple, collide=FR, key="plastic", name="MotorCap")
    a.box((0.015, 0.0, 0.185), (0.092, 0.152, 0.012), rgb=purple, name="Accent")
    a.box((-0.030, -0.0755, 0.120), (0.055, 0.002, 0.030), rgb=(0.92, 0.92, 0.90), name="Label")
    # wand and floor head (fixed visual props behind the body, +x)
    a.cyl((0.125, 0.0, 0.140), 0.012, 0.220, axis="X", rgb=steel, collide=FR, key="steel", name="Wand")
    a.box((0.245, 0.0, 0.040), (0.090, 0.150, 0.030), rgb=dark, collide=FR, key="plastic", name="FloorHead")
    a.cyl((0.245, 0.0, 0.060), 0.014, 0.040, axis="Z", rgb=purple, name="HeadNeck")
    # red bin-release button on a bracket forward of the motor, so the 10 mm press has clear travel.
    # The bracket sits above the motor top (0.206); joint origin local (-0.058,0,0.215) == world (0.302,0,TOP+0.215).
    a.box((-0.044, 0.0, 0.211), (0.050, 0.008, 0.006), rgb=dark, collide=FR, key="plastic", name="EjectBracket")
    a.link("Eject", 0.010, com=(-0.010, 0.0, 0.0), inertia=(2e-5, 2e-5, 2e-5), pos=(-0.058, 0.0, 0.215))
    a.cyl((-0.007, 0.0, 0.0), 0.005, 0.014, axis="X", rgb=steel, collide=(0.3, 0.25), key="steel", name="EjectStem")
    # cross-bar knob: its axis is y (perpendicular to the push), so the pinch pads hook it instead of sliding off
    a.cyl((-0.010, 0.0, 0.0), EJECT_R, 0.020, axis="Y", rgb=red, collide=GRIP_FR, key="rubber", name="EjectKnob")
    a.cyl((-0.010, 0.0, 0.0), 0.0088, 0.016, axis="Y", rgb=(0.62, 0.06, 0.06), name="EjectSleeve")
    j = UsdPhysics.PrismaticJoint.Define(a.stage, "/StickVac/EjectPress")
    j.CreateBody0Rel().SetTargets(["/StickVac/Body"])
    j.CreateBody1Rel().SetTargets(["/StickVac/Eject"])
    j.CreateAxisAttr("X")
    j.CreateLocalPos0Attr(Gf.Vec3f(-0.058, 0.0, 0.215))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(0.010)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(EJECT_K)
    d.CreateDampingAttr(EJECT_D)
    d.CreateTargetPositionAttr(0.0)
    j.GetPrim().SetMetadata("documentation", "Red bin-release button: 0 = out (latched), +10 mm = pressed (bin "
                            "released). 600 N/m = 6 N at the stop; the spring returns it to 0 when let go (the click).")
    a.save()


def bin_asset():
    a = Asset("bin.usda", "DustBin",
              "Dyson V6-class 0.4 L clear dust bin: a 70 x 70 x 85 mm clear plastic box (0.42 L) with a sealed top, "
              "two rubber grip ribs at the sides, and a hinged bottom dump flap (0 = shut, -100 deg = open). Origin at "
              "the centre of the bottom face. Free articulation; six loose pieces of dust ride inside.", body=False)
    a.link("Can", BIN_MASS, com=(0.0, 0.0, BIN_H / 2), inertia=box_inertia(BIN_MASS, BIN_L, BIN_W, BIN_H))
    UsdPhysics.ArticulationRootAPI.Apply(a.stage.GetPrimAtPath("/DustBin/Can"))
    clear = (0.55, 0.70, 0.78)
    edge = (0.28, 0.36, 0.42)
    red = (0.85, 0.10, 0.10)
    dark = (0.18, 0.20, 0.22)
    hz = (BIN_H - BIN_WALL) / 2 + BIN_WALL / 2
    hw = (BIN_H - BIN_WALL) / 2
    # 4 thin walls: the interior stays hollow for the debris and the flap opening
    for sy in (-1, 1):
        a.box((0, sy * (BIN_W - BIN_WALL) / 2, hz), (BIN_L, BIN_WALL, BIN_H - BIN_WALL),
              rgb=clear, collide=FR, key="plastic", name="WallY")
    for sx in (-1, 1):
        a.box((sx * (BIN_L - BIN_WALL) / 2, 0, hz), (BIN_WALL, BIN_W - 2 * BIN_WALL, BIN_H - BIN_WALL),
              rgb=clear, collide=FR, key="plastic", name="WallX")
    a.box((0, 0, BIN_H - BIN_WALL / 2), (BIN_L, BIN_W, BIN_WALL), rgb=clear, collide=FR, key="plastic", name="TopCap")
    a.cyl((0, 0, BIN_H + 0.005), 0.018, 0.012, axis="Z", rgb=dark, collide=FR, key="plastic", name="Collar")
    for sy in (-1, 1):
        a.box((0, sy * (BIN_W / 2 + 0.0005), BIN_H / 2), (0.040, 0.003, 0.030), rgb=(0.12, 0.12, 0.13), name="GripPad")
        a.box((0, sy * (BIN_W / 2 + 0.002), BIN_H / 2), (0.028, 0.002, 0.022), rgb=red, name="GripMark")
    a.box((0, -BIN_W / 2 - 0.0005, 0.062), (0.040, 0.002, 0.016), rgb=(0.92, 0.92, 0.90), name="Label")
    # rigid carry bar on the top: axis x (perpendicular to the fingers' closing axis y), so the pads hook it
    for sx in (-1, 1):
        a.box((sx * 0.014, 0.0, BIN_H + 0.009), (0.006, 0.010, 0.018), rgb=dark, collide=FR,
              key="plastic", name="HandlePost")
    a.cyl((0.0, 0.0, BIN_H + 0.018), 0.005, 0.040, axis="X", rgb=dark, collide=GRIP_FR,
          key="rubber", name="CarryBar")
    a.cyl((0.0, 0.0, BIN_H + 0.018), 0.0065, 0.030, axis="X", rgb=red, name="CarryGrip")
    # hinged bottom dump flap: hinge at the +x bottom edge (x=+BIN_L/2); the plate covers x -L/2..+L/2 when shut.
    a.link("Flap", 0.020, com=(-BIN_L / 2, 0.0, FLAP_T / 2),
           inertia=box_inertia(0.020, BIN_L, BIN_W, FLAP_T), pos=(BIN_L / 2, 0.0, 0.0))
    # the flap's underside is flush with the rigid wall bottoms (z=0), so the seated bin rests on its walls, not on
    # a pivoting flap; the flap still swings down and out when the declared dump opens it
    a.box((-BIN_L / 2, 0.0, FLAP_T / 2), (BIN_L, BIN_W, FLAP_T), rgb=clear, collide=FR, key="plastic", name="FlapPlate")
    a.cyl((-BIN_L + 0.006, 0.0, FLAP_T / 2), 0.006, 0.014, axis="Y", rgb=red, collide=FR, key="plastic", name="FlapTab")
    j = UsdPhysics.RevoluteJoint.Define(a.stage, "/DustBin/FlapHinge")
    j.CreateBody0Rel().SetTargets(["/DustBin/Can"])
    j.CreateBody1Rel().SetTargets(["/DustBin/Flap"])
    j.CreateAxisAttr("Y")
    j.CreateLocalPos0Attr(Gf.Vec3f(BIN_L / 2, 0.0, 0.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(-100.0)
    j.CreateUpperLimitAttr(0.0)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(8.0)
    d.CreateDampingAttr(0.2)
    d.CreateTargetPositionAttr(0.0)
    j.GetPrim().SetMetadata("documentation", "Bottom dump flap: 0 = shut; -100 deg = swung down and open. The drive "
                            "holds it shut until the bin is inverted over the waste bin, then opens it.")
    a.save()


def waste_bin():
    a = Asset("waste_bin.usda", "WasteBin",
              "An open-top 180 x 180 x 130 mm household waste bin (fixed on the bench): 6 mm walls, a 10 mm floor and "
              "a rolled rim. The stick-vac's dust bin is held over it and emptied into it.",
              mass=0.30, com=(0, 0, 0.07), inertia=(0.002, 0.002, 0.002))
    plastic = (0.20, 0.21, 0.23)
    rim = (0.42, 0.44, 0.46)
    fr = (0.55, 0.45)
    a.box((0, 0, 0.005), (WASTE_W, WASTE_W, 0.010), rgb=plastic, collide=fr, key="plastic", name="Floor")
    for sy in (-1, 1):
        a.box((0, sy * (WASTE_W - WASTE_WALL) / 2, WASTE_H / 2), (WASTE_W, WASTE_WALL, WASTE_H),
              rgb=plastic, collide=fr, key="plastic", name="Wall")
    for sx in (-1, 1):
        a.box((sx * (WASTE_W - WASTE_WALL) / 2, 0, WASTE_H / 2), (WASTE_WALL, WASTE_W - 2 * WASTE_WALL, WASTE_H),
              rgb=plastic, collide=fr, key="plastic", name="Wall")
    for sy in (-1, 1):
        a.box((0, sy * (WASTE_W / 2 + 0.004), WASTE_H), (WASTE_W + 0.016, 0.008, 0.008),
              rgb=rim, collide=fr, key="plastic", name="Rim")
    for sx in (-1, 1):
        a.box((sx * (WASTE_W / 2 + 0.004), 0, WASTE_H), (0.008, WASTE_W + 0.016, 0.008),
              rgb=rim, collide=fr, key="plastic", name="Rim")
    a.save()


def debris_assets():
    for name, size, mass, colour in (
            ("debris_pebble", (0.012, 0.010, 0.009), 0.010, (0.33, 0.27, 0.20)),
            ("debris_chip", (0.016, 0.004, 0.011), 0.004, (0.62, 0.55, 0.40)),
            ("debris_twig", (0.020, 0.005, 0.004), 0.003, (0.28, 0.20, 0.13))):
        a = Asset(name + ".usda", name.title().replace("_", ""),
                  "A light piece of real vacuum debris (dust grit / dry leaf fragment), a free rigid body.",
                  mass=mass, com=(0, 0, 0), inertia=box_inertia(mass, *size))
        a.box((0, 0, 0), (size[0], size[1], size[2]), rgb=colour, collide=(0.6, 0.5),
              key="debris", name="Debris")
        a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    vac()
    bin_asset()
    waste_bin()
    debris_assets()
