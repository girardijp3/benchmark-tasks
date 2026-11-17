"""Writes this task's USD assets with pxr (boxes only; no outside meshes).

Run:  ../../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A short (1U-style) rack switch on a bench riser with two SFP/DAC port cages, a cable-management comb over the
near port, and a staging cradle.  The 40 mm DAC plug is its own gravity-free free body so the task file can pose
it as the grippers carry, align, insert and tug it.  Real product class: a 40 mm DAC/SFP-style copper plug,
13.5 x 8.6 mm body, 12 mm strain-relief boot, in a 14.5 x 9.5 mm switch cage with a spring latch
(4-7 N insertion force).  See REFERENCE.md for the measured/estimated split.
"""
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

# -- colours -----------------------------------------------------------------------------------------------------------
STEEL = (0.70, 0.72, 0.75)
DARK = (0.10, 0.11, 0.12)
PCB = (0.10, 0.32, 0.20)
GOLD = (0.85, 0.68, 0.20)
RUBBER = (0.06, 0.06, 0.07)
GREEN = (0.12, 0.62, 0.24)
AMBER = (0.95, 0.60, 0.10)
ALU = (0.58, 0.61, 0.65)


class Asset:
    def __init__(self, fname, name, doc, mass=None, kinematic=False, no_gravity=False):
        self.path = OUT / fname
        s = self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(s, 1.0)
        self.name = name
        root = UsdGeom.Xform.Define(s, f"/{name}")
        s.SetDefaultPrim(root.GetPrim())
        self.root = root.GetPrim()
        self.root.SetMetadata("documentation", doc)
        UsdPhysics.RigidBodyAPI.Apply(self.root)
        if kinematic:
            self.root.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        if no_gravity:
            self.root.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        if mass is not None:
            m = UsdPhysics.MassAPI.Apply(self.root)
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-6, 1e-6, 1e-6))
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        self.mats = {}
        self.n = 0

    def color(self, key, rgb, metal=0.0, rough=0.6):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def _finish(self, g, rgb, collide, metal, rough):
        prim = g.GetPrim()
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0005)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
        if rgb is not None:
            mat = self.color("m%02d%02d%02d_%d%d" % (*tuple(int(c * 99) for c in rgb), int(metal * 9), int(rough * 9)),
                             rgb, metal, rough)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def box(self, center, size, rgb, collide=False, metal=0.0, rough=0.6, name=None):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"/{self.name}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, metal, rough)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


# switch local frame: origin at switch centre, x forward (0 at centre), y left, z above the table.
# The task places the switch origin at world (0.35, 0, TOP).  Front face is at local x -0.040 = world 0.310.
FRONT = -0.040           # world x of the port mouth plane: 0.350 - 0.040 = 0.310
PORT_Z = 0.100           # local z of the port centre (world TOP + 0.100)
TARGET_Y = 0.015         # local/world y of the target (near) port
DECOY_Y = -0.015         # the other port: a wrong insertion caps the score
PORT_W = 0.0145
PORT_H = 0.0095
CAGE_DEPTH = 0.045
COMB_X = FRONT + 0.002   # comb teeth at the mouth / boot flange plane (world 0.312)


def build_switch():
    """The 1U-style switch, its bench riser, two port cages, the comb over the target and the plug cradle."""
    a = Asset("switch.usda", "Switch",
              "short rack switch on a bench riser, two DAC/SFP port cages, cable comb and a plug staging cradle",
              kinematic=True)

    # bench riser / base plate (local z 0 .. 0.060)
    a.box((0, 0, 0.004), (0.140, 0.170, 0.008), ALU, metal=0.4, name="BasePlate")
    for y in (-0.070, 0.070):
        a.box((0.030, y, 0.032), (0.010, 0.010, 0.060), STEEL, metal=0.6, name="RiserPost")
    a.box((0, 0, 0.063), (0.080, 0.150, 0.006), STEEL, metal=0.6, name="RiserTop")

    # switch chassis: world x 0.310 .. 0.390 (local -0.040 .. +0.040), y +-0.060, z 0.066 .. 0.134
    a.box((0, 0, 0.100), (0.080, 0.120, 0.068), STEEL, metal=0.7, rough=0.35, name="Chassis")
    a.box((FRONT + 0.001, 0, 0.100), (0.002, 0.120, 0.068), DARK, metal=0.3, name="FacePlate")
    # green board visible on top, status LEDs
    a.box((0, 0, 0.135), (0.070, 0.110, 0.002), PCB, name="Board")
    for i, y in enumerate((-0.040, -0.026, 0.040, 0.052)):
        a.box((FRONT + 0.0005, y, 0.124), (0.001, 0.006, 0.003), GREEN if i % 2 else AMBER,
              name=f"Led{i}")

    for label, py in (("Target", TARGET_Y), ("Decoy", DECOY_Y)):
        # dark socket interior (open at the mouth) and the four steel cage walls
        a.box((FRONT + CAGE_DEPTH / 2, py, PORT_Z), (CAGE_DEPTH, PORT_W, PORT_H), DARK,
              name=label + "Socket")
        a.box((FRONT + CAGE_DEPTH / 2, py - PORT_W / 2 - 0.0008, PORT_Z),
              (CAGE_DEPTH, 0.0016, PORT_H + 0.002), STEEL, metal=0.7, name=label + "CageLow")
        a.box((FRONT + CAGE_DEPTH / 2, py + PORT_W / 2 + 0.0008, PORT_Z),
              (CAGE_DEPTH, 0.0016, PORT_H + 0.002), STEEL, metal=0.7, name=label + "CageHigh")
        a.box((FRONT + CAGE_DEPTH / 2, py, PORT_Z + PORT_H / 2 + 0.0008),
              (CAGE_DEPTH, PORT_W + 0.003, 0.0016), STEEL, metal=0.7, name=label + "CageTop")
        a.box((FRONT + CAGE_DEPTH / 2, py, PORT_Z - PORT_H / 2 - 0.0008),
              (CAGE_DEPTH, PORT_W + 0.003, 0.0016), STEEL, metal=0.7, name=label + "CageBot")
        # the latch notch on the top wall, where the plug's spring tab clicks (target only drawn bright)
        a.box((FRONT + 0.035, py, PORT_Z + PORT_H / 2 + 0.001), (0.004, 0.004, 0.001),
              GOLD if label == "Target" else DARK, name=label + "LatchNotch")

    # cable-management comb over the target port: two vertical teeth forming a 0.014 m slot around TARGET_Y
    for k, yy in enumerate((TARGET_Y - 0.008, TARGET_Y + 0.008)):
        a.box((COMB_X, yy, PORT_Z + 0.002), (0.004, 0.002, 0.040), STEEL, metal=0.6,
              name=f"CombTooth{k}")
    a.box((COMB_X, TARGET_Y, PORT_Z + 0.023), (0.004, 0.018, 0.003), STEEL, metal=0.6, name="CombBridge")

    # staging cradle for the plug (local x -0.100 -> world 0.250, y +0.060, top at PORT_Z)
    sx, sy = -0.100, 0.060
    a.box((sx, sy, 0.050), (0.010, 0.010, 0.100), STEEL, metal=0.5, name="CradlePost")
    a.box((sx, sy, PORT_Z - 0.006), (0.050, 0.022, 0.010), ALU, metal=0.4, name="CradlePlate")
    a.box((sx, sy - 0.012, PORT_Z), (0.050, 0.003, 0.014), DARK, name="CradleWallA")
    a.box((sx, sy + 0.012, PORT_Z), (0.050, 0.003, 0.014), DARK, name="CradleWallB")
    a.save()


def build_plug():
    """The 40 mm DAC plug: gold contact tongue, keyed steel body, spring latch tab and a rubber boot flange.

    Local frame: long axis +x, tip at +0.020, rear at -0.020, origin at the body centre.  Local +z is up.
    """
    a = Asset("plug.usda", "Plug", "40 mm DAC/SFP-style copper plug with latch tab and strain-relief boot",
              mass=0.020, no_gravity=True)
    # keyed body 40 x 13.5 x 8.6 mm
    a.box((0, 0, 0), (0.040, 0.0135, 0.0086), STEEL, collide=True, metal=0.75, rough=0.3, name="Body")
    # gold contact tongue at the tip (first 12 mm)
    a.box((0.014, 0, -0.0005), (0.012, 0.0100, 0.0030), GOLD, metal=0.85, rough=0.25, name="Contacts")
    # key rib (off-centre) so the plug only fits one way round
    a.box((0.004, -0.0050, 0), (0.024, 0.0015, 0.0060), DARK, name="KeyRib")
    # spring latch tab on top near the tip: clicks into the cage notch at full depth
    a.box((0.010, 0, 0.0052), (0.008, 0.0040, 0.0020), STEEL, metal=0.6, name="LatchTab")
    a.box((0.010, 0, 0.0064), (0.006, 0.0030, 0.0008), AMBER, name="LatchMark")
    # strain-relief boot flange at the rear (12 mm wide): seats between the comb teeth
    a.box((-0.018, 0, 0), (0.010, 0.0120, 0.0110), RUBBER, rough=0.95, name="BootFlange")
    a.box((-0.025, 0, 0), (0.010, 0.0080, 0.0070), RUBBER, rough=0.95, name="BootTail")
    a.save()


if __name__ == "__main__":
    build_switch()
    build_plug()
