"""Writes this task's USD assets with pxr (boxes and cylinders only; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A real 100 mm steel angle-bracket gusset located on a bench fixture and held there by two hold-down toggle
clamps.  The fixture + locator nest + clamp bases are one welded, collider-free visual body.  The gusset and the
two toggle-clamp levers are each their own gravity-free free body, so the task file can pose them as the grippers
carry, seat and flip them.  Product class and every dimension/mass are in REFERENCE.md.
"""
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

# -- colours -----------------------------------------------------------------------------------------------------------
STEEL = (0.62, 0.65, 0.68)
ALU = (0.58, 0.62, 0.66)
DARK = (0.10, 0.11, 0.12)
RED = (0.82, 0.13, 0.11)      # the toggle-clamp handles
RUBBER = (0.07, 0.07, 0.075)
YELLOW = (0.95, 0.72, 0.12)   # locator pins
BLACK = (0.03, 0.03, 0.03)


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

    def cyl(self, center, radius, height, rgb, collide=False, axis="Z", metal=0.0, rough=0.6, name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"/{self.name}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        self._finish(g, rgb, collide, metal, rough)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


def build_fixture():
    """Welded fixture: bench plate, the gusset locator nest (base, back wall, two pins, end stop), two clamp bases
    and the gusset staging tray. Local origin is the fixture centre; bench top z 0, fixture top z 0.045."""
    a = Asset("fixture.usda", "Fixture", "welded gusset fixture, locator nest, two toggle-clamp bases (no colliders)",
              kinematic=True)
    # bench plate: 160 x 240 x 45 mm, centred at y +0.02 so the staging tray fits at y +0.10
    a.box((0, 0.020, 0.0225), (0.160, 0.240, 0.045), ALU, metal=0.5, rough=0.4, name="FixturePlate")
    a.box((0, 0.020, 0.0455), (0.162, 0.242, 0.002), DARK, name="TopPlate")
    # locator nest for the gusset inner corner: back wall (gusset upright leans on it) + end stop + two pins
    a.box((0, -0.007, 0.070), (0.112, 0.006, 0.055), ALU, metal=0.5, name="LocatorBackWall")
    a.box((0.054, 0.014, 0.052), (0.004, 0.030, 0.014), ALU, metal=0.5, name="LocatorEndStop")
    for x in (-0.020, 0.020):
        a.cyl((x, 0.012, 0.049), 0.003, 0.008, YELLOW, metal=0.3, name="LocatorPin")
    # two toggle-clamp bases (hold-down style): a foot and a vertical pivot post at x +-0.035, y -0.037
    for x in (-0.035, 0.035):
        a.box((x, -0.037, 0.048), (0.030, 0.030, 0.006), DARK, name="ClampBase")
        a.cyl((x, -0.037, 0.062), 0.005, 0.030, ALU, metal=0.6, name="ClampPivotPost")
    # gusset staging tray at y +0.10: a shallow pocket with two side rails
    a.box((0, 0.100, 0.002), (0.130, 0.055, 0.004), DARK, name="StageTray")
    for y in (0.074, 0.126):
        a.box((0, y, 0.010), (0.130, 0.003, 0.016), ALU, metal=0.4, name="StageRail")
    a.save()


def build_gusset():
    """Real 100 mm angle-bracket gusset: a 100 x 30 x 4 mm foot and a 100 x 4 x 100 mm upright, meeting at the
    inner corner (body origin), plus two diagonal web ribs. Steel, 210 g."""
    a = Asset("gusset.usda", "Gusset", "100 mm steel angle-bracket gusset (foot + upright + web)", mass=0.210,
              no_gravity=True)
    # foot: lies flat on the fixture top, extends +y from the inner corner
    a.box((0, 0.015, 0.002), (0.100, 0.030, 0.004), STEEL, collide=True, metal=0.7, rough=0.35, name="Foot")
    # upright: stands +z from the inner corner, back face at y 0
    a.box((0, -0.002, 0.050), (0.100, 0.004, 0.100), STEEL, collide=True, metal=0.7, rough=0.35, name="Upright")
    # two small triangular-style web ribs (stepped boxes) bracing foot to upright
    for i, x in enumerate((-0.030, 0.030)):
        a.box((x, 0.010, 0.010), (0.006, 0.018, 0.004), STEEL, metal=0.7, name=f"WebLow{i}")
        a.box((x, 0.006, 0.022), (0.006, 0.010, 0.004), STEEL, metal=0.7, name=f"WebMid{i}")
    a.save()


def build_clamp():
    """One 90 mm hold-down toggle clamp lever (gravity-free, 90 g). Body origin is the red handle grip. A link arm
    runs to the jaw pad 45 mm away in body +x; yawing the body +70 deg swings the pad onto the gusset foot."""
    a = Asset("clamp.usda", "Clamp", "hold-down toggle clamp lever: red handle, link arm and rubber jaw pad",
              mass=0.090, no_gravity=True)
    # the red handle grip the robot closes on (at the body origin)
    a.cyl((0, 0, 0), 0.007, 0.030, RED, collide=True, axis="X", rough=0.7, name="Handle")
    a.cyl((0.014, 0, 0), 0.005, 0.004, DARK, axis="X", name="HandleCap")
    # the link arm out to the jaw, with a pivot boss at the far end
    a.box((0.024, 0, -0.004), (0.045, 0.008, 0.008), STEEL, collide=True, metal=0.6, name="LinkArm")
    a.cyl((0.045, 0, -0.004), 0.006, 0.010, ALU, metal=0.6, name="JawPivot")
    # the rubber hold-down pad that presses the gusset foot when over-centre
    a.box((0.045, 0, -0.012), (0.016, 0.014, 0.006), RUBBER, collide=True, rough=0.95, name="JawPad")
    a.save()


if __name__ == "__main__":
    build_fixture()
    build_gusset()
    build_clamp()
