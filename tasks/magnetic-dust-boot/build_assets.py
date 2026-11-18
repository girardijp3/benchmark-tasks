"""Writes this task's USD assets with pxr (boxes and cylinders only; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A real 80 mm magnetic dust boot presented on a 65 mm machine-tool spindle nose at bench height.  The spindle
fixture (base, shoulder boss, 65 mm nose and the boot staging pocket) is one welded, visual-only body.  The boot
is a gravity-free free body (80 mm OD, 65 mm bore, 40 mm tall, with a visible embedded magnet ring at its lower
inner rim) that the task file poses from the gripper, so the concentric alignment, press stroke, seat against the
shoulder and all three failure caps are real geometry.  True sizes: boot OD 80 mm, bore and nose 65 mm.  Mass and
the 4-7 N press force are from the product class (see REFERENCE.md).
"""
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

# -- colours -----------------------------------------------------------------------------------------------------------
BASE = (0.16, 0.17, 0.19)
BOSS = (0.34, 0.36, 0.39)
STEEL = (0.68, 0.71, 0.74)
ELAST = (0.10, 0.11, 0.12)      # oil-resistant black NBR elastomer
MAGNET = (0.30, 0.32, 0.36)     # embedded magnet ring band
ACCENT = (0.85, 0.55, 0.10)     # alignment mark on the boot
ALU = (0.60, 0.63, 0.66)
DARK = (0.10, 0.11, 0.12)


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


def build_spindle():
    """Fixed spindle-nose fixture: base, 100 mm shoulder boss, 65 mm nose and the boot staging pocket."""
    a = Asset("spindle.usda", "Spindle",
              "machine-tool spindle nose: 65 mm nose on a shoulder, with a magnetic-boot staging pocket (welded)",
              kinematic=True)
    # pedestal and shoulder boss; the shoulder face the boot seats on is at local z 0.050
    a.box((0, 0, 0.012), (0.160, 0.160, 0.024), BASE, metal=0.1, rough=0.7, name="Base")
    a.cyl((0, 0, 0.032), 0.050, 0.036, BOSS, metal=0.6, rough=0.5, name="ShoulderBoss")
    a.cyl((0, 0, 0.050), 0.050, 0.001, STEEL, metal=0.7, rough=0.35, name="ShoulderFace")
    # 65 mm diameter nose, 45 mm tall, from z 0.050 to z 0.095
    a.cyl((0, 0, 0.0725), 0.0325, 0.045, STEEL, metal=0.8, rough=0.3, name="Nose65")
    a.cyl((0, 0, 0.095), 0.020, 0.001, BOSS, metal=0.7, name="NoseFace")
    # staging pocket (boot rests with its bottom at local z 0): base + two low walls
    a.box((-0.050, -0.060, 0.004), (0.095, 0.095, 0.008), ALU, metal=0.4, name="StageBase")
    a.box((-0.050, -0.108, 0.011), (0.095, 0.003, 0.014), DARK, name="StageWallA")
    a.box((-0.050, -0.012, 0.011), (0.095, 0.003, 0.014), DARK, name="StageWallB")
    a.save()


def build_boot():
    """The 80 mm magnetic dust boot: oil-resistant NBR elastomer body with a moulded-in magnet ring."""
    a = Asset("boot.usda", "Boot", "80 mm oil-resistant elastomer magnetic dust boot (free body)",
              mass=0.090, no_gravity=True)
    # 80 mm OD x 40 mm tall elastomer body, centred at local z 0; bore 65 mm fits the spindle nose.
    # Visual-only (no collider): the robot grips the small central boss below, so the fingers can close on it.
    a.cyl((0, 0, 0), 0.040, 0.040, ELAST, collide=False, metal=0.0, rough=0.85, name="ElastomerBody")
    # the bore (visual): 65 mm through-hole
    a.cyl((0, 0, 0), 0.0325, 0.0405, DARK, metal=0.0, rough=0.9, name="Bore65")
    # embedded magnet ring at the lower inner rim (the band that centres on the steel nose)
    a.cyl((0, 0, -0.014), 0.0365, 0.006, MAGNET, metal=0.5, rough=0.5, name="MagnetRing")
    a.cyl((0, 0, -0.014), 0.0325, 0.0062, DARK, metal=0.0, rough=0.9, name="MagnetRingBore")
    # a raised flange at the top and an alignment tick so concentricity is readable from the top camera
    a.cyl((0, 0, 0.018), 0.040, 0.004, ELAST, metal=0.0, rough=0.85, name="TopFlange")
    a.box((0, -0.036, 0.0205), (0.004, 0.010, 0.001), ACCENT, name="AlignTick")
    # small central grasp boss on top (20 mm wide): the only collider, so a closed grip can take the boot
    a.cyl((0, 0, 0.026), 0.010, 0.012, ELAST, collide=True, metal=0.0, rough=0.8, name="GraspBoss")
    a.save()


if __name__ == "__main__":
    build_spindle()
    build_boot()
