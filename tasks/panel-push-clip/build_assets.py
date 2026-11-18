"""Writes this task's USD assets with pxr (boxes and cylinders only; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A real 8 mm nylon panel push-clip (20 mm cap, 8 mm stem, ~18 mm stem, barbs near the tip, ~2 g) staged upright in a
small tray, beside a fixed trim panel with an 8 mm mounting hole.  The panel and tray are welded, kinematic,
visual-only bodies with no colliders; the clip is its own gravity-free rigid body with an authored 2 g mass, so the
task file can pose it as the gripper carries, presses and pull-tests it.  See REFERENCE.md for the sources and the
measured/estimated split.  Nothing is scaled down.
"""
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

PANEL = (0.30, 0.33, 0.36)
PANEL_D = (0.20, 0.22, 0.25)
HOLE_D = (0.04, 0.045, 0.05)
TRAY = (0.16, 0.18, 0.21)
CLIP = (0.10, 0.11, 0.13)
BARB = (0.05, 0.06, 0.07)
STEEL = (0.55, 0.58, 0.62)


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

    def _finish(self, g, rgb, metal, rough):
        prim = g.GetPrim()
        if rgb is not None:
            mat = self.color("m%02d%02d%02d_%d%d" % (*tuple(int(c * 99) for c in rgb), int(metal * 9), int(rough * 9)),
                             rgb, metal, rough)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def box(self, center, size, rgb, metal=0.0, rough=0.6, name=None):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"/{self.name}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, metal, rough)

    def cyl(self, center, radius, height, rgb, axis="Z", metal=0.0, rough=0.6, name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"/{self.name}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        self._finish(g, rgb, metal, rough)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


def build_panel():
    """A fixed trim panel, 120 x 100 x 3 mm, with an 8 mm mounting hole at its centre. No colliders."""
    a = Asset("panel.usda", "Panel", "fixed trim panel with an 8 mm mounting hole, welded visual-only", kinematic=True)
    t, h = 0.003, 0.0015
    # four strips around a 10 mm square opening (the 8 mm hole, slightly open so it reads from the top camera)
    a.box((0, 0.0275, h), (0.120, 0.045, t), PANEL, name="PlateYPos")
    a.box((0, -0.0275, h), (0.120, 0.045, t), PANEL, name="PlateYNeg")
    a.box((0.0325, 0, h), (0.055, 0.010, t), PANEL, name="PlateXPos")
    a.box((-0.0325, 0, h), (0.055, 0.010, t), PANEL, name="PlateXNeg")
    # dark bore under the opening so the hole reads as a hole, not the table
    a.cyl((0, 0, -0.006), 0.004, 0.001, HOLE_D, name="Bore")
    a.cyl((0, 0, 0.0015), 0.005, 0.0008, PANEL_D, name="HoleRim")
    # four panel screws for realism
    k = 0
    for sx in (-1, 1):
        for sy in (-1, 1):
            k += 1
            a.cyl((sx * 0.050, sy * 0.038, 0.0032), 0.0035, 0.001, STEEL, metal=0.8,
                  rough=0.4, name=f"Screw{k}")
    a.save()


def build_tray():
    """A small staging tray holding the clip upright, with a dark central socket. No colliders."""
    a = Asset("tray.usda", "Tray", "small staging tray with a socket for the clip, welded visual-only", kinematic=True)
    t, h = 0.008, 0.004
    a.box((0, 0.011, h), (0.032, 0.005, t), TRAY, name="TrayYPos")
    a.box((0, -0.011, h), (0.032, 0.005, t), TRAY, name="TrayYNeg")
    a.box((0.011, 0, h), (0.005, 0.017, t), TRAY, name="TrayXPos")
    a.box((-0.011, 0, h), (0.005, 0.017, t), TRAY, name="TrayXNeg")
    a.cyl((0, 0, 0.002), 0.0045, 0.001, HOLE_D, name="Socket")
    a.save()


def build_clip():
    """The real 8 mm push-clip, origin at the cap top, stem pointing down (local -z), 2 g, gravity off."""
    a = Asset("clip.usda", "Clip", "real 8 mm nylon panel push-clip: 20 mm cap, 8 mm stem, barbs, 2 g",
              mass=0.002, no_gravity=True)
    # 20 mm cap, 3 mm thick: top at local z 0, underside at -0.003
    a.cyl((0, 0, -0.0015), 0.010, 0.003, CLIP, rough=0.7, name="Cap")
    a.cyl((0, 0, 0.0004), 0.0095, 0.0008, BARB, rough=0.7, name="CapTop")
    # 8 mm stem, 18 mm long, from -0.003 to -0.021
    a.cyl((0, 0, -0.012), 0.004, 0.018, CLIP, rough=0.7, name="Stem")
    # four flexible barbs near the tip (the retention feature); flared to ~13 mm across
    for k, (bx, by, sx, sy) in enumerate(((0.0053, 0, 0.004, 0.0032), (-0.0053, 0, 0.004, 0.0032),
                                          (0, 0.0053, 0.0032, 0.004), (0, -0.0053, 0.0032, 0.004))):
        a.box((bx, by, -0.0175), (sx, sy, 0.005), BARB, rough=0.7, name=f"Barb{k}")
    # pilot tip that leads the stem into the hole
    a.cyl((0, 0, -0.022), 0.003, 0.0025, CLIP, rough=0.7, name="Tip")
    a.save()


if __name__ == "__main__":
    build_panel()
    build_tray()
    build_clip()
