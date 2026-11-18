"""Writes this task's USD assets with pxr (boxes, cylinders and spheres only; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A two-piece rigid cervical collar on a care-training manikin.  The manikin (shoulders, neck, head, chin) is one
welded, kinematic, visual-only body fixed to the bench.  The two rigid shells -- a front chin-support shell and a
back occipital-support shell -- are each their own gravity-free free body, 30 mm side tabs included, so the task
file can pose them as the grippers carry and seat them.  Real product class: a two-piece rigid cervical collar,
~300 mm across the tabs, 0.3 kg total (front 0.14 kg, back 0.16 kg); see REFERENCE.md for the sources and the
measured/estimated split.  Nothing is scaled down.
"""
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

SKIN = (0.86, 0.72, 0.60)
SKIN_D = (0.74, 0.58, 0.47)
BASE = (0.16, 0.18, 0.22)
FRONT = (0.15, 0.45, 0.78)
BACK = (0.80, 0.32, 0.20)
FOAM = (0.92, 0.90, 0.84)
TAB = (0.20, 0.22, 0.26)
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

    def sphere(self, center, radius, rgb, collide=False, metal=0.0, rough=0.6, name=None):
        self.n += 1
        g = UsdGeom.Sphere.Define(self.stage, f"/{self.name}/{name or 'Sphere'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        self._finish(g, rgb, collide, metal, rough)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


def build_manikin():
    """Welded, visual-only care-training manikin: base, shoulders, neck, head and chin.  No colliders."""
    a = Asset("manikin.usda", "Manikin",
              "care-training manikin torso / neck / head, welded kinematic visual-only", kinematic=True)
    a.box((0, 0, 0.006), (0.26, 0.30, 0.012), BASE, name="BasePlate")
    a.box((0, 0, 0.028), (0.22, 0.26, 0.048), SKIN_D, name="Shoulders")
    a.cyl((0, 0, 0.100), 0.042, 0.120, SKIN, name="Neck")             # z 0.040 - 0.160
    a.sphere((0, 0, 0.215), 0.055, SKIN, name="Head")                 # head, chin below front
    a.box((0.050, 0, 0.160), (0.034, 0.068, 0.020), SKIN_D, name="Chin")
    a.cyl((0, 0, 0.161), 0.048, 0.008, SKIN_D, name="JawLine")
    a.save()


def _shell(fname, name, doc, color, mass, back=False):
    """A rigid collar shell, ~120 mm wide, with two 30 mm side tabs sticking out in +/-y."""
    a = Asset(fname, name, doc, mass=mass, no_gravity=True)
    # main curved plate (approximated by three flat segments), ~65 mm tall, 6 mm thick
    a.box((0, 0, 0.000), (0.007, 0.105, 0.040), color, rough=0.5, name="PlateLow")
    a.box((0, 0, 0.028), (0.010, 0.110, 0.038), color, rough=0.5, name="PlateHigh")
    a.box((0.004, 0, 0.014), (0.006, 0.100, 0.050), color, rough=0.5, name="Spine")
    # soft foam liner on the inner face
    a.box((-0.006 if not back else -0.008, 0, 0.014), (0.003, 0.090, 0.044), FOAM, rough=0.95, name="Liner")
    # the chin cup (front) or occipital rim (back)
    if back:
        a.cyl((-0.010, 0, 0.038), 0.034, 0.006, color, axis="Y", rough=0.5, name="OccipitalRim")
    else:
        a.box((0.010, 0, 0.034), (0.018, 0.070, 0.020), color, rough=0.5, name="ChinCup")
    # two 30 mm side tabs (the 4-7 N press targets): local y +/-0.050, 30 mm long in x
    for sgn, lab in ((1, "L"), (-1, "R")):
        a.box((0.0, sgn * 0.050, 0.012), (0.030, 0.030, 0.008), TAB, name="Tab" + lab)
        a.box((0.0, sgn * 0.040, 0.012), (0.010, 0.030, 0.014), color, name="TabRoot" + lab)
    a.save()


if __name__ == "__main__":
    build_manikin()
    _shell("front.usda", "FrontShell", "rigid cervical collar FRONT shell (chin support), 0.14 kg",
           FRONT, 0.140, back=False)
    _shell("back.usda", "BackShell", "rigid cervical collar BACK shell (occipital support), 0.16 kg",
           BACK, 0.160, back=True)
