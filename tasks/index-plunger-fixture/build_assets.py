"""Writes this task's USD assets with pxr (boxes and cylinders only; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A rotary fixture plate locked by a spring-loaded hand index plunger.  The fixed base (pedestal, bearing ring and
plunger bracket) is one welded, visual-only body.  The 120 mm index plate and the plunger (16 mm knob, 7 mm pin) are
each their own gravity-free free body so the task file can pose them from the grippers.  Real product: a 16 mm class
hand-retractable / index plunger (Carr Lane CLM-10-HRPK, M16x1.5 body, 8 mm pin, ~31 mm knob; task knob 16 mm per the
brief), pin seated in one of 8 holes in the rim of a 120 mm indexing plate.  Sizes and masses are from the product
class / brief (see REFERENCE.md).
"""
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

# -- colours -----------------------------------------------------------------------------------------------------------
ALU = (0.60, 0.63, 0.66)
STEEL = (0.70, 0.72, 0.75)
DARK = (0.10, 0.11, 0.12)
BLACK = (0.03, 0.03, 0.03)
KNOB = (0.09, 0.09, 0.10)
RED = (0.80, 0.14, 0.12)


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

    def radial_cyl(self, center, radius, height, rgb, direction, collide=False, metal=0.0, rough=0.6, name=None):
        """A cylinder whose local Z axis is rotated to point along ``direction`` (for radial rim holes)."""
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"/{self.name}/{name or 'Rad'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr("Z")
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        d = Gf.Vec3d(*direction)
        d.Normalize()
        rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), d)
        q = rot.GetQuat()
        g.AddOrientOp().Set(Gf.Quatf(q.GetReal(), *q.GetImaginary()))
        self._finish(g, rgb, collide, metal, rough)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


N_HOLES = 8
PITCH = 2.0 * math.pi / N_HOLES
HOLE_RING = 0.072


def build_base():
    """The welded pedestal, bearing ring and plunger bracket (visual only, no colliders)."""
    a = Asset("base.usda", "Base", "rotary fixture base / pedestal, bearing ring and plunger bracket", kinematic=True)
    a.box((0, 0, 0.045), (0.150, 0.150, 0.090), ALU, metal=0.5, rough=0.4, name="Pedestal")
    a.box((0, 0, 0.0915), (0.154, 0.154, 0.003), DARK, name="TopDeck")
    a.cyl((0, 0, 0.100), 0.055, 0.018, STEEL, metal=0.7, rough=0.35, name="BearingRing")
    # plunger bracket on the +y side at local z 0.110 (world z 0.8575)
    a.box((0, 0.140, 0.110), (0.030, 0.030, 0.030), ALU, metal=0.5, name="PlungerBracket")
    a.cyl((0, 0.140, 0.110), 0.0105, 0.030, DARK, axis="Y", name="PlungerBore")
    # home stop index mark (red) under the plunger at angle 0
    a.box((0, 0.080, 0.092), (0.006, 0.010, 0.002), RED, name="HomeMark")
    a.save()


def build_plate():
    """The 150 mm index plate: disc, central hub knob and 8 radial index holes in the rim."""
    a = Asset("plate.usda", "Plate", "rotary 8-stop index plate with central hub and rim holes",
              mass=0.600, no_gravity=True)
    a.cyl((0, 0, 0), 0.075, 0.020, ALU, collide=True, metal=0.5, rough=0.4, name="Disc")
    a.cyl((0, 0, 0), 0.055, 0.021, DARK, metal=0.4, name="Recess")
    a.cyl((0, 0, 0.011), 0.012, 0.002, STEEL, metal=0.7, name="HubCollar")
    # central hub knob: the gripper grabs this to turn the plate about its centre
    a.cyl((0, 0, 0.020), 0.012, 0.020, STEEL, collide=True, metal=0.7, rough=0.35, name="Hub")
    a.cyl((0, 0, 0.031), 0.013, 0.002, DARK, metal=0.6, name="HubRim")
    # 8 radial index holes opening on the rim (dark), 45 deg apart, first at angle 0 (+y).
    for k in range(N_HOLES):
        th = k * PITCH
        u = (math.sin(th), math.cos(th), 0.0)
        c = (HOLE_RING * u[0], HOLE_RING * u[1], 0.0)
        a.radial_cyl(c, 0.0035, 0.014, BLACK, u, name=f"IndexHole{k}")
        a.radial_cyl(c, 0.005, 0.002, STEEL, u, metal=0.6, name=f"HoleLip{k}")
    a.save()


def build_plunger():
    """The hand index plunger: 16 mm knob, 8 mm pin with a tapered nose, pointing along local -y."""
    a = Asset("plunger.usda", "Plunger", "16 mm class spring index plunger: 16 mm knob and 7 mm pin",
              mass=0.120, no_gravity=True)
    # knob at local origin, axis along y; the gripper closes across its 16 mm diameter
    a.cyl((0, 0, 0), 0.008, 0.016, KNOB, collide=True, axis="Y", rough=0.85, name="Knob")
    a.cyl((0, 0.0085, 0), 0.007, 0.002, DARK, axis="Y", name="KnobCollar")
    # pin extends from the knob's inner face (y -0.008) toward local -y; tip at y -0.080
    a.cyl((0, -0.046, 0), 0.0035, 0.072, STEEL, collide=True, axis="Y", metal=0.8, rough=0.3, name="Pin")
    a.cyl((0, -0.083, 0), 0.0030, 0.003, STEEL, axis="Y", metal=0.8, name="PinTip")
    a.save()


if __name__ == "__main__":
    build_base()
    build_plate()
    build_plunger()
