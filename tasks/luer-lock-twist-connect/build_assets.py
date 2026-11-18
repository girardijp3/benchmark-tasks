"""Writes this task's USD assets with pxr (boxes and cylinders only; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A 6 mm luer-lock syringe tip and a 20 mm vial adapter on a vial in a bench fixture.  The fixture, vial and adapter
(with its female luer-lock port) are one welded, visual-only body.  The syringe is its own gravity-free free body so
the task file can pose it as the grippers carry, seat and twist it.  Real product: a 6% (1:20) male luer tip on a
10 mL syringe with a 20 mm barrel and a threaded luer-lock collar, and a 20 mm-neck vial adapter with a female
luer-lock port; sizes and the mass are from the product class / brief (see REFERENCE.md).
"""
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

# -- colours -----------------------------------------------------------------------------------------------------------
ALU = (0.60, 0.63, 0.66)
DARK = (0.10, 0.11, 0.12)
GLASS = (0.72, 0.80, 0.84)
PLASTIC = (0.78, 0.86, 0.90)
RUBBER = (0.07, 0.08, 0.09)
STEEL = (0.70, 0.72, 0.75)
RED = (0.85, 0.13, 0.11)
BLUE = (0.12, 0.28, 0.78)
GREEN = (0.10, 0.55, 0.22)


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


def build_adapter():
    """The welded bench fixture, 20 mm vial, adapter skirt and female luer-lock port (all visual-only)."""
    a = Asset("adapter.usda", "Adapter", "20 mm vial adapter with female luer-lock port, in a bench fixture (welded)",
              kinematic=True)
    # bench fixture: 90 x 90 x 12 mm plate, top at z 0.012
    a.box((0, 0, 0.006), (0.090, 0.090, 0.012), ALU, metal=0.5, rough=0.4, name="Fixture")
    a.box((0, 0, 0.0125), (0.092, 0.092, 0.002), DARK, name="TopPlate")
    # a 20 mm injection vial: glass body 0.012 -> 0.060, neck / crimp 0.060 -> 0.066
    a.cyl((0, 0, 0.036), 0.010, 0.048, GLASS, rough=0.25, name="VialBody")
    a.cyl((0, 0, 0.0605), 0.0102, 0.004, STEEL, metal=0.7, name="VialCrimp")
    a.cyl((0, 0, 0.064), 0.007, 0.004, RUBBER, rough=0.9, name="VialStopper")
    # the vial adapter: 20 mm skirt over the crimp, then a female luer-lock port rising to z 0.100
    a.cyl((0, 0, 0.070), 0.0100, 0.012, PLASTIC, name="AdapterSkirt")
    a.cyl((0, 0, 0.078), 0.0075, 0.004, PLASTIC, name="AdapterShoulder")
    a.cyl((0, 0, 0.089), 0.0055, 0.022, PLASTIC, name="FemalePort")     # port 0.078 -> 0.100
    a.cyl((0, 0, 0.089), 0.0028, 0.024, DARK, name="PortBore")          # the female socket
    for i in range(4):                                                   # visual internal thread ridges
        a.cyl((0, 0, 0.081 + i * 0.005), 0.0040, 0.0009, STEEL, metal=0.5, name=f"PortThread{i}")
    a.cyl((0, 0, 0.100), 0.0068, 0.0015, PLASTIC, name="PortLip")
    # an orientation mark on the adapter skirt (visual only)
    a.box((0.0, 0.0092, 0.0765), (0.003, 0.002, 0.010), BLUE, name="AdapterMark")
    a.save()


def build_syringe():
    """The 10 mL luer-lock syringe: 20 mm barrel, luer-lock collar, 6 mm male tip, plunger and index mark."""
    a = Asset("syringe.usda", "Syringe", "10 mL luer-lock syringe: 20 mm barrel, threaded collar, 6% male luer tip",
              mass=0.012, no_gravity=True)
    # barrel: 20 mm diameter, 46 mm long, centred on the part origin
    a.cyl((0, 0, 0), 0.010, 0.046, PLASTIC, collide=True, rough=0.35, name="Barrel")
    for i in range(5):                                                   # graduation marks (visual)
        a.cyl((0, 0, -0.016 + i * 0.008), 0.0101, 0.0006, DARK, name=f"Grad{i}")
    # finger flanges at the barrel top
    a.box((0, 0.013, 0.024), (0.010, 0.010, 0.003), PLASTIC, name="FlangeA")
    a.box((0, -0.013, 0.024), (0.010, 0.010, 0.003), PLASTIC, name="FlangeB")
    # plunger rod and thumb pad
    a.cyl((0, 0, 0.038), 0.003, 0.030, STEEL, metal=0.3, name="PlungerRod")
    a.cyl((0, 0, 0.054), 0.010, 0.004, PLASTIC, name="ThumbPad")
    a.cyl((0, 0, 0.0565), 0.008, 0.002, RUBBER, rough=0.95, name="ThumbRubber")
    # the luer-lock collar: 11 mm across, 10 mm tall, with a visual thread and a lock lug
    a.cyl((0, 0, -0.028), 0.0055, 0.010, PLASTIC, name="LockCollar")
    for i in range(2):
        a.cyl((0, 0, -0.032 + i * 0.004), 0.0058, 0.0008, STEEL, metal=0.5, name=f"CollarThread{i}")
    # the 6% male luer tip: tapered, 8 mm long, tip bottom at z = -0.041
    a.cyl((0, 0, -0.037), 0.0028, 0.008, PLASTIC, name="MaleTip")
    a.cyl((0, 0, -0.0408), 0.0020, 0.0012, PLASTIC, name="TipEnd")
    a.cyl((0, 0, -0.0375), 0.0016, 0.006, DARK, name="TipBore")
    # an index mark on the thumb pad so the twist is visible from the top camera
    a.box((0.0055, 0, 0.0578), (0.006, 0.003, 0.0012), RED, name="IndexMark")
    a.save()


if __name__ == "__main__":
    build_adapter()
    build_syringe()
