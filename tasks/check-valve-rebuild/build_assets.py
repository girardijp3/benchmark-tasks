"""Writes this task's USD assets with pxr (boxes and cylinders only; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A 20 mm brass spring check valve held in a small bench fixture.  The fixture and valve body (with its seat,
inlet/outlet and two test buttons) are one welded body.  The three loose parts -- threaded cap, poppet/disc and
compression spring -- are each their own gravity-free free body, so the task file can pose them as the grippers
carry and seat them.  Real product: a 20 mm (3/4 in) brass spring check valve, cap 26 mm across flats, poppet
disc 16 mm, spring 14 mm OD x 24 mm free; these sizes and the masses are from the product class (see REFERENCE.md).
"""
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent / "assets"

# -- colours -----------------------------------------------------------------------------------------------------------
BRASS = (0.72, 0.55, 0.20)
ALU = (0.60, 0.63, 0.66)
DARK = (0.10, 0.11, 0.12)
STEEL = (0.70, 0.72, 0.75)
RUBBER = (0.05, 0.05, 0.055)
RED = (0.80, 0.14, 0.12)
BLUE = (0.10, 0.30, 0.80)
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


def build_valve():
    """The welded fixture + brass body + seat + two test buttons + the parts' staging tray and cap peg."""
    a = Asset("valve.usda", "Valve", "20 mm brass spring check valve in a bench fixture (welded)", kinematic=True)
    # bench fixture: 90 x 90 x 50 mm plate, top at z 0.050
    a.box((0, 0, 0.025), (0.090, 0.090, 0.050), ALU, metal=0.5, rough=0.4, name="Fixture")
    a.box((0, 0, 0.0505), (0.092, 0.092, 0.003), DARK, name="TopPlate")
    # brass valve body: cylinder from z 0.050 to 0.090
    a.cyl((0, 0, 0.070), 0.014, 0.040, BRASS, metal=0.8, rough=0.3, name="Body")
    # inlet / outlet port collars
    a.cyl((0, 0, 0.047), 0.010, 0.008, BRASS, metal=0.8, rough=0.3, name="Inlet")
    a.cyl((0, 0, 0.092), 0.010, 0.006, BRASS, metal=0.8, rough=0.3, name="Outlet")
    # the internal bore (dark) and the seat ledge at z 0.062
    a.cyl((0, 0, 0.070), 0.0062, 0.038, BLACK, name="Bore")
    a.cyl((0, 0, 0.062), 0.0090, 0.002, STEEL, metal=0.6, name="Seat")
    # two test buttons on the front edge (visual only; press is detected geometrically)
    a.cyl((-0.032, -0.025, 0.0305), 0.005, 0.004, RED, name="InletButton")
    a.cyl((-0.032, 0.025, 0.0305), 0.005, 0.004, BLUE, name="OutletButton")
    # staging pockets on the bench to the valve's left (robot's +y side): poppet, spring, cap peg
    for name, (x, y) in {"PoppetPocket": (0.0, 0.045), "SpringPocket": (0.0, 0.070),
                         "CapPeg": (0.0, -0.045)}.items():
        a.box((x, y, 0.004), (0.040, 0.032, 0.008), ALU, metal=0.4, name=name + "Base")
        a.box((x, y - 0.016, 0.010), (0.040, 0.003, 0.014), DARK, name=name + "WallA")
        a.box((x, y + 0.016, 0.010), (0.040, 0.003, 0.014), DARK, name=name + "WallB")
    a.cyl((0.0, -0.045, 0.014), 0.004, 0.012, DARK, name="CapPegPin")
    a.save()


def build_cap():
    """The threaded bonnet: 26 mm across flats, 14 mm tall, with a knurled rim and a hex collar."""
    a = Asset("cap.usda", "Cap", "threaded brass bonnet / cap of the check valve", mass=0.030, no_gravity=True)
    a.cyl((0, 0, 0), 0.013, 0.014, BRASS, collide=True, metal=0.8, rough=0.3, name="CapHex")
    a.cyl((0, 0, 0.006), 0.0135, 0.003, DARK, metal=0.5, name="Knurl")
    a.cyl((0, 0, -0.0075), 0.006, 0.003, BRASS, metal=0.8, name="Thread")
    a.save()


def build_poppet():
    """The replacement poppet/disc: 16 mm disc, rubber seal face down, 5 mm guide stem up."""
    a = Asset("poppet.usda", "Poppet", "replacement poppet disc with rubber seal face and guide stem",
              mass=0.008, no_gravity=True)
    a.cyl((0, 0, 0), 0.008, 0.003, STEEL, collide=True, metal=0.6, name="Disc")
    a.cyl((0, 0, -0.0020), 0.007, 0.0016, RUBBER, rough=0.95, name="SealFace")  # the sealing face (down)
    a.cyl((0, 0, 0.0095), 0.0025, 0.014, STEEL, metal=0.6, name="GuideStem")
    a.save()


def build_spring():
    """The replacement 14 mm OD, 24 mm free compression spring, with a flat top grip collar."""
    a = Asset("spring.usda", "Spring", "replacement compression spring for the check valve",
              mass=0.004, no_gravity=True)
    a.cyl((0, 0, 0), 0.007, 0.024, STEEL, collide=True, metal=0.7, rough=0.35, name="Coil")
    for i in range(7):
        a.cyl((0, 0, -0.010 + i * 0.0034), 0.0030, 0.0012, DARK, name=f"Bore{i}")
    a.cyl((0, 0, 0.0125), 0.007, 0.002, STEEL, metal=0.7, name="TopCollar")
    a.save()


if __name__ == "__main__":
    build_valve()
    build_cap()
    build_poppet()
    build_spring()
