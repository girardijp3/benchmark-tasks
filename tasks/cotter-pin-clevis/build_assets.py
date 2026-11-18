"""Writes this task's USD assets with pxr (boxes, cylinders and generated ring meshes; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A real 3 mm (nominal) ISO 1234 steel split/cotter pin and an 8 mm clevis-pin joint in a bench fixture.  The fixture,
fork, clevis pin (with its 3.2 mm cross-hole), rod end and pull tab are one welded, visual-only body with no colliders.
The cotter pin is three free, gravity-free bodies so the task file can pose the eye+shank and bend its two legs
independently: body (eye + shank), long leg and short leg.  Product class and sizes are documented in REFERENCE.md;
no size is scaled down.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"

# -- real product numbers (metres) ---------------------------------------------------------------------------------------
WIRE_R = 0.0014          # 3 mm nominal split pin: 2.8 mm wire (ISO 1234 d=3 -> wire ~0.9 d)
SHANK = 0.012            # straight part from the eye's seat face to the legs' roots
LEG_GAP = 0.0006         # gap between the two legs at the root
LONG = 0.036             # long (extended-prong) leg past the shank
SHORT = 0.028            # short leg past the shank
EYE_R = 0.0030           # eye loop centreline radius
EYE_X = -0.0045          # eye loop centre along the cotter's x axis (behind the seat face)

# -- fixture numbers (local frame, table top z = 0) ----------------------------------------------------------------------
PIN_R = 0.005            # 10 mm clevis pin
HOLE_Z = 0.055           # cross-hole centre height above the table
HOLE_D = 0.0032          # cross-hole diameter (clearance for a 3 mm cotter)
TAB_X = 0.058            # pull tab offset from the clevis pin axis, along +x

# -- colours ------------------------------------------------------------------------------------------------------------
ALU = (0.60, 0.63, 0.66)
DARK = (0.10, 0.11, 0.12)
STEEL = (0.70, 0.72, 0.75)
ZINC = (0.82, 0.83, 0.80)
YELLOW = (0.88, 0.68, 0.10)
RED = (0.85, 0.14, 0.12)
GREEN = (0.12, 0.62, 0.24)
BLUE = (0.12, 0.36, 0.85)
WHITE = (0.92, 0.92, 0.90)


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

    def mesh(self, name, points, quads, rgb, collide=False, metal=0.0, rough=0.6):
        tris = []
        for q in quads:
            for k in range(1, len(q) - 1):
                tris.append((q[0], q[k], q[k + 1]))
        g = UsdGeom.Mesh.Define(self.stage, f"/{self.name}/{name}")
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for t in tris for i in t])
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateDoubleSidedAttr(True)
        lo = [min(p[i] for p in points) for i in range(3)]
        hi = [max(p[i] for p in points) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        self._finish(g, rgb, collide, metal, rough)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


def _eye_ring():
    """A flat torus in the cotter's x-y plane: the eye loop of the split pin, at EYE_X."""
    M, N = 48, 10
    pts, quads = [], []
    for i in range(M):
        t = 2 * math.pi * i / M
        cx, cy = EYE_X + EYE_R * math.cos(t), EYE_R * math.sin(t)
        for j in range(N):
            p = 2 * math.pi * j / N
            r = WIRE_R * math.cos(p)
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t), WIRE_R * math.sin(p)))
    for i in range(M):
        for j in range(N):
            quads.append([i * N + j, ((i + 1) % M) * N + j,
                          ((i + 1) % M) * N + (j + 1) % N, i * N + (j + 1) % N])
    return pts, quads


def build_fixture():
    """Welded clevis joint fixture: steel stand, fork, 10 mm clevis pin with 3.2 mm cross-hole, rod end, pull tab,
    white staging holder for the cotter.  Visual-only, no colliders."""
    a = Asset("clevis.usda", "Clevis",
              "clevis joint fixture: fork, 10 mm clevis pin with 3.2 mm cross-hole, rod end, pull tab, cotter holder",
              kinematic=True)
    # bench plate and upright stand
    a.box((0, 0, 0.004), (0.110, 0.070, 0.008), ALU, metal=0.5, rough=0.4, name="BenchPlate")
    a.box((-0.028, 0, 0.022), (0.040, 0.046, 0.028), DARK, name="Stand")
    # fork clevis: two horizontal prongs with rounded ends around the pin, joined by a web at -x
    a.box((-0.019, 0, 0.047), (0.026, 0.034, 0.005), STEEL, metal=0.7, name="UpperProng")
    a.box((-0.019, 0, 0.028), (0.026, 0.034, 0.005), STEEL, metal=0.7, name="LowerProng")
    a.box((-0.031, 0, 0.0375), (0.006, 0.034, 0.025), STEEL, metal=0.7, name="YokeWeb")
    # yellow rod end between the prongs, running +x
    a.box((0.024, 0, 0.0375), (0.070, 0.020, 0.010), YELLOW, metal=0.2, name="RodEnd")
    # the upright 10 mm clevis pin through all three, from z 0.025 to 0.066
    a.cyl((0, 0, 0.0455), PIN_R, 0.041, STEEL, metal=0.85, rough=0.3, name="ClevisPin")
    a.cyl((0, 0, 0.0260), 0.0070, 0.0025, STEEL, metal=0.85, name="PinHead")
    # the 3.2 mm cross-hole: a dark cylinder along x through the pin at HOLE_Z (the cotter passes through this)
    a.cyl((0, 0, HOLE_Z), HOLE_D / 2, 2 * PIN_R + 0.002, DARK, axis="X", name="CrossHole")
    # pull tab on the rod end: red ring the robot grabs for the pull test
    a.cyl((TAB_X, 0, HOLE_Z), 0.006, 0.002, RED, axis="Y", name="PullTabRing")
    a.box(((TAB_X + 0.030) / 2, 0, 0.0375), (0.030, 0.008, 0.006), STEEL, name="TabStem")
    # white staging holder nearer the robot (+y): shallow V pocket where the straight cotter lies
    a.box((0, 0.100, 0.003), (0.075, 0.030, 0.006), WHITE, name="CotterHolder")
    a.box((0, 0.090, 0.0075), (0.075, 0.002, 0.004), WHITE, name="HolderLipA")
    a.box((0, 0.110, 0.0075), (0.075, 0.002, 0.004), WHITE, name="HolderLipB")
    a.save()


def build_cotter_body():
    """The eye loop and straight shank of the 3 mm split pin (the two leg roots are the ends of the shank)."""
    a = Asset("cotter.usda", "Cotter",
              "3 mm nominal ISO 1234 steel split pin: eye loop + 12 mm shank, 2.8 mm wire, 8 mm 3 mm nominal",
              mass=0.0008, no_gravity=True)
    pts, quads = _eye_ring()
    a.mesh("EyeLoop", pts, quads, ZINC, metal=0.8, rough=0.35)
    for s in (1, -1):
        y = s * LEG_GAP / 2
        a.cyl((SHANK / 2, y, 0), WIRE_R, SHANK, ZINC, axis="X", metal=0.8, rough=0.35,
              name="ShankLeg" + ("A" if s > 0 else "B"))
    # bright band where the eye seats against the clevis pin (read from the top camera for insertion depth)
    a.cyl((0.0008, 0, 0), WIRE_R + 0.0004, 0.0012, RED, axis="X", name="SeatBand")
    a.save()


def build_leg(fname, name, length, color):
    """One leg: a straight wire from its root (local origin) along +x, with a bright tip cap for legibility."""
    a = Asset(fname, name, f"split-pin leg, {int(length * 1000)} mm past the shank, 2.8 mm wire",
              mass=0.0004, no_gravity=True)
    a.cyl((length / 2, 0, 0), WIRE_R, length, ZINC, axis="X", metal=0.8, rough=0.35, name="Wire")
    a.cyl((length - 0.001, 0, 0), WIRE_R + 0.0008, 0.002, color, axis="X", name="TipCap")
    a.save()


if __name__ == "__main__":
    build_fixture()
    build_cotter_body()
    build_leg("leg_long.usda", "LegLong", LONG, GREEN)
    build_leg("leg_short.usda", "LegShort", SHORT, BLUE)
