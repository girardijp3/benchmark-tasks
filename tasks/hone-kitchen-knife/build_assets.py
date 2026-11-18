"""Writes this task's USD assets with pxr + scipy's convex hull (no outside mesh files).

Run:  local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

Frames: metres, kilograms, Z up.  The honing steel is a welded, collider-free fixture with its rod axis along
world +X; the knife is a free, gravity-free rigid body whose local origin is the blade heel, local +X running
heel->tip along a 200 mm blade, local +Z the spine direction (edge at local z = -0.030), local Y the 2.5 mm
blade thickness.  Real product dimensions (see REFERENCE.md): 200 mm chef's knife, 300 mm honing steel.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from scipy.spatial import ConvexHull

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001
TOP = 0.7475

# -- the real products (see REFERENCE.md for the measured / estimated split) -------------------------------------------
BLADE_LEN = 200 * MM          # 200 mm chef's knife, blade length (the named product size)
BLADE_H = 55 * MM             # heel blade height,EST for a 200 mm chef's knife
BLADE_T = 2.5 * MM            # blade thickness at the spine (EST)
HANDLE_LEN = 130 * MM         # handle length (EST)
HANDLE_W = 22 * MM            # handle thickness across the jaws (EST)
HANDLE_H = 26 * MM            # handle height (EST)
KNIFE_MASS = 0.205            # kg, typical 200 mm chef's knife (EST)

STEEL_LEN = 300 * MM          # 300 mm honing steel overall (the named product size)
STEEL_HANDLE = 100 * MM       # handle portion (EST)
STEEL_ROD = STEEL_LEN - STEEL_HANDLE   # 200 mm exposed rod (EST)
STEEL_R = 8 * MM              # rod radius, 16 mm diameter (EST)
STEEL_HANDLE_R = 13 * MM      # handle radius (EST)

# -- colours -----------------------------------------------------------------------------------------------------------
STEEL = (0.66, 0.68, 0.71)
STEEL_DARK = (0.33, 0.36, 0.39)
STEEL_EDGE = (0.90, 0.92, 0.95)
WOOD = (0.20, 0.11, 0.055)
WOOD_DARK = (0.11, 0.06, 0.03)
BOLSTER = (0.74, 0.76, 0.78)
RIVET = (0.82, 0.84, 0.86)
ALU = (0.55, 0.58, 0.61)
BLACK = (0.05, 0.05, 0.055)
GREEN = (0.10, 0.28, 0.16)


class Asset:
    def __init__(self, fname, name, doc, mass=None, kinematic=False, no_gravity=False, com=(0, 0, 0)):
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
            m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(7e-4, 7e-4, 3e-4))
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

    def box(self, center, size, rgb, collide=False, metal=0.0, rough=0.6, name=None, rot=(0, 0, 0)):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"/{self.name}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        if rot[0]:
            g.AddRotateXOp().Set(float(rot[0]))
        if rot[1]:
            g.AddRotateYOp().Set(float(rot[1]))
        if rot[2]:
            g.AddRotateZOp().Set(float(rot[2]))
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

    def mesh(self, pts, tris, rgb, collide=False, metal=0.0, rough=0.6, name="Mesh", guide=False):
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"/{self.name}/{name}{self.n}")
        P = np.asarray(pts, float)
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in p]) for p in P]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for tr in tris for i in tr])
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*P.min(0)), Gf.Vec3f(*P.max(0))]))
        if guide:
            g.CreatePurposeAttr("guide")
        self._finish(g, rgb, collide, metal, rough)

    def lathe_x(self, x0, x1, radius, n=64, rgb=None, metal=0.0, rough=0.6, name=None):
        """A closed smooth cylinder mesh along X (no USD Cylinder prim for a visible round part)."""
        pts = []
        for x in (x0, x1):
            for i in range(n):
                t = 2 * math.pi * i / n
                pts.append((x, radius * math.cos(t), radius * math.sin(t)))
        tris = []
        for i in range(n):
            j = (i + 1) % n
            tris += [(i, j, n + j), (i, n + j, n + i)]
        tris += [(i, (i + 1) % n, 0) for i in range(1, n - 1)]                 # x0 cap
        tris += [(n + i, n + (i + 1) % n, n) for i in range(1, n - 1)]         # x1 cap
        self.mesh(pts, tris, rgb, metal=metal, rough=rough, name=name or "Lathe")

    def hull(self, pts, rgb, collide=True, metal=0.0, rough=0.6, name="Hull", guide=False):
        P = np.asarray(pts, float)
        h = ConvexHull(P)
        self.mesh(P, h.simplices, rgb, collide=collide, metal=metal, rough=rough, name=name, guide=guide)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


def blade_sections():
    """Cross-sections of a 200 mm chef's blade: (x, edge_z, spine_z) with a gently curved belly."""
    return [
        (0.000, -0.030, 0.025),
        (0.038, -0.032, 0.025),
        (0.090, -0.030, 0.023),
        (0.140, -0.024, 0.020),
        (0.175, -0.014, 0.014),
        (0.196, -0.004, 0.005),
        (0.200, 0.000, 0.000),
    ]


def build_knife():
    """A real-size 200 mm chef's knife, free body, local origin at the blade heel."""
    a = Asset("knife.usda", "Knife",
              "200 mm chef's knife: stainless blade, full tang, three-rivet dark wood handle; origin at the "
              "blade heel, local +X heel->tip, local +Z the spine (edge at z=-0.030), local Y the blade thickness.",
              mass=KNIFE_MASS, no_gravity=True, com=(-0.060, 0.0, 0.0))
    # blade: convex hull of the true-size sections (edge thin, spine full thickness)
    pts = []
    for x, ze, zs in blade_sections():
        hy = 0.0010 if x >= 0.19 else BLADE_T / 2
        pts += [(x, -hy, ze), (x, hy, ze), (x, -hy, zs), (x, hy, zs)]
    a.hull(pts, STEEL, collide=True, metal=0.85, rough=0.28, name="Blade")
    # a brighter ground bevel strip just above the edge (visual, tells the story of the honed edge)
    bevel = []
    for x, ze, zs in blade_sections()[:-1]:
        bevel += [(x, -0.0011, ze), (x, 0.0011, ze), (x, -0.0013, ze + 0.006), (x, 0.0013, ze + 0.006)]
    a.hull(bevel, STEEL_EDGE, collide=False, metal=0.9, rough=0.18, name="EdgeBevel")
    # bolster / full tang at the heel, then the dark wood handle and three rivets
    a.box((-0.006, 0, -0.001), (0.014, HANDLE_W - 0.001, 0.050), BOLSTER, metal=0.8, rough=0.3, name="Bolster")
    a.box((-0.072, 0, -0.002), (HANDLE_LEN, HANDLE_W, HANDLE_H), WOOD, collide=True, rough=0.55, name="HandleCore")
    # chamfered handle scales (slightly narrower than the core) give the rounded slab silhouette
    for sy in (-1, 1):
        a.box((-0.072, sy * (HANDLE_W / 2 - 0.0015), -0.002), (HANDLE_LEN - 0.004, 0.003, HANDLE_H - 0.004),
              WOOD_DARK, rough=0.6, name="Scale")
    for x in (-0.038, -0.072, -0.106):
        a.cyl((x, -HANDLE_W / 2 - 0.0006, -0.002), 0.0032, 0.0012, RIVET, axis="Y", metal=0.8, name="RivetA")
        a.cyl((x, HANDLE_W / 2 + 0.0006, -0.002), 0.0032, 0.0012, RIVET, axis="Y", metal=0.8, name="RivetB")
    a.box((-0.137, 0, -0.002), (0.006, HANDLE_W - 0.002, HANDLE_H - 0.004), BOLSTER, metal=0.8, name="EndCap")
    a.save()


def build_steel():
    """A 300 mm honing steel clamped horizontally in a bench holder (welded, no colliders)."""
    a = Asset("steel.usda", "Steel",
              "300 mm honing steel in a bench clamp: a 200 mm fine-grooved chromium rod with a 100 mm knurled "
              "handle. Local origin at the rod heel (guard), rod axis along +X. Welded visual-only fixture.",
              kinematic=True)
    # the exposed rod and a dark fine-grooved honing surface band
    a.lathe_x(0.0, STEEL_ROD, STEEL_R, 64, rgb=STEEL, metal=0.9, rough=0.3, name="Rod")
    a.lathe_x(0.004, STEEL_ROD - 0.004, STEEL_R + 0.0002, 64, rgb=STEEL_DARK, metal=0.7, rough=0.72, name="HoneSurface")
    for i in range(1, 34):
        x = 0.004 + i * (STEEL_ROD - 0.008) / 34
        a.lathe_x(x, x + 0.0006, STEEL_R + 0.0004, 48, rgb=STEEL, metal=0.9, rough=0.25, name="Groove")
    # guard / collar and the knurled handle behind the rod heel
    a.lathe_x(-0.006, 0.002, STEEL_HANDLE_R, 64, rgb=BOLSTER, metal=0.8, rough=0.35, name="Guard")
    a.lathe_x(-STEEL_HANDLE, -0.004, STEEL_HANDLE_R, 64, rgb=WOOD, metal=0.0, rough=0.6, name="Handle")
    for i in range(1, 12):
        x = -STEEL_HANDLE + i * (STEEL_HANDLE - 0.010) / 12
        a.lathe_x(x, x + 0.0012, STEEL_HANDLE_R + 0.0005, 48, rgb=WOOD_DARK, rough=0.7, name="Knurl")
    a.lathe_x(-STEEL_HANDLE - 0.006, -STEEL_HANDLE, STEEL_HANDLE_R, 48, rgb=BOLSTER, metal=0.8, name="ButtCap")
    # bench clamp under the handle and a V-rest under the rod tip (visual only; the rod is welded)
    a.box((-0.050, 0, -0.030), (0.115, 0.055, 0.040), ALU, metal=0.5, rough=0.45, name="HandleClamp")
    a.box((-0.050, 0, -0.010), (0.120, 0.060, 0.004), BLACK, rough=0.7, name="ClampPad")
    a.box((STEEL_ROD - 0.008, 0, -0.026), (0.030, 0.040, 0.036), ALU, metal=0.5, rough=0.45, name="TipRest")
    a.box((STEEL_ROD - 0.008, 0, -0.008), (0.030, 0.030, 0.004), GREEN, rough=0.8, name="RestPad")
    a.save()


def build_rest():
    """The wooden knife rest the knife starts on and is parked on (welded, visual only)."""
    a = Asset("rest.usda", "Rest",
              "A two-cradle wooden knife rest on the bench; the knife parks flat with its blade along +X.",
              kinematic=True)
    a.box((0, 0, 0.006), (0.250, 0.070, 0.012), WOOD_DARK, rough=0.7, name="BasePlate")
    for x in (-0.050, 0.060):
        a.box((x, 0, 0.020), (0.022, 0.070, 0.016), WOOD, rough=0.6, name="Cradle")
        a.box((x, 0, 0.0285), (0.024, 0.050, 0.003), GREEN, rough=0.8, name="CradlePad")
    a.save()


if __name__ == "__main__":
    build_knife()
    build_steel()
    build_rest()
