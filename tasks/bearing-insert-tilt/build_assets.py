"""Writes this task's USD assets with pxr (boxes and generated revolved meshes; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A real 20 mm spherical-OD bearing insert in a pressed pillow-block-style housing.  The housing's concave spherical
seat has its axis inclined 8 deg from vertical (a sloped-conveyor mounting) and one grease port on the rim at a fixed
azimuth.  The bearing is one free body with a true spherical outer band (sphere radius 10 mm, 7 mm wide, 8 mm bore),
two lubrication holes 180 deg apart and a red index dot at the primary hole.  Product class: a radial insert ball
bearing with a spherical outside surface for a matched spherical housing seat (ISO 9628 / UC series), 4-7 N press.
Sizes are from the product class and the brief (see REFERENCE.md); no size is scaled down.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"

# -- product numbers (metres) -------------------------------------------------------------------------------------------
BALL_R = 0.010                 # spherical OD radius = 20 mm / 2 (true size)
BEARING_W = 0.007              # axial width
BORE_R = 0.004                 # 8 mm shaft bore
SEAT_TILT = math.radians(8.0)  # seat-axis inclination, leans +x
PORT_PSI = math.radians(60.0)  # grease-port azimuth about the seat axis from the minimal-tilt index direction
SEAT_CZ = 0.024                # bearing centre z when seated flush (housing local frame)
SEAT_R_IN = 0.0103             # concave seat radius (0.3 mm radial clearance over the 20 mm ball)
SEAT_R_OUT = 0.0133            # seat shell outer radius

# -- orientation targets (same formulas as the task file) ---------------------------------------------------------------
def _axis_angle(axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(ang), math.sin(ang)
    C = 1.0 - c
    return np.array([[c + x*x*C, x*y*C - z*s, x*z*C + y*s],
                     [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
                     [z*x*C - y*s, z*y*C + x*s, c + z*z*C]])


_TILT = _axis_angle([0, 1, 0], SEAT_TILT)
_SEAT_AXIS = _TILT @ np.array([0.0, 0.0, 1.0])
_SEAT_ROT = _axis_angle(_SEAT_AXIS, PORT_PSI) @ _TILT
_GREASE_DIR = _SEAT_ROT @ np.array([1.0, 0.0, 0.0])

# -- colours ------------------------------------------------------------------------------------------------------------
ALU = (0.60, 0.63, 0.66)
CAST = (0.22, 0.24, 0.27)
DARK = (0.10, 0.11, 0.12)
STEEL = (0.70, 0.72, 0.75)
BEARING_C = (0.82, 0.84, 0.87)
RED = (0.85, 0.14, 0.12)
GREEN = (0.12, 0.60, 0.22)


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
        """A mesh from quads (each a list of vertex indices), emitted as triangles."""
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


def _spherical_seat():
    """Concave spherical seat shell: inner radius r_in, outer r_out, lower hemisphere, axis tilted SEAT_TILT."""
    M, NR = 64, 14
    phis = np.linspace(0.0, math.pi / 2.0, NR)
    angles = np.linspace(0.0, 2 * math.pi, M, endpoint=False)
    pts = []

    def oi(k, j):
        return 2 * M * k + j

    def ii(k, j):
        return 2 * M * k + M + j

    # two hemispherical surfaces, in the untilted seat frame (axis = local z, opening faces +z)
    for r in (SEAT_R_IN, SEAT_R_OUT):
        for phi in phis:
            rad, z = r * math.sin(phi), -r * math.cos(phi)
            for t in angles:
                pts.append((rad * math.cos(t), rad * math.sin(t), z))

    quads = []
    for k in range(NR - 1):
        for j in range(M):
            jn = (j + 1) % M
            quads.append([oi(k, j), oi(k, jn), oi(k + 1, jn), oi(k + 1, j)])      # inner surface
            quads.append([ii(k, jn), ii(k, j), ii(k + 1, j), ii(k + 1, jn)])      # outer surface
    for j in range(M):  # rim annulus at the equator
        jn = (j + 1) % M
        quads.append([oi(NR - 1, j), oi(NR - 1, jn), ii(NR - 1, jn), ii(NR - 1, j)])
        quads.append([ii(0, j), ii(0, jn), oi(0, jn), oi(0, j)])                  # close the bottom pole

    c = np.array([0.0, 0.0, SEAT_CZ])
    world = [_TILT @ (np.asarray(p)) + c for p in pts]
    return world, quads


def build_housing():
    """Pressed pillow-block-style housing: base, cast boss, inclined spherical seat, one grease port, staging pocket."""
    a = Asset("housing.usda", "Housing",
              "pillow-block-style bearing housing: inclined concave spherical seat, grease port, staging pocket",
              kinematic=True)
    # bench plate 90 x 60 x 8 mm
    a.box((0, 0, 0.004), (0.090, 0.060, 0.008), ALU, metal=0.5, rough=0.4, name="BasePlate")
    a.box((0, 0, 0.0085), (0.092, 0.062, 0.001), DARK, name="TopPlate")
    # cast housing boss 56 x 48 x 12 mm (z 0.008 .. 0.020); the seat mouth and grease port stand proud of it
    a.box((0, 0, 0.014), (0.056, 0.048, 0.012), CAST, metal=0.2, rough=0.6, name="HousingBoss")
    # the concave spherical seat, axis inclined 8 deg
    pts, quads = _spherical_seat()
    a.mesh("SphericalSeat", pts, quads, DARK, metal=0.4, rough=0.5)
    # seat rim band: an annular ring at the mouth, tilted with the seat, bright for legibility
    M = 64
    ring_pts, ring_quads = [], []
    rim_r = SEAT_R_OUT
    for k, r in enumerate((rim_r - 0.0012, rim_r + 0.0006)):
        for t in np.linspace(0, 2 * math.pi, M, endpoint=False):
            v = np.array([r * math.cos(t), r * math.sin(t), 0.0])
            ring_pts.append(_TILT @ v + np.array([0.0, 0.0, SEAT_CZ]))
    for j in range(M):
        jn = (j + 1) % M
        ring_quads.append([j, jn, M + jn, M + j])
    a.mesh("SeatRim", ring_pts, ring_quads, STEEL, metal=0.85, rough=0.3)

    # grease port at the fixed azimuth on the rim (the bearing's primary grease hole must register here)
    gp = np.array([0.0, 0.0, SEAT_CZ]) + (SEAT_R_OUT + 0.0004) * _GREASE_DIR
    a.cyl(tuple(gp), 0.0022, 0.0022, GREEN, metal=0.3, name="GreasePort")
    # the opposite, blank side of the rim (the bearing's second hole is 180 deg away)
    bp = np.array([0.0, 0.0, SEAT_CZ]) - (SEAT_R_OUT + 0.0004) * _GREASE_DIR
    a.cyl(tuple(bp), 0.0012, 0.0018, DARK, metal=0.3, name="BlankRim")

    # staging pocket for the upright bearing on the robot's +y side
    a.box((0.0, 0.062, 0.008), (0.032, 0.032, 0.004), DARK, name="PocketFloor")
    for dy in (-0.016, 0.016):
        a.box((0.0, 0.062 + dy, 0.012), (0.032, 0.0016, 0.008), STEEL, name="PocketWall")
    for dx in (-0.016, 0.016):
        a.box((dx, 0.062, 0.012), (0.0016, 0.032, 0.008), STEEL, name="PocketWall")
    a.save()


def _spherical_band():
    """Points/quads for the spherical-OD bearing band: outer radius sqrt(R^2 - z^2), cylindrical bore."""
    M, NZ = 64, 10
    zs = np.linspace(-BEARING_W / 2, BEARING_W / 2, NZ)
    angles = np.linspace(0.0, 2 * math.pi, M, endpoint=False)
    pts = []

    def oi(k, j):
        return 2 * M * k + j

    def ii(k, j):
        return 2 * M * k + M + j

    for z in zs:
        ro = math.sqrt(BALL_R ** 2 - z ** 2)
        for t in angles:
            pts.append((ro * math.cos(t), ro * math.sin(t), z))
        for t in angles:
            pts.append((BORE_R * math.cos(t), BORE_R * math.sin(t), z))
    quads = []
    for k in range(NZ - 1):
        for j in range(M):
            jn = (j + 1) % M
            quads.append([oi(k, j), oi(k, jn), oi(k + 1, jn), oi(k + 1, j)])      # outer spherical surface
            quads.append([ii(k, jn), ii(k, j), ii(k + 1, j), ii(k + 1, jn)])      # inner bore
    for j in range(M):
        jn = (j + 1) % M
        quads.append([oi(NZ - 1, j), oi(NZ - 1, jn), ii(NZ - 1, jn), ii(NZ - 1, j)])  # top face
        quads.append([ii(0, j), ii(0, jn), oi(0, jn), oi(0, j)])                     # bottom face
    return pts, quads


def build_bearing():
    """The free 20 mm spherical-OD bearing: spherical outer band, 8 mm bore, two grease holes and a red index dot."""
    a = Asset("bearing.usda", "Bearing",
              "20 mm spherical-OD insert bearing: spherical outer ring, 8 mm bore, two grease holes 180 deg apart",
              mass=0.030, no_gravity=True)
    pts, quads = _spherical_band()
    a.mesh("SphericalOD", pts, quads, BEARING_C, collide=False, metal=0.8, rough=0.28)
    # a plain cylindrical collider (the visual outer band is convex; this stands in for it)
    a.cyl((0, 0, 0), 0.0093, BEARING_W, None, collide=True, name="BearingCollider")
    # bore sleeve: dark ring so the bore is legible from the top camera
    zs = np.linspace(-BEARING_W / 2 - 0.0003, BEARING_W / 2 + 0.0003, 2)
    bpts, bquads, M = [], [], 64
    for z in zs:
        for t in np.linspace(0, 2 * math.pi, M, endpoint=False):
            bpts.append(((BORE_R + 0.0004) * math.cos(t), (BORE_R + 0.0004) * math.sin(t), z))
    for j in range(M):
        jn = (j + 1) % M
        bquads.append([j, jn, M + jn, M + j])
    a.mesh("BoreSleeve", bpts, bquads, DARK, metal=0.6, rough=0.4)
    # primary grease hole + red index dot at local +x on the equator; secondary hole 180 deg away
    ro_eq = math.sqrt(BALL_R ** 2 - 0.0)
    a.cyl((ro_eq + 0.0001, 0, 0), 0.0014, 0.0008, RED, axis="X", name="IndexDot")
    a.cyl((ro_eq - 0.0006, 0, 0), 0.0009, 0.0010, DARK, axis="X", name="GreaseHole")
    a.cyl((-ro_eq - 0.0001, 0, 0), 0.0009, 0.0010, DARK, axis="X", name="GreaseHole2")
    a.save()


if __name__ == "__main__":
    build_housing()
    build_bearing()
