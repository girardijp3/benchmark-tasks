"""Writes this task's USD assets with pxr (boxes, cylinders and generated meshes; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A real 15 mm eccentric furniture cam, its M6 connecting bolt and a screwdriver, with two 16 mm melamine particleboard
panels.  The face panel carries a 15 mm blind cam bore 24 mm in from its joining edge (Minifix drilling distance B=24);
the mating panel carries an 8 mm bolt hole in that edge and starts 3 mm short of the face panel.  Sizes come from the
product class and the brief (see REFERENCE.md); no size is scaled down.  The two panels are visual-only (no colliders).
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"

# -- product numbers (metres) -------------------------------------------------------------------------------------------
CAM_R = 0.0075                 # 15 mm cam diameter / 2
CAM_H = 0.012
BOLT_R = 0.003                 # M6 shank
BOLT_HEAD_R = 0.0035           # 7 mm head for an 8 mm hole
BOLT_HOLE_R = 0.004
PANEL_T = 0.016                # 16 mm particleboard

# fixture-local frame: origin at the face panel's centre, bench top z = 0
PANEL_A_SIZE = (0.160, 0.140, PANEL_T)
CAM_BORE_LOCAL = (0.0, -0.046, 0.009)   # CAM_XY - PANEL_A_XY = (0, -0.046)
CAM_STAGE_LOCAL = (0.0, 0.035, 0.010)
BOLT_STAGE_LOCAL = (-0.040, 0.105, 0.006)
SCREW_STAGE_LOCAL = (0.075, 0.105, 0.030)
BOLT_HOLE_LOCAL = (0.0, 0.050, 0.008)   # in panel B's far (+y) edge

# -- colours ------------------------------------------------------------------------------------------------------------
BOARD = (0.90, 0.88, 0.83)
EDGE = (0.78, 0.75, 0.68)
DARK = (0.10, 0.11, 0.12)
STEEL = (0.70, 0.72, 0.75)
ZINC = (0.82, 0.84, 0.87)
RED = (0.85, 0.14, 0.12)
GREEN = (0.12, 0.60, 0.22)
HANDLE = (0.16, 0.18, 0.22)


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


def build_fixture():
    """Face panel with its 15 mm cam bore, cam staging pocket, bolt staging pad and screwdriver holder."""
    a = Asset("fixture.usda", "Fixture",
              "16 mm face panel: 15 mm cam bore 24 mm from the joining edge, staging pocket and screwdriver holder",
              kinematic=True)
    a.box((0, 0, PANEL_T / 2), PANEL_A_SIZE, BOARD, rough=0.8, name="FacePanel")
    a.box((0, CAM_BORE_LOCAL[1], PANEL_T + 0.0002), (0.036, 0.010, 0.0004), GREEN, name="EdgeArrow")
    # the 15 mm blind bore (dark void) and a bright steel rim ring at the mouth
    a.cyl((CAM_BORE_LOCAL[0], CAM_BORE_LOCAL[1], 0.009), CAM_R, 0.013, DARK, name="CamBore")
    M = 48
    pts, quads = [], []
    for r in (CAM_R - 0.0008, CAM_R + 0.0008):
        for t in np.linspace(0, 2 * math.pi, M, endpoint=False):
            pts.append((r * math.cos(t), CAM_BORE_LOCAL[1] + r * math.sin(t), PANEL_T + 0.0001))
    for j in range(M):
        jn = (j + 1) % M
        quads.append([j, jn, M + jn, M + j])
    a.mesh("BoreRim", pts, quads, STEEL, metal=0.8, rough=0.3)
    # staging pocket for the cam, a flat pad for the bolt, and a screwdriver holder
    a.box((CAM_STAGE_LOCAL[0], CAM_STAGE_LOCAL[1], 0.004), (0.034, 0.034, 0.004), DARK, name="CamPocket")
    for dy in (-0.017, 0.017):
        a.box((CAM_STAGE_LOCAL[0], CAM_STAGE_LOCAL[1] + dy, 0.008), (0.034, 0.0016, 0.008), STEEL, name="PocketWall")
    for dx in (-0.017, 0.017):
        a.box((CAM_STAGE_LOCAL[0] + dx, CAM_STAGE_LOCAL[1], 0.008), (0.0016, 0.034, 0.008), STEEL, name="PocketWall")
    a.box((BOLT_STAGE_LOCAL[0], BOLT_STAGE_LOCAL[1], 0.003), (0.060, 0.020, 0.003), DARK, name="BoltPad")
    a.box((SCREW_STAGE_LOCAL[0], SCREW_STAGE_LOCAL[1], 0.010), (0.030, 0.030, 0.020), HANDLE, name="ScrewHolder")
    a.cyl((SCREW_STAGE_LOCAL[0], SCREW_STAGE_LOCAL[1], 0.021), 0.014, 0.002, DARK, name="HolderLip")
    a.save()


def build_panel_b():
    """Mating panel: 16 mm board with an 8 mm bolt hole in the joining (+y) edge. Visual-only."""
    a = Asset("panel_b.usda", "MatingPanel",
              "16 mm mating panel: 8 mm bolt hole in the joining edge, slides +y as the cam draws it flush",
              no_gravity=True)
    a.box((0, 0, PANEL_T / 2), (0.160, 0.100, PANEL_T), BOARD, rough=0.8, name="MatingPanelBody")
    a.box((0, BOLT_HOLE_LOCAL[1], PANEL_T + 0.0002), (0.024, 0.008, 0.0004), RED, name="HoleMark")
    a.cyl((0, BOLT_HOLE_LOCAL[1], 0.008), BOLT_HOLE_R, 0.018, DARK, axis="Y", name="BoltHole")
    M = 40
    pts, quads = [], []
    for r in (BOLT_HOLE_R - 0.0006, BOLT_HOLE_R + 0.0008):
        for t in np.linspace(0, 2 * math.pi, M, endpoint=False):
            pts.append((r * math.cos(t), BOLT_HOLE_LOCAL[1] + 0.0002, 0.008 + r * math.sin(t)))
    for j in range(M):
        jn = (j + 1) % M
        quads.append([j, jn, M + jn, M + j])
    a.mesh("HoleRim", pts, quads, STEEL, metal=0.8, rough=0.3)
    a.save()


def build_cam():
    """The free 15 mm eccentric cam: drum, cross recess, side slot for the bolt head and a red index dot."""
    a = Asset("cam.usda", "Cam",
              "15 mm die-cast eccentric furniture cam: cross/PZ drive recess, eccentric bolt-head slot, index dot",
              mass=0.015, no_gravity=True)
    a.cyl((0, 0, 0), CAM_R, CAM_H, ZINC, metal=0.75, rough=0.3, name="CamDrum")
    a.cyl((0, 0, 0), CAM_R - 0.0004, CAM_H * 0.96, None, collide=True, name="CamCollider")
    # cross/PZ recess on the top face (two dark slots crossing)
    a.box((0, 0, CAM_H / 2 + 0.0001), (0.0090, 0.0018, 0.0006), DARK, name="CrossRecessA")
    a.box((0, 0, CAM_H / 2 + 0.0002), (0.0018, 0.0090, 0.0006), DARK, name="CrossRecessB")
    # eccentric slot open toward -y (the bolt edge) at the start orientation
    a.box((0, -CAM_R + 0.0006, 0.0), (0.0060, 0.0030, 0.0065), DARK, name="EccentricSlot")
    a.cyl((0, -CAM_R - 0.0001, 0.0), 0.0011, 0.0008, RED, axis="Y", name="IndexDot")
    a.save()


def build_bolt():
    """The free M6 connecting bolt: threaded shank into the panel, collar, 25 mm projection and 7 mm head."""
    a = Asset("bolt.usda", "Bolt",
              "M6 connecting bolt: 27 mm thread, stop collar, 25 mm projection and a 7 mm eccentric head (B=24)",
              mass=0.010, no_gravity=True)
    # local +z is the shank axis; at the scored pose local +z maps to world +y
    a.cyl((0, 0, -0.0265), BOLT_R, 0.027, STEEL, metal=0.8, rough=0.35, name="Thread")
    a.cyl((0, 0, -0.013), 0.0045, 0.002, STEEL, metal=0.8, rough=0.35, name="StopCollar")
    a.cyl((0, 0, -0.001), 0.0028, 0.022, STEEL, metal=0.8, rough=0.35, name="Projection")
    a.cyl((0, 0, 0.011), BOLT_HEAD_R, 0.004, ZINC, metal=0.8, rough=0.3, name="BoltHead")
    a.box((0.0018, 0, 0.011), (0.0016, 0.006, 0.004), DARK, name="HeadDrive")
    a.cyl((0, 0, -0.0135), 0.0036, 0.053, None, collide=True, name="BoltCollider")
    a.save()


def build_screwdriver():
    """The free PZ2 screwdriver: handle at local +z (grasp side), shaft and tip at local -z."""
    a = Asset("screwdriver.usda", "Screwdriver",
              "hand screwdriver: PZ2/Phillips cross tip at local -z, handle and grasp at local +z",
              mass=0.060, no_gravity=True)
    a.cyl((0, 0, 0.020), 0.012, 0.045, HANDLE, rough=0.7, name="Handle")
    for k in range(4):
        t = 2 * math.pi * k / 4
        a.box((0.009 * math.cos(t), 0.009 * math.sin(t), 0.020), (0.004, 0.004, 0.040), RED, name="HandleFlute")
    a.cyl((0, 0, -0.017), 0.0032, 0.044, STEEL, metal=0.85, rough=0.25, name="Shaft")
    a.cyl((0, 0, -0.043), 0.0034, 0.008, ZINC, metal=0.85, rough=0.25, name="Tip")
    a.cyl((0, 0, -0.003), 0.011, 0.070, None, collide=True, name="ScrewCollider")
    a.save()


if __name__ == "__main__":
    build_fixture()
    build_panel_b()
    build_cam()
    build_bolt()
    build_screwdriver()
