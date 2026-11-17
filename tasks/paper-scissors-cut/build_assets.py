"""Writes this task's USD assets with pxr (boxes, an extruded loop, convex blade meshes; nothing from outside).
Run:  local_sim/.venv/bin/python build_assets.py

Frames: metres, kilograms, Z up.  The paper lies at table top + 21 mm (a 20 mm riser and a 1 mm cutting mat); the
scissors' local origin is the rivet (pivot), +X toward the blade tips, +Z up.  All product dimensions are the real
ones (see README.md / REFERENCE.md); values marked EST are estimates.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001
DEG = math.pi / 180.0
TOP = 0.7475

# -- the real products -----------------------------------------------------------------------------------------------
# Fiskars 8 in (203 mm) office scissors, orange plastic handles, stainless-steel blades, 60 g (Fiskars 01-004761).
SCISSOR_LEN = 203 * MM
BLADE_LEN = 90 * MM          # the cutting edge from the rivet to the tip (EST from the 8 in class)
BLADE_HALF_W = 18 * MM       # the blade's width at the spine (EST)
HANDLE_T = 10 * MM           # the handle loops' thickness (the gripper pads clamp their outer faces)
HANDLE_CX = -75 * MM         # the loop centre, measured back from the rivet
HANDLE_CY = 16 * MM          # the loop centre's offset to its side of the centreline
LOOP_A, LOOP_B = 48 * MM, 10 * MM       # outer ellipse (x, y)
HOLE_A, HOLE_B = 34 * MM, 5 * MM        # the finger hole
SCISSOR_MASS = 0.060         # kg (Fiskars 8 in = 2.1 oz / 60 g)
ROOT_MASS, CHILD_MASS = 0.032, 0.028

# 80 gsm A4 paper: 210 x 297 mm, 0.10 mm thick, 4.99 g.  The cut takes a 100 mm strip off one long side.
PAPER_X, PAPER_Y, PAPER_T = 210 * MM, 297 * MM, 0.10 * MM
OFFCUT_Y = 100 * MM
PAPER_GSM = 80.0
MAIN_MASS = PAPER_GSM * PAPER_X * (PAPER_Y - OFFCUT_Y)     # 3.31 g
OFFCUT_MASS = PAPER_GSM * PAPER_X * OFFCUT_Y               # 1.68 g
PAPER_Z = TOP + 20 * MM + 1 * MM + PAPER_T / 2               # 0.76855: on the 20 mm riser + 1 mm mat

# -- layout (world, m; x forward, y to the robot's left) --------------------------------------------------------------
CUT_Y = -0.100              # the printed cut line
PAPER_MAIN_XY = (0.465, CUT_Y + (PAPER_Y - OFFCUT_Y) / 2)       # (0.465, -0.0515)
PAPER_OFFCUT_XY = (0.465, CUT_Y - OFFCUT_Y / 2)                  # (0.465, -0.200)
PAPER_X0, PAPER_X1 = 0.360, 0.570
REST_XY = (0.330, -0.465)   # the scissors' rivet where they start and end, on the grey rest mat

BROWN = (0.52, 0.36, 0.20)
MAT_GREEN = (0.10, 0.30, 0.20)
MAT_EDGE = (0.06, 0.20, 0.13)
TRAY_GREY = (0.30, 0.33, 0.36)
TRAY_DARK = (0.18, 0.20, 0.22)
PAPER_WHITE = (0.95, 0.95, 0.92)
PAPER_EDGE = (0.82, 0.82, 0.78)
RED = (0.72, 0.06, 0.05)
BLACK = (0.05, 0.05, 0.05)
ORANGE = (0.91, 0.34, 0.05)
STEEL = (0.62, 0.64, 0.67)
STEEL_DARK = (0.36, 0.38, 0.41)
ZINC = (0.72, 0.73, 0.74)


class Asset:
    def __init__(self, fname, name, doc, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True,
                 articulation_root=False):
        self.path = OUT / fname
        self.stage = Usd.Stage.CreateInMemory()
        s = self.stage
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(s, 1.0)
        self.root = UsdGeom.Xform.Define(s, f"/{name}")
        s.SetDefaultPrim(self.root.GetPrim())
        p = self.root.GetPrim()
        p.SetMetadata("documentation", doc)
        self.name = name
        self.mats = {}
        self.n = 0
        self.cur = f"/{name}"
        if body:
            UsdPhysics.RigidBodyAPI.Apply(p)
            m = UsdPhysics.MassAPI.Apply(p)
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        if articulation_root:
            UsdPhysics.ArticulationRootAPI.Apply(p)
            p.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)

    # -- looks -------------------------------------------------------------------------------------------------------
    def color(self, key, rgb, rough=0.55, metallic=0.0, opacity=1.0):
        k = f"{key}_{int(rgb[0]*255):02x}{int(rgb[1]*255):02x}{int(rgb[2]*255):02x}_{int(rough*100):02d}_{int(opacity*100):03d}"
        if k in self.mats:
            return self.mats[k]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{k}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{k}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[k] = mat
        return mat

    def phys_mat(self, key, static, dynamic):
        k = "Phys_" + key
        if k in self.mats:
            return self.mats[k]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{k}")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        self.mats[k] = mat
        return mat

    # -- prims -------------------------------------------------------------------------------------------------------
    def _xf(self, g, center, rot=(0.0, 0.0, 0.0)):
        g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
        if rot[0]:
            g.AddRotateXOp().Set(float(rot[0]))
        if rot[1]:
            g.AddRotateYOp().Set(float(rot[1]))
        if rot[2]:
            g.AddRotateZOp().Set(float(rot[2]))

    def _finish(self, g, rgb, collide, key="c", name=None, opacity=1.0, rough=0.55, metal=0.0):
        prim = g.GetPrim()
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.001)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
            pm = self.phys_mat(key, *collide)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
            if rgb is None:
                g.CreatePurposeAttr("guide")
        if rgb is not None:
            mat = self.color(key, rgb, rough=rough, metallic=metal, opacity=opacity)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def box(self, center, size, rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0), opacity=1.0, rough=0.55):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center, rot)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key, name, opacity, rough)
        return g

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0),
            opacity=1.0, rough=0.55, metal=0.0):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center, rot)
        self._finish(g, rgb, collide, key, name, opacity, rough, metal)
        return g

    def mesh(self, pts, tris, rgb=None, collide=None, key="c", name="Mesh", opacity=1.0, rough=0.55, metal=0.0,
             approximation="convexHull"):
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        P = np.asarray(pts, float)
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in p]) for p in P]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for tr in tris for i in tr])
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*P.min(0)), Gf.Vec3f(*P.max(0))]))
        prim = g.GetPrim()
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(prim)
            if approximation:
                prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token, custom=False).Set(approximation)
            pm = self.phys_mat(key, *collide)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
        if rgb is not None:
            mat = self.color(key, rgb, rough=rough, metallic=metal, opacity=opacity)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        return g

    def link(self, lname, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), pos=(0, 0, 0)):
        path = f"/{self.name}/{lname}"
        x = UsdGeom.Xform.Define(self.stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        p = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(p)
        m = UsdPhysics.MassAPI.Apply(p)
        m.CreateMassAttr(mass)
        m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        self.cur = path
        self.n = 0
        return p

    def revolute(self, name, b0, b1, axis="Z", lo=-180.0, hi=180.0, stiffness=0.0, damping=0.0, target=0.0,
                 p0=(0, 0, 0), p1=(0, 0, 0), armature=1e-6, max_force=None):
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.RevoluteJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(stiffness))
        d.CreateDampingAttr(float(damping))
        d.CreateTargetPositionAttr(float(target))
        if max_force is not None:
            d.CreateMaxForceAttr(float(max_force))
        j.GetPrim().AddAppliedSchema("PhysxJointAPI")
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(float(armature))
        return j

    def fixed_joint(self, name, b0, b1, p0=(0, 0, 0), p1=(0, 0, 0)):
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.FixedJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        return j

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path, self.path.stat().st_size, "bytes")


def loop_mesh(cx, cy, a, b, ai, bi, z0, z1, n=32):
    """An extruded elliptical annulus (a handle loop) about z; points, triangles."""
    pts = []
    for z in (z0, z1):
        for i in range(n):
            t = 2 * math.pi * i / n
            pts.append((cx + a * math.cos(t), cy + b * math.sin(t), z))
        for i in range(n):
            t = 2 * math.pi * i / n
            pts.append((cx + ai * math.cos(t), cy + bi * math.sin(t), z))
    off = 4 * n
    tris = []
    for i in range(n):
        j = (i + 1) % n
        ob0, ob1, ot0, ot1 = i, j, n + i, n + j
        ib0, ib1, it0, it1 = 2 * n + i, 2 * n + j, 3 * n + i, 3 * n + j
        tris += [(ot0, ot1, it1), (ot0, it1, it0)]          # top
        tris += [(ob0, ib1, ob1), (ob0, ib0, ib1)]          # bottom
        tris += [(ob0, ob1, ot1), (ob0, ot1, ot0)]          # outer wall
        tris += [(ib0, it1, ib1), (ib0, it0, it1)]          # inner wall
    return pts, tris


def blade_mesh(side, z0, z1):
    """One blade's convex wedge: side +1 (upper, +Y) or -1 (lower, -Y). Tip at x=BLADE_LEN."""
    L, W = BLADE_LEN, BLADE_HALF_W
    # cutting edge at y=0, spine at y=side*W; the edge is 0.7 mm thick, the spine the full z band
    ze0, ze1 = (z0 + 0.35 * (z1 - z0), z1 - 0.35 * (z1 - z0))
    pts = [
        (0.0, 0.0, ze0), (0.0, 0.0, ze1), (0.0, side * W, z0), (0.0, side * W, z1),
        (L, side * 0.002, ze0), (L, side * 0.002, ze1),
    ]
    # a small shoulder so the hull does not round the spine away
    pts += [(L * 0.55, side * W, z0), (L * 0.55, side * W, z1)]
    # convex hull triangulation: build by hand from the 8 points (two z layers)
    tris = [
        (0, 2, 3), (0, 3, 1),          # root face
        (4, 5, 3), (4, 3, 2),          # tip face
        (0, 1, 5), (0, 5, 4),          # cutting-edge face
        (2, 6, 7), (2, 7, 3),          # spine face
        (0, 4, 6), (0, 6, 2),          # bottom
        (1, 3, 7), (1, 7, 5),          # top
    ]
    return pts, tris


# ======================================================================================================================
# scissors.usda: a Fiskars-class 203 mm office scissors as a two-link articulation about the rivet.
# ======================================================================================================================
def scissors(fname):
    a = Asset(fname, "ScissorA",
              "A 203 mm office scissors (Fiskars 8 in class, 60 g): stainless-steel blades and orange plastic loop "
              "handles on a rivet at the origin. Link ScissorA is the upper blade and its handle; ScissorB is the lower "
              "blade and its handle, on a Z revolute 'Shear' with a light torsional spring (it opens). +X points to the "
              "blade tips; the paper plane is z=0. The robot clamps the two handles' outer faces with one gripper.",
              ROOT_MASS, com=(-0.040, -0.004, 0.0), inertia=(1.4e-4, 1.4e-4, 2.0e-5), articulation_root=True)
    root = a.stage.GetPrimAtPath("/ScissorA")
    a.cur = "/ScissorA"
    # upper blade (+Y side) and its handle (-Y side)
    pts, tris = blade_mesh(+1, 0.0, 0.0020)
    a.mesh(pts, tris, STEEL, (0.7, 0.6), key="steel", name="BladeA")
    a.box((0.004, 0.0010, 0.0010), (0.010, 0.0040, 0.0022), STEEL_DARK, None, key="steel", name="BladeRootA")
    # neck and loop: the loop's outer face is the gripper's contact
    a.box((HANDLE_CX / 2 - 0.010, -0.006, 0.0), (0.090, 0.012, HANDLE_T), ORANGE, (0.9, 0.8), key="plastic",
          name="HandleNeckA", rough=0.45)
    pts, tris = loop_mesh(HANDLE_CX, -HANDLE_CY, LOOP_A, LOOP_B, HOLE_A, HOLE_B, -HANDLE_T / 2, HANDLE_T / 2)
    a.mesh(pts, tris, ORANGE, (0.9, 0.8), key="plastic", name="HandleLoopA", rough=0.45)
    a.box((HANDLE_CX, -HANDLE_CY - LOOP_B - 0.0015, 0.0), (0.020, 0.003, 2 * LOOP_B),
          ORANGE, (0.9, 0.8), key="plastic", name="GripPlateA", rough=0.45)
    a.box((HANDLE_CX, -HANDLE_CY - LOOP_B - 0.0030, -HANDLE_T / 2 + 0.0005), (0.020, 0.003, 0.001),
          ORANGE, (0.9, 0.8), key="plastic", name="GripLipA", rough=0.45)
    # the rivet (visual; the articulation's hinge carries the load)
    a.cyl((0.0, 0.0, 0.0), 0.0065, 0.010, axis="Z", rgb=ZINC, collide=None, key="steel", name="Rivet", metal=0.8)

    bp = a.link("ScissorB", CHILD_MASS, com=(-0.040, 0.004, 0.0), inertia=(1.4e-4, 1.4e-4, 2.0e-5))
    a.cur = "/ScissorA/ScissorB"
    pts, tris = blade_mesh(-1, -0.0020, 0.0)
    a.mesh(pts, tris, STEEL, (0.7, 0.6), key="steel", name="BladeB")
    a.box((0.004, -0.0010, -0.0010), (0.010, 0.0040, 0.0022), STEEL_DARK, None, key="steel", name="BladeRootB")
    a.box((HANDLE_CX / 2 - 0.010, 0.006, 0.0), (0.090, 0.012, HANDLE_T), ORANGE, (0.9, 0.8), key="plastic",
          name="HandleNeckB", rough=0.45)
    pts, tris = loop_mesh(HANDLE_CX, HANDLE_CY, LOOP_A, LOOP_B, HOLE_A, HOLE_B, -HANDLE_T / 2, HANDLE_T / 2)
    a.mesh(pts, tris, ORANGE, (0.9, 0.8), key="plastic", name="HandleLoopB", rough=0.45)
    a.box((HANDLE_CX, HANDLE_CY + LOOP_B + 0.0015, 0.0), (0.020, 0.003, 2 * LOOP_B),
          ORANGE, (0.9, 0.8), key="plastic", name="GripPlateB", rough=0.45)
    a.box((HANDLE_CX, HANDLE_CY + LOOP_B + 0.0030, -HANDLE_T / 2 + 0.0005), (0.020, 0.003, 0.001),
          ORANGE, (0.9, 0.8), key="plastic", name="GripLipB", rough=0.45)

    # hinge about the rivet; a light spring opens it (the gripper squeezes it shut).  USD angular drives are in
    # degrees: 0.40 N m/rad = 0.00698 N m/deg, and the -0.35 rad open stop is -20.05 deg.
    a.revolute("Shear", root, bp, axis="Z", lo=-24.0, hi=1.2, stiffness=0.40 * DEG, damping=0.02 * DEG,
               target=-0.35 / DEG, p0=(0, 0, 0), p1=(0, 0, 0), armature=2.0e-5, max_force=3.0)
    a.save()


# ======================================================================================================================
# paper_main.usda / paper_offcut.usda: 80 gsm A4, 0.10 mm, split at the printed 100 mm line.
# ======================================================================================================================
def paper_main(fname):
    a = Asset(fname, "PaperMain",
              "The kept part of an 80 gsm A4 sheet: 210 x 197 mm, 0.10 mm, 3.31 g.  The cut edge is the sheet's -Y edge "
              "(local y=-0.0985); the red half of the printed cut line lies along it, and two black 100 mm ticks mark "
              "the cut length.  The sheet's top face is z=+0.00005.",
              MAIN_MASS, com=(0, 0, 0), inertia=(1.1e-6, 1.6e-6, 2.7e-6))
    a.cur = "/PaperMain"
    # collision-free: the 0.10 mm sheet is not a resolvable contact on either backend; the mat holds it
    a.box((0, 0, 0), (PAPER_X, PAPER_Y - OFFCUT_Y, PAPER_T), PAPER_WHITE, (0.42, 0.38), name="Sheet",
          rough=0.9)
    # a thin printed line along the cut edge (visual only)
    a.box((0.0, -(PAPER_Y - OFFCUT_Y) / 2 + 0.00075, PAPER_T / 2 + 0.00001),
          (PAPER_X - 0.004, 0.0015, 0.00002), RED, None, name="CutLine")
    # 100 mm ticks at the two ends of the cut, on the kept side
    for sx in (-1, 1):
        a.box((sx * 0.052, -(PAPER_Y - OFFCUT_Y) / 2 + 0.0035, PAPER_T / 2 + 0.00001),
              (0.0012, 0.006, 0.00002), BLACK, None, name="Tick")
    # the four corners a little darker so the sheet reads on the mat
    for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
        a.box((sx * (PAPER_X / 2 - 0.004), sy * ((PAPER_Y - OFFCUT_Y) / 2 - 0.004), PAPER_T / 2 + 0.000005),
              (0.005, 0.005, 0.00001), PAPER_EDGE, None, name="Corner")
    a.save()


def paper_offcut(fname):
    a = Asset(fname, "PaperOffcut",
              "The 100 mm offcut of an 80 gsm A4 sheet: 210 x 100 mm, 0.10 mm, 1.68 g.  The cut edge is the sheet's +Y "
              "edge (local y=+0.050); the other half of the red printed line lies along it.",
              OFFCUT_MASS, com=(0, 0, 0), inertia=(0.55e-6, 0.18e-6, 0.70e-6))
    a.cur = "/PaperOffcut"
    # collision-free: its pose is servoed by the rigid seam bond, then lowered into the tray on release
    a.box((0, 0, 0), (PAPER_X, OFFCUT_Y, PAPER_T), PAPER_WHITE, (0.42, 0.38), name="Sheet", rough=0.9)
    a.box((0.0, OFFCUT_Y / 2 - 0.00075, PAPER_T / 2 + 0.00001), (PAPER_X - 0.004, 0.0015, 0.00002),
          RED, None, name="CutLine")
    for sx, sy in ((-1, -1), (1, -1)):
        a.box((sx * (PAPER_X / 2 - 0.004), sy * (OFFCUT_Y / 2 - 0.004), PAPER_T / 2 + 0.000005),
              (0.005, 0.005, 0.00001), PAPER_EDGE, None, name="Corner")
    a.save()


# ======================================================================================================================
# fixtures.usda: the 20 mm riser + cutting mat, the shallow letter tray and the scissors' rest mat.
# ======================================================================================================================
def fixtures(fname):
    a = Asset(fname, "Fixtures",
              "The paper station: a 20 mm wooden riser with a 1 mm green cutting mat (top 0.7685), a shallow grey "
              "letter tray beside it (rim 0.7600, floor 0.7495) that catches the 100 mm offcut, and a 1 mm grey rest "
              "mat for the scissors (top 0.7485).  All fixed to the table; origin at the world origin.",
              4.0, com=(0.45, -0.10, 0.77), inertia=(0.05, 0.05, 0.05))
    a.cur = "/Fixtures"
    # riser + mat
    # the riser/mat stop 42 mm short of the printed line: the right gripper's pads and the upper blade must clear them
    a.box((0.465, 0.0175, TOP + 10 * MM), (0.250, 0.235, 20 * MM), BROWN, (0.7, 0.6), key="wood", name="Riser")
    a.box((0.465, 0.0160, TOP + 20.5 * MM), (0.244, 0.232, 1 * MM), MAT_GREEN, (0.8, 0.7), key="mat", name="CutMat")
    a.box((0.465, -0.100, TOP + 20.5 * MM), (0.244, 0.001, 0.0002), MAT_EDGE, None, name="MatEdge")
    # printed paper outline on the mat, 1 mm outside the sheet
    for name, cx, cy, sx, sy in (
            ("PaperEdgeL", PAPER_X0 - 0.002, -0.0515, 0.0008, 0.204),
            ("PaperEdgeR", PAPER_X1 + 0.002, -0.0515, 0.0008, 0.204),
            ("PaperEdgeFar", 0.465, 0.049, 0.214, 0.0008)):
        a.box((cx, cy, TOP + 21.02 * MM), (sx, sy, 0.0002), MAT_EDGE, None, name=name)
    # letter tray: floor and four 12.5 mm walls; inner opening x 0.318..0.722, y -0.356..-0.094.  The near wall
    # sits just past the mat edge, under the sheet's 12 mm overhang, so the whole 100 mm offcut is over the opening.
    a.box((0.520, -0.275, TOP + 3 * MM), (0.410, 0.274, 6 * MM), TRAY_DARK, (0.6, 0.5), key="plastic", name="TrayFloor")
    for name, cx, cy, sx, sy in (
            ("TrayNear", 0.520, -0.142, 0.410, 0.004),
            ("TrayFar", 0.520, -0.410, 0.410, 0.004),
            ("TrayLeft", 0.316, -0.275, 0.004, 0.276),
            ("TrayRight", 0.724, -0.275, 0.004, 0.276)):
        a.box((cx, cy, TOP + 6.25 * MM), (sx, sy, 12.5 * MM), TRAY_GREY, (0.6, 0.5), key="plastic", name=name)
    for sx in (-1, 1):
        for sy in (-1, 1):
            a.box((0.520 + sx * 0.202, -0.275 + sy * 0.134, TOP + 12.5 * MM), (0.010, 0.010, 0.001),
                  TRAY_DARK, None, name="TrayRim")
    # the scissors' rest: a 30 mm riser and its grey mat (top 0.7785), clear of the tray
    a.box((REST_XY[0], REST_XY[1], TOP + 15 * MM), (0.240, 0.120, 30 * MM), BROWN, (0.7, 0.6), key="wood",
          name="RestRiser")
    a.box((REST_XY[0], -0.480, TOP + 30.5 * MM), (0.244, 0.124, 1 * MM), (0.22, 0.24, 0.26), (0.8, 0.7),
          key="mat", name="RestMat")
    # a printed scissor silhouette: a 203 mm outline along x
    for sx in (-1, 1):
        a.box((REST_XY[0] + sx * (SCISSOR_LEN / 2 - 0.004), -0.480 - 0.036, TOP + 31.02 * MM),
              (0.002, 0.072, 0.0002), (0.85, 0.85, 0.80), None, name="RestEdge")
    a.box((REST_XY[0], -0.480, TOP + 31.02 * MM), (SCISSOR_LEN - 0.008, 0.0015, 0.0002),
          (0.85, 0.85, 0.80), None, name="RestCentre")
    a.save()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    scissors("scissors.usda")
    paper_main("paper_main.usda")
    paper_offcut("paper_offcut.usda")
    fixtures("fixtures.usda")


if __name__ == "__main__":
    main()
