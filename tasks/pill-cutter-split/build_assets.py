"""Writes the task's USD assets with pxr (boxes, cylinders and small convex prisms; no meshes from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up."""
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001


class Asset:
    def __init__(self, fname, name, doc, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True, no_gravity=False):
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
        if body:
            UsdPhysics.RigidBodyAPI.Apply(p)
            m = UsdPhysics.MassAPI.Apply(p)
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
            if no_gravity:
                p.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        self.mats = {}
        self.n = 0
        self.cur = f"/{name}"  # the prim the next shapes go under (a link, in an articulation)

    def link(self, lname, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), pos=(0, 0, 0)):
        """Start a rigid link under the root (articulation): later shapes go in it."""
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
        return p

    def color(self, key, rgb, rough=0.6):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
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

    def _xf(self, g, center, rot_x=0.0):
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        if rot_x:
            g.AddRotateXOp().Set(rot_x)

    def box(self, center, size, rgb=None, collide=None, key="c", name=None):
        """A box (size = x, y, z edge lengths). rgb: visible, coloured; collide=(static, dynamic): a collider."""
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key)

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

    def capsule(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None):
        """A capsule: ``height`` is the straight part between the two hemispheres."""
        self.n += 1
        g = UsdGeom.Capsule.Define(self.stage, f"{self.cur}/{name or 'Capsule'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

    def prism_x(self, poly_yz, x0, x1, rgb, collide=None, key="c", name="Wall"):
        """A convex prism along x: cross-section ``poly_yz`` (list of (y, z), counter-clockwise seen from +x)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        pts = [(x, y, z) for x in (x0, x1) for (y, z) in poly_yz]
        k = len(poly_yz)
        faces, counts = [], []
        for i in range(1, k - 1):  # caps
            faces += [0, i + 1, i]
            faces += [k, k + i, k + i + 1]
            counts += [3, 3]
        for i in range(k):  # sides
            j = (i + 1) % k
            faces += [i, j, k + j, i, k + j, k + i]
            counts += [3, 3]
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        g.CreateFaceVertexCountsAttr(counts)
        g.CreateFaceVertexIndicesAttr(faces)
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        self._finish(g, rgb, collide, key, mesh=True)

    def prism_y(self, poly_xz, y0, y1, rgb, collide=None, key="c", name="Prism"):
        """A convex prism along y: cross-section ``poly_xz`` (list of (x, z), counter-clockwise seen from -y)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        pts = [(x, y, z) for y in (y0, y1) for (x, z) in poly_xz]
        k = len(poly_xz)
        faces, counts = [], []
        for i in range(1, k - 1):
            faces += [0, i, i + 1]
            faces += [k, k + i + 1, k + i]
            counts += [3, 3]
        for i in range(k):
            j = (i + 1) % k
            faces += [i, k + j, j, i, k + i, k + j]
            counts += [3, 3]
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        g.CreateFaceVertexCountsAttr(counts)
        g.CreateFaceVertexIndicesAttr(faces)
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        self._finish(g, rgb, collide, key, mesh=True)

    def _finish(self, g, rgb, collide, key, mesh=False):
        prim = g.GetPrim()
        if collide is not None:
            UsdPhysics.CollisionAPI.Apply(prim)
            if mesh:
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.001)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
            pm = self.phys_mat(key, *collide)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
            if rgb is None:
                g.CreatePurposeAttr("guide")
        if rgb is not None:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.color("m%02d%02d%02d" % tuple(int(c * 99) for c in rgb), rgb))

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path, self.path.stat().st_size, "bytes")


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))




class Asset2(Asset):
    """``Asset`` plus a box turned about z (``yaw`` degrees)."""

    def boxz(self, center, size, yaw, rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        g.AddRotateZOp().Set(yaw)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key)


# -- a typical pill splitter: base with a V cradle, hinged lid with a blade ----------------------------------------------
# Splitter frame: origin on the table at the base's middle; x away from the robot (the hinge is on the robot's side, x = -40),
# y along the cradle (left-right), z up. The lid lies away from the robot when shut and swings up and back over the hinge
# toward the robot (the joint angle is negative when open, stop at -95 deg: past upright, so the lid stays open).
BASE_X, BASE_Y, BASE_H = 80 * MM, 40 * MM, 11 * MM
V_DEPTH, V_HALF = 5.5 * MM, 10 * MM  # cradle: 20 mm wide at the rim, 5.5 mm deep (about 29 degree walls)
HINGE = (-40 * MM, 0.0, BASE_H)  # hinge axis (along y), at the rim height on the robot's side
OPEN_DEG = 95.0
LID_MASS = 0.012
DAMP = 0.6  # N m s/rad: hinge friction
D_DEG = DAMP * math.pi / 180


def splitter():
    a = Asset2("pill_cutter.usda", "PillCutter",
               "A typical pill splitter, 80 x 40 mm, 11 mm tall base with a 20 mm wide V cradle along y (5.5 mm deep) and a hinged "
               "lid (80 x 40 mm, 12 mm clearance, a blade over the cradle's middle, a 24 mm tall, 6 mm thick pull fin standing on its top near the free edge). The hinge "
               "(axis along y, at the rim, on the robot's side) turns 0 shut to -95 deg open with 0.6 N m s/rad of friction. "
               "Origin on the table at the base's middle.", mass=0.03, body=False)
    base_c, lid_c, steel, tab_c = (0.93, 0.93, 0.9), (0.2, 0.55, 0.8), (0.72, 0.74, 0.78), (0.15, 0.4, 0.62)
    fr = (0.5, 0.4)
    a.link("Base", 0.022, com=(0, 0, 6 * MM), inertia=box_inertia(0.022, BASE_X, BASE_Y, BASE_H))
    UsdPhysics.ArticulationRootAPI.Apply(a.stage.GetPrimAtPath("/PillCutter/Base"))  # on the root link (fix_root_link needs a rigid-body root)
    zb = 0.0
    left = [(-40 * MM, 0.0), (0.0, 0.0), (0.0, V_DEPTH), (-V_HALF, BASE_H), (-40 * MM, BASE_H)]
    right = [(0.0, 0.0), (40 * MM, 0.0), (40 * MM, BASE_H), (V_HALF, BASE_H), (0.0, V_DEPTH)]
    a.prism_y(left, -BASE_Y / 2, BASE_Y / 2, base_c, collide=fr, key="abs", name="BaseL")
    a.prism_y(right, -BASE_Y / 2, BASE_Y / 2, base_c, collide=fr, key="abs", name="BaseR")
    a.box((0, 0, V_DEPTH - 0.2 * MM), (60 * MM, 1.2 * MM, 0.4 * MM), rgb=(0.6, 0.6, 0.6))  # the blade slot's line
    for sx in (-30 * MM, 30 * MM):
        for sy in (-14 * MM, 14 * MM):
            a.box((sx, sy, 0.5 * MM), (8 * MM, 8 * MM, 1.0 * MM), rgb=(0.1, 0.1, 0.1))  # rubber feet (visual)
    # lid, in its own frame (origin = hinge axis, closed lid extends +x)
    a.link("Lid", LID_MASS, com=(40 * MM, 0, 8 * MM), inertia=box_inertia(LID_MASS, 0.08, 0.04, 0.016), pos=HINGE)
    a.box((40 * MM, 0, 13.5 * MM), (80 * MM, 40 * MM, 3 * MM), rgb=lid_c, collide=fr, key="abs", name="LidTop")
    for sy in (-19.25 * MM, 19.25 * MM):
        a.box((40 * MM, sy, 6 * MM), (80 * MM, 1.5 * MM, 12 * MM), rgb=lid_c, collide=fr, key="abs", name="LidRail")
    a.box((78.5 * MM, 0, 6 * MM), (3 * MM, 37 * MM, 12 * MM), rgb=lid_c, collide=fr, key="abs", name="LidFront")
    a.box((1.5 * MM, 0, 6 * MM), (3 * MM, 37 * MM, 12 * MM), rgb=lid_c, collide=fr, key="abs", name="LidBack")
    a.box((68 * MM, 0, 27 * MM), (14 * MM, 6 * MM, 24 * MM), rgb=tab_c, collide=fr, key="abs", name="Fin")
    a.box((40 * MM, 0, 3 * MM), (16 * MM, 0.8 * MM, 18 * MM), rgb=steel, name="Blade")  # blade (visual)
    # Pass-through collider for the blade: only the part above the caplet's top when the caplet is in the V cradle
    # (table z 15.5 mm = lid-frame z 4.5 mm with the lid shut), so it never touches the caplet or the halves. The
    # lower 11 mm of the visible blade is the cutting edge that enters the caplet at the cut, left to the declared cut.
    a.box((40 * MM, 0, 8.5 * MM), (16 * MM, 0.8 * MM, 7 * MM), collide=fr, key="abs", name="BladeCollider")
    j = UsdPhysics.RevoluteJoint.Define(a.stage, "/PillCutter/Hinge")
    j.CreateBody0Rel().SetTargets(["/PillCutter/Base"])
    j.CreateBody1Rel().SetTargets(["/PillCutter/Lid"])
    j.CreateAxisAttr("Y")
    j.CreateLocalPos0Attr(Gf.Vec3f(*HINGE))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(-OPEN_DEG)
    j.CreateUpperLimitAttr(1.0)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(D_DEG)
    d.CreateTargetPositionAttr(0.0)
    j.GetPrim().SetMetadata("documentation", "Lid hinge: 0 shut (lid away from the robot), negative opens up and back toward the robot, stop at -95 deg.")
    a.save()


# -- the tablet (a 20 x 9 x 7.5 mm scored caplet) and its half --------------------------------------------------------------
TAB_L, TAB_W, TAB_T = 20 * MM, 9 * MM, 7.5 * MM
TAB_MASS = 0.0018


def tablet(fname, name, length, mass, score_line):
    a = Asset2(fname, name, f"A {length * 1000:g} x 9 x 7.5 mm piece of a scored white caplet, {mass * 1000:g} g. Origin at its centre.",
               mass=mass, inertia=box_inertia(mass, TAB_W, length, TAB_T))
    fr = (0.6, 0.5)
    a.box((0, 0, 0), (TAB_W, length, TAB_T), rgb=(0.96, 0.95, 0.9), collide=fr, key="tab")
    if score_line:
        a.box((0, 0, TAB_T / 2 + 0.05 * MM), (TAB_W - 1 * MM, 0.5 * MM, 0.2 * MM), rgb=(0.55, 0.54, 0.5))
    a.save()


# -- the dosing tray: two 30 mL medicine cups (fixtures) ----------------------------------------------------------------------
CUP_R_IN, CUP_WALL, CUP_H = 19 * MM, 2 * MM, 30 * MM
TRAY_X, TRAY_Y, TRAY_T = 56 * MM, 120 * MM, 4 * MM
CUP_Y = 30 * MM


def tray():
    a = Asset2("cup_tray.usda", "CupTray",
               "A dosing tray: a 56 x 120 x 4 mm base holding two 30 mL medicine cups (38 mm inside, 30 mm tall), 60 mm apart "
               "along y. Origin on the table at the tray's middle. Fixed.", mass=0.2)
    fr = (0.4, 0.3)
    a.box((0, 0, TRAY_T / 2), (TRAY_X, TRAY_Y, TRAY_T), rgb=(0.85, 0.86, 0.88), collide=fr, key="tray")
    for sy, rgb in ((-CUP_Y, (0.95, 0.75, 0.3)), (CUP_Y, (0.4, 0.55, 0.85))):
        a.cyl((0, sy, TRAY_T + 1.0 * MM), CUP_R_IN + CUP_WALL, 2.0 * MM, rgb=rgb, collide=fr, key="cup", name="CupFloor")
        n = 14
        seg = 2 * (CUP_R_IN + CUP_WALL / 2) * math.tan(math.pi / n) * 1.06
        for i in range(n):
            th = 2 * math.pi * i / n
            r = CUP_R_IN + CUP_WALL / 2
            a.boxz((r * math.cos(th), sy + r * math.sin(th), TRAY_T + CUP_H / 2), (CUP_WALL, seg, CUP_H),
                   math.degrees(th), rgb=rgb, collide=fr, key="cup", name="CupWall")
    a.save()


# -- the pill dish: a plate with an oblong pocket (chamfered all round) that centres the tablet ---------------------------
POCKET_HX, POCKET_HY, POCKET_FLOOR, CHAMFER = 5.5 * MM, 10.75 * MM, 1.0 * MM, 2.0 * MM
DISH_HX, DISH_HY, DISH_H = 20 * MM, 13 * MM, 3 * MM


def dish():
    a = Asset2("pill_dish.usda", "PillDish",
               "A pill dish: a 40 x 26 x 3 mm plate with an oblong pocket (11 x 21.5 mm at the floor, 1 mm above the table, "
               "2 mm chamfer all round) that holds one caplet. Origin on the table at the pocket's middle. Fixed.", mass=0.05)
    fr = (0.4, 0.3)
    c = (0.9, 0.9, 0.86)
    px, py, f, ch, hx, hy, h = POCKET_HX, POCKET_HY, POCKET_FLOOR, CHAMFER, DISH_HX, DISH_HY, DISH_H
    a.prism_y([(-hx, 0.0), (-px, 0.0), (-px, f), (-px - ch, h), (-hx, h)], -hy, hy, c, collide=fr, key="dish", name="SideL")
    a.prism_y([(px, 0.0), (hx, 0.0), (hx, h), (px + ch, h), (px, f)], -hy, hy, c, collide=fr, key="dish", name="SideR")
    a.prism_x([(-hy, 0.0), (-py, 0.0), (-py, f), (-py - ch, h), (-hy, h)], -px, px, c, collide=fr, key="dish", name="EndN")
    a.prism_x([(py, 0.0), (hy, 0.0), (hy, h), (py + ch, h), (py, f)], -px, px, c, collide=fr, key="dish", name="EndS")
    a.box((0, 0, f / 2), (2 * px, 2 * py, f), rgb=c, collide=fr, key="dish", name="Floor")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    splitter()
    tablet("tablet.usda", "Tablet", TAB_L, TAB_MASS, True)
    tablet("half_tablet.usda", "HalfTablet", TAB_L / 2, TAB_MASS / 2, False)
    tray()
    dish()
