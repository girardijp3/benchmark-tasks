"""Writes this task's USD assets with pxr (boxes, cylinders, closed lathes; no meshes from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)

Frames: metres, kilograms, Z up.  The bench asset's origin is the table top.  The board asset's origin is the
motherboard's top face at the CPU's centre.  The CPU asset's origin is the centre of the IHS top face.  The paste
asset's origin is the same point.  The wipe asset's origin is the centre of the wipe's contact face (its underside).
Every contract is a true-size primitive; the visuals are the real product's shapes (see REFERENCE.md)."""
import math
import numpy as np
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001




# ---- visual-mesh helpers (copy of submission_tools/meshkit.py): closed, outward-wound round shapes ----


def _orient(pts, tris):
    """Flip all triangles if the signed volume is negative (so the normals point outward)."""
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[a], np.cross(p[b], p[c])) for a, b, c in tris) / 6.0
    return tris if vol >= 0 else [(a, c, b) for a, b, c in tris]


def check_closed(pts, tris):
    """True if every directed edge has exactly one opposite twin (watertight, consistent winding) and the volume is positive."""
    from collections import Counter
    e = Counter()
    for a, b, c in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            e[(u, v)] += 1
    ok = all(n == 1 and e.get((v, u)) == 1 for (u, v), n in e.items())
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[a], np.cross(p[b], p[c])) for a, b, c in tris) / 6.0
    return ok and vol > 0, vol


def lathe(profile, n=96):
    """Revolve a closed (r, z) profile about z. Returns (pts, tris), outward-wound."""
    k = len(profile)
    pts = [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z) for i in range(n) for (r, z) in profile]
    tris = []
    for i in range(n):
        i2 = (i + 1) % n
        for j in range(k):
            j2 = (j + 1) % k
            a, b, c, d = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            tris += [(a, b, c), (a, c, d)]
    return pts, _orient(pts, tris)


def disc(r, z0, z1, n=96, axis="Z"):
    """A solid cylinder of radius r between z0 and z1 along ``axis`` (Z, Y or X), centred on the axis."""
    pts, tris = lathe([(0.0, z0), (r, z0), (r, z1), (0.0, z1)], n)
    # the axis points (r=0) repeat n times: weld them
    return _weld(_to_axis(pts, axis), tris)


def _to_axis(pts, axis):
    if axis == "Z":
        return pts
    if axis == "Y":
        return [(x, z, -y) for x, y, z in pts]
    return [(z, y, -x) for x, y, z in pts]  # X


def _weld(pts, tris, tol=1e-9):
    m, out, new = {}, [], []
    for p in pts:
        key = tuple(round(v / tol) for v in p)
        if key not in m:
            m[key] = len(out)
            out.append(p)
        new.append(m[key])
    t = [(new[a], new[b], new[c]) for a, b, c in tris]
    t = [x for x in t if len(set(x)) == 3]  # drop the degenerate triangles at the axis
    return out, t


METALS = set()


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
        metal = tuple(round(c, 2) for c in rgb) in METALS
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.9 if metal else 0.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.32 if metal else rough)
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

    def set_compliant(self, key, stiffness, damping):
        """Soft (compliant) contact on the physics material ``key`` (N/m, N s/m): a coating that gives and loses energy."""
        prim = self.mats["Phys_" + key].GetPrim()
        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:compliantContactStiffness", Sdf.ValueTypeNames.Float).Set(stiffness)
        prim.CreateAttribute("physxMaterial:compliantContactDamping", Sdf.ValueTypeNames.Float).Set(damping)

    def _xf(self, g, center, rot_x=0.0, rot_y=0.0, rot_z=0.0):
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        if rot_x:
            g.AddRotateXOp().Set(rot_x)
        if rot_y:
            g.AddRotateYOp().Set(rot_y)
        if rot_z:
            g.AddRotateZOp().Set(rot_z)

    def box(self, center, size, rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0)):
        """A box (size = x, y, z edge lengths). rgb: visible, coloured; collide=(static, dynamic): a collider."""
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"{self.cur}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center, *rot)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key)

    def vmesh(self, name, pts, tris, rgb, center=(0, 0, 0)):
        """A visible-only closed mesh (no physics) under the current link."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        P = [Gf.Vec3f(x + center[0], y + center[1], z + center[2]) for x, y, z in pts]
        g.CreatePointsAttr(Vt.Vec3fArray(P))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for t in tris for i in t])
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateDoubleSidedAttr(True)
        lo = [min(p[i] for p in P) for i in range(3)]
        hi = [max(p[i] for p in P) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.color("m%02d%02d%02d" % tuple(int(c * 99) for c in rgb), rgb))

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None):
        """A cylinder: the visible shape is a 96-sided closed mesh; a collider is the same-size USD Cylinder, invisible."""
        self.n += 1
        if rgb is not None:
            pts, tris = disc(radius, -height / 2, height / 2, 96, axis)
            self.vmesh((name or "Cyl") + "Vis", pts, tris, rgb, center)
        if collide is not None:
            g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
            g.CreateRadiusAttr(radius)
            g.CreateHeightAttr(height)
            g.CreateAxisAttr(axis)
            self._xf(g, center)
            self._finish(g, None, collide, key)

    def capsule(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0)):
        """A capsule: ``height`` is the straight part between the two hemispheres."""
        self.n += 1
        g = UsdGeom.Capsule.Define(self.stage, f"{self.cur}/{name or 'Capsule'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center, *rot)
        self._finish(g, rgb, collide, key)

    def articulation_root(self):
        UsdPhysics.ArticulationRootAPI.Apply(self.root.GetPrim())

    def sph_joint(self, name, b0, b1, p0, p1, armature=1e-6):
        """A spherical joint (free to turn every way) between bodies ``b0`` and ``b1`` (prims), at the point given in each
        body's frame by ``p0`` and ``p1``."""
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.SphericalJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr("X")
        j.GetPrim().AddAppliedSchema("PhysxJointAPI")
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(armature)

    def prismatic(self, name, b0, b1, axis="Z", lo=-0.04, hi=0.0, stiffness=0.0, damping=5.0, target=0.0, p0=(0, 0, 0), p1=(0, 0, 0)):
        """A prismatic joint along ``axis`` (limits ``lo``..``hi`` m) from body ``b0`` to ``b1``, with a linear drive."""
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.PrismaticJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(stiffness)
        d.CreateDampingAttr(damping)
        d.CreateTargetPositionAttr(target)
        return j

    def revolute(self, name, b0, b1, axis="X", lo=-60.0, hi=0.0, stiffness=0.0, damping=0.0, target=0.0, p0=(0, 0, 0), p1=(0, 0, 0), armature=1e-6):
        """A revolute joint about ``axis`` (limits ``lo``..``hi`` degrees) from body ``b0`` to ``b1``, with an angular drive
        (stiffness in N m per degree, damping in N m s per degree, target in degrees)."""
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
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(stiffness)
        d.CreateDampingAttr(damping)
        d.CreateTargetPositionAttr(target)
        j.GetPrim().AddAppliedSchema("PhysxJointAPI")
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(armature)
        return j

    def sphere(self, center, radius, rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Sphere.Define(self.stage, f"{self.cur}/{name or 'Sphere'}{self.n}")
        g.CreateRadiusAttr(radius)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

    def mass_props(self, mass, com, inertia):
        m = UsdPhysics.MassAPI(self.root.GetPrim())
        m.GetMassAttr().Set(mass)
        m.GetCenterOfMassAttr().Set(Gf.Vec3f(*com))
        m.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*inertia))

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

    def prism_z(self, poly_xy, z0, z1, rgb, collide=None, key="c", name="Hex", poly_top=None):
        """A convex prism along z: cross-section ``poly_xy`` (list of (x, y), counter-clockwise seen from +z)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        pts = [(x, y, z0) for (x, y) in poly_xy] + [(x, y, z1) for (x, y) in (poly_top or poly_xy)]
        k = len(poly_xy)
        faces, counts = [], []
        for i in range(1, k - 1):
            faces += [0, i + 1, i]
            faces += [k, k + i, k + i + 1]
            counts += [3, 3]
        for i in range(k):
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


# =====================================================================================================================
# Thermal-paste-removal assets.  Real products and sizes in REFERENCE.md; all lengths in metres, kg, Z up.
# =====================================================================================================================
TOP = 0.7475  # yam_arena station table top
CPU_X, CPU_Y = 0.36, -0.07
MAT_TOP = TOP + 0.003
STANDOFF, PCB_T = 0.0065, 0.0016
BOARD_TOP = MAT_TOP + STANDOFF + PCB_T  # world z of the motherboard's top face
IHS_TOP = BOARD_TOP + 0.00735  # AMD AM4: PCB top to IHS top 7.35 mm (Nobu's caliper measurement, [H]ardForum)
BOARD_Z = BOARD_TOP - TOP  # board asset local z when spawned at (CPU_X, CPU_Y, BOARD_TOP)
IHS_Z = IHS_TOP - TOP
TRAY_TOP = TOP + 0.006
PAD_START_Z = TOP + 0.0062

# -- colours ---------------------------------------------------------------------------------------------------------
PCB = (0.055, 0.115, 0.072)
PCB_EDGE = (0.075, 0.150, 0.095)
TRACE = (0.105, 0.195, 0.125)
GOLD = (0.80, 0.63, 0.27)
SILVER = (0.73, 0.74, 0.76)
COPPER = (0.80, 0.52, 0.30)
STEEL = (0.62, 0.64, 0.68)
DARK = (0.035, 0.036, 0.040)
PLASTIC = (0.075, 0.078, 0.085)
PLASTIC_EDGE = (0.16, 0.16, 0.17)
WHITE = (0.955, 0.955, 0.940)
WHITE_EDGE = (0.88, 0.89, 0.90)
PASTE = (0.345, 0.340, 0.320)
PASTE_EDGE = (0.28, 0.28, 0.26)
MAT = (0.075, 0.085, 0.105)
MAT_EDGE = (0.12, 0.14, 0.17)
TRAY = (0.17, 0.25, 0.36)
TRAY_IN = (0.30, 0.40, 0.52)
FOIL = (0.80, 0.81, 0.83)
TUBE = (0.30, 0.31, 0.34)
CAP = (0.10, 0.30, 0.62)
RED = (0.62, 0.09, 0.10)
BLUE = (0.18, 0.35, 0.72)
SILK = (0.90, 0.90, 0.86)
METALS.update(tuple(round(c, 2) for c in k) for k in (GOLD, SILVER, COPPER, STEEL, (0.62, 0.48, 0.22)))

SOCKET_W = 0.054
SOCKET_OPEN = 0.0402
SOCKET_H = 0.00265
LOAD_H = 0.0008
LEVER_R = 0.0015
LEVER_Z = 0.0042

PAD_W = 0.050
PAD_T = 0.0020
FACE_W = 0.024
FACE_PROUD = 0.0008
PAD_MASS = 0.0025
PASTE_T = 0.00005
NX, NY, CELL = 15, 10, 0.002
PATCH_RX, PATCH_RY = 0.015, 0.010


def rounded_rect(w, h, r, n=4):
    """A counter-clockwise rounded rectangle polygon, 4 arcs of n segments."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, -h / 2 + r, -90.0), (w / 2 - r, h / 2 - r, 0.0),
                       (-w / 2 + r, h / 2 - r, 90.0), (-w / 2 + r, -h / 2 + r, 180.0)):
        for k in range(n):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


def paste_cells():
    """The paste patch's 2 mm cells inside a 30 x 20 mm ellipse, row-major (j outer, i inner); the asset and task
    use this same order so joint C<n> is cell n."""
    out = []
    for j in range(NY):
        for i in range(NX):
            x = (i + 0.5) * CELL - PATCH_RX
            y = (j + 0.5) * CELL - PATCH_RY
            if (x / PATCH_RX) ** 2 + (y / PATCH_RY) ** 2 <= 1.0:
                out.append((x, y))
    return out


def bench(fname):
    """The bench around the board: anti-static mat, the wipe's foil tray, the waste tray, the paste tube, the CPU's
    box, a screwdriver and the wipe packet.  Origin: table top."""
    a = Asset(fname, "Bench", "The PC-repair bench: a 500 x 440 x 3 mm dark anti-static mat at (0.36, -0.07), a "
              "80 x 80 x 3 mm blue wipe tray at (0.24, -0.24) with its foil packet, a 80 x 80 x 3 mm grey waste tray at "
              "(0.46, -0.24), an Arctic MX-4-class 4 g paste tube at (0.18, 0.10), the CPU's box at (0.56, 0.10), a "
              "Phillips #2 screwdriver at (0.54, -0.16), a folded-wipe packet at (0.15, -0.02) and an anti-static bag "
              "at (0.14, -0.06).  All fixed; the mat alone carries the board's standoffs.", 1.0,
              com=(0, 0, 0.02), inertia=(1, 1, 1))
    a.phys_mat("mat", 1.2, 1.0)
    a.phys_mat("tray", 0.7, 0.6)
    a.phys_mat("metal", 0.6, 0.5)
    a.phys_mat("plastic", 0.5, 0.4)
    a.box((0.36, -0.07, 0.0015), (0.500, 0.440, 0.003), MAT, (1.2, 1.0), key="mat", name="Mat")
    a.box((0.36, -0.07, 0.00302), (0.490, 0.430, 0.0003), MAT_EDGE, None, key="mat_edge", name="MatEdge")
    # the wipe's foil tray
    a.box((0.24, -0.24, 0.0045), (0.080, 0.080, 0.003), TRAY, (0.7, 0.6), key="tray", name="WipeTray")
    a.box((0.24, -0.24, 0.00602), (0.074, 0.074, 0.0004), TRAY_IN, None, key="tray_in", name="WipeTrayFloor")
    a.box((0.24, -0.24, 0.00626), (0.070, 0.070, 0.0003), FOIL, None, key="foil", name="Foil")
    a.box((0.24, -0.24, 0.00643), (0.064, 0.064, 0.0002), (0.93, 0.93, 0.91), None, key="foil_in", name="FoilInner")
    # the waste tray
    a.box((0.46, -0.24, 0.0045), (0.080, 0.080, 0.003), TRAY, (0.7, 0.6), key="tray", name="WasteTray")
    a.box((0.46, -0.24, 0.00602), (0.074, 0.074, 0.0004), TRAY_IN, None, key="tray_in", name="WasteTrayFloor")
    # the paste tube, lying along x
    a.cyl((0.180, 0.100, 0.0110), 0.0060, 0.070, "X", TUBE, (0.5, 0.4), key="plastic", name="PasteTube")
    a.cyl((0.220, 0.100, 0.0110), 0.0068, 0.012, "X", CAP, (0.5, 0.4), key="plastic", name="PasteCap")
    a.cyl((0.180, 0.100, 0.0110), 0.0062, 0.020, "X", WHITE, None, key="label", name="PasteLabel")
    a.box((0.155, 0.100, 0.0110), (0.006, 0.004, 0.004), DARK, None, key="nozzle", name="PasteNozzle")
    # the CPU's box, with a label panel
    a.box((0.560, 0.100, 0.0170), (0.060, 0.060, 0.028), DARK, (0.5, 0.4), key="plastic", name="CpuBox")
    a.box((0.560, 0.100, 0.0312), (0.046, 0.034, 0.0006), (0.16, 0.16, 0.17), None, key="box_in", name="CpuBoxTop")
    a.box((0.560, 0.084, 0.0316), (0.038, 0.008, 0.0004), SILK, None, key="silk", name="CpuBoxLabel")
    a.box((0.535, 0.078, 0.0317), (0.006, 0.004, 0.0003), BLUE, None, key="blue", name="CpuBoxMark")
    # the screwdriver, lying along y
    a.cyl((0.540, -0.160, 0.0130), 0.011, 0.095, "Y", RED, (0.6, 0.5), key="plastic", name="ScrewdriverHandle")
    a.cyl((0.540, -0.218, 0.0070), 0.0035, 0.120, "Y", STEEL, (0.6, 0.5), key="metal", name="ScrewdriverShaft")
    a.box((0.540, -0.280, 0.0070), (0.006, 0.014, 0.003), STEEL, None, key="metal", name="ScrewdriverTip")
    a.box((0.540, -0.130, 0.0130), (0.012, 0.012, 0.004), DARK, None, key="plastic", name="ScrewdriverCap")
    # the folded-wipe packet and the anti-static bag
    a.box((0.150, -0.020, 0.0040), (0.090, 0.110, 0.002), FOIL, (0.6, 0.5), key="foil", name="WipePacket")
    a.box((0.150, -0.020, 0.0051), (0.076, 0.086, 0.0004), WHITE, None, key="white", name="WipePacketLabel")
    a.box((0.140, -0.060, 0.0018), (0.090, 0.140, 0.0010), (0.55, 0.62, 0.70), None, key="bag", name="AntiStaticBag")
    a.save()


def board(fname):
    """An ATX motherboard with an AM4 socket, its load plate and lever, and the usual components.  Origin: the
    motherboard's top face at the CPU's centre; the socket's centre is the origin."""
    a = Asset(fname, "Board", "ATX 305 x 244 x 1.6 mm motherboard on 6.5 mm brass standoffs: dark-green PCB with "
              "silkscreen and traces, a black AM4 socket (54 x 54 mm frame, 40.2 mm opening), its steel load plate and "
              "lever, the 54 x 90 mm cooler holes, four DDR4 DIMM slots, the 24-pin ATX header, two VRM heatsinks, the "
              "chipset heatsink, two PCIe slots, an M.2 slot, capacitors and the rear I/O shield.  Fixed.", 0.90,
              com=(0, 0, 0.015), inertia=(0.01, 0.01, 0.01))
    a.phys_mat("pcb", 0.5, 0.4)
    a.phys_mat("plastic", 0.4, 0.3)
    a.phys_mat("metal", 0.5, 0.4)
    a.phys_mat("gold", 0.6, 0.5)
    # PCB and its edge
    a.box((0, 0, -PCB_T / 2), (0.305, 0.244, PCB_T), PCB, (0.5, 0.4), key="pcb", name="PCB")
    a.box((0, 0, -0.00005), (0.305, 0.244, 0.0001), PCB_EDGE, None, key="pcb_edge", name="PCBTop")
    # brass standoffs at the ATX holes
    for sx in (-1, 0, 1):
        for sy in (-1, 0, 1):
            if sx == 0 and sy == 0:
                continue
            a.cyl((sx * 0.140, sy * 0.100, -0.00485), 0.0035, STANDOFF, "Z", (0.62, 0.48, 0.22), (0.6, 0.5),
                  key="gold", name="Standoff")
    # the socket frame (54 x 54 outside, 40.2 inside)
    for sx in (-1, 1):
        a.box((sx * (SOCKET_OPEN / 2 + (SOCKET_W - SOCKET_OPEN) / 4), 0, SOCKET_H / 2),
              ((SOCKET_W - SOCKET_OPEN) / 2, SOCKET_W, SOCKET_H), PLASTIC, (0.4, 0.3), key="plastic", name="SocketX")
    for sy in (-1, 1):
        a.box((0, sy * (SOCKET_OPEN / 2 + (SOCKET_W - SOCKET_OPEN) / 4), SOCKET_H / 2),
              (SOCKET_OPEN, (SOCKET_W - SOCKET_OPEN) / 2, SOCKET_H), PLASTIC, (0.4, 0.3), key="plastic", name="SocketY")
    # a pale inner lip on the frame
    for sx in (-1, 1):
        a.box((sx * (SOCKET_OPEN / 2 + 0.0008), 0, SOCKET_H - 0.0002),
              (0.0008, SOCKET_OPEN, 0.0004), PLASTIC_EDGE, None, key="lip", name="SocketLipX")
    for sy in (-1, 1):
        a.box((0, sy * (SOCKET_OPEN / 2 + 0.0008), SOCKET_H - 0.0002),
              (SOCKET_OPEN, 0.0008, 0.0004), PLASTIC_EDGE, None, key="lip", name="SocketLipY")
    # the gold pin field (under the CPU) and its rim
    a.box((0, 0, 0.00255), (0.0398, 0.0398, 0.0002), GOLD, (0.6, 0.5), key="gold", name="PinField")
    # the steel load plate around the CPU, 40.6 mm opening, 53 mm outside
    for sx in (-1, 1):
        a.box((sx * (0.0406 / 2 + (0.053 - 0.0406) / 4), 0, SOCKET_H + LOAD_H / 2),
              ((0.053 - 0.0406) / 2, 0.053, LOAD_H), DARK, (0.4, 0.3), key="metal", name="LoadX")
    for sy in (-1, 1):
        a.box((0, sy * (0.0406 / 2 + (0.053 - 0.0406) / 4), SOCKET_H + LOAD_H / 2),
              (0.0406, (0.053 - 0.0406) / 2, LOAD_H), DARK, (0.4, 0.3), key="metal", name="LoadY")
    a.box((0.0285, 0.0, SOCKET_H + LOAD_H / 2), (0.008, 0.010, LOAD_H), DARK, (0.4, 0.3), key="metal", name="LoadTab")
    # the lever: a 3 mm steel rod along y on the +x side, its handle and its plastic grip
    a.cyl((0.0295, 0.0, LEVER_Z), LEVER_R, 0.052, "Y", STEEL, (0.5, 0.4), key="metal", name="LeverRod")
    a.cyl((0.0295, 0.0330, LEVER_Z), LEVER_R, 0.014, "Y", STEEL, (0.5, 0.4), key="metal", name="LeverBend")
    a.cyl((0.0345, 0.0400, LEVER_Z), LEVER_R, 0.010, "X", STEEL, (0.5, 0.4), key="metal", name="LeverHandle")
    a.cyl((0.0340, 0.0400, LEVER_Z), 0.0030, 0.012, "X", PLASTIC, None, key="grip", name="LeverGrip")
    a.box((0.0310, 0.0260, LEVER_Z + 0.0015), (0.005, 0.004, 0.003), PLASTIC, None, key="clip", name="LeverClip")
    # the 54 x 90 mm cooler mounting holes
    for sx in (-1, 1):
        for sy in (-1, 1):
            a.cyl((sx * 0.027, sy * 0.045, 0.0006), 0.0060, 0.0012, "Z", (0.65, 0.50, 0.24), (0.6, 0.5),
                  key="gold", name="CoolerHole")
            a.cyl((sx * 0.027, sy * 0.045, 0.0013), 0.0032, 0.0014, "Z", (0.08, 0.08, 0.09), None,
                  key="hole", name="CoolerHoleIn")
    # DIMM slots (4), with clips
    for k in range(4):
        y = 0.060 + k * 0.0135
        a.box((-0.005, y, 0.00425), (0.110, 0.0075, 0.0085), PLASTIC, (0.4, 0.3), key="plastic", name="Dimm")
        a.box((-0.005, y, 0.0086), (0.104, 0.0020, 0.0008), PLASTIC_EDGE, None, key="dimm_edge", name="DimmSlot")
        for sx in (-1, 1):
            a.box((-0.005 + sx * 0.058, y, 0.0045), (0.004, 0.010, 0.009), WHITE_EDGE, None, key="clip", name="DimmClip")
    # 24-pin ATX header
    a.box((0.105, 0.045, 0.0075), (0.052, 0.012, 0.015), PLASTIC, (0.4, 0.3), key="plastic", name="Atx24")
    a.box((0.105, 0.045, 0.0152), (0.046, 0.006, 0.0004), SILK, None, key="silk", name="AtxLabel")
    # VRM heatsinks with fins
    for sy in (-1, 1):
        a.box((-0.055, sy * 0.060, 0.0110), (0.030, 0.018, 0.022), DARK, (0.5, 0.4), key="metal", name="Vrm")
        for k in range(6):
            a.box((-0.055, sy * 0.060 + (k - 2.5) * 0.0028, 0.0222), (0.028, 0.0012, 0.0012), STEEL, None,
                  key="fin", name="VrmFin")
    # chipset heatsink
    a.box((-0.100, 0.055, 0.0065), (0.030, 0.030, 0.013), DARK, (0.5, 0.4), key="metal", name="Chipset")
    a.box((-0.100, 0.055, 0.0131), (0.022, 0.022, 0.0004), SILVER, None, key="chip", name="ChipsetTop")
    # PCIe slots
    for x in (-0.095, -0.130):
        a.box((x, -0.020, 0.0055), (0.089, 0.012, 0.011), PLASTIC, (0.4, 0.3), key="plastic", name="Pcie")
        a.box((x, -0.0265, 0.0111), (0.080, 0.0010, 0.0004), SILVER, None, key="pcie", name="PcieContact")
    # M.2 slot and its screw
    a.box((-0.015, -0.075, 0.0015), (0.022, 0.080, 0.003), PLASTIC, None, key="plastic", name="M2")
    a.cyl((-0.015, -0.110, 0.0018), 0.0025, 0.003, "Z", SILVER, None, key="metal", name="M2Screw")
    # capacitors around the socket
    for x, y in ((0.030, 0.030), (0.040, 0.024), (0.024, 0.040), (-0.036, 0.030), (-0.030, -0.036), (0.035, -0.030)):
        a.cyl((x, y, 0.0030), 0.0035, 0.006, "Z", DARK, (0.4, 0.3), key="plastic", name="Cap")
        a.cyl((x, y, 0.0061), 0.0035, 0.0004, "Z", SILVER, None, key="cap_top", name="CapTop")
    # CPU fan header
    a.box((0.015, 0.038, 0.0020), (0.012, 0.006, 0.004), WHITE, None, key="header", name="FanHeader")
    # rear I/O shield
    a.box((-0.145, 0.0, 0.020), (0.005, 0.158, 0.040), SILVER, (0.5, 0.4), key="metal", name="IoShield")
    a.box((-0.1415, 0.0, 0.020), (0.002, 0.150, 0.034), DARK, None, key="io", name="IoFace")
    # silkscreen and traces
    for i in range(8):
        a.box((0.055 + i * 0.008, -0.090 + (i % 3) * 0.010, 0.00002), (0.004, 0.0005, 0.00004), TRACE, None,
              key="trace", name="Trace")
    for i in range(5):
        a.box((-0.050 + i * 0.008, 0.095, 0.00002), (0.0005, 0.020, 0.00004), TRACE, None, key="trace", name="Trace2")
    a.box((0, 0, 0.00002), (0.060, 0.0004, 0.00004), SILK, None, key="silk", name="SocketLine")
    a.box((0, 0, 0.00002), (0.0004, 0.060, 0.00004), SILK, None, key="silk", name="SocketLine2")
    a.box((-0.045, 0.0, 0.00002), (0.020, 0.006, 0.00004), SILK, None, key="silk", name="SocketLabel")
    a.box((-0.031, -0.031, 0.00003), (0.004, 0.004, 0.00006), SILK, None, key="silk", name="Pin1")
    a.save()


def cpu(fname):
    """An AMD AM4 Ryzen-class CPU with its IHS up: 40 x 40 mm package, 37.5 x 37.5 mm nickel-plated copper IHS,
    7.35 mm from the PCB to the IHS top.  Origin: the centre of the IHS top face.  Fixed in the socket."""
    a = Asset(fname, "CPU", "AMD AM4 Ryzen-class desktop CPU, true size: 40 x 40 mm package, 37.5 x 37.5 mm "
              "nickel-plated copper IHS, 3.6 mm thick, 7.35 mm from the motherboard's top to the IHS top; dark-green "
              "substrate with gold edge traces, a pin-1 triangle and a laser-etched die area.  Origin: the centre of "
              "the IHS top.  Fixed.", 0.050, com=(0, 0, -0.0025), inertia=(1.2e-5, 1.2e-5, 2.3e-5))
    a.phys_mat("metal", 0.5, 0.4)
    a.phys_mat("plastic", 0.5, 0.4)
    a.phys_mat("gold", 0.6, 0.5)
    # the IHS, with a 0.15 mm top chamfer
    a.prism_z(rounded_rect(0.0375, 0.0375, 0.0008, 6), -0.0036, -0.00015, SILVER, (0.5, 0.4), key="metal", name="IHS")
    a.prism_z(rounded_rect(0.0372, 0.0372, 0.0008, 6), -0.00015, 0.0, SILVER, (0.5, 0.4), key="metal", name="IHSTop")
    # the copper fillet at the IHS's foot
    a.prism_z(rounded_rect(0.0378, 0.0378, 0.0008, 6), -0.0036, -0.00345, COPPER, None, key="copper", name="IHSFoot")
    # the laser-etched area and the brand line (no readable text; a real CPU's etched lid)
    a.box((-0.002, 0.0, 0.00001), (0.012, 0.0055, 0.00002), (0.66, 0.66, 0.67), None, key="etch", name="Etch")
    a.box((-0.002, 0.0, 0.00002), (0.010, 0.0004, 0.00002), (0.58, 0.58, 0.59), None, key="etch_line", name="EtchLine")
    # the substrate and its gold edge traces
    a.box((0, 0, -0.00415), (0.040, 0.040, 0.0011), (0.035, 0.065, 0.045), (0.5, 0.4), key="plastic", name="Substrate")
    a.box((0, 0, -0.00358), (0.0402, 0.0402, 0.00008), (0.045, 0.080, 0.055), None, key="sub_edge", name="SubEdge")
    for sx in (-1, 1):
        a.box((sx * 0.0195, 0, -0.00357), (0.0005, 0.038, 0.00008), GOLD, None, key="gold", name="GoldEdge")
        a.box((0, sx * 0.0195, -0.00357), (0.038, 0.0005, 0.00008), GOLD, None, key="gold", name="GoldEdge2")
    a.box((-0.0185, -0.0185, -0.00356), (0.002, 0.002, 0.00006), WHITE, None, key="silk", name="Pin1Mark")
    for x, y in ((0.016, 0.016), (0.018, -0.012), (-0.015, 0.016), (-0.017, -0.015)):
        a.box((x, y, -0.00354), (0.0012, 0.0008, 0.0002), (0.55, 0.50, 0.42), None, key="smd", name="Smd")
    # the pin field, on the underside (hidden in the socket; drawn for a lifted-CPU view)
    a.box((0, 0, -0.00482), (0.0392, 0.0392, 0.0002), GOLD, None, key="pins", name="Pins")
    a.save()


def paste(fname):
    """The old thermal-paste patch: a 30 x 20 mm ellipse of 2 mm x 2 mm x 0.05 mm grey cells, each on a 0.5 mm
    downward slide.  Visual only (no colliders); the scorer's pad-IHS contact does the removing.  Origin: IHS top."""
    cells = paste_cells()
    a = Asset(fname, "Paste", f"The old thermal paste on the IHS: {len(cells)} cells of 2 x 2 x 0.05 mm inside a "
              "30 x 20 mm ellipse, grey, on 0.5 mm slides so a clean cell sinks into the lid.  Visual only; the pad's "
              "real contact with the IHS and its rubbing remove cells (see task.json fidelity).  Origin: IHS top.", 1e-3,
              inertia=(1e-7, 1e-7, 1e-7), no_gravity=True)
    a.phys_mat("paste", 0.8, 0.6)
    a.articulation_root()
    root = a.root.GetPrim()
    for n, (x, y) in enumerate(cells):
        lk = a.link(f"C{n}", 1e-6, inertia=(1e-12, 1e-12, 1e-12), pos=(x, y, 0.000025))
        a.box((0, 0, 0), (CELL - 0.00002, CELL - 0.00002, PASTE_T), PASTE, None, key="paste", name="Cell")
        a.prismatic(f"C{n}", root, lk, axis="Z", lo=-0.0005, hi=0.0, stiffness=50.0, damping=2.0,
                    target=0.0, p0=(x, y, 0.000025), p1=(0, 0, 0))
    print(f"paste cells: {len(cells)}")
    a.save()


def wipe(fname):
    """A folded 91% IPA lint-free wipe on a small bench applicator: a 50 x 50 mm folded Kimtech W4-class pad with a
    24 x 24 mm contact face standing 0.8 mm proud of a 50 x 50 mm flange, on a 40 x 40 x 6 mm backing and a
    20 x 20 x 26 mm grip bar.  The holder only replaces the hand's grip for the parallel-jaw gripper; the wipe, its
    91% IPA, its 24 x 24 mm working face and the white-pad check are unchanged.  Origin: the centre of the wipe's
    contact face (its underside).  Free body."""
    a = Asset(fname, "Wipe", "A folded 91% IPA lint-free wipe, 50 x 50 mm and 2 mm thick: a 24 x 24 mm contact face "
              "standing 0.8 mm proud of the 50 x 50 mm flange.  White nonwoven with a blue edge line, a woven emboss "
              "and a centre crease.  It is mounted on a 40 x 40 x 6 mm dark-blue bench applicator with a 20 x 20 x "
              "26 mm grip bar (the hand's grip stand-in).  Origin: the centre of the wipe's contact face.  Free body.",
              0.0275, com=(0, 0, 0.0090), inertia=(6.5e-6, 6.5e-6, 5.0e-6))
    a.phys_mat("pad", 1.2, 1.0)
    a.phys_mat("holder", 0.6, 0.5)
    # the wipe's flange and its contact face
    a.prism_z(rounded_rect(PAD_W, PAD_W, 0.0030, 6), FACE_PROUD, PAD_T, WHITE, (1.2, 1.0), key="pad", name="Flange")
    a.prism_z(rounded_rect(FACE_W, FACE_W, 0.0020, 6), 0.0, PAD_T, WHITE, (1.2, 1.0), key="pad", name="Face")
    for sx in (-1, 1):
        a.box((sx * (PAD_W / 2 - 0.0015), 0, PAD_T + 0.00002), (0.0012, PAD_W - 0.006, 0.00004), BLUE, None,
              key="blue", name="EdgeBlue")
        a.box((0, sx * (PAD_W / 2 - 0.0015), PAD_T + 0.00002), (PAD_W - 0.006, 0.0012, 0.00004), BLUE, None,
              key="blue", name="EdgeBlue2")
    a.box((0, 0, PAD_T + 0.00002), (PAD_W - 0.010, 0.0005, 0.00004), WHITE_EDGE, None, key="crease", name="Crease")
    for k in range(-2, 3):
        a.box((k * 0.0045, 0, 0.00001), (0.00025, FACE_W - 0.002, 0.00002), WHITE_EDGE, None, key="weave", name="WeaveX")
        a.box((0, k * 0.0045, 0.00001), (FACE_W - 0.002, 0.00025, 0.00002), WHITE_EDGE, None, key="weave", name="WeaveY")
    # the applicator: a 40 x 40 x 6 mm backing and a 20 x 20 x 26 mm grip bar
    a.prism_z(rounded_rect(0.040, 0.040, 0.0040, 6), PAD_T, PAD_T + 0.0060, (0.10, 0.16, 0.24),
              (0.6, 0.5), key="holder", name="Backing")
    a.prism_z(rounded_rect(0.020, 0.020, 0.0030, 6), PAD_T + 0.0060, PAD_T + 0.0060 + 0.0260,
              (0.13, 0.20, 0.30), (0.6, 0.5), key="holder", name="GripBar")
    for z in (0.014, 0.020, 0.026):
        a.box((0, 0, z), (0.0202, 0.0202, 0.0012), (0.20, 0.30, 0.42), None, key="rib", name="GripRib")
    a.box((0, 0, PAD_T + 0.0062), (0.036, 0.036, 0.0004), (0.18, 0.26, 0.36), None, key="backing_top", name="BackingTop")
    a.save()


if __name__ == "__main__":
    bench("bench.usda")
    board("board.usda")
    cpu("cpu.usda")
    paste("paste.usda")
    wipe("wipe.usda")
