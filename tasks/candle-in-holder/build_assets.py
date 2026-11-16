"""Writes this task's USD assets with pxr (boxes, capsules; no meshes from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up."""
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


def lathe_axis(profile, n=96):
    """Revolve an OPEN (r, z) profile that starts and ends on the axis (r = 0) into a closed solid."""
    pts, tris = lathe(list(profile), n)
    return _weld(pts, tris)


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


def square_plate_round_hole(half, r_in, z0, z1, n=96):
    """A square plate (+-half in x and y) between z0 and z1 with a round hole of radius r_in. n is a multiple of 8."""
    ti, to, bi, bo = [], [], [], []
    for i in range(n):
        a = 2 * math.pi * i / n
        c, s = math.cos(a), math.sin(a)
        m = half / max(abs(c), abs(s))  # where the ray meets the square: corners lie on the 45-degree rays
        ti.append((r_in * c, r_in * s, z1)); to.append((m * c, m * s, z1))
        bi.append((r_in * c, r_in * s, z0)); bo.append((m * c, m * s, z0))
    pts = ti + to + bi + bo
    T, O, B, P = 0, n, 2 * n, 3 * n
    tris = []
    for i in range(n):
        j = (i + 1) % n
        for quad in ((T + i, O + i, O + j, T + j),   # top
                     (B + i, B + j, P + j, P + i),   # bottom
                     (O + i, P + i, P + j, O + j),   # outer wall
                     (T + i, T + j, B + j, B + i)):  # hole wall
            a, b, c, d = quad
            tris += [(a, b, c), (a, c, d)]
    return pts, _orient(pts, tris)



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

    def color(self, key, rgb, rough=0.6, metal=0.0):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
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

    def vmesh(self, name, pts, tris, rgb, center=(0, 0, 0), rough=0.6, metal=0.0):
        """A visible-only closed mesh (no physics) under the current link."""
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
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.color("m%02d%02d%02d_%d_%d" % (*[int(c * 99) for c in rgb], int(rough * 10), int(metal * 10)), rgb, rough, metal))

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
        """The articulation root is a rigid link "Base" (not the root Xform): the root's body and mass move onto it, later shapes go under it."""
        p = self.root.GetPrim()
        m = UsdPhysics.MassAPI(p)
        mass, com, inertia = m.GetMassAttr().Get(), m.GetCenterOfMassAttr().Get(), m.GetDiagonalInertiaAttr().Get()
        p.RemoveAPI(UsdPhysics.MassAPI)
        p.RemoveAPI(UsdPhysics.RigidBodyAPI)
        self.base_prim = self.link("Base", mass, com=tuple(com), inertia=tuple(inertia))
        UsdPhysics.ArticulationRootAPI.Apply(self.base_prim)

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

    def prismatic(self, name, b0, b1, axis="Z", lo=-0.04, hi=0.0, stiffness=0.0, damping=5.0, target=0.0, p0=(0, 0, 0), p1=(0, 0, 0), rot_z=0.0):
        """A prismatic joint along ``axis`` (limits ``lo``..``hi`` m) from body ``b0`` to ``b1``, with a linear drive."""
        if not self.stage.GetPrimAtPath(f"/{self.name}/Joints"):
            UsdGeom.Scope.Define(self.stage, f"/{self.name}/Joints")
        j = UsdPhysics.PrismaticJoint.Define(self.stage, f"/{self.name}/Joints/{name}")
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        qz = Gf.Quatf(math.cos(math.radians(rot_z) / 2), 0, 0, math.sin(math.radians(rot_z) / 2))
        j.CreateLocalRot0Attr().Set(qz)
        j.CreateLocalRot1Attr().Set(qz)
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


# ======================================================================================================================
# Candle task assets. Metres, kilograms, Z up.
#   candle.usda        : an 8 in (203 mm) taper-base dinner candle, 46 g, one free rigid body. Origin: the middle of its base end.
#   candlestick.usda   : a 4 in solid-brass altar candlestick, 340 g, 127 mm foot, 7/8 in socket; ONE FREE articulation: the body ("Base",
#                        the root: foot, stem, cup, floor, lead-in, housing) and 12 spring wall segments (radial prismatic joints) that give
#                        the way a candle's wax does when the tapered base is pressed in. Origin: table level, the socket's axis.
# ======================================================================================================================
def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


MM_ = 0.001
CANDLE_H = 0.203
CANDLE_MASS = 0.046
TIP_R, CHAMF_R, CHAMF_Z = 0.0092, 0.0100, 0.0008  # the base end: 18.4 mm across, 1 mm chamfer-like round-over to 20.0 mm
TAPER_Z, TAPER_R = 0.025, 0.0117  # the 25 mm base taper widens to 23.4 mm across
TOP_R = 0.0085  # 17 mm across at the top
Z_RIM = 0.1016  # 4 in
SOCKET_DEPTH = 0.022
FLOOR_Z = Z_RIM - SOCKET_DEPTH
R_BORE = 0.011112  # 7/8 in socket: 22.225 mm across (the flats of the 12 wall segments)
LEAD = 0.004
STICK_MASS = 0.340
N_WALL = 12
WALL_MASS = 0.003
WALL_T = 0.004
WALL_TRAVEL = 0.0006
WALL_K = 7700.0  # N/m per segment (tuned so the measured seating push is about SEAT_PUSH_TARGET, see test_candle_in_holder.py)
WALL_D = 4.0  # N s/m per segment


def circle(r, n=32):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def candle(fname):
    m, H = CANDLE_MASS, CANDLE_H
    zc = 0.098
    ixx = m * (3 * 0.0105**2 + H**2) / 12
    a = Asset(fname, "Candle", "An 8 in (203 mm) taper-base dinner candle, 46 g: the base end 20 mm across widening over 25 mm to 23.4 mm (a 7/8 in nominal base), "
              "tapering to 17 mm at the top. Origin: the base end's middle, z up.", m, com=(0, 0, zc), inertia=(ixx, ixx, m * 0.0105**2 / 2))
    f = (0.40, 0.35)
    a.prism_z(circle(TIP_R), 0.0, CHAMF_Z, None, f, key="wax", name="Round", poly_top=circle(CHAMF_R))
    a.prism_z(circle(CHAMF_R), CHAMF_Z, TAPER_Z, None, f, key="wax", name="BaseTaper", poly_top=circle(TAPER_R))
    a.prism_z(circle(TAPER_R), TAPER_Z, H - 0.002, None, f, key="wax", name="Body", poly_top=circle(TOP_R))
    mm = MM_
    prof = [(0, 0), (TIP_R, 0), (CHAMF_R, CHAMF_Z), (TAPER_R, TAPER_Z), (TOP_R + 0.4 * mm, H - 3 * mm), (TOP_R, H - 1.2 * mm), (TOP_R - 1.5 * mm, H),
            (TOP_R - 4.0 * mm, H - 0.5 * mm), (3.5 * mm, H - 2.0 * mm), (1.5 * mm, H - 2.6 * mm), (0, H - 2.6 * mm)]
    pts, tris = lathe_axis(prof, 96)
    a.n += 1
    a.vmesh("WaxVis", pts, tris, (0.96, 0.93, 0.82), rough=0.45)
    pts, tris = lathe_axis([(0, H - 3.0 * mm), (0.9 * mm, H - 3.0 * mm), (0.8 * mm, H + 6.0 * mm), (0, H + 6.0 * mm)], 24)
    a.n += 1
    a.vmesh("WickVis", pts, tris, (0.86, 0.80, 0.62), rough=0.9)
    pts, tris = lathe_axis([(0, H + 5.8 * mm), (0.9 * mm, H + 5.8 * mm), (0.8 * mm, H + 9.5 * mm), (0.4 * mm, H + 10 * mm), (0, H + 10 * mm)], 24)
    a.n += 1
    a.vmesh("WickTipVis", pts, tris, (0.07, 0.06, 0.06), rough=0.9)
    a.save()


def stick(fname):
    mw = N_WALL * WALL_MASS
    mb = STICK_MASS - mw
    a = Asset(fname, "Candlestick", "A 4 in solid-brass altar candlestick, 340 g, 127 mm foot, 7/8 in socket, 22 mm deep, free on the table. Articulation: body Base (root) and 12 "
              "spring wall segments W0..W11 on radial prismatic joints (the give of a wax candle base). Origin: table level, the socket's axis.", mb,
              com=(0, 0, 0.028), inertia=(4.4e-4, 4.4e-4, 3.8e-4))
    a.articulation_root()
    f = (0.35, 0.30)
    # colliders on Base (invisible; what you see is the single turned-brass profile below)
    a.cyl((0, 0, 0.003), 0.0635, 0.006, "Z", None, f, key="brass", name="Foot")
    a.prism_z(circle(0.0600), 0.006, 0.020, None, f, key="brass", name="Dome", poly_top=circle(0.0250))
    a.cyl((0, 0, 0.045), 0.012, 0.050, "Z", None, f, key="brass", name="Stem")
    a.cyl((0, 0, 0.0385), 0.0185, 0.009, "Z", None, f, key="brass", name="Knop")
    a.prism_z(circle(0.0125), 0.062, 0.0796, None, f, key="brass", name="Flare", poly_top=circle(0.0185))
    a.cyl((0, 0, FLOOR_Z - 0.0015), 0.0135, 0.003, "Z", None, f, key="brass", name="Floor")
    for k in range(N_WALL):  # the cup's outer housing behind the wall segments
        ph = 2 * math.pi * k / N_WALL
        rc = 0.0175
        a.box((rc * math.cos(ph), rc * math.sin(ph), FLOOR_Z + SOCKET_DEPTH / 2), (0.003, 0.0105, SOCKET_DEPTH), None, f, key="brass", name="Housing", rot=(0, 0, math.degrees(ph)))
    skin, sl = 0.0015, math.radians(45.0)
    length = LEAD * math.sqrt(2.0)
    r_in = R_BORE + WALL_TRAVEL + 0.0002  # the lead-in starts just outside the walls' farthest swing
    rm = r_in + LEAD / 2 + 0.5 * skin * math.sin(sl)
    zm = Z_RIM + LEAD / 2 - 0.5 * skin * math.cos(sl)
    for k in range(N_WALL * 2):
        ph = 2 * math.pi * k / (N_WALL * 2)
        a.n += 1
        g = UsdGeom.Cube.Define(a.stage, f"{a.cur}/Lead{a.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(rm * math.cos(ph), rm * math.sin(ph), zm))
        g.AddRotateZOp().Set(math.degrees(ph))
        g.AddRotateYOp().Set(-45.0)
        g.AddScaleOp().Set(Gf.Vec3f(length, 2 * math.pi * r_in / (N_WALL * 2) * 1.6, skin))
        a._finish(g, None, f, "brass", mesh=False)
    # the look: a turned polished-brass profile: beaded domed foot, knop, waisted stem, flared cup, rolled lip, 4 mm mouth, bore to the floor
    prof_mm = [(0, 0), (61, 0), (63.5, 1.0), (63.5, 2.5), (61, 4.5), (52, 7.5), (42, 11), (33, 15), (26, 19), (20, 23), (16, 27), (14, 31),
               (14.5, 34), (17.5, 36.5), (18.5, 38.5), (17.5, 40.5), (14.5, 42.5), (12.5, 46), (12, 52), (12.5, 58), (14.5, 63), (16.5, 69),
               (17.8, 75), (18.2, 82), (18.5, 92), (18.7, 99), (19.4, 101.0), (19.2, 102.4), (18.4, 103.4), (17.6, 105.6),
               ((r_in + LEAD) / MM_, (Z_RIM + LEAD) / MM_), (r_in / MM_, Z_RIM / MM_), (R_BORE / MM_ - 0.15, Z_RIM / MM_ - 0.2),
               (R_BORE / MM_ - 0.15, FLOOR_Z / MM_), (0, FLOOR_Z / MM_)]
    pts, tris = lathe_axis([(r * MM_, z * MM_) for r, z in prof_mm], 96)
    a.n += 1
    a.vmesh("BrassVis", pts, tris, (0.82, 0.64, 0.26), rough=0.30, metal=0.9)
    # the 12 spring wall segments: links on radial prismatic joints (travel 0..0.6 mm outward, drive WALL_K)
    zc = FLOOR_Z + 0.0005 + (SOCKET_DEPTH - 0.0005) / 2
    hseg = SOCKET_DEPTH - 0.0005
    rc = R_BORE + WALL_T / 2
    for k in range(N_WALL):
        ph = 2 * math.pi * k / N_WALL
        pos = (rc * math.cos(ph), rc * math.sin(ph), zc)
        w = a.link(f"W{k}", WALL_MASS, com=(0, 0, 0), inertia=box_inertia(WALL_MASS, WALL_T, 0.0069, hseg), pos=pos)
        a.box((0, 0, 0), (WALL_T, 0.0069, hseg), None, f, key="brass", name="Seg", rot=(0, 0, math.degrees(ph)))
        a.prismatic(f"Wall{k}", a.base_prim, w, "X", lo=0.0, hi=WALL_TRAVEL, stiffness=WALL_K, damping=WALL_D, target=0.0, p0=pos, p1=(0, 0, 0), rot_z=math.degrees(ph))
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    candle("candle.usda")
    stick("candlestick.usda")
