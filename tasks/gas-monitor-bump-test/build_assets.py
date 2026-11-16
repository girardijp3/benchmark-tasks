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


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


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
# Gas-monitor bump-test assets. Metres, kilograms, Z up.
#   monitor.usda    : a BW GasAlertQuattro-class 4-gas monitor (130 x 81 x 47 mm, 316 g) lying on its back, face up;
#                     one free articulation: the body (root) and four LCD bar pointers on prismatic joints.
#   cap.usda        : the QT-TC-1 calibration cap (a 62 x 82 x 12 mm shell with a gasket, two side snap tabs and a hose
#                     barb) and its 1 ft hose; one free articulation: the cap body (root) and the two tabs on revolute
#                     joints. The hose is a visual tube with no collider; its far end is a slip fit in the regulator.
#   cylinder.usda   : a 34 L aluminium calibration-gas cylinder (76 x 289 mm, 2.0 kg full) lying in a bench cradle,
#                     with its C-10 valve toward the robot; fixed.
#   regulator.usda  : the BW REG-0.5 fixed-flow regulator (0.239 kg): inlet nut, body, outlet barb and the on/off
#                     knob on a revolute joint; fixed root.
#   cap_rest.usda   : the cap's bench rest, fixed.
# ======================================================================================================================
import meshkit2_inline as mk2  # rounded boxes (the two meshkit files are embedded copies of submission_tools/meshkit.py)

MON_L, MON_W, MON_H = 0.112, 0.076, 0.033      # MSA ALTAIR 4X 112 x 76 x 33 mm (MSA datasheet)
MON_MASS = 0.224                                # with battery and clip
SENSOR_X = -0.026
RIDGE_Y = 0.0388
CAP_L, CAP_W, CAP_H = 0.062, 0.084, 0.012
CAP_MASS = 0.078
CYL_L, CYL_R = 0.289, 0.038                     # 34 L aluminium cylinder: 289 mm x 76 mm, 2.0 kg full
REG_MASS = 0.239                                # BW REG-0.5


def tube_mesh(p0, p1, r, n=20):
    """A closed tube with rounded ends from p0 to p1 (points, triangles)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    if L < 1e-6:
        return [], []
    z = d / L
    a = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(z, a)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    profile = [(0.0, 0.0), (r * 0.55, 0.0), (r, r * 0.45), (r, L - r * 0.45), (r * 0.55, L), (0.0, L)]
    pts, tris = lathe_axis(profile, n)
    P = [tuple(p0 + R @ np.array(p)) for p in pts]
    return P, tris


def monitor(fname):
    I = box_inertia(MON_MASS, MON_L, MON_W, MON_H)
    a = Asset(fname, "Monitor", "An MSA ALTAIR 4X four-gas monitor, 112 x 76 x 33 mm, 224 g, lying face up on the bench. "
              "Free articulation: the body (root) and four LCD bar pointers on prismatic joints (the declared display effect). "
              "Origin: the middle of the underside, z up.", MON_MASS, com=(0, 0, MON_H / 2), inertia=I)
    a.articulation_root()
    f = (0.55, 0.45)
    # colliders: the body, the raised sensor plate and the two snap ridges (invisible guides)
    a.box((0, 0, MON_H / 2), (MON_L, MON_W, MON_H), None, f, key="rubber", name="BodyCol")
    a.box((SENSOR_X, 0, MON_H + 0.002), (0.050, 0.040, 0.004), None, f, key="rubber", name="SensorCol")
    for sy in (1, -1):
        a.box((SENSOR_X, sy * RIDGE_Y, 0.0185), (0.046, 0.0016, 0.003), None, f, key="rubber", name="RidgeCol")
    # the rubberised body and its yellow bumper band
    pts, tris = mk2.rounded_box(MON_L, MON_W, MON_H, 0.006, nc=7, steps=4, centre=(0, 0, MON_H / 2))
    a.n += 1
    a.vmesh("BodyVis", pts, tris, (0.10, 0.10, 0.11), rough=0.72)
    pts, tris = mk2.rounded_box(MON_L + 0.001, MON_W + 0.001, 0.012, 0.005, nc=7, steps=3, centre=(0, 0, 0.006))
    a.n += 1
    a.vmesh("BumperVis", pts, tris, (0.86, 0.63, 0.05), rough=0.75)
    # top bezel, a little proud, with the LCD panel and the sensor plate
    # LCD: dark green glass with a silver frame
    a.box((0.030, 0, MON_H + 0.0025), (0.040, 0.034, 0.0012), (0.03, 0.09, 0.07), None, name="LcdVis")
    for sx, sw in ((1, 0.002), (-1, 0.002)):
        a.box((0.030 + sx * 0.021, 0, MON_H + 0.0025), (sw, 0.036, 0.0016), (0.62, 0.64, 0.66), None, name="LcdFrame")
    for sy, sh in ((1, 0.002), (-1, 0.002)):
        a.box((0.030, sy * 0.018, MON_H + 0.0025), (0.042, sh, 0.0016), (0.62, 0.64, 0.66), None, name="LcdFrame")
    # four rows of scale ticks and coloured pointers (the pointers are the articulation's links)
    rows = (("H2S", 0.016, (0.92, 0.80, 0.10)), ("CO", 0.026, (0.93, 0.48, 0.08)),
            ("LEL", 0.036, (0.85, 0.12, 0.10)), ("O2", 0.046, (0.10, 0.55, 0.90)))
    for name, x, rgb in rows:
        for k in range(5):
            a.box((x, -0.012 + 0.006 * k, MON_H + 0.0034), (0.0009, 0.0009, 0.0005), (0.80, 0.82, 0.80), None, name=f"Tick{name}{k}")
        lk = a.link(f"Bar{name}", 0.0004, com=(0, 0, 0), inertia=(2e-10, 2e-10, 2e-10), pos=(x, -0.012, MON_H + 0.0042))
        a.box((0, 0, 0), (0.0032, 0.0042, 0.0012), rgb, None, name=f"Ptr{name}")
        a.prismatic(f"Bar{name}", a.base_prim, lk, "Y", lo=0.0, hi=0.024, stiffness=30.0, damping=0.25, target=0.0,
                    p0=(x, -0.012, MON_H + 0.0042), p1=(0, 0, 0))
    # the raised sensor plate: four dark ports in a 2 x 2 grid and a fine grille
    pts, tris = mk2.rounded_box(0.050, 0.040, 0.004, 0.0015, nc=5, steps=2, centre=(SENSOR_X, 0, MON_H + 0.002))
    a.n += 1
    a.vmesh("SensorVis", pts, tris, (0.08, 0.08, 0.09), rough=0.6)
    for sx in (1, -1):
        for sy in (1, -1):
            a.cyl((SENSOR_X + sx * 0.0105, sy * 0.0085, MON_H + 0.0042), 0.0023, 0.0008, "Z", (0.015, 0.015, 0.018), None, name="Port")
            a.cyl((SENSOR_X + sx * 0.0105, sy * 0.0085, MON_H + 0.0046), 0.0030, 0.0005, "Z", (0.55, 0.57, 0.58), None, name="PortRing")
    for k in range(7):
        a.box((SENSOR_X, -0.011 + 0.0037 * k, MON_H + 0.0043), (0.040, 0.0008, 0.0005), (0.45, 0.47, 0.47), None, name="Grille")
    # side snap ridges the cap hooks catch, and a label / LED strip
    for sy in (1, -1):
        a.box((SENSOR_X, sy * RIDGE_Y, 0.0185), (0.046, 0.0016, 0.003), (0.16, 0.16, 0.17), None, name="Ridge")
    a.box((-0.044, 0.030, MON_H + 0.0025), (0.014, 0.010, 0.0012), (0.90, 0.90, 0.88), None, name="Label")
    a.box((-0.044, 0.030, MON_H + 0.0028), (0.015, 0.004, 0.0013), (0.82, 0.12, 0.10), None, name="LabelBand")
    # the three front-panel buttons of the ALTAIR 4X, below the LCD
    for yy in (-0.022, 0.0, 0.022):
        a.cyl((0.004, yy, MON_H + 0.0022), 0.0040, 0.0015, "Z", (0.16, 0.16, 0.17), None, name="Button")
        a.cyl((0.004, yy, MON_H + 0.0030), 0.0034, 0.0008, "Z", (0.32, 0.33, 0.34), None, name="ButtonFace")
    for sx, rgb in ((-1, (0.10, 0.75, 0.25)), (1, (0.85, 0.12, 0.10))):
        a.cyl((0.052, sx * 0.008, MON_H + 0.0035), 0.0016, 0.0012, "Z", rgb, None, name="Led")
    a.save()


def cap(fname):
    I = box_inertia(CAP_MASS, CAP_L, CAP_W, 0.024)
    a = Asset(fname, "CalCap", "The MSA 10089321 calibration cap: a black 62 x 84 x 12 mm shell with a silicone gasket, two side snap "
              "tabs on revolute joints and a 6 mm hose barb, with one foot of 6 mm hose (8 links on spherical joints). Free "
              "articulation: cap body (root), the two tabs and the hose links. Origin: the middle of the sealing face, z up.",
              CAP_MASS, com=(0, 0, 0.002), inertia=I)
    a.articulation_root()
    f = (0.45, 0.38)
    # shell colliders: top plate, four skirt walls
    a.box((0, 0, 0.011), (CAP_L, CAP_W, 0.002), None, f, key="plastic", name="TopCol")
    for sy in (1, -1):
        a.box((0, sy * 0.04125, 0.0005), (0.059, 0.0015, 0.021), None, f, key="plastic", name="SkirtYCol")
    # the silicone gasket ring on the sealing face (compliant contact)
    for sx in (1, -1):
        a.box((sx * 0.0285, 0, -0.0005), (0.002, 0.085, 0.001), None, (0.60, 0.55), key="gasket", name="GasketXCol")
    for sy in (1, -1):
        a.box((0, sy * 0.0395, -0.0005), (0.059, 0.002, 0.001), None, (0.60, 0.55), key="gasket", name="GasketYCol")
    a.set_compliant("gasket", 2500.0, 4.0)
    # the shell look
    pts, tris = mk2.rounded_box(CAP_L, CAP_W, 0.003, 0.0015, nc=6, steps=2, centre=(0, 0, 0.0105))
    a.n += 1
    a.vmesh("TopVis", pts, tris, (0.055, 0.055, 0.060), rough=0.55)
    for sy in (1, -1):
        pts, tris = mk2.rounded_box(0.059, 0.002, 0.021, 0.0008, nc=5, steps=2, centre=(0, sy * 0.041, 0.0005))
        a.n += 1
        a.vmesh("SkirtYVis", pts, tris, (0.06, 0.06, 0.065), rough=0.55)
    # orange gasket ring
    for sx in (1, -1):
        a.box((sx * 0.0285, 0, -0.0005), (0.0022, 0.085, 0.0012), (0.85, 0.30, 0.08), None, name="Gasket")
    for sy in (1, -1):
        a.box((0, sy * 0.0395, -0.0005), (0.059, 0.0022, 0.0012), (0.85, 0.30, 0.08), None, name="Gasket")
    # two green push arrows and a white label
    for sx in (1, -1):
        a.box((sx * 0.016, 0.024, 0.0122), (0.012, 0.004, 0.0008), (0.15, 0.70, 0.30), None, name="Arrow")
    a.box((-0.022, -0.026, 0.0122), (0.020, 0.012, 0.0008), (0.92, 0.92, 0.90), None, name="CapLabel")
    a.box((-0.022, -0.026, 0.0123), (0.021, 0.003, 0.0009), (0.82, 0.12, 0.10), None, name="CapBand")
    # hose barb on top
    a.cyl((0, 0, 0.017), 0.0030, 0.012, "Z", (0.20, 0.20, 0.21), f, key="plastic", name="Barb")
    a.cyl((0, 0, 0.0230), 0.0042, 0.002, "Z", (0.20, 0.20, 0.21), None, name="BarbLip")
    # the two snap tabs: links hinged along x at the top edge, hanging 21 mm down, with an inward hook
    for side, name in ((1, "TabL"), (-1, "TabR")):
        lk = a.link(name, 0.0020, com=(0, 0, -0.0095), inertia=(8e-8, 8e-8, 2e-8),
                    pos=(0, side * 0.04375, 0.010))
        a.box((0, 0, -0.0095), (0.030, 0.0025, 0.019), None, f, key="plastic", name="TabCol")
        a.box((0, -side * 0.0025, -0.0185), (0.030, 0.0050, 0.0020), None, f, key="plastic", name="HookCol")
        # release ramp on the hook's top: the ridge slides it outward when the cap is pulled
        a.prism_x([(0.0, -0.0160), (0.0, -0.0175), (-side * 0.0050, -0.0175)], -0.015, 0.015, None, f, key="plastic", name="RampCol")
        a.box((0, 0, -0.0095), (0.030, 0.0025, 0.019), (0.07, 0.07, 0.075), None, name="TabVis")
        a.box((0, -side * 0.0025, -0.0185), (0.030, 0.0050, 0.0020), (0.07, 0.07, 0.075), None, name="HookVis")
        a.prism_x([(0.0, -0.0160), (0.0, -0.0175), (-side * 0.0050, -0.0175)], -0.015, 0.015, (0.07, 0.07, 0.075), None, name="RampVis")
        a.box((0, -side * 0.0015, -0.0200), (0.030, 0.0012, 0.0010), (0.55, 0.56, 0.56), None, name="HookChamfer")
        a.revolute(f"Tab{name[-1]}", a.base_prim, lk, axis="X", lo=-18.0, hi=18.0, stiffness=0.020, damping=0.002,
                   target=0.0, p0=(0, side * 0.04375, 0.010), p1=(0, 0, 0))
    # one foot of 6 mm calibration hose: a visual tube (no collider, no joints) from the barb to the regulator
    nodes = [(0, 0, 0.024), (0, 0.030, 0.070), (0.030, 0.030, 0.080), (0.030, -0.030, 0.050),
             (0.000, -0.080, 0.020), (-0.020, -0.090, -0.022)]
    for k in range(len(nodes) - 1):
        pts, tris = tube_mesh(nodes[k], nodes[k + 1], 0.0030, 20)
        a.n += 1
        a.vmesh(f"HoseVis{k}", pts, tris, (0.055, 0.055, 0.060), rough=0.8)
        if k == len(nodes) - 2:
            a.sphere(nodes[-1], 0.0036, (0.20, 0.20, 0.21), None, name="HoseCuff")
    a.save()


def cylinder(fname):
    a = Asset(fname, "Cylinder", "A 34 L disposable aluminium calibration-gas cylinder, 76 mm x 289 mm, 2.0 kg full, lying in a "
              "bench cradle with its C-10 valve toward the robot; fixed. Origin: the valve face on the cylinder axis, +x along the "
              "cylinder.", 2.0, com=(CYL_L / 2, 0, CYL_R), inertia=box_inertia(2.0, CYL_L, 2 * CYL_R, 2 * CYL_R))
    f = (0.35, 0.30)
    # body: a horizontal cylinder along x with domed ends; collider is one cylinder
    a.cyl((CYL_L / 2, 0, CYL_R), CYL_R, CYL_L - 0.02, "X", None, f, key="aluminium", name="CylCol")
    a.cyl((CYL_L / 2, 0, CYL_R), CYL_R, CYL_L - 0.02, "X", (0.63, 0.64, 0.66), None, name="CylVis")
    for x, sx in ((0.001, -1), (CYL_L - 0.001, 1)):
        a.sphere((x, 0, CYL_R), CYL_R * 0.999, (0.62, 0.63, 0.65), None, name="Dome")
    # C-10 valve: a brass block/neck at the near end
    a.cyl((0.012, 0, CYL_R), 0.018, 0.024, "X", None, f, key="brass", name="ValveCol")
    a.cyl((0.012, 0, CYL_R), 0.018, 0.024, "X", (0.80, 0.60, 0.20), None, name="ValveVis")
    a.cyl((-0.008, 0, CYL_R), 0.010, 0.020, "X", None, f, key="brass", name="OutletCol")
    a.cyl((-0.008, 0, CYL_R), 0.010, 0.020, "X", (0.86, 0.66, 0.24), None, name="OutletVis")
    for k in range(6):
        ph = 2 * math.pi * k / 6
        a.box((-0.016, 0.010 * math.cos(ph), CYL_R + 0.010 * math.sin(ph)), (0.010, 0.006, 0.006),
              (0.86, 0.66, 0.24), None, name="Hex", rot=(math.degrees(ph), 0, 0))
    # a white cylinder label with a red band and four gas squares (visible from the top camera)
    a.box((0.150, 0.000, CYL_R + 0.0382), (0.120, 0.046, 0.0012), (0.92, 0.92, 0.90), None, name="Label")
    a.box((0.150, 0.000, CYL_R + 0.0388), (0.120, 0.008, 0.0014), (0.82, 0.10, 0.10), None, name="LabelBand")
    for k, rgb in enumerate(((0.92, 0.80, 0.10), (0.93, 0.48, 0.08), (0.85, 0.12, 0.10), (0.10, 0.55, 0.90))):
        a.box((0.150 + (k - 1.5) * 0.026, 0.000, CYL_R + 0.0388), (0.018, 0.012, 0.0014), rgb, None, name="GasSquare")
    # the bench cradle: base plate, two V blocks and a strap
    a.box((0.145, 0, 0.005), (0.290, 0.120, 0.010), None, f, key="steel", name="CradleBase")
    a.box((0.145, 0, 0.005), (0.290, 0.120, 0.010), (0.18, 0.19, 0.20), None, name="CradleBaseVis")
    for x in (0.045, 0.245):
        for sy in (1, -1):
            a.box((x, sy * 0.045, 0.030), (0.040, 0.020, 0.060), None, f, key="steel", name="VBlock")
            a.box((x, sy * 0.045, 0.030), (0.040, 0.020, 0.060), (0.22, 0.23, 0.24), None, name="VBlockVis", rot=(0, 0, 0))
    a.box((0.145, 0, 0.080), (0.020, 0.110, 0.004), None, f, key="steel", name="Strap")
    a.box((0.145, 0, 0.080), (0.020, 0.110, 0.004), (0.55, 0.56, 0.58), None, name="StrapVis")
    a.save()


def regulator(fname):
    a = Asset(fname, "Regulator", "A 0.25 L/min fixed-flow calibration-gas regulator of the BW REG-0.5 class, 0.239 kg: C-10 inlet nut, stainless "
              "body, 6 mm outlet barb and a knurled on/off knob on a vertical revolute joint. Fixed root. Origin: the inlet face on the "
              "cylinder axis, +x into the regulator body (toward -x world when spawned), z up.", REG_MASS,
              com=(-0.025, 0, 0.010), inertia=box_inertia(REG_MASS, 0.070, 0.050, 0.040))
    a.articulation_root()
    f = (0.35, 0.30)
    # inlet nut and body
    a.cyl((0.004, 0, 0), 0.016, 0.018, "X", None, f, key="brass", name="NutCol")
    a.cyl((0.004, 0, 0), 0.016, 0.018, "X", (0.78, 0.58, 0.22), None, name="NutVis")
    for k in range(6):
        ph = 2 * math.pi * k / 6
        a.box((0.004, 0.014 * math.cos(ph), 0.014 * math.sin(ph)), (0.018, 0.006, 0.006), (0.86, 0.66, 0.24), None,
              name="NutHex", rot=(math.degrees(ph), 0, 0))
    a.box((-0.026, 0, 0), (0.042, 0.044, 0.040), None, f, key="steel", name="BodyCol")
    pts, tris = mk2.rounded_box(0.042, 0.044, 0.040, 0.003, nc=6, steps=3, centre=(-0.026, 0, 0))
    a.n += 1
    a.vmesh("BodyVis", pts, tris, (0.55, 0.56, 0.58), rough=0.35, metal=0.6)
    # outlet barb toward +y
    a.cyl((-0.030, 0.030, 0), 0.0060, 0.020, "Y", None, f, key="brass", name="BarbCol")
    a.cyl((-0.030, 0.030, 0), 0.0060, 0.020, "Y", (0.80, 0.62, 0.24), None, name="BarbVis")
    a.cyl((-0.030, 0.040, 0), 0.0080, 0.002, "Y", (0.80, 0.62, 0.24), None, name="BarbLip")
    # labels and the OPEN/CLOSE marks on the body top
    a.box((-0.026, 0, 0.0205), (0.028, 0.030, 0.0010), (0.92, 0.92, 0.90), None, name="RegLabel")
    a.box((-0.026, 0, 0.0206), (0.029, 0.010, 0.0011), (0.82, 0.10, 0.10), None, name="RegBand")
    a.box((-0.026, 0.018, 0.0208), (0.020, 0.003, 0.0008), (0.15, 0.70, 0.30), None, name="OpenMark")
    a.box((-0.008, 0.000, 0.0208), (0.003, 0.020, 0.0008), (0.82, 0.10, 0.10), None, name="CloseMark")
    # the knurled on/off knob on a revolute joint
    lk = a.link("Knob", 0.012, com=(0, 0, 0.007), inertia=box_inertia(0.012, 0.032, 0.032, 0.014), pos=(-0.026, 0, 0.020))
    a.cyl((0, 0, 0.007), 0.016, 0.014, "Z", None, f, key="plastic", name="KnobCol")
    a.cyl((0, 0, 0.007), 0.016, 0.014, "Z", (0.08, 0.08, 0.09), None, name="KnobVis")
    for k in range(24):
        ph = 2 * math.pi * k / 24
        a.box((0.0165 * math.cos(ph), 0.0165 * math.sin(ph), 0.007), (0.003, 0.003, 0.012), (0.14, 0.14, 0.15), None,
              name="Knurl", rot=(0, 0, math.degrees(ph)))
    a.box((0.007, 0, 0.0145), (0.014, 0.0035, 0.0012), (0.85, 0.12, 0.10), None, name="Pointer")
    a.box((-0.026, 0, 0.0148), (0.005, 0.005, 0.0014), (0.85, 0.12, 0.10), None, name="PointerHub")
    a.revolute("Knob", a.base_prim, lk, axis="Z", lo=-2.0, hi=92.0, stiffness=0.0, damping=0.004, target=0.0,
               p0=(-0.026, 0, 0.020), p1=(0, 0, 0))
    a.save()


def cap_rest(fname):
    a = Asset(fname, "CapRest", "The calibration cap's bench rest: a base, two uprights and a top plate, fixed. Origin: on the "
              "bench, the cap's sealing plane 60 mm up.", 0.30, com=(0, 0, 0.030), inertia=box_inertia(0.30, 0.080, 0.090, 0.060))
    f = (0.35, 0.30)
    a.box((0, 0, 0.005), (0.080, 0.090, 0.010), None, f, key="steel", name="BaseCol")
    a.box((0, 0, 0.005), (0.080, 0.090, 0.010), (0.20, 0.21, 0.22), None, name="BaseVis")
    for sx in (1, -1):
        a.box((sx * 0.030, 0, 0.0295), (0.010, 0.080, 0.039), None, f, key="steel", name="PostCol")
        a.box((sx * 0.030, 0, 0.0295), (0.010, 0.080, 0.039), (0.20, 0.21, 0.22), None, name="PostVis")
    a.box((0, 0, 0.046), (0.080, 0.060, 0.006), None, f, key="rubber", name="TopCol")
    a.box((0, 0, 0.046), (0.080, 0.060, 0.006), (0.08, 0.08, 0.09), None, name="TopVis")
    a.box((0, 0, 0.0495), (0.060, 0.032, 0.001), (0.80, 0.30, 0.08), None, name="Pad")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    monitor("monitor.usda")
    cap("cap.usda")
    cylinder("cylinder.usda")
    regulator("regulator.usda")
    cap_rest("cap_rest.usda")
