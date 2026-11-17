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

    def color(self, key, rgb, rough=0.6, metal=0.0, emissive=None, opacity=1.0):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        if emissive is not None:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        if opacity < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
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

    def vmesh(self, name, pts, tris, rgb, center=(0, 0, 0), rough=0.6, metal=0.0, emissive=None, opacity=1.0):
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
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.color("m%02d%02d%02d_%d_%d_%s_%d" % (*[int(c * 99) for c in rgb], int(rough * 10), int(metal * 10), "e" if emissive else "n", int(opacity * 10)), rgb, rough, metal, emissive, opacity))

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


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))
# ======================================================================================================================
# Match-light-candle assets. Metres, kilograms, Z up.  (the base class and mesh helpers above are shared with the other tasks)
#   matchbox.usda   : Swan Vestas class safety-match box, 52 x 36 x 16 mm (L x W x T), 12 g with its matches (box 6 g + about 50 matches at 0.12 g, estimate).
#                     An articulation with a free root: the box, plus the -y striker panel (48 x 12 x 0.5 mm) on a 2 mm prismatic joint
#                     held out by a spring: its deflection tells the task the head is pressed on it. Origin: the box's middle, long side along x.
#   match.usda      : one safety match, 47 mm long, 2.1 mm square splint, 3.2 mm head, 0.12 g (estimate). Origin: the base end's middle, z along the stick.
#   flame_match.usda, flame_candle.usda : the flames (visual only, kinematic; the task moves them): origin at the flame's base, z up.
#   candle.usda     : a 22 x 250 mm dinner (taper) candle, 80 g (estimate), wick 8 mm proud. Origin: the base end's middle, z up.
#   holder.usda     : a low brass chamberstick, 91 mm dish, socket 22.8 mm across and 25 mm deep, rim 30 mm up, 0.35 kg (free on the table).
#                     Origin: table level, the socket's axis.
#   ashtray.usda    : a round ceramic ashtray, 104 mm across, 24 mm tall, bowl 84 mm across, 0.20 kg (free). Origin: table level, the bowl's axis.
# ======================================================================================================================
from scipy.spatial import ConvexHull

MM = 0.001


def rbox_mesh(sx, sy, sz, r):
    """A box with bevelled edges (closed, outward-wound): the convex hull of each corner pulled in by ``r`` along each axis (mm-scale sizes in metres)."""
    pts = []
    for ix in (-1, 1):
        for iy in (-1, 1):
            for iz in (-1, 1):
                cx, cy, cz = ix * sx / 2, iy * sy / 2, iz * sz / 2
                pts += [(cx - ix * r, cy, cz), (cx, cy - iy * r, cz), (cx, cy, cz - iz * r)]
                pts += [(cx - ix * r, cy - iy * r, cz), (cx - ix * r, cy, cz - iz * r), (cx, cy - iy * r, cz - iz * r)]
    pts = np.array(pts)
    hull = ConvexHull(pts)
    idx = sorted(set(hull.vertices))
    remap = {v: i for i, v in enumerate(idx)}
    P = [tuple(pts[v]) for v in idx]
    T = [(remap[a], remap[b], remap[c]) for a, b, c in hull.simplices]
    return P, _orient(P, T)


def poly_prism(poly, z0, z1):
    """A closed prism over a convex polygon (list of (x, y)), z0..z1."""
    k = len(poly)
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    tris = []
    for i in range(1, k - 1):
        tris += [(0, i, i + 1), (k, k + i + 1, k + i)]
    for i in range(k):
        j = (i + 1) % k
        tris += [(i, j, k + j), (i, k + j, k + i)]
    return pts, _orient(pts, tris)


def ellipse(ax, ay, n=64):
    return [(ax * math.cos(2 * math.pi * i / n), ay * math.sin(2 * math.pi * i / n)) for i in range(n)]


def vis_rbox(a, name, center, size, r, rgb, rough=0.6, metal=0.0):
    pts, tris = rbox_mesh(*size, r)
    a.n += 1
    a.vmesh(name, pts, tris, rgb, center, rough, metal)


def vis_plate(a, name, center, ax, ay, z0, z1, rgb, rough=0.6, n=64):
    pts, tris = poly_prism(ellipse(ax, ay, n), z0, z1)
    a.n += 1
    a.vmesh(name, pts, tris, rgb, center, rough)


def speckles(a, name, cx, cy, cz, sx, sz, count, seed, rgb, axis_normal="y"):
    """Little light dots on a striker strip (visual only), at pseudo-random places within sx (x) by sz (z)."""
    rng = np.random.default_rng(seed)
    for i in range(count):
        x = cx + (rng.random() - 0.5) * (sx - 2 * MM)
        z = cz + (rng.random() - 0.5) * (sz - 2 * MM)
        s = (0.5 + rng.random() * 0.6) * MM
        pts, tris = rbox_mesh(s, 0.12 * MM, s, 0.1 * MM)
        a.n += 1
        a.vmesh(name, pts, tris, rgb, (x, cy, z), 0.9)


# ---- the match box ----
BX, BY, BZ = 0.052, 0.036, 0.016
PANEL_X, PANEL_Z, PANEL_T = 0.048, 0.012, 0.0005
PANEL_K = 4000.0  # N/m: the striker panel's spring (a compliant contact sensor: the sim needs 0.1 mm of push, not a force threshold)
PANEL_TRAVEL = 0.002


def matchbox(fname):
    m = 0.012
    a = Asset(fname, "Matchbox", "A Swan Vestas class safety-match box, 52 x 36 x 16 mm, 12 g with its matches, lying flat (long side along x). Its -y long side carries "
              "a striker strip (48 x 12 x 0.5 mm, feathered ends) on a 2 mm spring panel (4000 N/m): the panel's push-in tells the task the head is pressed on the strip. Origin: the box's middle.", m - 0.001,
              com=(0, 0, 0), inertia=box_inertia(m - 0.001, BX, BY, BZ))
    a.articulation_root()
    f = (0.50, 0.45)
    # the box's collider (invisible): the whole box except a 2.5 mm deep pocket under the striker strip (48 x 12 mm), which the spring panel sinks into
    pocket = 0.0025
    a.box((0, (pocket) / 2, 0), (BX, BY - pocket, BZ), None, f, key="box", name="Body")
    for sgn in (1, -1):
        a.box((0, -BY / 2 + pocket / 2, sgn * (PANEL_Z / 2 + 1.0 * MM)), (BX, pocket, 2.0 * MM), None, f, key="box", name="Rail")
        a.box((sgn * (PANEL_X / 2 + 1.0 * MM), -BY / 2 + pocket / 2, 0), (2.0 * MM, pocket, PANEL_Z), None, f, key="box", name="End")
    navy, red, white, gold = (0.07, 0.14, 0.40), (0.72, 0.09, 0.09), (0.95, 0.94, 0.90), (0.86, 0.68, 0.22)
    vis_rbox(a, "Sleeve", (0, 0, 0), (BX, BY, BZ), 0.6 * MM, navy, 0.7)
    for sgn in (1, -1):  # the tray's end plates, flush with the ends
        vis_rbox(a, "Tray", (sgn * (BX / 2 - 0.2 * MM), 0, 0), (0.4 * MM, BY - 2 * MM, BZ - 2 * MM), 0.1 * MM, (0.88, 0.80, 0.58), 0.8)
    for sgn in (1, -1):  # labels on both big faces: gold border, white oval, navy field, red band
        zc = sgn * BZ / 2
        vis_plate(a, "Gold", (0, 0, zc), 21.5 * MM, 13.3 * MM, 0.0, sgn * 0.12 * MM, gold, 0.4)
        vis_plate(a, "White", (0, 0, zc), 20.5 * MM, 12.3 * MM, 0.0, sgn * 0.22 * MM, white, 0.6)
        vis_plate(a, "Field", (0, 0, zc), 18.4 * MM, 10.3 * MM, 0.0, sgn * 0.32 * MM, navy, 0.7)
        pts, tris = rbox_mesh(30 * MM, 3.2 * MM, 0.12 * MM, 0.04 * MM)
        a.n += 1
        a.vmesh("Band", pts, tris, red, (0, 0, zc + sgn * 0.40 * MM), 0.6)
        vis_plate(a, "Swan", (-2 * MM, 0, zc), 5.0 * MM, 3.0 * MM, sgn * 0.30 * MM, sgn * 0.52 * MM, white, 0.5)  # the swan's body (a stylised oval)
        vis_plate(a, "Neck", (3.5 * MM, 0, zc), 1.0 * MM, 4.6 * MM, sgn * 0.30 * MM, sgn * 0.52 * MM, white, 0.5)
    # the fixed striker on the +y side (visual)
    pts, tris = rbox_mesh(PANEL_X, 0.5 * MM, PANEL_Z, 0.12 * MM)
    a.n += 1
    a.vmesh("StrikerFixed", pts, tris, (0.30, 0.10, 0.07), (0, BY / 2 + 0.2 * MM, 0), 0.95)
    speckles(a, "SpeckleF", 0, BY / 2 + 0.47 * MM, 0, PANEL_X, PANEL_Z, 40, 3, (0.62, 0.55, 0.50))
    # the sprung striker panel on the -y side
    root = a.root.GetPrim()
    y0 = -(BY / 2 + PANEL_T / 2)
    lk = a.link("Panel", 0.001, inertia=(1e-9, 1e-9, 1e-9), pos=(0, y0, 0))
    ramp = 2.0 * MM  # the strip's feathered ends (a glued abrasive paper): the head rides up them
    trap = [(-PANEL_X / 2 - ramp, PANEL_T / 2), (-PANEL_X / 2, -PANEL_T / 2), (PANEL_X / 2, -PANEL_T / 2), (PANEL_X / 2 + ramp, PANEL_T / 2)]
    a.prism_z(trap, -PANEL_Z / 2, PANEL_Z / 2, None, (0.20, 0.18), key="striker", name="PanelCol")  # collider (invisible)
    pts, tris = poly_prism(trap, -PANEL_Z / 2, PANEL_Z / 2)
    a.n += 1
    a.vmesh("PanelVis", pts, tris, (0.30, 0.10, 0.07), (0, 0, 0), 0.95)
    speckles(a, "Speckle", 0, -PANEL_T / 2 - 0.07 * MM, 0, PANEL_X, PANEL_Z, 40, 5, (0.62, 0.55, 0.50))
    a.prismatic("PanelJoint", root, lk, axis="Y", lo=0.0, hi=PANEL_TRAVEL, stiffness=PANEL_K, damping=3.0, target=0.0, p0=(0, y0, 0), p1=(0, 0, 0))
    a.save()


# ---- one match ----
M_L, M_S, M_HR = 0.047, 0.0021, 0.0016
M_MASS = 0.002


def match(fname):
    m = M_MASS
    a = Asset(fname, "Match", "A safety match: 47 mm long, 2.1 mm square splint, 3.2 mm head, 0.12 g (estimate: 0.45 g/cm3 splint plus head). Origin: the base end's middle, z along the stick, head up.",
              m, com=(0, 0, 0.023), inertia=(m * M_L**2 / 12, m * M_L**2 / 12, m * M_S**2 / 6))
    f = (1.2, 1.1)  # the splint's friction against the finger pads: raised for the weak pinch on a 2 mm stick (a rubber pad on dry wood is 0.8-1.2)
    fh = (0.20, 0.18)  # the head against the striker strip (real about 0.4; the strip has the same value so PhysX averages to 0.2; 0.4 was tried on Isaac: see the README)
    hz = M_L - M_HR  # the head sphere's centre
    a.box((0, 0, (hz) / 2), (M_S, M_S, hz), None, f, key="match", name="Stick")  # collider (invisible)
    a.sphere((0, 0, hz), M_HR, None, fh, key="head", name="Head")  # collider (invisible)
    vis_rbox(a, "StickVis", (0, 0, (M_L - 0.0042) / 2), (M_S, M_S, M_L - 0.0042), 0.2 * MM, (0.86, 0.72, 0.50), 0.85)
    prof = [(0, M_L - 4.4 * MM), (1.1 * MM, M_L - 4.2 * MM), (1.55 * MM, M_L - 2.6 * MM), (M_HR, M_L - 1.6 * MM), (1.35 * MM, M_L - 0.5 * MM), (0.7 * MM, M_L - 0.05 * MM), (0, M_L)]
    pts, tris = lathe_axis(prof, 48)
    a.n += 1
    a.vmesh("HeadVis", pts, tris, (0.72, 0.10, 0.08), rough=0.55)
    a.save()


# ---- flames (visual only, kinematic; the task sets their pose) ----
def flame(fname, name, height, width, doc):
    a = Asset(fname, name, doc, 1e-4, com=(0, 0, 0), inertia=(1e-9, 1e-9, 1e-9))
    k = height / 18.5
    w = width / 7.0
    prof = [(0, 0), (1.6, 0.3), (2.9, 1.8), (3.5, 4.0), (3.3, 7.0), (2.5, 10.5), (1.5, 14.0), (0.6, 17.0), (0, 18.5)]
    pts, tris = lathe_axis([(r * w * MM, z * k * MM) for r, z in prof], 48)
    a.n += 1
    a.vmesh("Outer", pts, tris, (1.0, 0.62, 0.12), rough=1.0, emissive=(1.0, 0.55, 0.08), opacity=0.78)
    prof2 = [(0, 0.2), (1.4, 0.8), (2.0, 2.2), (1.7, 4.5), (0.8, 6.5), (0, 7.5)]
    pts, tris = lathe_axis([(r * w * MM, z * k * MM) for r, z in prof2], 48)
    a.n += 1
    a.vmesh("Core", pts, tris, (0.25, 0.35, 0.95), rough=1.0, emissive=(0.20, 0.30, 0.90), opacity=0.85)
    prof3 = [(0, 3.5), (1.2, 4.5), (1.6, 7.5), (1.0, 11.0), (0, 13.0)]
    pts, tris = lathe_axis([(r * w * MM, z * k * MM) for r, z in prof3], 48)
    a.n += 1
    a.vmesh("Hot", pts, tris, (1.0, 0.95, 0.55), rough=1.0, emissive=(1.0, 0.92, 0.45), opacity=0.9)
    a.save()


# ---- the candle ----
C_R, C_H, C_RT = 0.011, 0.250, 0.0099
WICK_UP = 0.008
C_MASS = 0.080


def candle(fname):
    m = C_MASS
    a = Asset(fname, "Candle", "A dinner (taper) candle: 22 mm at the base tapering to 19.8 mm, 250 mm tall, 80 g (estimate), a wick 8 mm proud of the top. Origin: the base end's middle, z up.", m,
              com=(0, 0, 0.123), inertia=(m / 12 * (3 * C_R**2 + C_H**2), m / 12 * (3 * C_R**2 + C_H**2), m * C_R**2 / 2))
    f = (0.40, 0.35)
    a.cyl((0, 0, 0.0125), C_R - 0.0004, 0.025, "Z", None, f, key="wax", name="Foot")  # collider only (invisible)
    a.cyl((0, 0, 0.025 + (C_H - 0.025) / 2), 0.0103, C_H - 0.025, "Z", None, f, key="wax", name="Body")  # collider only (invisible)
    mm = MM
    prof = [(0, 0), (10.0 * mm, 0), (C_R, 1.0 * mm), (C_R, 25 * mm), (C_RT, C_H - 3.0 * mm), (C_RT - 0.3 * mm, C_H - 0.8 * mm), (C_RT - 1.2 * mm, C_H), (C_RT - 3.0 * mm, C_H - 0.5 * mm),
            (3.0 * mm, C_H - 3.0 * mm), (1.0 * mm, C_H - 4.0 * mm), (0, C_H - 4.0 * mm)]
    pts, tris = lathe_axis(prof, 96)
    a.n += 1
    a.vmesh("WaxVis", pts, tris, (0.95, 0.92, 0.80), rough=0.4)
    wick = [(0, C_H - 4.0 * mm), (0.8 * mm, C_H - 4.0 * mm), (0.8 * mm, C_H + 4.0 * mm), (0, C_H + 4.0 * mm)]
    pts, tris = lathe_axis(wick, 24)
    a.n += 1
    a.vmesh("WickVis", pts, tris, (0.84, 0.78, 0.60), rough=0.9)
    tip = [(0, C_H + 3.8 * mm), (0.8 * mm, C_H + 3.8 * mm), (0.7 * mm, C_H + 7.4 * mm), (0.35 * mm, C_H + WICK_UP * 1000 * mm), (0, C_H + WICK_UP * 1000 * mm)]
    pts, tris = lathe_axis(tip, 24)
    a.n += 1
    a.vmesh("WickTipVis", pts, tris, (0.07, 0.06, 0.06), rough=0.9)
    a.save()


# ---- the brass holder ----
SOCKET_R, SOCKET_DEPTH, LEAD, RIM = 0.0114, 0.025, 0.003, 0.030
H_MASS = 0.35


def socket_colliders(a, f, key, n=24):
    t0 = RIM - SOCKET_DEPTH
    wall_t = 0.0055
    width = 2 * math.pi * SOCKET_R / n * 1.3
    for k in range(n):
        ph = 2 * math.pi * k / n
        rc = SOCKET_R + wall_t / 2
        a.box((rc * math.cos(ph), rc * math.sin(ph), t0 + SOCKET_DEPTH / 2), (wall_t, width, SOCKET_DEPTH), None, f, key=key, name="Wall", rot=(0, 0, math.degrees(ph)))
    a.cyl((0, 0, t0 - 0.0015), SOCKET_R + 0.002, 0.003, "Z", None, f, key=key, name="Floor")
    skin, sl = 0.0015, math.radians(45.0)
    length = LEAD * math.sqrt(2.0)
    rm = SOCKET_R + LEAD / 2 + 0.5 * skin * math.sin(sl) + 0.0006
    zm = RIM + LEAD / 2 - 0.5 * skin * math.cos(sl)
    for k in range(n):
        ph = 2 * math.pi * k / n
        a.n += 1
        g = UsdGeom.Cube.Define(a.stage, f"{a.cur}/Lead{a.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(rm * math.cos(ph), rm * math.sin(ph), zm))
        g.AddRotateZOp().Set(math.degrees(ph))
        g.AddRotateYOp().Set(-45.0)
        g.AddScaleOp().Set(Gf.Vec3f(length, 2 * math.pi * SOCKET_R / n * 1.6, skin))
        a._finish(g, None, f, key, mesh=False)


def holder(fname):
    m = H_MASS
    a = Asset(fname, "Holder", "A low brass chamberstick: 91 mm drip dish, socket 22.8 mm across and 25 mm deep (3 mm 45 degree lead-in) with its rim 30 mm up, 0.35 kg, free on the table. "
              "Origin: table level, the socket's axis.", m, com=(0, 0, 0.011), inertia=(1.2e-4, 1.2e-4, 1.6e-4))
    f = (0.35, 0.30)
    a.cyl((0, 0, 0.0025), 0.0455, 0.005, "Z", None, f, key="brass", name="Dish")
    socket_colliders(a, f, "brass")
    prof_mm = [(0, 0), (39, 0), (43.5, 0.5), (45.5, 1.6), (45.5, 4.6), (45.0, 5.8), (43.8, 6.3), (42.6, 5.7), (41.0, 4.4), (34, 3.8), (24, 3.9), (19.5, 4.8), (17.6, 6.4), (16.4, 9.0),
               (15.7, 13.0), (15.4, 18.0), (15.6, 22.5), (16.2, 26.0), (16.7, 28.4), (16.8, 29.6), (16.1, 30.4), (14.4, 30.2), (14.4, 30.0), (SOCKET_R / MM, (RIM - LEAD) / MM),
               (SOCKET_R / MM, (RIM - SOCKET_DEPTH) / MM), (0, (RIM - SOCKET_DEPTH) / MM)]
    pts, tris = lathe_axis([(r * MM, z * MM) for r, z in prof_mm], 96)
    a.n += 1
    a.vmesh("BrassVis", pts, tris, (0.82, 0.64, 0.26), rough=0.32, metal=0.9)
    ring = [(15.0, 15.6), (16.7, 15.9), (17.3, 16.9), (16.7, 17.9), (15.0, 18.2)]  # a bead ring round the cup
    pts, tris = lathe([(r * MM, z * MM) for r, z in ring], 96)
    a.n += 1
    a.vmesh("BeadVis", pts, tris, (0.86, 0.68, 0.28), rough=0.3, metal=0.9)
    a.save()


# ---- the ashtray ----
A_R, A_H, A_IN, A_FLOOR = 0.052, 0.024, 0.042, 0.006
A_MASS = 0.20


def ashtray(fname):
    m = A_MASS
    a = Asset(fname, "Ashtray", "A round ceramic ashtray: 104 mm across, 24 mm tall, bowl 84 mm across and 18 mm deep, 0.20 kg, free on the table. Origin: table level, the bowl's axis.", m,
              com=(0, 0, 0.011), inertia=(m * (3 * A_R**2 + A_H**2) / 12, m * (3 * A_R**2 + A_H**2) / 12, m * A_R**2 / 2))
    f = (0.45, 0.40)
    a.cyl((0, 0, A_FLOOR / 2), A_R - 0.0005, A_FLOOR, "Z", None, f, key="clay", name="Base")
    n = 24
    wall_t = A_R - A_IN
    width = 2 * math.pi * (A_IN + wall_t / 2) / n * 1.3
    for k in range(n):
        ph = 2 * math.pi * k / n
        rc = A_IN + wall_t / 2
        a.box((rc * math.cos(ph), rc * math.sin(ph), A_FLOOR + (A_H - A_FLOOR) / 2), (wall_t, width, A_H - A_FLOOR), None, f, key="clay", name="Wall", rot=(0, 0, math.degrees(ph)))
    prof_mm = [(0, 0), (47, 0), (51, 1.2), (52, 3.5), (52, 19.5), (51.4, 22.4), (49.5, 24.0), (46, 24.0), (43.4, 23.4), (42.0, 22.0), (42.0, 9.5), (41.2, 7.6), (39.2, 6.4), (36.5, 6.0), (0, 6.0)]
    pts, tris = lathe_axis([(r * MM, z * MM) for r, z in prof_mm], 96)
    a.n += 1
    a.vmesh("GlazeVis", pts, tris, (0.93, 0.93, 0.89), rough=0.18)
    band = [(52.15, 9.0), (52.15, 12.5), (51.5, 12.5), (51.5, 9.0)]
    pts, tris = lathe([(r * MM, z * MM) for r, z in band], 96)
    a.n += 1
    a.vmesh("BandVis", pts, tris, (0.12, 0.25, 0.55), rough=0.2)
    for k in range(3):  # three cigarette rests: shallow dark grooves across the rim
        ph = math.radians(90 + 120 * k)
        pts, tris = rbox_mesh(10 * MM, 7 * MM, 1.0 * MM, 0.2 * MM)
        a.n += 1
        c, s = math.cos(ph), math.sin(ph)
        pts = [(x * c - y * s + 46.5 * MM * c, x * s + y * c + 46.5 * MM * s, z) for x, y, z in pts]
        a.vmesh("RestVis", pts, tris, (0.55, 0.55, 0.52), (0, 0, 23.7 * MM), 0.4)
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    matchbox("matchbox.usda")
    match("match.usda")
    flame("flame_match.usda", "FlameMatch", 20.0, 7.0, "The match's flame (visual only; the task moves it): 20 mm tall, 7 mm wide. Origin: the flame's base middle, z up.")
    flame("flame_candle.usda", "FlameCandle", 28.0, 9.0, "The candle's flame (visual only; the task moves it): 28 mm tall, 9 mm wide. Origin: the flame's base middle, z up.")
    candle("candle.usda")
    holder("holder.usda")
    ashtray("ashtray.usda")
