"""Writes this task's USD assets with pxr (boxes, cylinders, closed 96-sided round meshes; nothing from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up.

The brake rotor is shown in service: a worn Brembo 09.7012.11 (280 mm dia, 22.0 mm new, 19 mm discard) bolted to a
front hub and steering knuckle inside the wheel well, with the caliper unbolted and its bracket still on the knuckle.
The hub/knuckle and caliper bracket are fixed context; the micrometer only ever touches the rotor's friction faces."""
import math
import numpy as np
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001
METALLIC = set()


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

    def color(self, key, rgb, rough=0.6):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        metal = tuple(rgb) in METALLIC
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.9 if metal else 0.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set((0.5 if tuple(rgb) == (0.62, 0.62, 0.66) else 0.28) if metal else rough)
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


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


def lathe_y(profile, n=96):
    """Revolve a closed (r, y) profile about the Y axis (a closed, outward-wound 96-sided mesh)."""
    pts, tris = lathe(profile, n)
    return [(x, z, -y) for x, y, z in pts], tris  # (x, y, z) -> (x, z, -y): the profile's second coordinate becomes world y


# ======================================================================================================================
# Brake-rotor micrometer task assets. Metres, kilograms, Z up.
#   micrometer.usda : a 0-25 mm outside micrometer with ratchet stop and lock lever (Mitutoyo 103-137 class: 175 g, 18 mm thimble,
#                     6.35 mm carbide faces, 0.5 mm pitch). A free articulation: the frame is the root, the spindle on a prismatic
#                     joint along y (drive-limited to the 10 N measuring force), the thimble and the ratchet knob on revolute joints
#                     about y, the lock lever on a revolute joint about x. Origin: the anvil face's centre; the measuring axis is y
#                     (the anvil at +y, the thimble end toward -y), the frame's throat opens toward +x, z up.
#   rotor.usda      : a worn Brembo 09.7012.11 280 mm dia front brake rotor (22.0 mm new, 21.50 mm at the inspection
#                     band in the sim, 19 mm discard; 5.74 kg estimated worn mass) installed on its hub, axis along y
#                     (fixed). Origin: the middle of the rotor's thickness, on its axis. Eight scribe marks at radius
#                     135 mm show the inspection points; the 6 mm dot marks the thinnest one.
#   hub_knuckle.usda: the front wheel hub, bearing boss, steering knuckle and five wheel studs (fixed context).
#                     Origin: the rotor's hub centre, same as rotor.usda.
#   caliper_bracket.usda: the caliper carrier still bolted to the knuckle at the bottom of the rotor (fixed context);
#                     the caliper and pads are removed for the thickness check. Same origin.
#   rest.usda       : the micrometer's bench rest (fixed): a 6061-T6 base and two 46 x 6 x 146 mm uprights the frame lies
#                     across, with a 36 mm clear gap for the lower finger pad. Origin: on the table under the anvil face's centre.
# ======================================================================================================================
G_OPEN = 25.0 * MM  # spindle face to anvil face with the spindle fully open (0-25 mm range)
NEW_MM = 22.0  # the part's new thickness (Brembo 09.7012.11), for reference only
DISCARD_MM = 19.0  # the thickness stamped as the discard limit on this rotor
THICK = 21.50 * MM  # the worn in-service rotor's measured thickness at the inspection band in the simulator
INSPECTION_R = 0.135  # radius of the eight circumferential inspection points (m; 5 mm inboard of the friction rim)
ROT_R = 0.140
HUB_R = 0.0625  # hat outer radius (125 mm, a visual approximation of the real hat)
BORE_R = 0.0325  # 65 mm centre bore
HAT_D = 0.015  # hat depth behind the inboard friction face: 15 + 22 = 37 mm overall height
PLATE_T = 6.0 * MM
CHAM = 1.0 * MM
Z_AX = 0.152  # the measuring axis' height above the table (= the rotor's hub height)
SPINE_X0, SPINE_X1 = -39.3 * MM, -30.3 * MM
ARM_X1 = 9.5 * MM
FRAME_H = 12.0 * MM  # the frame's thickness (z)
Y_ANV_ARM = (4.0 * MM, 13.0 * MM)  # the anvil arm (anvil stub 4 mm out of it)
Y_SPD_ARM = (-35.0 * MM, -26.0 * MM)
SLEEVE_R, SLEEVE_END = 4.75 * MM, -68.0 * MM
LEV_X, LEV_Y, LEV_H = -0.027, -0.0305, 0.016  # lock lever: pivot on the spindle arm's top face, 16 mm tall
LEV_UNLOCK, LEV_LOCK = 35.0, -35.0  # degrees about +x (positive leans the top toward -y, the thimble)
LEV_PEAK = 0.03  # N m: the click's peak torque (an estimate)
PITCH = 0.5 * MM
MEAS_FORCE = 10.0  # N: the ratchet limits the spindle thrust to the top of the 5-10 N measuring force (lag x K_SPINDLE = 6 N); a tilted frame can push the spindle back open
RATCHET_SLIP = 0.006  # N m: the slip torque of the ratchet (10 N x thread torque arm: 10 N x (0.08 mm + 0.15 x 3.5 mm)).
K_SPINDLE = 2.0e5  # N/m: the drive that pushes the spindle (the force saturates at MEAS_FORCE at 0.05 mm of lag)
CHROME = (0.80, 0.81, 0.84)
ENAMEL = (0.12, 0.16, 0.30)
ENAMEL_M = (0.62, 0.64, 0.68)  # the satin steel of the lock pivot
BLACK = (0.07, 0.07, 0.08)
CARBIDE = (0.32, 0.33, 0.36)
LEVER_RED = (0.82, 0.18, 0.10)
IRON = (0.30, 0.30, 0.32)
IRON_FACE = (0.62, 0.62, 0.66)
IRON_WEAR = (0.50, 0.50, 0.53)  # the worn friction track: duller than the as-cast face
IRON_HAT = (0.12, 0.12, 0.13)  # the hat is black-coated
KNUCKLE = (0.38, 0.39, 0.41)  # cast-iron steering knuckle and hub flange
BRACKET = (0.26, 0.28, 0.31)  # painted-steel caliper carrier
STEEL = (0.66, 0.67, 0.70)  # the wheel studs
STAND = (0.10, 0.30, 0.55)  # blue anodised bench rest
HEAT = (0.05, 0.05, 0.06)  # the frame's heat-insulating plates
BRASS = (0.78, 0.60, 0.25)
METALLIC.update({CHROME, IRON_FACE, IRON_WEAR, CARBIDE, ENAMEL_M, BRASS, STEEL})


def _rounded_arm(x0, x1, y0, y1, nseg=16):
    """A frame arm's outline in x-y: a rectangle with a half-round tip at x1 (counter-clockwise)."""
    yc, r = (y0 + y1) / 2, (y1 - y0) / 2
    pts = [(x0, y0), (x1 - r, y0)]
    pts += [(x1 - r + r * math.sin(math.pi * i / nseg), yc - r * math.cos(math.pi * i / nseg)) for i in range(1, nseg)]
    pts += [(x1 - r, y1), (x0, y1)]
    return pts


def micrometer(fname):
    m_f, m_s, m_t, m_r, m_l = 0.118, 0.015, 0.030, 0.010, 0.003
    a = Asset(fname, "Micrometer", "A 0-25 mm outside micrometer with ratchet stop and lock lever (Mitutoyo 103-137 class: 175 g, 18 mm thimble, 6.35 mm carbide faces, "
              "0.5 mm pitch; frame dimensions typical). A free articulation: the frame is the root; the spindle slides along y (prismatic, drive-limited to the 10 N "
              "measuring force), the thimble and the ratchet knob turn about y, the lock lever turns about x. Origin: the anvil face's centre; the anvil is at +y, the "
              "thimble toward -y, the throat opens toward +x.", m_f, com=(-0.015, -0.011, 0.0), inertia=box_inertia(m_f, 0.05, 0.05, FRAME_H))
    a.articulation_root()
    root = a.root.GetPrim()
    rootpath = root.GetPath().pathString
    f_en = (0.60, 0.50)
    f_face = (0.20, 0.15)
    hz = FRAME_H / 2
    # frame (root body): the spine and the two arms
    a.box(((SPINE_X0 + SPINE_X1) / 2, (Y_SPD_ARM[0] + Y_ANV_ARM[1]) / 2, 0.0), (SPINE_X1 - SPINE_X0, Y_ANV_ARM[1] - Y_SPD_ARM[0], FRAME_H), ENAMEL, f_en, key="enamel", name="Spine")
    for (y0, y1), nm in ((Y_ANV_ARM, "AnvilArm"), (Y_SPD_ARM, "SpindleArm")):
        poly = _rounded_arm(SPINE_X0 + 0.001, ARM_X1, y0, y1)
        a.prism_z(poly, -hz, hz, ENAMEL, f_en, key="enamel", name=nm)
    a.cyl((0.0, 2.0 * MM, 0.0), 3.175 * MM, 4.0 * MM, "Y", CARBIDE, f_face, key="carbide", name="Anvil")  # y 0 .. 4 mm: the stub
    a.cyl((0.0, (SLEEVE_END + Y_SPD_ARM[0]) / 2, 0.0), SLEEVE_R, Y_SPD_ARM[0] - SLEEVE_END, "Y", CHROME, (0.5, 0.4), key="chrome", name="Sleeve")
    # graduations on the sleeve (visual): 25 mm of barrel in 0.5 mm... drawn as 1 mm marks every 5 mm along a line
    for k in range(0, 6):
        a.box((0.0, Y_SPD_ARM[0] - 3.0 * MM - k * 5.0 * MM, SLEEVE_R + 0.0002), (6.0 * MM, 0.5 * MM, 0.0005), BLACK, None, name="Grad")
    a.box((LEV_X, LEV_Y, hz + 0.001), (0.012, 0.010, 0.002), ENAMEL_M, None, name="LeverMount")  # visual: the pivot block
    # heat-insulating plates (visual, 0.4 mm proud) on both flat faces of the spine, and a legend plate on the spindle arm
    for sz in (1.0, -1.0):
        a.box(((SPINE_X0 + SPINE_X1) / 2, -0.011, sz * (hz + 0.0002)), (0.0085, 0.036, 0.0004), HEAT, None, name="HeatPlate")
        a.box((-0.008, -0.0305, sz * (hz + 0.0002)), (0.012, 0.0045, 0.0004), (0.92, 0.92, 0.94), None, name="Legend")
    pts, tris = disc(4.0 * MM, 0.0, 0.0012, 96, "Y")  # a steel collar where the sleeve meets the frame
    a.vmesh("Collar", pts, tris, ENAMEL_M, center=(0.0, Y_SPD_ARM[0] - 0.0012, 0.0))
    pts, tris = disc(4.2 * MM, 0.0, 0.0010, 96, "Y")
    a.vmesh("AnvilCollar", pts, tris, ENAMEL_M, center=(0.0, Y_ANV_ARM[0], 0.0))
    # spindle (prismatic along y)
    s = a.link("Spindle", m_s, com=(0, -0.022, 0), inertia=(1e-7, 1e-8, 1e-7), pos=(0.0, -G_OPEN, 0.0))
    a.cyl((0.0, -0.0225, 0.0), 3.175 * MM, 0.045, "Y", CARBIDE, f_face, key="carbide", name="Rod")  # face at local y 0, runs inside the barrel
    a.prismatic("Spindle", root, s, axis="Y", lo=0.0, hi=G_OPEN - 0.1 * MM, stiffness=K_SPINDLE, damping=20.0, target=0.0, p0=(0.0, -G_OPEN, 0.0), p1=(0.0, 0.0, 0.0))
    d = UsdPhysics.DriveAPI.Get(a.stage.GetPrimAtPath(f"/{a.name}/Joints/Spindle"), "linear")
    d.CreateMaxForceAttr(MEAS_FORCE)
    # thimble (turns about y; the spindle carries it)
    t = a.link("Thimble", m_t, com=(0, -0.014, 0), inertia=(1e-6, 1e-6, 1e-6), pos=(0.0, -0.060, 0.0))
    a.cyl((0.0, -0.014, 0.0), 9.0 * MM, 0.028, "Y", CHROME, (0.6, 0.5), key="chrome", name="Thimble")
    for k in range(20):  # the thimble's graduations: dark marks round its lower rim (visual) show it turning
        ph = 360.0 * k / 20
        c = (9.0 * MM + 0.0001) * math.sin(math.radians(ph)), -0.0035, (9.0 * MM + 0.0001) * math.cos(math.radians(ph))
        a.box(c, (0.5 * MM, 5.0 * MM, 0.0004), BLACK, None, name="Tick", rot=(0, ph, 0))
    a.prismatic  # (documentation anchor)
    a.revolute("Thimble", s, t, "Y", lo=-1.0e6, hi=1.0e6, stiffness=1.0 * math.pi / 180.0, damping=0.01 * math.pi / 180.0, target=0.0,
               p0=(0.0, -0.035, 0.0), p1=(0.0, 0.0, 0.0), armature=1e-6)
    # ratchet knob (turns about y; slips at RATCHET_SLIP)
    r = a.link("Ratchet", m_r, com=(0, -0.006, 0), inertia=(3e-7, 3e-7, 3e-7), pos=(0.0, -0.088, 0.0))
    a.cyl((0.0, -0.006, 0.0), 7.5 * MM, 0.012, "Y", BLACK, (0.8, 0.7), key="knurl", name="Knob")
    for k in range(24):  # knurl ribs (visual)
        ph = 360.0 * k / 24
        c = (7.5 * MM + 0.0001) * math.sin(math.radians(ph)), -0.006, (7.5 * MM + 0.0001) * math.cos(math.radians(ph))
        a.box(c, (0.8 * MM, 10.0 * MM, 0.0005), (0.28, 0.28, 0.30), None, name="Rib", rot=(0, ph, 0))
    a.revolute("Ratchet", s, r, "Y", lo=-1.0e6, hi=1.0e6, stiffness=0.1 * math.pi / 180.0, damping=0.0003 * math.pi / 180.0, target=0.0,
               p0=(0.0, -0.063, 0.0), p1=(0.0, 0.0, 0.0), armature=1e-6)
    d = UsdPhysics.DriveAPI.Get(a.stage.GetPrimAtPath(f"/{a.name}/Joints/Ratchet"), "angular")
    d.CreateMaxForceAttr(RATCHET_SLIP)
    # lock lever (turns about x on the spindle arm's top face)
    lv = a.link("Lever", m_l, com=(0, 0, 0.007), inertia=(1e-7, 1e-7, 1e-7), pos=(LEV_X, LEV_Y, hz))
    a.box((0.0, 0.0, LEV_H / 2), (0.009, 0.005, LEV_H), LEVER_RED, (0.5, 0.4), key="lever", name="Lever")
    a.cyl((0.0, 0.0, 0.0), 2.5 * MM, 0.011, "X", (0.55, 0.55, 0.58), None, key="pin", name="Pin")
    a.revolute("Lever", root, lv, "X", lo=-40.0, hi=40.0, stiffness=LEV_PEAK / LEV_UNLOCK, damping=0.003 * math.pi / 180.0, target=LEV_UNLOCK,
               p0=(LEV_X, LEV_Y, hz), p1=(0.0, 0.0, 0.0), armature=1e-7)
    a.save()


def rotor(fname):
    """A worn in-service Brembo 09.7012.11: 280 mm dia, 21.50 mm at the inspection band in the simulator (22.0 mm new,
    19 mm discard), 65 mm centre bore, 5 x 100 mm stud holes. Installed on the hub in hub_knuckle.usda. The hat, the
    36 visual vanes and the worn mass (5.74 kg, estimated from 0.25 mm of wear per face) are approximations; the 280 mm
    diameter, the 22.0 mm new thickness and the 19 mm discard limit are the real part's. Fixed, axis along y."""
    a = Asset(fname, "Rotor", "A worn in-service Brembo 09.7012.11 280 mm dia internally vented front brake rotor (22.0 mm new, "
              "21.50 mm at the eight inspection points in the sim, 19 mm discard, 65 mm centre bore, 5 x 100 mm stud holes, "
              "5.74 kg estimated grey cast iron), installed on a wheel hub and steering knuckle. Axis along y (the hat toward -y); "
              "fixed. Origin: the middle of the rotor's thickness, on its axis. Eight 2 mm scribe marks and one 6 mm paint dot at "
              "radius 135 mm show the inspection points; the dot marks the thinnest one. The hat outer diameter and 36 vanes are "
              "visual approximations.", 5.74, com=(0, 0, 0), inertia=(0.028, 0.028, 0.055))
    f = (0.40, 0.30)
    h = THICK / 2
    # the outboard plate (+y): an annulus 95 .. 140 mm with 1 mm chamfers, flat friction face at y = +h
    def plate(y0, y1, r0, r1):
        return [(r0, y0), (r1 - CHAM, y0), (r1, y0 + CHAM), (r1, y1 - CHAM), (r1 - CHAM, y1), (r0, y1)]
    pts, tris = lathe_y(plate(h - PLATE_T, h, 0.095, ROT_R))
    a.vmesh("PlateOut", pts, tris, IRON_FACE)
    prof = [(HUB_R, -h - 0.002), (0.095 - 0.004, -h - 0.002), (0.095, -h), (ROT_R - CHAM, -h), (ROT_R, -h + CHAM),
            (ROT_R, -h + PLATE_T - CHAM), (ROT_R - CHAM, -h + PLATE_T), (HUB_R, -h + PLATE_T)]
    pts, tris = lathe_y(prof)
    a.vmesh("PlateIn", pts, tris, IRON_FACE)
    # the worn friction tracks (visual): a duller annulus 100 .. 140 mm on both faces, 0.05 mm proud
    for sgn in (1.0, -1.0):
        yf = sgn * (h + 0.000025)
        pts, tris = lathe_y([(0.100, yf - 0.000025), (0.140, yf - 0.000025), (0.140, yf + 0.000025), (0.100, yf + 0.000025)])
        a.vmesh("WearTrack", pts, tris, IRON_WEAR)
    # eight circumferential inspection points at radius 135 mm: the thinnest (10 deg above the 9 o'clock line) is the 6 mm paint dot
    for k in range(8):
        ph = math.radians(170.0 - 45.0 * k)  # 170 deg = 10 deg above 9 o'clock
        c = (INSPECTION_R * math.cos(ph), h + 0.00006, INSPECTION_R * math.sin(ph))
        rr = 3.0 * MM if k == 0 else 1.0 * MM
        pts, tris = disc(rr, 0.0, 0.00005, 96, "Y")
        a.vmesh("RefDot" if k == 0 else "InspectMark", pts, tris, (0.05, 0.05, 0.05) if k == 0 else (0.18, 0.18, 0.18), center=c)
    hat = [(BORE_R, -h - HAT_D), (HUB_R, -h - HAT_D), (HUB_R, -h + PLATE_T), (BORE_R, -h + PLATE_T)]
    pts, tris = lathe_y(hat)
    a.vmesh("Hat", pts, tris, IRON_HAT)
    yf = -h - HAT_D  # the hat's mounting face: 5 stud holes on a 100 mm circle and the centre bore
    for k in range(5):
        ph = 2 * math.pi * k / 5
        pts, tris = disc(7.0 * MM, 0.0, 0.0006, 96, "Y")
        a.vmesh("StudHole", pts, tris, (0.02, 0.02, 0.02), center=(0.050 * math.cos(ph), yf - 0.0006, 0.050 * math.sin(ph)))
    pts, tris = disc(BORE_R - 0.002, 0.0, 0.0006, 96, "Y")
    a.vmesh("Bore", pts, tris, (0.03, 0.03, 0.03), center=(0.0, yf - 0.0006, 0.0))
    # 36 vanes between the plates (visual), each a box 4 mm thick across the 95 .. 140 mm band
    for k in range(36):
        ph = 360.0 * k / 36
        rc = 0.5 * (0.097 + 0.1385)
        c = (rc * math.cos(math.radians(ph)), 0.0, rc * math.sin(math.radians(ph)))
        a.box(c, (0.0415, THICK - 2 * PLATE_T + 0.0004, 0.004), IRON, None, name="Vane", rot=(0, -ph, 0))
    # colliders: one solid cylinder 280 x 21.50 mm and the hat (the vented core is not modelled: the micrometer only touches the faces)
    a.cyl((0, 0, 0), ROT_R, THICK, "Y", None, f, key="iron", name="RotorCol")
    a.cyl((0, -h - HAT_D / 2, 0), HUB_R, HAT_D, "Y", None, f, key="iron", name="HatCol")
    a.save()


def hub_knuckle(fname):
    """The fixed wheel hub and steering knuckle the worn rotor is bolted to, as seen in an installed wheel well. Origin:
    the rotor's hub centre, same as rotor.usda. The hub flange is inboard (-y) of the rotor's hat; five wheel studs pass
    through the hat holes. The bearing boss, upright, lower ball-joint boss, strut clamp and steering arm give the real
    installed context; the caliper is unbolted and its bracket (caliper_bracket.usda) stays on the knuckle."""
    a = Asset(fname, "HubKnuckle", "Fixed front wheel hub, bearing housing and cast-iron steering knuckle in an installed wheel "
              "well. Origin: the rotor's hub centre (same as rotor.usda). The hub flange and five M12 wheel studs hold the "
              "rotor's hat; the upright carries a lower ball-joint boss, an upper strut clamp and a steering arm. The "
              "caliper bracket mounts to the upright. Sizes are representative of a 2001-05 VW Golf IV front knuckle "
              "(estimates, not catalogue dimensions); visual and collider shapes are fixed.", 12.0,
              com=(0.0, -0.05, -0.04), inertia=(0.4, 0.2, 0.5))
    f = (0.50, 0.55)
    h = THICK / 2
    yhat = -h - HAT_D  # the hat's mounting face: the hub flange sits just inboard of it
    # hub flange: 110 mm dia x 12 mm, face at yhat, behind the rotor's hat
    y0, y1 = yhat - 0.012, yhat
    a.cyl((0, (y0 + y1) / 2, 0), 0.055, y1 - y0, "Y", KNUCKLE, f, key="iron", name="HubFlange")
    # five M12 wheel studs (radius 6 mm, 48 mm long), on the 100 mm bolt circle, through the hat toward the outboard side
    for k in range(5):
        ph = 2 * math.pi * k / 5
        c = (0.050 * math.cos(ph), yhat + 0.024, 0.050 * math.sin(ph))
        a.cyl(c, 0.006, 0.048, "Y", STEEL, None, key="iron", name="WheelStud")
        a.cyl((c[0], yhat + 0.047, c[2]), 0.007, 0.004, "Y", STEEL, None, key="iron", name="StudNut")
    # wheel-bearing housing: 90 mm dia boss, 40 mm deep, behind the flange (inboard, -y)
    a.cyl((0, y0 - 0.020, 0), 0.045, 0.040, "Y", KNUCKLE, f, key="iron", name="BearingBoss")
    # the upright: a cast web from the hub down to the lower ball joint and up to the strut clamp
    a.box((0, y0 - 0.020, -0.070), (0.052, 0.040, 0.180), KNUCKLE, f, key="iron", name="Upright")
    a.cyl((0, y0 - 0.020, -0.170), 0.020, 0.040, "Y", KNUCKLE, f, key="iron", name="BallJointBoss")
    a.box((0, y0 - 0.020, 0.085), (0.060, 0.042, 0.050), KNUCKLE, f, key="iron", name="StrutClamp")
    # the steering arm: a web to +x with a tie-rod-end boss
    a.box((0.070, y0 - 0.020, -0.055), (0.120, 0.030, 0.028), KNUCKLE, f, key="iron", name="SteeringArm")
    a.cyl((0.125, y0 - 0.020, -0.055), 0.014, 0.030, "Y", KNUCKLE, f, key="iron", name="TieRodBoss")
    a.save()


def caliper_bracket(fname):
    """The caliper carrier left bolted to the knuckle at the bottom of the installed rotor (origin: rotor hub centre).
    The caliper body and pads are removed for the thickness check, so the friction band is open; the carrier's bridge
    and abutment rails sit at the bottom of the disc, clear of the 9 o'clock inspection point. Representative sizes."""
    a = Asset(fname, "CaliperBracket", "Fixed painted-steel caliper carrier (bracket) still bolted to the steering knuckle at "
              "the bottom of the installed rotor; the caliper and pads are removed for the inspection so both friction faces "
              "are open. Origin: the rotor's hub centre (same as rotor.usda). The bridge straddles the disc rim at 6 o'clock; "
              "the two abutment rails and mounting ears are representative of a 2001-05 VW Golf IV front caliper (estimates).",
              2.4, com=(0.0, -0.01, -0.14), inertia=(0.02, 0.02, 0.01))
    f = (0.45, 0.50)
    yhat = -THICK / 2 - HAT_D
    y_ear = yhat - 0.020  # the ears bolt to the upright, inboard of the hat
    # a bridge over the rim at 6 o'clock: spans y across the disc, sits 4 mm clear of the 140 mm outer radius
    a.box((0, 0, -0.152), (0.110, THICK + 0.030, 0.016), BRACKET, f, key="iron", name="Bridge")
    # two mounting ears reaching inboard to the knuckle, with 12 mm bolt holes
    for x in (-0.050, 0.050):
        a.box((x, (y_ear + 0.016) / 2, -0.150), (0.024, 0.016 - y_ear, 0.030), BRACKET, f, key="iron", name="MountEar")
        a.cyl((x, y_ear - 0.001, -0.150), 0.006, 0.024, "Y", None, f, key="iron", name="BracketBoltHole")
    # pad abutment rails on both faces, just above the bridge, open toward the rim
    for sgn in (-1, 1):
        y = sgn * (THICK / 2 + 0.005)
        a.box((0, y, -0.131), (0.090, 0.008, 0.022), BRACKET, f, key="iron", name="AbutmentRail")
    a.save()


def rest(fname):
    """The micrometer's bench rest. Frame axis at Z_AX, so the frame's underside is at Z_AX - 6 mm."""
    a = Asset(fname, "Rest", "The micrometer's bench rest (fixed), 6061-T6 aluminium, blue anodised: an 80 x 80 x 30 mm base, two 46 x 6 x 146 mm uprights "
              "the frame's arms lie across, and a 36 mm clear gap between them, open toward -x, for the lower finger pad; 0.8 mm EPDM caps on top. "
              "Origin: on the table under the anvil face's centre.", 0.75, com=(-0.015, -0.011, 0.07), inertia=(1e-2, 1e-2, 1e-2))
    f = (0.45, 0.40)
    top = Z_AX - FRAME_H / 2
    for (y0, y1), nm in (((7.0 * MM, 13.0 * MM), "RibAnvil"), ((-35.0 * MM, -29.0 * MM), "RibSpindle")):
        a.box((-0.015, (y0 + y1) / 2, top / 2), (0.046, y1 - y0, top), STAND, f, key="plastic", name=nm)
    a.box((-0.015, -0.011, 0.015), (0.080, 0.080, 0.030), STAND, f, key="plastic", name="Base")
    for (y0, y1) in ((7.0 * MM, 13.0 * MM), (-35.0 * MM, -29.0 * MM)):
        a.box((-0.015, (y0 + y1) / 2, top + 0.0004), (0.046, y1 - y0, 0.0008), (0.04, 0.04, 0.04), None, name="RubberCap")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    micrometer("micrometer.usda")
    rotor("rotor.usda")
    hub_knuckle("hub_knuckle.usda")
    caliper_bracket("caliper_bracket.usda")
    rest("rest.usda")
