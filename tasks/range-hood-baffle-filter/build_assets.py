"""Writes this task's USD assets with pxr (boxes, cylinders, capsules; nothing from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up."""
import math
import numpy as np
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001
METAL = {}  # rgb -> (metallic, roughness), filled below with the stainless / brass finishes


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
        met, rgh = METAL.get(tuple(rgb), (0.0, rough))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(met)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rgh)
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

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"{self.cur}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

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
        """The fixed articulation root is a rigid link "Base" (not the root Xform): the root's body and mass move onto it, later shapes go under it."""
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

    def mesh(self, pts, tris, rgb=None, collide=None, key="c", name="Mesh"):
        """A triangle mesh from points and (a, b, c) index triples (closed, outward-wound for the round parts)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in p]) for p in pts]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for t in tris for i in t])
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


# ---- meshkit.py (inlined): closed, outward-wound meshes for round parts ----

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


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))
# ======================================================================================================================
# T8 tube / rotor lampholder task assets. Metres, kilograms, Z up.
#   batten.usda  : a fixed 600 mm T8 batten on a back board: two G13 rotor lampholders (a housing with a round pocket, a
#                  front entry notch with a lead-in, and a rotor with a diametral slot on a revolute joint about y). An
#                  articulation with a fixed root. Origin: table level, on the tube's axis (x), y 0 midway between the holders.
#   t8_tube.usda : a T8 fluorescent tube, 26 mm x 590 mm cap to cap, G13 bi-pins (2.47 mm, 12.7 mm apart, 7.5 mm out of each
#                  cap), 0.13 kg: an articulation (free root) with four visual strips that show when the tube is lit.

# ======================================================================================================================
# Range-hood baffle-filter task assets. Metres, kilograms, Z up. Origin of every asset: table level, x forward from the arms' midpoint.
#   hood.usda   : a fixed hood filter panel leaning back PHI degrees from vertical: back plate, a bottom trough (back wall = the plate,
#                 front lip with a lead-in) and a top spring latch (a plate that slides along the panel on a prismatic joint with a
#                 spring drive, a pull handle and a visible coil). An articulation with a fixed root.
#   filter.usda : a stainless baffle filter 406.4 x 254 x 44.45 mm (16 x 10 x 1.75 in), 1.18 kg (2.6 lb), with two drop handles on its
#                 front face; a free rigid body whose origin is its centre, local x = up the panel, y = along the filter, z = the front face.
# ======================================================================================================================
FW, FH, FT = 0.4064, 0.254, 0.04445  # filter: along y, up the panel, thickness
F_MASS = 1.18  # Component Hardware FR51-1016: 10 x 16 x 1.75 in stainless baffle filter, 2.6 lb
PHI = 60.0  # panel lean from vertical, degrees (30 degrees above horizontal)
XB, ZB = 0.27, 0.02  # the panel's trough corner (a = 0, b = 0): x forward, z above the table
SPH, CPH = math.sin(math.radians(PHI)), math.cos(math.radians(PHI))
STEEL, STEEL2, PAINT, BRASS, DARKG = (0.72, 0.74, 0.76), (0.60, 0.62, 0.65), (0.82, 0.83, 0.85), (0.80, 0.62, 0.25), (0.20, 0.21, 0.23)
SS = (0.30, 0.25)  # steel on steel friction (static, dynamic)
SLOT_B = 0.0495  # the trough's inner width (filter 44.45: 5 mm of play)
LIP_A = 0.018  # trough lip height
PLATE_B0, PLATE_B1 = 0.047, 0.050  # latch plate, in front of the filter face (b 44.45): 2.5 mm clear
PLATE_A0, PLATE_A1 = 0.246, 0.296  # plate rest span along the panel; its hook covers the filter's top 14 mm
PLATE_Y = -0.15  # latch plate centre (y)
LATCH_TRAVEL = 0.0165
TAB_Y = -0.235  # the T handle's y: beyond the filter's end (-0.2032)
SPRING_K = 150.0  # N/m; drive target -0.020 gives a 3 N preload (rest against its stop) and 5.5 N at full travel
SPRING_PRELOAD = 0.020
LATCH_MASS = 0.06


def H(a, b, y=0.0):
    """Hood coordinates (a up the panel, b out from its face) to world (x, y, z above the table)."""
    return (XB + a * SPH - b * CPH, y, ZB + a * CPH + b * SPH)


def hbox(asset, a0, a1, b0, b1, y0, y1, rgb, collide, key="steel", name="Box"):
    """A box with faces along the panel's axes: a from a0 to a1, b from b0 to b1, y from y0 to y1."""
    c = H((a0 + a1) / 2, (b0 + b1) / 2, (y0 + y1) / 2)
    asset.box(c, (a1 - a0, y1 - y0, b1 - b0), rgb, collide, key=key, name=name, rot=(0, -PHI + 90.0 - 90.0, 0))


def _ccw(poly):
    s = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))
    return poly if s > 0 else poly[::-1]


def helix(r, wire, turns, length, n_seg=24, per_turn=48):
    """A closed coil-spring mesh along local +z from 0 to ``length``: a round wire (n_seg sides) wound ``turns`` times about the axis."""
    steps = int(turns * per_turn)
    pts, tris = [], []
    for i in range(steps + 1):
        t = i / steps
        ang = 2 * math.pi * turns * t
        c = np.array([r * math.cos(ang), r * math.sin(ang), length * t])
        tan = np.array([-r * math.sin(ang) * 2 * math.pi * turns, r * math.cos(ang) * 2 * math.pi * turns, length])
        tan /= np.linalg.norm(tan)
        n1 = np.array([math.cos(ang), math.sin(ang), 0.0])
        n1 = n1 - tan * np.dot(n1, tan)
        n1 /= np.linalg.norm(n1)
        n2 = np.cross(tan, n1)
        for j in range(n_seg):
            w = 2 * math.pi * j / n_seg
            pts.append(tuple(c + wire * (math.cos(w) * n1 + math.sin(w) * n2)))
    for i in range(steps):
        for j in range(n_seg):
            j2 = (j + 1) % n_seg
            a, b, c2, d = i * n_seg + j, i * n_seg + j2, (i + 1) * n_seg + j2, (i + 1) * n_seg + j
            tris += [(a, b, c2), (a, c2, d)]
    k0 = len(pts)
    pts.append(tuple(np.array(pts[0:n_seg]).mean(axis=0)))
    pts.append(tuple(np.array(pts[steps * n_seg:(steps + 1) * n_seg]).mean(axis=0)))
    for j in range(n_seg):
        j2 = (j + 1) % n_seg
        tris.append((k0, j2, j))
        tris.append((k0 + 1, steps * n_seg + j, steps * n_seg + j2))
    return pts, _orient(pts, tris)


def to_hood(pts, a0, b0, y0, flip=False):
    """Local mesh points (axis z = up the panel) to world points: local z -> u, local x -> m (out of the face), local y -> y."""
    out = []
    for (x, y, z) in pts:
        out.append(H(a0 + z, b0 + x, y0 + y))
    return out


# ---- visual detailing helpers (visual meshes only: every collider keeps its box, now an invisible guide) ----
BRUSH = (0.78, 0.79, 0.81)   # brushed stainless filter frame and baffles
HOODSS = (0.58, 0.60, 0.63)  # the hood's darker stainless panel
RIVET = (0.88, 0.89, 0.90)
METAL.update({STEEL: (0.9, 0.34), STEEL2: (0.85, 0.42), PAINT: (0.75, 0.45), BRASS: (0.9, 0.28), BRUSH: (0.92, 0.30),
              HOODSS: (0.85, 0.40), RIVET: (0.95, 0.2), DARKG: (0.2, 0.6)})


def rrect(u0, u1, v0, v1, r, n=6):
    """A rounded rectangle (counter-clockwise polygon, convex), corner radius r."""
    out = []
    for cu, cv, a0 in ((u1 - r, v1 - r, 0.0), (u0 + r, v1 - r, 90.0), (u0 + r, v0 + r, 180.0), (u1 - r, v0 + r, 270.0)):
        for k in range(n + 1):
            t = math.radians(a0 + 90.0 * k / n)
            out.append((cu + r * math.cos(t), cv + r * math.sin(t)))
    return out


def extrude(poly, mapf, w0, w1):
    """A convex prism: ``poly`` (u, v) swept from w0 to w1; ``mapf(u, v, w)`` gives the point. Closed, outward-wound."""
    k = len(poly)
    pts = [mapf(u, v, w0) for (u, v) in poly] + [mapf(u, v, w1) for (u, v) in poly]
    tris = []
    for i in range(1, k - 1):
        tris += [(0, i + 1, i), (k, k + i, k + i + 1)]
    for i in range(k):
        j = (i + 1) % k
        tris += [(i, j, k + j), (i, k + j, k + i)]
    return pts, _orient(pts, tris)


def dome(r, hgt, n=48, m=5):
    """A rivet head: a flat-bottomed dome (axis +z), closed and welded at the axis."""
    prof = [(0.0, 0.0), (r, 0.0)] + [(r * math.cos(math.pi / 2 * k / m), hgt * math.sin(math.pi / 2 * k / m)) for k in range(1, m)] + [(0.0, hgt)]
    pts, tris = lathe(prof, n)
    return _weld(pts, tris)


def place(pts, origin, R):
    R = np.asarray(R, float)
    return [tuple(np.asarray(origin, float) + R @ np.asarray(p, float)) for p in pts]


def rod(r, p0, p1, n=48):
    """A closed round rod (cylinder mesh) from point p0 to p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    L = float(np.linalg.norm(ax))
    z = ax / L
    x = np.cross(z, [0.0, 0.0, 1.0] if abs(z[2]) < 0.9 else [1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    pts, tris = disc(r, 0.0, L, n)
    return place(pts, p0, np.column_stack([x, y, z])), tris


def hood(fname):
    a = Asset(fname, "Hood", "A fixed range-hood filter panel leaning back 60 degrees from vertical: back plate, a bottom trough with a front lip and lead-in, "
              "and a top spring latch (a plate on a prismatic joint along the panel, spring 150 N/m with a 3 N preload, pull handle, visible coil). "
              "Origin: table level; x forward from the arms' midpoint.", 3.0, com=(XB + 0.1, 0, 0.1), inertia=(1e-1, 1e-1, 1e-1))
    a.articulation_root()
    root = a.base_prim
    th = -(90.0 - PHI)
    U = np.array([SPH, 0.0, CPH]); M = np.array([-CPH, 0.0, SPH]); Y = np.array([0.0, 1.0, 0.0])
    RH = np.column_stack([U, Y, M])  # local x -> up the panel, y -> y, z -> out of the face

    def box(a0, a1, b0, b1, y0, y1, rgb, collide, key="steel", name="Box"):
        c = H((a0 + a1) / 2, (b0 + b1) / 2, (y0 + y1) / 2)
        a.box(c, (a1 - a0, y1 - y0, b1 - b0), rgb, collide, key=key, name=name, rot=(0, th, 0))

    def vis_ext(poly_ab, y0, y1, rgb, name):  # visual prism: polygon in (a, b), swept along y
        pts, tris = extrude(poly_ab, lambda u, v, w: H(u, v, w), y0, y1)
        a.mesh(pts, tris, rgb, None, name=name)

    def vis_ext_ya(poly_ya, b0, b1, rgb, name):  # visual prism: polygon in (y, a), swept along b
        pts, tris = extrude(poly_ya, lambda u, v, w: H(v, w, u), b0, b1)
        a.mesh(pts, tris, rgb, None, name=name)

    def rivets(spots, b, rgb=RIVET, r=0.0027, hgt=0.0022):
        dp, dt = dome(r, hgt)
        for (aa, yy) in spots:
            a.mesh(place(dp, H(aa, b, yy), RH), dt, rgb, None, name="Rivet")

    YW = 0.290
    # --- colliders (unchanged sizes; invisible guides) ---
    box(-0.014, 0.300, -0.006, 0.0, -YW, YW, None, SS, name="Plate")
    box(-0.014, 0.0, -0.006, SLOT_B + 0.005, -YW, YW, None, SS, name="Floor")
    box(-0.014, LIP_A, SLOT_B, SLOT_B + 0.005, -YW, YW, None, SS, name="Lip")
    L = 0.010 * math.sqrt(2.0)
    c = H(LIP_A + 0.005, SLOT_B + 0.005)
    a.box(c, (L, 2 * YW, 0.004), None, SS, key="steel", name="LeadIn", rot=(0, th - 45.0, 0))
    # --- visual: the filter panel, a brushed stainless sheet with a folded border and a riveted trim frame ---
    vis_ext(rrect(-0.014, 0.300, -0.006, 0.0, 0.0008), -YW, YW, HOODSS, "Panel")
    for s_ in (1.0, -1.0):  # folded side hems and the filter's side guide channels
        y0, y1 = (s_ * (YW - 0.012), s_ * YW)
        vis_ext(rrect(-0.014, 0.300, -0.006, 0.014, 0.003), min(y0, y1), max(y0, y1), HOODSS, "Hem")
        g0, g1 = (s_ * 0.2078, s_ * 0.2205)
        vis_ext(rrect(0.0, 0.296, 0.0, 0.030, 0.0025), min(g0, g1), max(g0, g1), STEEL2, "SideGuide")
    # --- visual: the trough, a formed gutter with chamfered lip and lead-in, end caps and a rolled front edge ---
    vis_ext(rrect(-0.014, 0.0, -0.006, SLOT_B + 0.005, 0.002), -YW, YW, STEEL2, "Floor")
    vis_ext(rrect(-0.014, LIP_A, SLOT_B, SLOT_B + 0.005, 0.0022), -YW, YW, STEEL2, "Lip")
    ang = math.radians(th - 45.0)
    ca, sa = math.cos(ang), math.sin(ang)
    # lead-in plate (a flat strip at 45 degrees)
    cl = (LIP_A + 0.005, SLOT_B + 0.005)
    ua, ub = 0.5 * L, 0.002
    d0 = np.array([math.cos(math.radians(45.0)), math.sin(math.radians(45.0))])
    n0 = np.array([-d0[1], d0[0]])
    quad = [tuple(np.array(cl) + sx * 0.5 * L * d0 + sy * 0.002 * n0) for (sx, sy) in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    vis_ext(quad, -YW, YW, STEEL2, "LeadIn")
    for s_ in (1.0, -1.0):  # end caps of the gutter
        e0, e1 = (s_ * (YW - 0.0015), s_ * YW)
        vis_ext(rrect(-0.014, LIP_A + 0.006, -0.006, SLOT_B + 0.005, 0.003), min(e0, e1), max(e0, e1), STEEL2, "EndCap")
    rivets([(-0.007, yy) for yy in np.arange(-0.27, 0.271, 0.045)], SLOT_B + 0.0072 - 0.0022 + 0.0022 - 0.0022, r=0.0022, hgt=0.0018) if False else None
    # rivets along the gutter floor's front strip (the strip is horizontal: rivets stand up out of it)
    for yy in np.arange(-0.27, 0.271, 0.045):
        dp, dt = dome(0.0022, 0.0018)
        a.mesh(place(dp, H(-0.0, 0.022, yy), np.column_stack([-M, Y, U])), dt, RIVET, None, name="Rivet")
    # rolled front edge of the lip (a round bead along y at the lip's outer top corner)
    bp, bt = rod(0.0030, H(LIP_A + 0.0005, SLOT_B + 0.0072, -YW), H(LIP_A + 0.0005, SLOT_B + 0.0072, YW))
    a.mesh(bp, bt, STEEL2, None, name="LipBead")
    # --- visual: the hood body above the filter: upper panel, folded side walls, a skirt behind and a top canopy ---
    AUP = 0.62
    vis_ext(rrect(0.300, AUP, -0.006, 0.0, 0.0008), -YW, YW, HOODSS, "UpperPanel")
    for s_ in (1.0, -1.0):
        wp = [H(-0.014, -0.006)[::2], H(AUP, -0.006)[::2], (H(AUP, -0.006)[0], 0.0), (H(-0.014, -0.006)[0], 0.0)]
        y0, y1 = (s_ * YW, s_ * (YW + 0.006)) if s_ > 0 else (s_ * (YW + 0.006), s_ * YW)
        a.prism_y(_ccw(wp), y0, y1, HOODSS, None, name="Wedge")
    box(0.300, AUP, -0.140, -0.006, -YW, YW, HOODSS, None, name="Skirt")
    vis_ext(rrect(AUP - 0.012, AUP + 0.030, -0.140, 0.085, 0.006), -YW - 0.006, YW + 0.006, HOODSS, "Canopy")  # the top canopy lip
    vis_ext(rrect(AUP + 0.018, AUP + 0.036, -0.140, 0.085, 0.004), -YW - 0.006, YW + 0.006, STEEL, "CanopyRim")
    rivets([(aa, yy) for aa in (0.318, 0.600) for yy in np.arange(-0.27, 0.271, 0.09)], 0.0)
    # --- latch guides, spring bracket, coil and plunger (visual; static) ---
    ya, yb = PLATE_Y - 0.020, PLATE_Y + 0.020
    for yy in (ya - 0.008, yb + 0.002):
        vis_ext(rrect(PLATE_A0 - 0.020, 0.346, PLATE_B0 - 0.002, PLATE_B1 + 0.006, 0.0012), yy, yy + 0.006, STEEL2, "Guide")
    vis_ext(rrect(0.332, 0.340, PLATE_B0 - 0.002, PLATE_B1 + 0.012, 0.0012), ya - 0.008, yb + 0.008, STEEL2, "Bracket")
    rivets([(0.336, ya - 0.005), (0.336, yb + 0.005)], PLATE_B1 + 0.012, r=0.0022, hgt=0.0016)
    cp, ct = helix(0.0058, 0.0008, 8, 0.034)
    a.mesh(to_hood(cp, PLATE_A1, 0.0485, PLATE_Y), ct, STEEL, None, name="Coil")
    rp, rt = disc(0.0022, 0.0, 0.034, n=96)
    a.mesh(to_hood(rp, PLATE_A1, 0.0485, PLATE_Y), rt, BRASS, None, name="Plunger")
    # --- the latch plate: its own link on a prismatic joint (colliders unchanged, invisible), spring drive ---
    lk = a.link("Latch", LATCH_MASS, com=H((PLATE_A0 + PLATE_A1) / 2, 0.052, PLATE_Y), inertia=(2e-5, 2e-5, 2e-5), pos=(0.0, 0.0, 0.0))

    def lbox(a0, a1, b0, b1, y0, y1, rgb, collide, key="steel", name="Box"):
        c = H((a0 + a1) / 2, (b0 + b1) / 2, (y0 + y1) / 2)
        a.box(c, (a1 - a0, y1 - y0, b1 - b0), rgb, collide, key=key, name=name, rot=(0, th, 0))
    LF = (0.18, 0.14)
    lbox(PLATE_A0, PLATE_A1, PLATE_B0, PLATE_B1, ya, yb, None, LF, key="latch", name="Hook")
    lbox(PLATE_A0, PLATE_A0 + 0.020, PLATE_B0, PLATE_B1, -0.255, ya, None, LF, key="latch", name="Ear")
    lbox(0.248, 0.263, PLATE_B1, 0.074, TAB_Y - 0.006, TAB_Y + 0.006, None, LF, key="latch", name="Stem")
    lbox(0.248, 0.263, 0.074, 0.084, TAB_Y - 0.020, TAB_Y + 0.020, None, LF, key="latch", name="Head")
    # visuals of the latch: a brass plate with rounded corners (hook end and the ear in one outline), a rounded stem and a rolled T handle
    plate = rrect(ya, yb, PLATE_A0, PLATE_A1, 0.004)
    pts, tris = extrude(plate, lambda u, v, w: H(v, w, u), PLATE_B0 + 0.0003, PLATE_B1 - 0.0003)
    a.mesh(pts, tris, BRASS, None, name="HookPlate")
    ear = rrect(-0.2548, ya + 0.004, PLATE_A0, PLATE_A0 + 0.020, 0.004)
    pts, tris = extrude(ear, lambda u, v, w: H(v, w, u), PLATE_B0 + 0.0003, PLATE_B1 - 0.0003)
    a.mesh(pts, tris, BRASS, None, name="EarPlate")
    # a bent-over hook lip on the plate's lower end (the part that catches the filter's top edge)
    lip = rrect(ya + 0.002, yb - 0.002, PLATE_A0 - 0.0035, PLATE_A0 + 0.0005, 0.0006)
    pts, tris = extrude(lip, lambda u, v, w: H(v, w, u), PLATE_B0 + 0.0003, PLATE_B1 - 0.0003)
    a.mesh(pts, tris, BRASS, None, name="HookLip")
    sp, st = extrude(rrect(0.2495, 0.2615, TAB_Y - 0.0058, TAB_Y + 0.0058, 0.0035), lambda u, v, w: H(u, w, v), PLATE_B1 - 0.0005, 0.0745)
    a.mesh(sp, st, BRASS, None, name="StemRound")
    hp, ht = extrude(rrect(0.2485, 0.2625, 0.0745, 0.0835, 0.0045), lambda u, v, w: H(u, v, w), TAB_Y - 0.0195, TAB_Y + 0.0195)
    a.mesh(hp, ht, BRASS, None, name="HeadBar")
    for (aa, yy) in ((PLATE_A0 + 0.010, ya + 0.007), (PLATE_A0 + 0.010, yb - 0.007), (PLATE_A1 - 0.008, PLATE_Y)):
        dp, dt = dome(0.0026, 0.0018)
        a.mesh(place(dp, H(aa, PLATE_B1 - 0.0003, yy), RH), dt, RIVET, None, name="Rivet")
    a.cur = a.base_prim.GetPath().pathString
    j = a.prismatic("Latch", root, lk, axis="X", lo=0.0, hi=LATCH_TRAVEL, stiffness=SPRING_K, damping=2.0, target=-SPRING_PRELOAD)
    q = Gf.Quatf(math.cos(math.radians(th) / 2), 0.0, math.sin(math.radians(th) / 2), 0.0)
    j.CreateLocalRot0Attr().Set(q)
    j.CreateLocalRot1Attr().Set(q)
    a.save()


def filt(fname):
    ia = F_MASS / 12 * (FH ** 2 + FT ** 2)
    ib = F_MASS / 12 * (FW ** 2 + FT ** 2)
    ic = F_MASS / 12 * (FH ** 2 + FW ** 2)
    a = Asset(fname, "Filter", "A stainless baffle hood filter 406.4 x 254 x 44.45 mm (16 x 10 x 1.75 in, Component Hardware FR51-1016 class), 1.18 kg, with two drop handles on its front face. "
              "Origin: its centre; local x up the panel (254 mm), y along the filter (406.4 mm), z out of the front face (44.45 mm).",
              F_MASS, inertia=(ia, ib, ic))
    a.box((0, 0, 0), (FH, FW, FT), None, SS, key="steel", name="Core")  # invisible collider: the solid envelope
    ident = lambda u, v, w: (u, v, w)
    rail, tw, bead = 0.012, 0.0015, 0.0022
    zf = FT / 2

    def ex_xy(poly, z0, z1, rgb, name):  # prism: polygon in (x, y), swept along z
        pts, tris = extrude(poly, lambda u, v, w: (u, v, w), z0, z1)
        a.mesh(pts, tris, rgb, None, name=name)

    def ex_xz(poly, y0, y1, rgb, name):  # polygon in (x, z), swept along y
        pts, tris = extrude(poly, lambda u, v, w: (u, w, v), y0, y1)
        a.mesh(pts, tris, rgb, None, name=name)

    def ex_yz(poly, x0, x1, rgb, name):  # polygon in (y, z), swept along x
        pts, tris = extrude(poly, lambda u, v, w: (w, u, v), x0, x1)
        a.mesh(pts, tris, rgb, None, name=name)

    # frame: a sheet-metal channel round all four edges (web on the edge, a flange on each face turned in 12 mm, rolled bead on each flange)
    for sx in (1, -1):  # top and bottom rails, along y
        xe = sx * FH / 2
        xi = sx * (FH / 2 - rail)
        ex_xz([(min(xe, xe - sx * tw), -zf), (max(xe, xe - sx * tw), -zf), (max(xe, xe - sx * tw), zf), (min(xe, xe - sx * tw), zf)], -FW / 2, FW / 2, BRUSH, "Web")
        for sz in (1, -1):
            zc = sz * (zf - tw / 2)
            ex_xz([(min(xe, xi), zc - tw / 2), (max(xe, xi), zc - tw / 2), (max(xe, xi), zc + tw / 2), (min(xe, xi), zc + tw / 2)], -FW / 2, FW / 2, BRUSH, "Flange")
            bp, bt = rod(bead, (xi, -FW / 2, sz * (zf - bead)), (xi, FW / 2, sz * (zf - bead)))
            a.mesh(bp, bt, BRUSH, None, name="Bead")
    for sy in (1, -1):  # the two ends, along x
        ye = sy * FW / 2
        yi = sy * (FW / 2 - rail)
        ex_yz([(min(ye, ye - sy * tw), -zf), (max(ye, ye - sy * tw), -zf), (max(ye, ye - sy * tw), zf), (min(ye, ye - sy * tw), zf)], -FH / 2, FH / 2, BRUSH, "Web")
        for sz in (1, -1):
            zc = sz * (zf - tw / 2)
            ex_yz([(min(ye, yi), zc - tw / 2), (max(ye, yi), zc - tw / 2), (max(ye, yi), zc + tw / 2), (min(ye, yi), zc + tw / 2)], -FH / 2, FH / 2, BRUSH, "Flange")
            bp, bt = rod(bead, (-FH / 2, yi, sz * (zf - bead)), (FH / 2, yi, sz * (zf - bead)))
            a.mesh(bp, bt, BRUSH, None, name="Bead")
    # two interlocked rows of chevron baffles (apex out in front, apex in at the back), thin sheet legs
    n = 12
    pitch = (FW - 2 * rail) / n
    t = 0.0013
    y_in = -FW / 2 + rail

    def chevron(yc, zlo, zhi, up):  # an inverted / upright V with its apex at yc
        za, zb = (zhi, zlo) if up else (zlo, zhi)
        legs = (((yc - pitch / 2, zb), (yc - pitch / 2 + t, zb), (yc + t, za), (yc, za)),
                ((yc, za), (yc + t, za), (yc + pitch / 2 + t, zb), (yc + pitch / 2, zb)))
        for poly in legs:
            pts, tris = extrude(list(poly), lambda u, v, w: (w, u, v), -FH / 2 + rail, FH / 2 - rail)
            a.mesh(pts, tris, BRUSH, None, name="Baffle")
    for i in range(n):
        chevron(y_in + pitch * (i + 0.5), 0.0008, zf - tw - 0.0006, True)
    for i in range(1, n):
        chevron(y_in + pitch * i, -(zf - tw - 0.0006), -0.0008, False)
    # rivets: dome heads through the front flanges, along every rail
    dp, dt = dome(0.0027, 0.0021)
    spots = [(sx * (FH / 2 - 0.0065), yy) for sx in (1, -1) for yy in np.arange(-0.18, 0.181, 0.06)]
    spots += [(xx, sy * (FW / 2 - 0.0065)) for sy in (1, -1) for xx in np.arange(-0.10, 0.101, 0.05)]
    for (xx, yy) in spots:
        a.mesh(place(dp, (xx, yy, zf), np.eye(3)), dt, RIVET, None, name="Rivet")
    # two drop handles on the front face: round-cornered posts and a flat 15 x 10 mm crossbar (collider boxes unchanged, invisible)
    for sy in (1, -1):
        yc = sy * 0.16
        for sp in (1, -1):
            a.box((0.072, yc + sp * 0.040, FT / 2 + 0.011), (0.015, 0.010, 0.022), None, SS, key="steel", name="Post")
            pts, tris = extrude(rrect(0.072 - 0.0075, 0.072 + 0.0075, yc + sp * 0.040 - 0.005, yc + sp * 0.040 + 0.005, 0.0025), ident, zf, zf + 0.0225)
            a.mesh(pts, tris, BRUSH, None, name="PostVis")
            wp, wt = disc(0.0095, zf, zf + 0.0015)  # mounting washer under the post
            a.mesh([(p[0] + 0.072, p[1] + yc + sp * 0.040, p[2]) for p in wp], wt, STEEL2, None, name="Washer")
            dp2, dt2 = dome(0.0030, 0.0022)
            a.mesh(place(dp2, (0.072 + 0.0, yc + sp * 0.040, zf + 0.0015), np.eye(3)), dt2, RIVET, None, name="Rivet") if False else None
        a.box((0.072, yc, FT / 2 + 0.027), (0.015, 0.090, 0.010), None, SS, key="steel", name="Bar")
        pts, tris = extrude(rrect(0.072 - 0.0075, 0.072 + 0.0075, zf + 0.0215, zf + 0.0325, 0.0042), lambda u, v, w: (u, w, v), yc - 0.045, yc + 0.045)
        a.mesh(pts, tris, BRUSH, None, name="BarVis")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    hood("hood.usda")
    filt("filter.usda")
