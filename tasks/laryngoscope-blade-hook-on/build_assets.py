"""Writes this task's USD assets with pxr (boxes, cylinders, convex prisms; nothing from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: see each asset's doc; metres, kg, Z up."""
import math
import numpy as np
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001


# ---- visual-mesh helpers (copy of submission_tools/meshkit.py): closed, outward-wound shapes; colliders stay as they are ----
def _orient(pts, tris):
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[a], np.cross(p[b], p[c])) for a, b, c in tris) / 6.0
    return tris if vol >= 0 else [(a, c, b) for a, b, c in tris]


def _weld(pts, tris, tol=1e-9):
    m, out, new = {}, [], []
    for p in pts:
        key = tuple(round(v / tol) for v in p)
        if key not in m:
            m[key] = len(out)
            out.append(p)
        new.append(m[key])
    t = [(new[a], new[b], new[c]) for a, b, c in tris]
    return out, [x for x in t if len(set(x)) == 3]


def lathe(profile, n=64):
    """Revolve a closed (r, z) profile about z."""
    k = len(profile)
    pts = [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z) for i in range(n) for (r, z) in profile]
    tris = []
    for i in range(n):
        i2 = (i + 1) % n
        for j in range(k):
            j2 = (j + 1) % k
            a, b, c, d = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            tris += [(a, b, c), (a, c, d)]
    pts, tris = _weld(pts, tris)
    return pts, _orient(pts, tris)


def to_axis(pts, axis):
    if axis == "Z":
        return pts
    if axis == "Y":
        return [(x, z, -y) for x, y, z in pts]
    return [(z, y, -x) for x, y, z in pts]


def disc(r, z0, z1, n=64, axis="Z"):
    pts, tris = lathe([(0.0, z0), (r, z0), (r, z1), (0.0, z1)], n)
    return to_axis(pts, axis), tris


def knurl(r, depth, y0, y1, n=72):
    """A knurled (straight-ridged) cylinder along y: 2n ridge points alternate between r and r - depth."""
    pts = []
    for y in (y0, y1):
        for i in range(2 * n):
            a = math.pi * i / n
            rr = r if i % 2 == 0 else r - depth
            pts.append((rr * math.cos(a), y, rr * math.sin(a)))
    c0, c1 = len(pts), len(pts) + 1
    pts += [(0.0, y0, 0.0), (0.0, y1, 0.0)]
    m = 2 * n
    tris = []
    for i in range(m):
        j = (i + 1) % m
        tris += [(i, j, m + j), (i, m + j, m + i), (c0, j, i), (c1, m + i, m + j)]
    return pts, _orient(pts, tris)


def tbox(x0, x1, a0, a1, b0, b1, z0, z1):
    """A box tapered along x: y range (a0, a1) at x0 and (b0, b1) at x1, z range (z0, z1). Convex."""
    pts = [(x0, a0, z0), (x0, a1, z0), (x0, a1, z1), (x0, a0, z1), (x1, b0, z0), (x1, b1, z0), (x1, b1, z1), (x1, b0, z1)]
    q = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (3, 2, 6, 7), (0, 3, 7, 4), (1, 5, 6, 2)]
    tris = []
    for a, b, c, d in q:
        tris += [(a, b, c), (a, c, d)]
    return pts, _orient(pts, tris)


def shift(m, dx=0.0, dy=0.0, dz=0.0):
    pts, tris = m
    return [(x + dx, y + dy, z + dz) for x, y, z in pts], tris


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

    def pbr(self, key, rgb, metal=0.0, rough=0.5, emit=None, opacity=None):
        k = "P_" + key
        if k in self.mats:
            return self.mats[k]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{k}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{k}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        if emit is not None:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emit))
        if opacity is not None:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[k] = mat
        return mat

    def vis(self, m, mat, name="Vis"):
        """A visual-only triangle mesh (no collision). m = (points, triangles); mat = a material from pbr()."""
        self.n += 1
        pts, tris = m
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(v) for v in p]) for p in pts]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for t in tris for i in t])
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
        return g

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
# Laryngoscope hook-on task assets. Metres, kilograms, Z up.
#   handle.usda  : a medium C-cell fibre-optic laryngoscope handle (Welch Allyn 60813 class, ISO 7376 green standard), one rigid
#                  body, 0.25 kg. Origin: the middle of the 28 mm x 150 mm grip cylinder (axis along y; +y is the base end, -y the
#                  head end); z up. The head carries a bridge (a 24 x 26 mm plate 21-27 mm above the tray) and, under its outer
#                  end, the 4.5 mm hinge bar (a vertical post): this is the hinge the blade hooks onto, lying on its side.
#   blade.usda   : a Macintosh 3 style fibre-optic blade (ISO 7376 hook-on), 0.08 kg, an articulation (free root) plus one
#                  visual-only lamp marker. Origin: the middle of the base face, half-way up; +x toward the tip (130 mm long);
#                  the hook slot is cut in the base face (5 mm wide, 6.5 mm deep, 3 mm 45 degree lead-in).
#   tray.usda    : the fixed tray, 260 x 400 x 3 mm. Origin: table level at its centre.
# ======================================================================================================================
H_R, H_L, H_M = 14.0 * MM, 150.0 * MM, 0.25  # handle: 28 mm grip, 6 in overall (reseller listing), 0.25 kg (estimate)
BAR_R, BAR_H = 2.2525 * MM, 21.0 * MM  # hinge bar 4.505 mm: ISO 7376:2009 Fig. 2 (green fibre-illuminated handle), 4.505 +-0.065 mm
BAR_LOC = (30.0 * MM, -(H_L / 2 + 12.0 * MM), -3.2 * MM)  # bar centre in the handle frame (tray 14 mm below the axis: the bar spans 0.3 to 21.3 mm above it)
B_L, B_H = 130.0 * MM, 20.0 * MM  # blade length and height (the bar's direction)
SLOT_W, SLOT_D, LEAD = 5.0 * MM, 6.5 * MM, 3.0 * MM
GREEN, STEEL, DARK, LAMP = (0.10, 0.55, 0.25), (0.70, 0.72, 0.75), (0.15, 0.16, 0.18), (1.0, 0.92, 0.35)


def handle(fname):
    m = H_M
    iyy = 0.5 * m * H_R**2
    ixx = m / 12 * (3 * H_R**2 + H_L**2)
    a = Asset(fname, "Handle", "A medium C-cell fibre-optic laryngoscope handle (Welch Allyn 60813 class): 28 mm x 150 mm grip, 0.25 kg, "
              "with the head bridge and the 4.5 mm hinge bar. Origin: middle of the grip; y along it (+y the base end); z up.",
              m, com=(0.004, 0, 0), inertia=(ixx, iyy, ixx))
    f = (0.30, 0.25)
    # colliders (invisible: purpose guide), unchanged
    a.cyl((0, 0, 0), H_R, H_L, "Y", None, f, key="handle", name="Grip")
    bx, by, bz = BAR_LOC
    a.box((bx / 2 + 2 * MM, -(H_L / 2 + 12.0 * MM), 10.0 * MM), (bx + 8 * MM, 26 * MM, 6 * MM), None, f, key="head", name="Bridge")
    a.box((0, -(H_L / 2 + 6 * MM), 0), (2 * H_R, 12 * MM, 2 * H_R), None, f, key="head", name="Head")
    a.cyl(BAR_LOC, BAR_R, BAR_H, "Z", None, f, key="head", name="Bar")
    # visible parts (no collision)
    chrome = a.pbr("chrome", (0.80, 0.82, 0.86), 0.75, 0.28)
    dark = a.pbr("dark", (0.10, 0.11, 0.13), 0.3, 0.45)
    green = a.pbr("green", (0.08, 0.50, 0.22), 0.0, 0.5)
    smoke = a.pbr("smoke", (0.45, 0.52, 0.60), 0.2, 0.2, opacity=0.35)
    a.vis(knurl(H_R, 0.5 * MM, -58 * MM, 62 * MM, 72), chrome, "Knurl")
    a.vis(disc(H_R - 0.3 * MM, 62 * MM, 75 * MM, 64, "Y"), dark, "BaseCap")
    a.vis(disc(H_R + 0.4 * MM, 52 * MM, 58 * MM, 64, "Y"), green, "Band")
    a.vis(disc(H_R - 0.6 * MM, -76 * MM, -58 * MM, 64, "Y"), dark, "Neck")
    a.vis(disc(H_R + 0.8 * MM, -76 * MM, -72 * MM, 64, "Y"), chrome, "NeckRing")
    hy = -(H_L / 2 + 6 * MM)
    a.vis(_head_block(hy), dark, "Head")
    # the hook-on bridge (clear smoky plastic so the bar and slot can be seen through it from above) and the bar
    a.vis(_bridge_block(bx, hy), smoke, "Bridge")
    a.vis(shift(disc(BAR_R, -BAR_H / 2, BAR_H / 2, 48, "Z"), bx, by, bz), chrome, "Bar")
    a.vis(shift(disc(BAR_R + 0.9 * MM, 0.0, 1.2 * MM, 48, "Z"), bx, by, bz + BAR_H / 2 - 1.4 * MM), chrome, "BarHead")
    a.save()


def _head_block(hy):
    """The handle's head: a dark block under the bridge, rounded by chamfering the corners."""
    return _box_mesh(-14 * MM, 14 * MM, hy - 6 * MM, hy + 6 * MM, -14 * MM, 14 * MM)


def _bridge_block(bx, hy):
    return _box_mesh(-2 * MM, bx + 6 * MM, hy - 13 * MM, hy + 13 * MM, 7 * MM, 13 * MM)


def _box_mesh(x0, x1, y0, y1, z0, z1):
    return tbox(x0, x1, y0, y1, y0, y1, z0, z1)


def blade(fname):
    m = 0.08
    L = B_L
    a = Asset(fname, "Blade", "A Macintosh 3 style fibre-optic laryngoscope blade (ISO 7376 hook-on), 130 mm, 0.08 kg, with a flared hook slot "
              "in the base face and a visual lamp marker at the tip. Origin: middle of the base face, half-way up; +x toward the tip.",
              m - 1e-4, com=(0.055, 0, 0), inertia=(m / 12 * (0.02**2 + B_H**2), m / 12 * (L**2 + B_H**2), m / 12 * (L**2 + 0.02**2)))
    a.articulation_root()
    f = (0.35, 0.30)
    z = (-B_H / 2, B_H / 2)
    w = SLOT_W / 2
    polys = []
    for sgn in (1, -1):  # the two sides of the slot with the lead-in: convex prisms (colliders, invisible)
        poly = [(0, sgn * (w + LEAD)), (LEAD, sgn * w), (SLOT_D, sgn * w), (SLOT_D, sgn * 12.5 * MM), (0, sgn * 12.5 * MM)]
        if sgn < 0:
            poly = poly[::-1]
        polys.append(poly)
        a.prism_z(poly, z[0], z[1], None, f, key="blade", name="Hook")
    a.box(((SLOT_D + 40 * MM) / 2 + 0 * MM, 0, 0), (40 * MM - SLOT_D, 25 * MM, B_H), None, f, key="blade", name="Base")
    a.box((65 * MM, 0, 0), (50 * MM, 20 * MM, B_H), None, f, key="blade", name="Mid")
    a.box((110 * MM, 0, 0), (40 * MM, 14 * MM, B_H), None, f, key="blade", name="Tip")
    # visible Macintosh 3 style blade (no collision): flange block with the hook slot, then a tapering channel
    steel = a.pbr("steel", (0.74, 0.76, 0.80), 0.7, 0.3)
    dark = a.pbr("bdark", (0.06, 0.06, 0.07), 0.2, 0.5)
    green = a.pbr("bgreen", (0.08, 0.50, 0.22), 0.0, 0.5)
    for poly in polys:  # the hook pieces again, chromed (same shape as the colliders)
        pts = [(x, y, z[0]) for (x, y) in poly] + [(x, y, z[1]) for (x, y) in poly]
        k = len(poly)
        tris = []
        for i in range(1, k - 1):
            tris += [(0, i + 1, i), (k, k + i, k + i + 1)]
        for i in range(k):
            j = (i + 1) % k
            tris += [(i, j, k + j), (i, k + j, k + i)]
        a.vis((pts, _orient(pts, tris)), steel, "HookV")
    a.vis(tbox(SLOT_D, 40 * MM, -12.5 * MM, 12.5 * MM, -12.5 * MM, 12.5 * MM, -B_H / 2, B_H / 2), steel, "FlangeV")
    def wid(x):
        return 25 * MM - (25 - 14) * MM * (x - 40 * MM) / (130 * MM - 40 * MM)
    xs = [40 * MM, 70 * MM, 100 * MM, 130 * MM]
    for x0, x1 in zip(xs[:-1], xs[1:]):
        h0, h1 = wid(x0) / 2, wid(x1) / 2
        a.vis(tbox(x0, x1, h0 - 2 * MM, h0, h1 - 2 * MM, h1, -B_H / 2, B_H / 2), steel, "WebV")          # back wall
        a.vis(tbox(x0, x1, -h0, h0, -h1, h1, -B_H / 2, -B_H / 2 + 2.5 * MM), steel, "FloorV")           # floor
        a.vis(tbox(x0, x1, h0 - 7 * MM, h0, h1 - 7 * MM, h1, B_H / 2 - 2.5 * MM, B_H / 2), steel, "LipV")  # upper lip
    a.vis(tbox(112 * MM, 130 * MM, -wid(112 * MM) / 2, wid(112 * MM) / 2, -7 * MM, 7 * MM, 4 * MM, B_H / 2), steel, "TipCapV")
    a.vis(tbox(128 * MM, 130 * MM, -7 * MM, 7 * MM, -7 * MM, 7 * MM, -B_H / 2, B_H / 2), steel, "TipEndV")
    a.vis(tbox(20 * MM, 38 * MM, -4 * MM, 4 * MM, -4 * MM, 4 * MM, B_H / 2, B_H / 2 + 0.3 * MM), green, "StripeV")
    a.vis(shift(disc(6.2 * MM, 0.0, 0.5 * MM, 48, "Z"), 124 * MM, 0.0, B_H / 2), steel, "LampRing")
    a.vis(shift(disc(4.8 * MM, 0.0, 0.8 * MM, 48, "Z"), 124 * MM, 0.0, B_H / 2), dark, "LampSocket")
    root = a.root.GetPrim()
    lk = a.link("Lamp", 1e-4, inertia=(1e-10,) * 3, pos=(124 * MM, 0, B_H / 2 - 3.0 * MM))
    glow = a.pbr("lamp", LAMP, 0.0, 0.3, emit=(1.0, 0.85, 0.2))
    dome = lathe([(0.0, -1.5 * MM), (3.6 * MM, -1.5 * MM), (3.6 * MM, 0.4 * MM), (2.8 * MM, 1.6 * MM), (1.4 * MM, 2.3 * MM), (0.0, 2.5 * MM)], 48)
    a.vis(dome, glow, "LampDome")
    a.prismatic("Lamp", root, lk, axis="Z", lo=0.0, hi=4.5 * MM, stiffness=20.0, damping=0.3, target=0.0, p0=(124 * MM, 0, B_H / 2 - 3.0 * MM), p1=(0, 0, 0))
    a.save()


def tray(fname):
    a = Asset(fname, "Tray", "A fixed dark crash-cart tray 260 x 400 x 3 mm with a low rim. Origin: table level at its centre.", 1.0, com=(0, 0, 0.0015), inertia=(1e-2,) * 3)
    a.box((0, 0, 1.5 * MM), (0.26, 0.40, 3.0 * MM), None, (0.40, 0.35), key="steel", name="Plate")
    dark = a.pbr("tray", (0.09, 0.10, 0.12), 0.5, 0.45)
    rim = a.pbr("rim", (0.30, 0.32, 0.36), 0.7, 0.35)
    a.vis(tbox(-0.13, 0.13, -0.20, 0.20, -0.13, 0.13, 0.0, 3.0 * MM), dark, "PlateV")
    for (x0, x1, y0, y1) in ((-0.13, 0.13, -0.20, -0.198), (-0.13, 0.13, 0.198, 0.20), (-0.13, -0.128, -0.20, 0.20), (0.128, 0.13, -0.20, 0.20)):
        a.vis(tbox(x0, x1, y0, y1, y0, y1, 3.0 * MM, 5.5 * MM), rim, "RimV")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    handle("handle.usda")
    blade("blade.usda")
    tray("tray.usda")
