"""Writes this task's USD assets with pxr (boxes, cylinders and two small convex meshes; nothing from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up."""
import math
import numpy as np
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001


# ---- visual-mesh helpers (as submission_tools/meshkit.py): closed, outward-wound round shapes; colliders stay separate ----
def _orient(pts, tris):
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[a], np.cross(p[b], p[c])) for a, b, c in tris) / 6.0
    return tris if vol >= 0 else [(a, c, b) for a, b, c in tris]


def lathe(profile, n=96):
    """Revolve a closed (r, z) profile (metres) about z; r = 0 points are welded."""
    k = len(profile)
    pts, idx = [], {}
    for i in range(n):
        for j, (r, z) in enumerate(profile):
            if r == 0.0:
                if j not in idx:
                    idx[j] = len(pts)
                    pts.append((0.0, 0.0, z))
                continue
            idx[(i, j)] = len(pts)
            pts.append((r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z))
    def at(i, j):
        return idx[j] if profile[j][0] == 0.0 else idx[(i % n, j)]
    tris = []
    for i in range(n):
        for j in range(k):
            j2 = (j + 1) % k
            a, b, c, d = at(i, j), at(i, j2), at(i + 1, j2), at(i + 1, j)
            for t in ((a, b, c), (a, c, d)):
                if len({*t}) == 3:
                    tris.append(t)
    return pts, _orient(pts, tris)


def loft_square(layers):
    """A closed prism from square layers [(half_side, z), ...] bottom to top (chamfers and bevels)."""
    pts, tris = [], []
    for h, z in layers:
        pts += [(-h, -h, z), (h, -h, z), (h, h, z), (-h, h, z)]
    for L in range(len(layers) - 1):
        for e in range(4):
            e2 = (e + 1) % 4
            a, b, c, d = 4 * L + e, 4 * L + e2, 4 * (L + 1) + e2, 4 * (L + 1) + e
            tris += [(a, b, c), (a, c, d)]
    top = 4 * (len(layers) - 1)
    tris += [(0, 2, 1), (0, 3, 2), (top, top + 1, top + 2), (top, top + 2, top + 3)]
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

    def color(self, key, rgb, rough=0.6, metallic=0.0):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
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

    def vmesh(self, name, pts, tris, rgb, rough=0.5, metallic=0.0):
        """A visible-only closed mesh (no physics) under the current link; ``name`` is used as given."""
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}")
        P = [Gf.Vec3f(*[float(v) for v in p]) for p in pts]
        g.CreatePointsAttr(Vt.Vec3fArray(P))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for t in tris for i in t])
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateDoubleSidedAttr(True)
        lo = [min(p[i] for p in P) for i in range(3)]
        hi = [max(p[i] for p in P) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        key = "v%02d%02d%02d_%d_%d" % (*[int(c * 99) for c in rgb], int(rough * 10), int(metallic * 10))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.color(key, rgb, rough, metallic))

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
# Copper-tube deburring task assets. Metres, kilograms, Z up.
#   copper_tube.usda : a 22 x 0.9 mm EN 1057 R250 copper tube cut to 100 mm (0.053 kg), an articulation: the tube (free root)
#                      plus 36 visual-only marker links on prismatic joints that show the inner burr (two 0.2 mm layers in each
#                      of 12 sectors, slid outward into the wall when reamed away) and the chamfer (a bright mark on the rim).
#                      Origin: the middle of the lower end; z along the tube. Open end up. Bore 20.2 mm (collider ring on the top
#                      14 mm, solid below).
#   reamer.usda      : an inner-outer deburring reamer (typical hand tool of the RIDGID 223 class: a 45 mm cutting cup, a 30 mm
#                      handle, a conical blade), 0.090 kg, one rigid body. Origin: the cone's tip; z along the tool, tip down.
#   tube_rack.usda   : a fixed bench rack: a 4 mm base plate with a 27 mm bore through a 20 mm wall, 7 mm lead-in. Origin: table
#                      level at the bore's axis.
#   reamer_rest.usda : a fixed rest for the reamer: a 6 mm plate with an 18 mm bore through a 12 mm wall, 5 mm lead-in.
# ======================================================================================================================
TUBE_OD, TUBE_WALL, TUBE_L = 22.0 * MM, 0.9 * MM, 100.0 * MM  # EN 1057 22 x 0.9 mm
TUBE_RI = TUBE_OD / 2 - TUBE_WALL  # 10.1 mm
TUBE_MASS = 8.94e3 * math.pi / 4 * (TUBE_OD**2 - (2 * TUBE_RI) ** 2) * TUBE_L  # copper 8.94 g/cm3: 0.053 kg
RING_H = 14.0 * MM  # the collider ring at the open end; solid below
RING_N = 32
LAYER = 0.2 * MM  # the inner burr: two layers of 0.2 mm
N_SECT = 12
SECT_DEG = [-165.0 + 30.0 * i for i in range(N_SECT)]  # sector middles in the tube frame
CHIP_W = 5.0 * MM  # a chip is one 30 degree sector (5.2 mm of arc at the bore)
CHIP_H = 2.0 * MM
HIDE_SLIDE = 0.5 * MM  # a burr layer slides this far outward into the wall to disappear
SHOW_SLIDE = 0.4 * MM  # a bevel mark slides this far up out of the rim
CONE_TIP_R, CONE_BASE_R, CONE_H = 4.0 * MM, 13.0 * MM, 14.0 * MM
CUP_R, CUP_T = 22.5 * MM, 8.0 * MM  # the 45 mm cutting cup
HANDLE_R, HANDLE_H = 15.0 * MM, 50.0 * MM
TOOL_L = CONE_H + CUP_T + HANDLE_H  # 72 mm
TOOL_MASS = 0.090
RACK_PLATE, RACK_WALL, RACK_BORE_R, RACK_FUNNEL = 4.0 * MM, 20.0 * MM, 13.5 * MM, 7.0 * MM  # wall = height above the table
REST_PLATE, REST_WALL, REST_BORE_R, REST_FUNNEL = 6.0 * MM, 18.0 * MM, 9.0 * MM, 5.0 * MM
COPPER, STEEL, PLASTIC = (0.80, 0.45, 0.28), (0.66, 0.68, 0.72), (0.18, 0.20, 0.24)
ORANGE, BRIGHT = (1.0, 0.62, 0.05), (0.95, 0.96, 0.98)


def ngon(r, n=24, a0=0.0):
    return [(r * math.cos(math.radians(a0 + 360.0 * k / n)), r * math.sin(math.radians(a0 + 360.0 * k / n))) for k in range(n)]


def ring_walls(a, r_in, thick, z0, height, n, rgb, f, key, name="Wall"):
    """n wall boxes round a bore of inner radius ``r_in`` (at the flats), from z0 up ``height``."""
    for k in range(n):
        phi = 360.0 * k / n
        ph = math.radians(phi)
        rc = r_in + thick / 2
        w = 2 * (r_in + thick) * math.tan(math.pi / n) * 1.03
        a.box((rc * math.cos(ph), rc * math.sin(ph), z0 + height / 2), (thick, w, height), rgb, f, key=key, name=name, rot=(0, 0, phi))


def funnel(a, r_in, ztop, depth, n, rgb, f, key):
    """A 45 degree lead-in ``depth`` tall flaring out above ``ztop`` (n tilted plates)."""
    skin = 1.5 * MM
    for k in range(n):
        phi = 360.0 * k / n
        ph = math.radians(phi)
        length = depth * math.sqrt(2.0)
        rm = r_in + depth / 2 + 0.5 * skin * math.sin(math.radians(45.0)) + 0.0005
        zm = ztop + depth / 2 - 0.5 * skin * math.cos(math.radians(45.0))
        a.n += 1
        g = UsdGeom.Cube.Define(a.stage, f"{a.cur}/Flange{a.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(rm * math.cos(ph), rm * math.sin(ph), zm))
        g.AddRotateZOp().Set(phi)
        g.AddRotateYOp().Set(-45.0)
        g.AddScaleOp().Set(Gf.Vec3f(length, 2 * (r_in + depth) * math.tan(math.pi / n) * 1.1, skin))
        a._finish(g, rgb, f, key, mesh=False)


def slide(a, name, b0, b1, p0, axis, hi, rotz_deg, stiffness=5.0, damping=0.1):
    """A prismatic joint 0..hi along ``axis`` of the joint frame (turned by ``rotz_deg`` about z in body 0's frame)."""
    j = a.prismatic(name, b0, b1, axis=axis, lo=0.0, hi=hi, stiffness=stiffness, damping=damping, target=0.0, p0=p0, p1=(0, 0, 0))
    h = math.radians(rotz_deg) / 2
    j.CreateLocalRot0Attr().Set(Gf.Quatf(math.cos(h), 0, 0, math.sin(h)))
    return j


def tube(fname):
    m, L = TUBE_MASS, TUBE_L
    m_root = m - 36 * 1e-4  # the root body's share: with the 36 marker links the tube weighs TUBE_MASS
    ro, ri = TUBE_OD / 2, TUBE_RI
    izz = 0.5 * m * (ro**2 + ri**2)
    ixx = m / 12 * (3 * (ro**2 + ri**2) + L**2)
    a = Asset(fname, "CopperTube", "A 22 x 0.9 mm EN 1057 R250 copper tube cut to 100 mm (0.053 kg), open end up, with its inner burr (36 "
              "visual marker links on prismatic joints: two 0.2 mm layers in each of 12 sectors, and a rim mark per sector for the "
              "chamfer). Origin: the middle of the lower end; z along the tube.", m_root, com=(0, 0, L / 2), inertia=(ixx, ixx, izz))
    a.articulation_root()
    f = (0.50, 0.45)
    CH = 0.6 * MM  # the foot's edge is chamfered 0.6 mm (a cut tube's edge is never sharp; a sharp base edge chattered in the simulator)
    a.prism_z(ngon(ro - CH, 48), 0.0, CH, None, f, key="copper", name="Foot", poly_top=ngon(ro, 48))
    a.cyl((0, 0, CH + (L - RING_H - CH) / 2), ro, L - RING_H - CH, "Z", None, f, key="copper", name="Body")
    ring_walls(a, ri, TUBE_WALL, L - RING_H, RING_H, RING_N, None, f, "copper", name="Rim")
    # what you see: a smooth round hollow copper tube (96 segments), rolled rims, a bright inside edge
    rm = ri + 0.45 * MM  # outer shell bright copper, inner liner darker so the burr stands out
    pts, tris = lathe([(ro, 0.0), (ro, L - 0.1 * MM), (ro - 0.1 * MM, L), (rm, L), (rm, 0.0)], 96)
    a.vmesh("TubeVis", pts, tris, (0.78, 0.40, 0.24), rough=0.28, metallic=0.95)
    pts, tris = lathe([(rm, 0.0), (rm, L), (ri, L), (ri, 0.0)], 96)
    a.vmesh("TubeBore", pts, tris, (0.30, 0.15, 0.09), rough=0.6, metallic=0.5)
    pts, tris = lathe([(ro + 0.05 * MM, 3.0 * MM), (ro + 0.05 * MM, 4.2 * MM), (ro - 0.1 * MM, 4.2 * MM), (ro - 0.1 * MM, 3.0 * MM)], 96)
    a.vmesh("TubeBand", pts, tris, (0.55, 0.28, 0.17), rough=0.5, metallic=0.9)
    root = a.root.GetPrim()
    inert = (1e-10, 1e-10, 1e-10)
    MARK_M = 1e-4  # a marker link weighs 0.1 g (36 of them 3.6 g, taken off the tube's own mass so it stays 0.053 kg in all)
    for i, deg in enumerate(SECT_DEG):
        th = math.radians(deg)
        # BurrA is the inner 0.2 mm (reamed away first), BurrB the outer 0.2 mm (against the wall)
        for tag, r_c in (("BurrA", ri - 1.5 * LAYER), ("BurrB", ri - 0.5 * LAYER)):
            p0 = (r_c * math.cos(th), r_c * math.sin(th), L - CHIP_H / 2 - 0.5 * MM)  # top 0.5 mm below the rim face: a spent chip sunk in the wall never shows through the rim
            lk = a.link(f"{tag}{i}", MARK_M, inertia=inert, pos=p0)
            UsdGeom.Xformable(lk).AddRotateZOp().Set(deg)
            a.box((0, 0, 0), (LAYER, CHIP_W, CHIP_H), ORANGE, None, key="burr", name="Burr")
            slide(a, f"{tag}{i}", root, lk, p0, "X", HIDE_SLIDE, deg)
        rb = ri + 0.40 * MM
        p0 = (rb * math.cos(th), rb * math.sin(th), L - 0.3 * MM)
        lk = a.link(f"Bevel{i}", MARK_M, inertia=inert, pos=p0)
        UsdGeom.Xformable(lk).AddRotateZOp().Set(deg)
        a.box((0, 0, 0), (0.5 * MM, 3.0 * MM, 0.4 * MM), BRIGHT, None, key="bevel", name="Bevel")
        slide(a, f"Bevel{i}", root, lk, p0, "Z", SHOW_SLIDE, deg)
    a.save()


def reamer(fname):
    a = Asset(fname, "Reamer", "An inner-outer deburring reamer of the RIDGID 223 class: a conical blade (8 mm tip, 26 mm base, 14 mm "
              "long), a 45 mm cutting cup 8 mm thick and a 30 mm handle 50 mm long: 72 mm, 0.090 kg. Origin: the cone's tip; z along "
              "the tool, tip down.", TOOL_MASS, com=(0, 0, 0.040),
              inertia=(TOOL_MASS * (3 * 0.02**2 + 0.072**2) / 12, TOOL_MASS * (3 * 0.02**2 + 0.072**2) / 12, 0.5 * TOOL_MASS * 0.018**2))
    f = (0.45, 0.40)
    a.prism_z(ngon(CONE_TIP_R), 0.0, CONE_H, STEEL, f, key="steel", name="Cone", poly_top=ngon(CONE_BASE_R))  # collider (purpose guide)
    a.cyl((0, 0, CONE_H + CUP_T / 2), CUP_R, CUP_T, "Z", None, f, key="steel", name="Cup")
    a.cyl((0, 0, CONE_H + CUP_T + HANDLE_H / 2), HANDLE_R, HANDLE_H, "Z", None, f, key="handle", name="Handle")
    # what you see (RIDGID 223S-like inner-outer reamer): bright ground steel cone with three flutes, a steel cutting cup with
    # chamfered edges and a ring of cutter slots, a chrome ferrule, a red ribbed grip and a black end cap
    h0 = CONE_H
    pts, tris = lathe([(0.0, 0.0), (CONE_TIP_R, 0.0), (CONE_BASE_R, CONE_H), (0.0, CONE_H)], 96)
    a.vmesh("ConeVis", pts, tris, (0.82, 0.84, 0.88), rough=0.22, metallic=0.95)
    for k in range(3):  # flutes: dark slots down the cone's flank
        ph = math.radians(120.0 * k + 20.0)
        r0, r1 = CONE_TIP_R + 0.6 * MM, CONE_BASE_R + 0.2 * MM
        w = 1.6 * MM
        c, sn = math.cos(ph), math.sin(ph)
        t = (-sn, c)
        P = [(r0 * c + t[0] * w / 2, r0 * sn + t[1] * w / 2, 1.0 * MM), (r0 * c - t[0] * w / 2, r0 * sn - t[1] * w / 2, 1.0 * MM),
             (r1 * c - t[0] * w, r1 * sn - t[1] * w, CONE_H - 0.2 * MM), (r1 * c + t[0] * w, r1 * sn + t[1] * w, CONE_H - 0.2 * MM)]
        fl = [(P[i][0] * 1.0, P[i][1] * 1.0, P[i][2]) for i in range(4)]
        # a thin dark wedge slightly proud of the cone (closed: two layers)
        P2 = [(x * 0.97, y * 0.97, z) for x, y, z in fl]
        pts = fl + P2
        tr = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
        a.vmesh(f"Flute{k}", pts, _orient(pts, tr), (0.08, 0.08, 0.09), rough=0.6, metallic=0.2)
    z0, z1 = CONE_H, CONE_H + CUP_T
    pts, tris = lathe([(0.0, z0), (CUP_R - 1.5 * MM, z0), (CUP_R, z0 + 1.5 * MM), (CUP_R, z1 - 1.5 * MM), (CUP_R - 1.5 * MM, z1), (0.0, z1)], 96)
    a.vmesh("CupVis", pts, tris, (0.70, 0.72, 0.76), rough=0.35, metallic=0.9)
    for k in range(6):  # cutter slots in the cup's face
        ph = math.radians(60.0 * k)
        c, sn = math.cos(ph), math.sin(ph)
        P = [(rr * c - ww * sn, rr * sn + ww * c, z0 - 0.15 * MM) for rr, ww in ((CONE_BASE_R + 1 * MM, -1.2 * MM), (CUP_R - 2 * MM, -1.2 * MM),
                                                                       (CUP_R - 2 * MM, 1.2 * MM), (CONE_BASE_R + 1 * MM, 1.2 * MM))]
        P2 = [(x, y, z0 + 0.5 * MM) for x, y, z in P]
        pts = P + P2
        tr = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
        a.vmesh(f"Slot{k}", pts, _orient(pts, tr), (0.10, 0.10, 0.11), rough=0.6, metallic=0.3)
    z2 = z1 + 3.0 * MM
    pts, tris = lathe([(0.0, z1), (HANDLE_R + 1.0 * MM, z1), (HANDLE_R + 1.0 * MM, z2), (0.0, z2)], 96)
    a.vmesh("Ferrule", pts, tris, (0.88, 0.89, 0.92), rough=0.15, metallic=1.0)
    z3 = CONE_H + CUP_T + HANDLE_H - 4.0 * MM
    prof, zz, up = [(0.0, z2)], z2, True
    while zz < z3 - 1e-9:  # ribbed grip: ten ribs
        prof += [(HANDLE_R, zz), (HANDLE_R, zz + 2.0 * MM), (HANDLE_R - 0.7 * MM, zz + 2.6 * MM), (HANDLE_R - 0.7 * MM, zz + 3.4 * MM),
                 (HANDLE_R, zz + 4.0 * MM)]
        zz += 4.0 * MM
    prof = [p for p in prof if p[1] <= z3 + 1e-9] + [(HANDLE_R, z3), (0.0, z3)]
    a.vmesh("Grip", *lathe(prof, 96), (0.78, 0.09, 0.07), rough=0.55, metallic=0.0)
    ztop = CONE_H + CUP_T + HANDLE_H
    a.vmesh("EndCap", *lathe([(0.0, z3), (HANDLE_R, z3), (HANDLE_R - 1.5 * MM, ztop - 0.8 * MM), (HANDLE_R - 3.5 * MM, ztop), (0.0, ztop)], 96),
            (0.08, 0.08, 0.09), rough=0.5, metallic=0.1)
    a.save()


def rack(fname, name, doc, plate, wall, bore_r, funnel_d, mass, n, ring_thick):
    a = Asset(fname, name, doc, mass, com=(0, 0, wall / 2), inertia=(1e-3, 1e-3, 1e-3))
    f = (0.45, 0.40)
    a.box((0, 0, plate / 2), (0.090, 0.090, plate), None, f, key="plastic", name="Plate")
    ring_walls(a, bore_r, ring_thick, plate, wall - plate, n, None, f, "plastic")
    funnel(a, bore_r, wall, funnel_d, n, None, f, "plastic")
    # what you see: a plate with chamfered edges in anodised aluminium, a round satin-black ring with a smooth 45 degree lead-in
    plate_rgb = (0.20, 0.30, 0.45) if name == "TubeRack" else (0.30, 0.31, 0.33)
    pts, tris = loft_square([(0.045 - 1.2 * MM, 0.0), (0.045, 1.2 * MM), (0.045, plate - 1.2 * MM), (0.045 - 1.2 * MM, plate)])
    a.vmesh("PlateVis", pts, tris, plate_rgb, rough=0.35, metallic=0.85)
    t = ring_thick
    pts, tris = lathe([(bore_r, plate), (bore_r + t, plate), (bore_r + t, wall + funnel_d), (bore_r + funnel_d, wall + funnel_d), (bore_r, wall)], 96)
    a.vmesh("RingVis", pts, tris, (0.06, 0.06, 0.07), rough=0.5, metallic=0.2)
    pts, tris = lathe([(bore_r + t + 0.2 * MM, plate + 3 * MM), (bore_r + t + 0.2 * MM, plate + 4.2 * MM), (bore_r + t - 0.3 * MM, plate + 4.2 * MM),
                       (bore_r + t - 0.3 * MM, plate + 3 * MM)], 96)
    a.vmesh("RingBand", pts, tris, (0.85, 0.55, 0.10), rough=0.4, metallic=0.5)
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    tube("copper_tube.usda")
    reamer("reamer.usda")
    rack("tube_rack.usda", "TubeRack", "A fixed bench rack for cut tube: a 90 x 90 x 4 mm base plate with a 27 mm bore through a 20 mm "
         "wall (7 mm 45 degree lead-in). Origin: table level at the bore's axis.", RACK_PLATE, RACK_WALL, RACK_BORE_R, RACK_FUNNEL, 0.6, 24, 15.0 * MM)
    rack("reamer_rest.usda", "ReamerRest", "A fixed rest for the reamer: a 90 x 90 x 6 mm plate with an 18 mm bore through an 18 mm wall "
         "(5 mm lead-in); the cone stands in it. Origin: table level at the bore's axis.", REST_PLATE, REST_WALL, REST_BORE_R, REST_FUNNEL, 0.4, 24, 14.0 * MM)
