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
        g = UsdGeom.Cube.Define(self.stage, f"/{self.name}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        self._xf(g, center)
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, key)

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"/{self.name}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center)
        self._finish(g, rgb, collide, key)

    def prism_x(self, poly_yz, x0, x1, rgb, collide=None, key="c", name="Wall"):
        """A convex prism along x: cross-section ``poly_yz`` (list of (y, z), counter-clockwise seen from +x)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"/{self.name}/{name}{self.n}")
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
        g = UsdGeom.Mesh.Define(self.stage, f"/{self.name}/{name}{self.n}")
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


# -- parts ------------------------------------------------------------------------------------------------------------
PROFILE = 20 * MM  # 2020 extrusion
POST_L = 100 * MM
POST_MASS = 0.043  # 2020 profile: about 0.43 kg/m
CUBE = 34 * MM  # the corner cube's outside edge
SOCK = 22 * MM  # the sockets' inside width (2 mm of play around the 20 mm profile, 1 mm a side)
FLOOR = 12 * MM  # the up socket's floor above the table (22 mm deep)
CHAMF = 4 * MM  # the socket mouth's lead-in
RAIL_Z0 = 7 * MM  # the rails' underside (the rails sit in the cube's side sockets)
RAIL_L = 70 * MM  # the rail stubs beyond the cube


def corner_cube():
    a = Asset("corner_cube.usda", "CornerCube",
              "A 3-way corner cube connector for 2020 extrusion on two rail stubs, a fixture: a 34 mm cube with a "
              "22 x 22 mm socket opening upward (22 mm deep, 4 mm lead-in chamfer at the mouth) and two side "
              "sockets holding two 70 mm stubs of 20 x 20 mm rail, one along +x and one along -y. Origin on the "
              "table under the cube's middle.", mass=1.0, inertia=(1e-3, 1e-3, 1e-3))
    fr = (0.35, 0.3)
    cube = (0.2, 0.22, 0.25)
    alu = (0.78, 0.8, 0.83)
    h = CUBE
    c, s = CUBE / 2, SOCK / 2
    a.box((0, 0, FLOOR / 2), (CUBE, CUBE, FLOOR), rgb=cube, collide=fr, key="alu")
    for t in (-1, 1):
        # walls along y (x = +-[s, c]), full width
        poly = [(t * s, FLOOR), (t * c, FLOOR), (t * c, h), (t * (s + CHAMF), h), (t * s, h - CHAMF)]
        if t < 0:
            poly = poly[::-1]
        a.prism_y(poly, -c, c, cube, collide=fr, key="alu")
        # walls along x (y = +-[s, c]) between the x walls
        poly = [(t * s, FLOOR), (t * c, FLOOR), (t * c, h), (t * (s + CHAMF), h), (t * s, h - CHAMF)]
        if t < 0:
            poly = poly[::-1]
        a.prism_x(poly, -s, s, cube, collide=fr, key="alu")
    # the rail stubs, fused into the cube's side sockets
    a.box((c + RAIL_L / 2, 0, RAIL_Z0 + PROFILE / 2), (RAIL_L, PROFILE, PROFILE), rgb=alu, collide=fr, key="alu")
    a.box((0, -(c + RAIL_L / 2), RAIL_Z0 + PROFILE / 2), (PROFILE, RAIL_L, PROFILE), rgb=alu, collide=fr, key="alu")
    dark = (0.1, 0.1, 0.12)
    a.box((c + RAIL_L / 2, 0, RAIL_Z0 + PROFILE + 0.1 * MM), (RAIL_L, 6.2 * MM, 0.3 * MM), rgb=dark)
    a.box((0, -(c + RAIL_L / 2), RAIL_Z0 + PROFILE + 0.1 * MM), (6.2 * MM, RAIL_L, 0.3 * MM), rgb=dark)
    a.save()


def post_rack():
    a = Asset("post_rack.usda", "PostRack",
              "A holder for a 2020 post: a 50 x 50 x 30 mm block with a 22 x 22 mm pocket 18 mm deep (3 mm lead-in "
              "chamfer), a fixture. Origin on the table under the pocket's middle.", mass=0.4,
              inertia=(1e-3, 1e-3, 1e-3))
    fr = (0.45, 0.35)
    col = (0.2, 0.22, 0.5)
    H, D, c, s, ch = 30 * MM, 18 * MM, 25 * MM, SOCK / 2, 3 * MM
    f = H - D
    a.box((0, 0, f / 2), (2 * c, 2 * c, f), rgb=col, collide=fr, key="plastic")
    for t in (-1, 1):
        poly = [(t * s, f), (t * c, f), (t * c, H), (t * (s + ch), H), (t * s, H - ch)]
        if t < 0:
            poly = poly[::-1]
        a.prism_y(poly, -c, c, col, collide=fr, key="plastic")
        a.prism_x(poly, -s, s, col, collide=fr, key="plastic")
    a.save()


def post():
    m = POST_MASS
    a = Asset("post.usda", "Post",
              "A 100 mm length of 20 x 20 mm T-slot aluminium extrusion (2020), 43 g, clear anodized, shown as a solid "
              "bar with the slots painted on its four faces and ends. Origin at the middle of its lower end; axis "
              "along z.", mass=m, com=(0, 0, POST_L / 2), inertia=box_inertia(m, PROFILE, PROFILE, POST_L))
    a.box((0, 0, POST_L / 2), (PROFILE, PROFILE, POST_L), rgb=(0.78, 0.8, 0.83), collide=(0.35, 0.3), key="alu")
    dark = (0.1, 0.1, 0.12)
    o = PROFILE / 2 + 0.1 * MM
    a.box((o, 0, POST_L / 2), (0.3 * MM, 6.2 * MM, POST_L - 4 * MM), rgb=dark)
    a.box((-o, 0, POST_L / 2), (0.3 * MM, 6.2 * MM, POST_L - 4 * MM), rgb=dark)
    a.box((0, o, POST_L / 2), (6.2 * MM, 0.3 * MM, POST_L - 4 * MM), rgb=dark)
    a.box((0, -o, POST_L / 2), (6.2 * MM, 0.3 * MM, POST_L - 4 * MM), rgb=dark)
    a.box((0, 0, POST_L + 0.1 * MM), (6.2 * MM, 6.2 * MM, 0.3 * MM), rgb=dark)
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    corner_cube()
    post_rack()
    post()
