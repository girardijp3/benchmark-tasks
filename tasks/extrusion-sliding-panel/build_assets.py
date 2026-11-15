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


# -- frame: a bottom and a top 2020 rail along y with their slots facing each other, an end post at each end ---------------
# Frame: origin on the table at the middle of the frame, in the slots' plane (x across the slot); y along the rails, z up.
P = 20 * MM
OPEN_W, LIP = 6.2 * MM, 1.8 * MM
CAV_W, CAV_D = 11.0 * MM, 6.0 * MM
RAIL_HALF = 110 * MM  # the rails run from y = -110 to +110 mm
FLOOR_Z = P - LIP - CAV_D  # the bottom slot's cavity floor: 12.2 mm above the table
PANEL_W, PANEL_H, PANEL_T = 100 * MM, 80 * MM, 3 * MM
TOP_UNDER = FLOOR_Z + PANEL_H - 4.5 * MM  # the top rail's underside (the panel's top edge goes 4.5 mm into its slot)
PANEL_MASS = 0.037


def rail(a, z0, up, y0, y1, alu, fr):
    """A 20 x 20 rail from y0 to y1 whose slot opens upward (up=+1: slot on its top face, z0 the underside) or downward."""
    for t in (-1, 1):
        xa, xb = OPEN_W / 2, P / 2
        zl0, zl1 = (z0 + P - LIP, z0 + P) if up > 0 else (z0, z0 + LIP)
        a.box((t * (xa + xb) / 2, (y0 + y1) / 2, (zl0 + zl1) / 2), (xb - xa, y1 - y0, LIP), rgb=alu, collide=fr, key="alu")
        zw0, zw1 = (z0, z0 + P - LIP) if up > 0 else (z0 + LIP, z0 + P)
        xa2 = CAV_W / 2
        a.box((t * (xa2 + xb) / 2, (y0 + y1) / 2, (zw0 + zw1) / 2), (xb - xa2, y1 - y0, zw1 - zw0), rgb=alu, collide=fr, key="alu")
    zf0, zf1 = (z0, z0 + P - LIP - CAV_D) if up > 0 else (z0 + LIP + CAV_D, z0 + P)
    a.box((0, (y0 + y1) / 2, (zf0 + zf1) / 2), (P, y1 - y0, zf1 - zf0), rgb=alu, collide=fr, key="alu")


def frame():
    a = Asset("slide_frame.usda", "SlideFrame",
              "A frame of 20 x 20 mm aluminium extrusion for a sliding panel: a bottom and a top rail 220 mm long along y with "
              "their T-slots facing each other (opening 6.2 mm, lips 1.8 mm, cavity 11 x 6 mm), 67.7 mm apart between the lips, "
              "and a 20 x 20 mm end post at each end (the far one, at +y, is the end stop). A fixture. Origin on the table "
              "at the middle of the frame in the slots' plane; x across the slots.", mass=3.0, inertia=(1e-2, 1e-2, 1e-2))
    fr = (0.35, 0.3)
    alu = (0.78, 0.8, 0.83)
    rail(a, 0.0, +1, -RAIL_HALF, RAIL_HALF, alu, fr)
    rail(a, TOP_UNDER, -1, -RAIL_HALF, RAIL_HALF, alu, fr)
    H = TOP_UNDER + P
    for s in (-1, 1):
        y0, y1 = (RAIL_HALF, RAIL_HALF + P) if s > 0 else (-RAIL_HALF - P, -RAIL_HALF)
        a.box((0, (y0 + y1) / 2, H / 2), (P, P, H), rgb=alu, collide=fr, key="alu")
        a.box((0, s * (RAIL_HALF + P) + 0.1 * MM * s, H / 2), (6.2 * MM, 0.3 * MM, H - 4 * MM), rgb=(0.1, 0.1, 0.12))
        a.box((0, (y0 + y1) / 2, 2 * MM), (30 * MM, 36 * MM, 4 * MM), rgb=(0.3, 0.31, 0.34), collide=fr, key="alu")
    a.box((0, RAIL_HALF - 0.15 * MM, FLOOR_Z + 20 * MM), (P - 4 * MM, 0.3 * MM, 30 * MM), rgb=(0.7, 0.1, 0.1))  # the stop's red face
    a.save()


def panel():
    m = PANEL_MASS
    a = Asset("sliding_panel.usda", "SlidingPanel",
              "A 3 mm clear acrylic sliding panel, 100 x 80 mm (thickness along x), with a pull knob on its front face (-x): "
              "an 8 mm stem and a 16 (y) x 14 (x) x 16 mm (z) head, 37 g. Origin at the middle of its lower edge.",
              mass=m, com=(-3 * MM, 0, PANEL_H / 2), inertia=box_inertia(m, PANEL_T + 0.03, PANEL_W, PANEL_H))
    a.box((0, 0, PANEL_H / 2), (PANEL_T, PANEL_W, PANEL_H), rgb=(0.62, 0.82, 0.92), collide=(0.4, 0.3), key="acrylic")
    zc = 55 * MM
    a.box((-(PANEL_T / 2 + 6 * MM), 0, zc), (12 * MM, 8 * MM, 8 * MM), rgb=(0.15, 0.15, 0.17), collide=(0.4, 0.3), key="plastic")
    a.box((-(PANEL_T / 2 + 12 * MM + 7 * MM), 0, zc), (14 * MM, 16 * MM, 16 * MM), rgb=(0.85, 0.15, 0.1), collide=(0.4, 0.3),
          key="plastic")
    a.box((0, 0, PANEL_H - 1.5 * MM), (PANEL_T + 0.4 * MM, PANEL_W, 3 * MM), rgb=(0.9, 0.95, 1.0))
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    frame()
    panel()
