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

    def cyl(self, center, radius, height, axis="Z", rgb=None, collide=None, key="c", name=None, segments=64):
        """A closed cylinder mesh with ``segments`` sides (smooth when rendered close up); a collider is its convex hull."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"/{self.name}/{name or 'Cyl'}{self.n}")
        ring = [(radius * math.cos(2 * math.pi * i / segments), radius * math.sin(2 * math.pi * i / segments))
                for i in range(segments)]
        h = height / 2
        pts = [(u, v, -h) for u, v in ring] + [(u, v, h) for u, v in ring]  # axis along local z
        faces, counts = [], []
        for i in range(1, segments - 1):  # caps
            faces += [0, i + 1, i]
            faces += [segments, segments + i, segments + i + 1]
            counts += [3, 3]
        for i in range(segments):  # sides
            j = (i + 1) % segments
            faces += [i, j, segments + j, i, segments + j, segments + i]
            counts += [3, 3]
        if axis == "X":  # local (u, v, w) -> (w, u, v): the cylinder's axis becomes x
            pts = [(w, u, v) for u, v, w in pts]
        elif axis == "Y":
            pts = [(v, w, u) for u, v, w in pts]
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        g.CreateFaceVertexCountsAttr(counts)
        g.CreateFaceVertexIndicesAttr(faces)
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        self._xf(g, center)
        self._finish(g, rgb, collide, key, mesh=True)

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


# -- the light curtain: an emitter and a receiver post (one fixture) ------------------------------------------------
# Frame: origin on the table at the middle of the protected field's footprint; x = the field's normal (toward the
# machine), y along the beams, z up. Emitter at y = -0.22, receiver at y = +0.22.
POST = 40 * MM  # housing cross-section (typical Type 4 hand-protection curtain: about 30 to 50 mm)
POST_Y = 0.22
HOUSING_BOT, HOUSING_TOP = 12 * MM, 242 * MM
BEAMS, PITCH, FIRST = 8, 20 * MM, 57 * MM  # 8 beams every 20 mm from 57 mm: protected height 52..202 mm (150 mm)
LENS_R = 5 * MM


def curtain():
    a = Asset("curtain.usda", "Curtain", "A 30 mm-resolution safety light curtain, protected height 150 mm: two 40 x 40 mm "
              "housings 440 mm apart (centres), 230 mm tall, on 90 x 90 x 12 mm foot plates. 8 beams, 20 mm pitch, 10 mm "
              "lenses (resolution = pitch + lens). Origin on the table at the field's footprint; x normal to the field.",
              mass=6.0, inertia=(0.2, 0.2, 0.2))
    fr = (0.5, 0.35)
    for side, nm in ((-1, "emitter"), (1, "receiver")):
        y = side * POST_Y
        a.box((0, y, 6 * MM), (90 * MM, 90 * MM, 12 * MM), rgb=(0.2, 0.2, 0.22), collide=fr, key="steel")
        h = HOUSING_TOP - HOUSING_BOT - 40 * MM
        a.box((0, y, HOUSING_BOT + 20 * MM + h / 2), (POST, POST, h), rgb=(0.95, 0.78, 0.08), collide=fr, key="steel")
        a.box((0, y, HOUSING_BOT + 10 * MM), (POST, POST, 20 * MM), rgb=(0.08, 0.08, 0.09), collide=fr, key="steel")
        a.box((0, y, HOUSING_TOP - 10 * MM), (POST, POST, 20 * MM), rgb=(0.08, 0.08, 0.09), collide=fr, key="steel")
        face = -side  # the lenses face the other post
        lens = (0.45, 0.02, 0.02) if side < 0 else (0.05, 0.05, 0.12)
        for k in range(BEAMS):
            a.cyl((0, y + face * (POST / 2), FIRST + k * PITCH), LENS_R, 0.6 * MM, axis="Y", rgb=lens)
    a.save()


# -- the tag holder (also the tag rack): a block with a slot the tag stands in ----------------------------------------
SLOT_W, SLOT_L, WALL_H, FLOOR = 8 * MM, 90 * MM, 30 * MM, 6 * MM


def holder():
    a = Asset("tag_holder.usda", "TagHolder", "A tag holder bolted to the table: a 100 x 28 mm base with a slot 8 mm wide, "
              "90 mm long, 24 mm deep (flared to 20 mm at the top), open to the sides at its ends by 5 mm end stops. The "
              "red tag (5 mm thick) stands in it. Origin on the table at the slot's centre; slot long side along x.",
              mass=0.5, inertia=(1e-3, 1e-3, 1e-3))
    fr = (0.45, 0.35)
    col = (0.18, 0.2, 0.55)
    a.box((0, 0, FLOOR / 2), (100 * MM, 28 * MM, FLOOR), rgb=col, collide=fr, key="plastic")
    for s in (-1, 1):
        poly = [(s * 4 * MM, FLOOR), (s * 14 * MM, FLOOR), (s * 14 * MM, WALL_H), (s * 10 * MM, WALL_H),
                (s * 4 * MM, 22 * MM)]
        if s < 0:
            poly = poly[::-1]
        a.prism_x(poly, -50 * MM, 50 * MM, col, collide=fr, key="plastic")
    for s in (-1, 1):
        a.box((s * 47.5 * MM, 0, (FLOOR + WALL_H) / 2), (5 * MM, 28 * MM, WALL_H - FLOOR), rgb=col, collide=fr, key="plastic")
    a.save()


# -- the test rod ----------------------------------------------------------------------------------------------------
ROD_L, ROD_R = 0.30, 0.015
ROD_M = 0.11


def rod():
    m = ROD_M
    a = Asset("test_rod.usda", "TestRod", "A 30 mm test rod, 300 mm long: a hollow aluminium tube, 1.5 mm wall, 110 g, "
              "yellow and black bands, a black grip band in the middle (where the centre of mass is). Origin at the middle; "
              "axis along x.", mass=m, inertia=(m * ROD_R ** 2, m * (ROD_L ** 2 / 12 + ROD_R ** 2 / 2),
                                                  m * (ROD_L ** 2 / 12 + ROD_R ** 2 / 2)))
    a.cyl((0, 0, 0), ROD_R, ROD_L, axis="X", collide=(0.5, 0.4), key="alu")
    seg = 0.05
    for i in range(6):
        x = -ROD_L / 2 + seg * (i + 0.5)
        black = i % 2 == 1
        if i in (2, 3):
            rgb = (0.05, 0.05, 0.06)
        else:
            rgb = (0.05, 0.05, 0.06) if black else (0.96, 0.8, 0.05)
        a.cyl((x, 0, 0), ROD_R, seg, axis="X", rgb=rgb)
    a.save()


def saddles():
    a = Asset("rod_saddles.usda", "RodSaddles", "Two saddles for a test rod, 200 mm apart along x (centres at x = -0.10 and "
              "+0.10): a 30 x 60 x 12 mm block with two 6 mm cheeks 34 mm apart that stop a 30 mm rod rolling. The rod's axis "
              "sits 27 mm above the table. Origin on the table midway.", mass=0.4, inertia=(1e-3, 1e-3, 1e-3))
    fr = (0.45, 0.35)
    col = (0.12, 0.12, 0.14)
    for x in (-0.10, 0.10):
        a.box((x, 0, 6 * MM), (30 * MM, 60 * MM, 12 * MM), rgb=col, collide=fr, key="plastic")
        for s in (-1, 1):
            a.box((x, s * 20 * MM, 16 * MM), (30 * MM, 6 * MM, 8 * MM), rgb=col, collide=fr, key="plastic")
    a.save()


TAG_W, TAG_T, TAG_H, TAG_M = 76 * MM, 5 * MM, 127 * MM, 0.046


def tag():
    a = Asset("tag.usda", "Tag", "A 'DO NOT USE' tag, 76 x 127 mm (3 x 5 in), 5 mm HDPE, 46 g, red with a white band; its "
              "thickness is along y. Origin at the middle of its lower edge.", mass=TAG_M, com=(0, 0, TAG_H / 2),
              inertia=box_inertia(TAG_M, TAG_W, TAG_T, TAG_H))
    a.box((0, 0, TAG_H / 2), (TAG_W, TAG_T, TAG_H), rgb=(0.85, 0.05, 0.05), collide=(0.4, 0.3), key="hdpe")
    for s in (-1, 1):
        a.box((0, s * (TAG_T / 2 + 0.1 * MM), TAG_H * 0.62), (TAG_W - 12 * MM, 0.3 * MM, 34 * MM), rgb=(0.97, 0.97, 0.97))
    a.save()


def lamp(fname, name, rgb, doc):
    a = Asset(fname, name, doc, mass=0.001, inertia=(1e-7, 1e-7, 1e-7), no_gravity=True)
    a.cyl((0, 0, 0.3 * MM), 14 * MM, 0.6 * MM, rgb=rgb)
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    curtain()
    holder()
    rod()
    saddles()
    tag()
    lamp("lamp_green.usda", "LampGreen", (0.1, 0.9, 0.2), "The receiver's green lamp (field clear): a 28 mm disc 0.6 mm thick, "
         "visual only (no collider, no gravity). It waits inside the housing; the task puts it on the post's top cap.")
    lamp("lamp_red.usda", "LampRed", (0.95, 0.08, 0.05), "The receiver's red lamp (field interrupted): see the green lamp.")
