"""Writes this task's USD assets with pxr (boxes, capsules; no meshes from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up."""
import math
import numpy as np
from pathlib import Path

import sys
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "submission_tools"))
import meshkit  # noqa: E402

METAL = {}  # rgb -> (metallic, roughness) for the visual materials

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

    def mesh(self, pts, tris, rgb, name="Mesh", offset=(0, 0, 0)):
        """A visual-only closed mesh (no collider)."""
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]) for p in pts]))
        g.CreateFaceVertexCountsAttr([3] * len(tris))
        g.CreateFaceVertexIndicesAttr([int(i) for t in tris for i in t])
        g.CreateSubdivisionSchemeAttr("none")
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.color("m%02d%02d%02d" % tuple(int(c * 99) for c in rgb), rgb))

    def tex_quad(self, center, size, normal_y, png, name="Label"):
        """A visual-only textured quad in the x-z plane facing +y (normal_y > 0) or -y; ``png`` is relative to the asset folder."""
        self.n += 1
        w, h = size[0] / 2, size[1] / 2
        cx, cy, cz = center
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        quad = [(-w, -h), (w, -h), (w, h), (-w, h)]
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(cx + x, cy, cz + z) for x, z in quad]))
        g.CreateFaceVertexCountsAttr([4])
        g.CreateFaceVertexIndicesAttr([0, 1, 2, 3] if normal_y < 0 else [0, 3, 2, 1])
        g.CreateSubdivisionSchemeAttr("none")
        st = UsdGeom.PrimvarsAPI(g).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)] if normal_y < 0 else [Gf.Vec2f(1, 0), Gf.Vec2f(0, 0), Gf.Vec2f(0, 1), Gf.Vec2f(1, 1)]))
        key = "Tex_" + Path(png).stem
        if key not in self.mats:
            mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
            sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/PreviewSurface")
            sh.CreateIdAttr("UsdPreviewSurface")
            tex = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/Tex")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(png))
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/Reader").ConnectableAPI(), "result")
            rd = UsdShade.Shader(self.stage.GetPrimAtPath(f"/{self.name}/Looks/{key}/Reader"))
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
            rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.mats[key] = mat
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.mats[key])

    def root_link(self):
        """Move the root body's physics and shapes into a rigid child link "Base" that carries the articulation root (a fixed
        root needs a rigid-body root link, not the root Xform)."""
        r = self.root.GetPrim()
        root_path = r.GetPath()
        base = UsdGeom.Xform.Define(self.stage, f"{root_path}/Base").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(base)
        m = UsdPhysics.MassAPI.Apply(base)
        src = UsdPhysics.MassAPI(r)
        m.CreateMassAttr(src.GetMassAttr().Get())
        m.CreateCenterOfMassAttr(src.GetCenterOfMassAttr().Get())
        m.CreateDiagonalInertiaAttr(src.GetDiagonalInertiaAttr().Get())
        r.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        r.RemoveAPI(UsdPhysics.MassAPI)
        r.RemoveAPI(UsdPhysics.RigidBodyAPI)
        for a in ("physics:mass", "physics:centerOfMass", "physics:diagonalInertia"):
            if r.HasAttribute(a):
                r.RemoveProperty(a)
        UsdPhysics.ArticulationRootAPI.Apply(base)
        layer = self.stage.GetRootLayer()
        edit = Sdf.BatchNamespaceEdit()
        for ch in r.GetChildren():
            if ch.GetName() in ("Looks", "Joints", "Base") or ch.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            edit.Add(Sdf.NamespaceEdit.Reparent(ch.GetPath(), base.GetPath(), -1))
        assert layer.Apply(edit)
        for j in self.stage.Traverse():
            if j.IsA(UsdPhysics.Joint):
                rel = UsdPhysics.Joint(j).GetBody0Rel()
                if list(rel.GetTargets()) == [root_path]:
                    rel.SetTargets([base.GetPath()])

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path, self.path.stat().st_size, "bytes")


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))
# ======================================================================================================================
# Fuse-puller assets. Metres, kilograms, Z up. A MAXI blade fuse (29.2 x 8.5 mm, typical) and a 100 mm tweezer-type puller.
#   fuse_block.usda : a fuse block lying on the table (one articulation, fixed root): a 52 x 84 x 20 mm base with three fuse positions 28 mm apart
#                     along y: the two outer ones carry fixed fuses (22 mm of body above the base), the middle one is an open pocket
#                     (30.6 x 9.5 x 12 mm deep) with two spring clips (joints ClipP, ClipM: prismatic along y, 1.5 mm of interference, 4 N a side).
#                     A 54 mm square tray (10 mm rim) lies 110 mm to the +y side and 50 mm to the -x side. Origin: table level, middle of the base.
#   fuse.usda       : the middle fuse, a free body: 29.2 x 8.5 x 34 mm (12 mm in the pocket), 8 g. Origin: the middle of its underside.
#   puller.usda     : a tweezer puller (one articulation, free root): a head block and two 3 mm arms 100 mm long, 12 mm tall, hinged about Z at
#                     (2 mm, +-10.5 mm), jaw plates on their ends (28 mm long, inner faces 14 mm apart open); joints ArmP and ArmM, the springs push them to the open stops.
#                     Origin: table level at the hinge line, middle; the jaws point toward +x.
# ======================================================================================================================
BASE_X, BASE_Y, BASE_Z = 0.052, 0.084, 0.020
PITCH = 0.028
FUSE_W, FUSE_T, FUSE_H = 0.0292, 0.0085, 0.034
POCKET_X, POCKET_Y, POCKET_FLOOR = 0.0306, 0.0095, 0.008
FUSE_MASS = 0.008
CLIP_REST, CLIP_K = 0.00275, 2700.0  # the tooth tip at 2.75 mm from the middle; 1.5 mm of interference with the fuse: 4 N
TRAY_IN, TRAY_T, TRAY_RIM = 0.054, 0.003, 0.010
TRAY_DX, TRAY_DY = -0.050, 0.110
ARM_L, ARM_T, ARM_H = 0.100, 0.003, 0.012
ARM_Y, ARM_X0 = 0.0105, 0.002  # the hinge pins
JAW_X0, JAW_X1, JAW_IN = 0.074, 0.102, 0.002  # the jaw plates stand 2 mm inside the arms' inner faces: 14 mm apart open
PULLER_MASS, ARM_MASS = 0.030, 0.006
ARM_K = 3.0  # N m / rad: about 6 N at the pads, preloaded against the open stops
CLOSE_MAX = 5.0  # degrees


# ATO/MAXI blade fuse colour code (ISO 8820-3): 20 A yellow, 30 A green, 50 A red
FUSE_COL = {50: (0.80, 0.08, 0.07), 20: (0.96, 0.78, 0.08), 30: (0.12, 0.62, 0.24)}
STEEL = (0.78, 0.79, 0.81)
METAL[STEEL] = (0.9, 0.3)
BRONZE = (0.72, 0.45, 0.20)
METAL[BRONZE] = (0.85, 0.4)
TRAYSTEEL = (0.74, 0.76, 0.79)
METAL[TRAYSTEEL] = (0.9, 0.32)
TEXDIR = OUT / "textures"


def make_textures():
    import cv2
    TEXDIR.mkdir(exist_ok=True)
    for amp, col in FUSE_COL.items():
        img = np.zeros((128, 256, 3), np.uint8)
        img[:] = [int(c * 255) for c in col[::-1]]
        txt = f"{amp}"
        dark = (20, 20, 20) if amp == 20 else (245, 245, 245)
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_DUPLEX, 3.0, 8)
        cv2.putText(img, txt, (128 - tw // 2 - 20, 64 + th // 2), cv2.FONT_HERSHEY_DUPLEX, 3.0, dark, 8, cv2.LINE_AA)
        cv2.putText(img, "A", (128 + tw // 2 - 10, 64 + th // 2 - 14), cv2.FONT_HERSHEY_DUPLEX, 1.3, dark, 4, cv2.LINE_AA)
        cv2.imwrite(str(TEXDIR / f"fuse_{amp}.png"), img)


def fuse_visual(a, cx, cy, z0, amp, height=FUSE_H):
    """A MAXI blade fuse's looks (visual only): coloured housing with chamfered top corners, a window with the metal link, the amp rating printed on both
    faces. ``z0`` is the housing's underside; the box collider stays as it is."""
    col = FUSE_COL[amp]
    hw, ht, ch = FUSE_W / 2, FUSE_T / 2, 0.004
    prof = [(cx - hw, z0), (cx + hw, z0), (cx + hw, z0 + height - ch), (cx + hw - 0.002, z0 + height), (cx - hw + 0.002, z0 + height), (cx - hw, z0 + height - ch)]
    a.prism_y(prof, cy - ht, cy + ht, col, name="FuseHousing")
    for sgn in (-1, 1):
        a.tex_quad((cx, cy + sgn * (ht + 0.00006), z0 + height - 0.0105), (0.018, 0.009), sgn, f"./textures/fuse_{amp}.png", name="FuseRating")
        a.box((cx, cy + sgn * (ht + 0.00004), z0 + height - 0.0235), (0.017, 0.00008, 0.006), (0.92, 0.92, 0.88), None, name="FuseWindow")
        a.box((cx, cy + sgn * (ht + 0.00008), z0 + height - 0.0235), (0.014, 0.00008, 0.0012), STEEL, None, name="FuseLink")
    a.box((cx, cy, z0 + height + 0.0003), (0.020, FUSE_T * 0.6, 0.0006), tuple(c * 0.6 for c in col), None, name="FuseTop")


def block(fname):
    a = Asset(fname, "Block", "A fuse block lying on the table: a 52 x 84 x 20 mm base with three fuse pockets 28 mm apart along y (each 30.6 x 9.5 x 12 mm deep with two spring "
              "clips, joints Clip<A|B|C><P|M>, prismatic along y; the fuses are separate free bodies) and a 54 mm square tray "
              "110 mm to +y and 50 mm to -x. Fixed root. Origin: table level, middle of the base.", 1.0, com=(0, 0, 0.01), inertia=(1e-3, 1e-3, 1e-3))
    a.articulation_root()
    blk = (0.16, 0.17, 0.20)
    red = (0.85, 0.20, 0.15)
    yel = (0.95, 0.80, 0.15)
    f = (0.35, 0.30)
    hx, hy = BASE_X / 2, BASE_Y / 2
    a.box((0, 0, POCKET_FLOOR / 2), (BASE_X, BASE_Y, POCKET_FLOOR), None, f, key="base", name="Floor")
    zc, zh = (POCKET_FLOOR + BASE_Z) / 2, BASE_Z - POCKET_FLOOR
    py = POCKET_Y / 2
    px = POCKET_X / 2
    ivs = [(-hy, -PITCH - py), (-PITCH + py, -py), (py, PITCH - py), (PITCH + py, hy)]  # the solid strips between the three pockets
    for i, (y0, y1) in enumerate(ivs):
        a.box((0, (y0 + y1) / 2, zc), (BASE_X, y1 - y0, zh), None, f, key="base", name=f"Slab{i}")
    for k in (-1, 0, 1):
        a.box((-(hx + px) / 2, k * PITCH, zc), (hx - px, POCKET_Y, zh), None, f, key="base", name="EndN")
        a.box(((hx + px) / 2, k * PITCH, zc), (hx - px, POCKET_Y, zh), None, f, key="base", name="EndF")
    # (the three fuses are free bodies of their own, each held by two clips in its pocket)
    # the tray
    cx, cy = TRAY_DX, TRAY_DY
    a.box((cx, cy, TRAY_T / 2), (TRAY_IN + 2 * TRAY_T, TRAY_IN + 2 * TRAY_T, TRAY_T), None, f, key="tray", name="TrayFloor")
    for s in (-1, 1):
        a.box((cx + s * (TRAY_IN / 2 + TRAY_T / 2), cy, TRAY_T + TRAY_RIM / 2), (TRAY_T, TRAY_IN + 2 * TRAY_T, TRAY_RIM), None, f, key="tray", name="TrayRimX")
        a.box((cx, cy + s * (TRAY_IN / 2 + TRAY_T / 2), TRAY_T + TRAY_RIM / 2), (TRAY_IN, TRAY_T, TRAY_RIM), None, f, key="tray", name="TrayRimY")
    # looks (visual only; the colliders above are unchanged and invisible)
    dark, edge = (0.11, 0.12, 0.13), 0.003
    def chamfered(x0, x1, y0, y1, cut):
        pts = [(x0 + (cut if x0 == -hx and y0 == -hy else 0), y0), (x1 - (cut if x1 == hx and y0 == -hy else 0), y0), (x1, y0 + (cut if y0 == -hy else 0)),
               (x1, y1 - (cut if y1 == hy else 0)), (x1 - (cut if y1 == hy else 0), y1), (x0 + (cut if y1 == hy else 0), y1), (x0, y1 - (cut if y1 == hy else 0)),
               (x0, y0 + (cut if y0 == -hy else 0))]
        out = []
        for p in pts:
            if not out or abs(p[0] - out[-1][0]) > 1e-9 or abs(p[1] - out[-1][1]) > 1e-9:
                out.append(p)
        return out
    a.box((0, 0, POCKET_FLOOR / 2), (BASE_X, BASE_Y, POCKET_FLOOR), dark, None, name="BaseFloorLook")
    for i, (y0, y1) in enumerate(ivs):
        if i in (0, 3):
            a.prism_z(chamfered(-hx, hx, y0, y1, edge), POCKET_FLOOR, BASE_Z, dark, name="BaseLook")
        else:
            a.box((0, (y0 + y1) / 2, zc), (BASE_X, y1 - y0, zh), dark, None, name="BaseLook")
    for k in (-1, 0, 1):
        a.box((-(hx + px) / 2, k * PITCH, zc), (hx - px, POCKET_Y, zh), dark, None, name="BaseLookN")
        a.box(((hx + px) / 2, k * PITCH, zc), (hx - px, POCKET_Y, zh), dark, None, name="BaseLookF")
    for k in (-1, 1):  # mounting ears with screws
        a.box((0, k * (hy + 0.005), 0.0015), (0.020, 0.010, 0.003), dark, None, name="Ear")
        pts, tris = meshkit.disc(0.0035, 0.003, 0.0042, 48)
        a.mesh(pts, tris, STEEL, "EarScrew", offset=(0, k * (hy + 0.005), 0))
    # tray: pressed stainless steel
    steel = TRAYSTEEL
    a.box((cx, cy, TRAY_T / 2), (TRAY_IN + 2 * TRAY_T, TRAY_IN + 2 * TRAY_T, TRAY_T), steel, None, name="TrayFloorLook")
    for s_ in (-1, 1):
        a.box((cx + s_ * (TRAY_IN / 2 + TRAY_T / 2), cy, TRAY_T + TRAY_RIM / 2), (TRAY_T, TRAY_IN + 2 * TRAY_T, TRAY_RIM), steel, None, name="TrayRimXLook")
        a.box((cx, cy + s_ * (TRAY_IN / 2 + TRAY_T / 2), TRAY_T + TRAY_RIM / 2), (TRAY_IN, TRAY_T, TRAY_RIM), steel, None, name="TrayRimYLook")
        a.box((cx + s_ * (TRAY_IN / 2 + TRAY_T / 2), cy, TRAY_T + TRAY_RIM + 0.0005), (TRAY_T * 1.4, TRAY_IN + 2 * TRAY_T, 0.001), steel, None, name="TrayLipX")
        a.box((cx, cy + s_ * (TRAY_IN / 2 + TRAY_T / 2), TRAY_T + TRAY_RIM + 0.0005), (TRAY_IN, TRAY_T * 1.4, 0.001), steel, None, name="TrayLipY")
    # the clips: a tooth 8 mm wide, 3 mm thick, 8 mm tall at each side of the pocket, tip 2.75 mm from the middle at rest
    for pos, k in (("A", -1), ("B", 0), ("C", 1)):  # A: the 50 A fuse (-y), B: the middle 30 A fuse, C: the 20 A fuse (+y)
        for name, s in ((f"Clip{pos}P", 1), (f"Clip{pos}M", -1)):
            ln = a.link(name, 0.003, com=(0, 0, 0), inertia=(1e-7, 1e-7, 1e-7), pos=(0, k * PITCH + s * (CLIP_REST + 0.0015), 0.014))
            a.box((0, 0, 0), (0.008, 0.003, 0.008), BRONZE, (0.50, 0.45), key="clip", name="Tooth")
            a.prismatic(name, a.root.GetPrim(), ln, axis="Y", lo=0.0 if s > 0 else -0.0035, hi=0.0035 if s > 0 else 0.0, stiffness=CLIP_K, damping=4.0, target=0.0,
                        p0=(0, k * PITCH + s * (CLIP_REST + 0.0015), 0.014), p1=(0, 0, 0))
    a.root_link()
    a.save()


def fuse(fname, amp=30):
    m = FUSE_MASS
    a = Asset(fname, "Fuse", "A MAXI blade fuse, 29.2 x 8.5 mm, 34 mm tall (its lower 12 mm sits in the pocket), 8 g. Origin: the middle of its underside.", m,
              com=(0, 0, FUSE_H / 2), inertia=box_inertia(m, FUSE_W, FUSE_T, FUSE_H))
    a.box((0, 0, FUSE_H / 2), (FUSE_W, FUSE_T, FUSE_H), None, (0.50, 0.45), key="fuse", name="Body")
    fuse_visual(a, 0.0, 0.0, 0.0, amp)
    for sx_ in (-1, 1):  # the two blades under the housing (inside the pocket)
        a.box((sx_ * 0.0085, 0, -0.0005), (0.0065, 0.0008, 0.001), STEEL, None, name="Blade")
    a.save()


def puller(fname):
    m = PULLER_MASS
    a = Asset(fname, "Puller", "A tweezer-type fuse puller: a head block and two 3 mm arms 100 mm long and 12 mm tall hinged about Z at (2 mm, +-10.5 mm), with jaw plates "
              "(28 mm long, inner faces 14 mm apart open); joints ArmP and ArmM, springs to the open stops. Free root. Origin: table level at the hinge line, middle; jaws toward +x.",
              m, com=(0.02, 0, ARM_H / 2), inertia=(2e-5, 2e-5, 2e-5))
    a.articulation_root()
    pl = (0.15, 0.30, 0.65)
    f = (0.60, 0.50)
    pl = (0.82, 0.12, 0.08)  # red moulded plastic
    dk = (0.35, 0.05, 0.04)
    a.box((-0.006, 0, ARM_H / 2), (0.016, 0.034, ARM_H), None, f, key="puller", name="Head")
    hx0, hx1, hy = -0.014, 0.002, 0.017  # the head's looks: chamfered plan, a pivot rivet, a lanyard hole
    a.prism_z([(hx0 + 0.003, -hy), (hx1, -hy), (hx1, hy), (hx0 + 0.003, hy), (hx0, hy - 0.003), (hx0, -hy + 0.003)], 0.0, ARM_H, pl, name="HeadLook")
    pts, tris = meshkit.disc(0.0022, ARM_H, ARM_H + 0.0004, 48)
    for sy_ in (-1, 1):
        a.mesh(pts, tris, STEEL, "Rivet", offset=(ARM_X0, sy_ * ARM_Y, 0))
    for name, s in (("ArmP", 1), ("ArmM", -1)):
        ln = a.link(name, ARM_MASS, com=(0.05, 0, ARM_H / 2), inertia=(2e-7, 2e-6, 2e-6), pos=(ARM_X0, s * ARM_Y, 0.0))
        a.box((ARM_L / 2, 0, ARM_H / 2), (ARM_L, ARM_T, ARM_H), None, f, key="puller", name="Arm")
        a.box((ARM_L / 2, 0, ARM_H / 2), (ARM_L, ARM_T, ARM_H), pl, None, name="ArmLook")
        for i in range(8):  # grip ribs printed flush on the outer face
            a.box((0.020 + i * 0.005, s * (ARM_T / 2 + 0.00004), ARM_H / 2), (0.0022, 0.00008, ARM_H - 0.002), dk, None, name="Rib")
        jx0, jx1 = JAW_X0 - ARM_X0, JAW_X1 - ARM_X0
        # the jaw plate stands inside the arm; its lead-in (a 3 mm flare over 6 mm) at the tip
        yi = -s * ARM_T / 2
        poly = [(jx0, yi), (jx1, yi), (jx1, yi - s * (JAW_IN - 0.0005)), (jx1 - 0.004, yi - s * JAW_IN), (jx0, yi - s * JAW_IN)]
        if s > 0:
            poly = poly[::-1]
        a.prism_z(poly, 0.0, ARM_H, STEEL, (0.90, 0.80), key="jaw", name="Jaw")
        flare = [(jx1, yi - s * (JAW_IN - 0.0005)), (jx1 + 0.0005, yi - s * (JAW_IN - 0.0005)), (jx1 + 0.0005, yi), (jx1, yi)]
        # open stop at 0; the spring (target beyond the stop) pushes the arm open
        tgt = 6.0 if s > 0 else -6.0
        a.revolute(name, a.root.GetPrim(), ln, axis="Z", lo=-CLOSE_MAX if s > 0 else 0.0, hi=0.0 if s > 0 else CLOSE_MAX, stiffness=ARM_K * math.pi / 180.0,
                   damping=0.002 * math.pi / 180.0, target=tgt, p0=(ARM_X0, s * ARM_Y, 0.0), p1=(0, 0, 0), armature=2e-6)
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    make_textures()
    block("fuse_block.usda")
    fuse("fuse.usda")
    fuse("fuse_50.usda", 50)
    fuse("fuse_20.usda", 20)
    puller("puller.usda")
