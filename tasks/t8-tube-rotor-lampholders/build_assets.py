"""Writes this task's USD assets with pxr (boxes, cylinders, capsules; nothing from outside).
Run:  python build_assets.py        (writes ./assets/*.usda)
Frames: every asset's origin is on the table (z = 0 at its underside) unless its doc says otherwise; metres, kg, Z up."""
import math
import numpy as np
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
        met, rgh = METALS.get(tuple(rgb), (0.0, rough))
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
# T8 tube / rotor lampholder task assets. Metres, kilograms, Z up.
#   batten.usda  : a fixed 600 mm T8 batten on a back board: two G13 rotor lampholders (a housing with a round pocket, a
#                  front entry notch with a lead-in, and a rotor with a diametral slot on a revolute joint about y). An
#                  articulation with a fixed root. Origin: table level, on the tube's axis (x), y 0 midway between the holders.
#   t8_tube.usda : a T8 fluorescent tube, 26 mm x 590 mm cap to cap, G13 bi-pins (2.47 mm, 12.7 mm apart, 7.5 mm out of each
#                  cap), 0.13 kg: an articulation (free root) with four visual strips that show when the tube is lit.
#   cradles.usda : two fixed bench trays (with end stops) that the tube lies on before it is fitted.
# ======================================================================================================================
AX = 0.115  # the tube's axis above the table
TUBE_D, TUBE_L, TUBE_MASS = 26.0 * MM, 590.0 * MM, 0.13
PIN_R, PIN_SP, PIN_OUT = 1.235 * MM, 12.7 * MM, 7.5 * MM  # G13: pin 2.47 mm, 12.7 mm apart, projecting about 7.5 mm
HALF_GAP = 298.0 * MM  # a housing's inner face is this far from the middle (caps clear by 3 mm each)
HOUSE_T = 14.0 * MM  # housing thickness along y
SLOT_W = 3.2 * MM  # the rotor's slot (pin 2.47 mm)
NOTCH_W = 3.4 * MM  # the housing's entry notch
GROOVE = 7.0 * MM  # slot depth
ROTOR_R, POCKET_R = 9.4 * MM, 10.0 * MM
PAN_X = TUBE_D / 2 + 40.0 * MM  # the pan's face, behind the tube's axis
HOUSE_FRONT, HOUSE_H = -24.0 * MM, 24.0 * MM  # housing from 24 mm in front of the axis; half height
ROTOR_MASS = 0.020
IVORY, DARK, WOOD, PAN, GLASS, METAL, BRIGHT = (0.90, 0.88, 0.80), (0.12, 0.12, 0.14), (0.62, 0.50, 0.34), (0.93, 0.93, 0.93), (0.88, 0.90, 0.93), (0.70, 0.70, 0.72), (1.0, 0.78, 0.05)
PHOS, ALU, BRASS = (0.97, 0.97, 0.94), (0.78, 0.79, 0.80), (0.82, 0.63, 0.25)  # tube: white phosphor glass, aluminium caps, brass pins
HOUSE_W, ROTOR_C, PANW, BOARD, SCREW = (0.95, 0.95, 0.93), (0.80, 0.79, 0.74), (0.96, 0.96, 0.95), (0.86, 0.82, 0.74), (0.66, 0.67, 0.69)
LABEL, LABEL2 = (0.80, 0.86, 0.94), (0.10, 0.25, 0.55)
METALS = {ALU: (0.85, 0.30), BRASS: (0.90, 0.30), SCREW: (0.85, 0.35), PHOS: (0.0, 0.22), PANW: (0.0, 0.40)}  # (metallic, roughness) by colour
PL = (0.30, 0.25)  # plastic friction (PA/PBT on brass: about 0.2-0.3)
D_ROTOR = 0.0004  # N m s per degree of rotor drag


def batten(fname):
    a = Asset(fname, "Batten", "A fixed 600 mm T8 batten on a back board with two G13 rotor lampholders. Each holder: a housing with a round "
              "10 mm pocket, an entry notch from the front with a lead-in, and a rotor (9.4 mm) with a 3.2 mm diametral slot on a revolute joint about y. "
              "The notch lines up with the slot at 0 degrees; at -90 degrees the pins are behind the pocket wall. Origin: table level on the tube's axis, y 0 midway.",
              2.0, com=(0.03, 0, 0.1), inertia=(1e-1, 1e-1, 1e-1))
    a.articulation_root()
    f = PL
    a.box((PAN_X + 0.0015, 0, AX), (0.003, 0.680, 0.080), PANW, f, key="pan", name="Pan")
    a.box((PAN_X + 0.003 + 0.009, 0, 0.095), (0.018, 0.760, 0.190), BOARD, f, key="wood", name="Board")
    root = a.base_prim
    for s, nm in ((1.0, "RotL"), (-1.0, "RotR")):
        yf = s * HALF_GAP
        yc = lambda d: yf + s * d
        # housing: a visual block, colliders only where the pins and the rotor are (pocket ring, notch walls, lead-in, back plate)
        hx0, hx1 = HOUSE_FRONT, PAN_X
        a.box(((hx0 + hx1) / 2, yc(HOUSE_T / 2), AX), (hx1 - hx0, HOUSE_T, 2 * HOUSE_H), HOUSE_W, None, name="Housing" + nm)
        n = 16
        for k in range(n):
            phi = 360.0 * k / n
            if 146.0 < phi < 214.0:  # the front three segments are the notch
                continue
            ph = math.radians(phi)
            th = 5.0 * MM
            rc = POCKET_R + th / 2
            w = 2 * (POCKET_R + th) * math.tan(math.pi / n) * 1.03
            # box in the x-z plane: radial size th along its x, tangential w along its z; rotate about y
            a.box((rc * math.cos(ph), yc((GROOVE + 1.5 * MM) / 2), AX - rc * math.sin(ph)), (th, GROOVE + 1.5 * MM, w), HOUSE_W, f, key="plastic", name="Ring" + nm, rot=(0, phi, 0))
        # the entry notch's lead-in: one plate each side, at 30 degrees from the pocket's front edge (-10 mm) out to -17 mm, opening from 3.4 to 13.4 mm
        for sz in (1.0, -1.0):
            zi = NOTCH_W / 2
            dx, dz = 8.66, 5.0
            ang = math.degrees(math.atan2(dz, dx))
            L = math.hypot(dx, dz) * MM
            nrm = (dz / math.hypot(dx, dz), dx / math.hypot(dx, dz))  # outward normal (x, z) for the upper plate
            tcn = 1.25 * MM
            mx, mz = -(POCKET_R + 4.33 * MM), zi + 2.5 * MM
            a.box((mx, yc((GROOVE + 1.5 * MM) / 2), AX + sz * (mz + tcn * nrm[1])), (L, GROOVE + 1.5 * MM, 2.5 * MM), HOUSE_W, f, key="plastic", name="Lead" + nm, rot=(0, sz * ang, 0))
        a.box((0, yc(GROOVE + 1.5 * MM + 1.0 * MM), AX), (24 * MM, 2.0 * MM, 24 * MM), HOUSE_W, f, key="plastic", name="Back" + nm)
        # the rotor
        lk = a.link(nm, ROTOR_MASS, com=(0, s * 3 * MM, 0), inertia=(1.5e-6, 1.0e-6, 1.5e-6), pos=(0.0, yf, AX))
        zi = SLOT_W / 2
        for sz in (1.0, -1.0):
            a.box((0, s * GROOVE / 2, sz * (zi + 0.85 * MM)), (17.6 * MM, GROOVE, 1.7 * MM), ROTOR_C, f, key="plastic", name="Wall" + nm)
        a.cyl((0, s * (GROOVE + 0.75 * MM), 0), ROTOR_R, 1.5 * MM, "Y", ROTOR_C, f, key="plastic", name="Floor" + nm)
        a.cur = a.base_prim.GetPath().pathString
        a.revolute(nm, root, lk, "Y", lo=-93.0, hi=3.0, stiffness=0.1 / 45.0, damping=D_ROTOR, target=0.0, p0=(0.0, yf, AX), p1=(0, 0, 0), armature=3e-5)
    a.cur = a.base_prim.GetPath().pathString
    for yy in (0.250, -0.250, 0.0):  # visual only: pan screws
        for zz in (0.025, -0.025):
            a.cyl((PAN_X - 1.5 * MM - 0.5 * MM, yy, AX + zz), 3.0 * MM, 1.5 * MM, "X", SCREW, None, name="Screw")
    for zz in (0.0405, -0.0405):  # visual only: the pan's folded edges (thin lips toward the front)
        a.box((PAN_X - 4.0 * MM, 0, AX + zz), (8.0 * MM, 0.680, 1.2 * MM), PANW, None, name="Lip")
    a.save()


def _slide(a, name, b0, b1, axis, lo, hi, p0):
    a.prismatic(name, b0, b1, axis=axis, lo=lo, hi=hi, stiffness=20.0, damping=0.3, target=0.0, p0=p0, p1=(0, 0, 0))


def tube(fname):
    m = TUBE_MASS
    ro = TUBE_D / 2
    ia = m * ro * ro
    it = m * (3 * ro * ro + TUBE_L ** 2) / 12
    a = Asset(fname, "T8Tube", "A T8 fluorescent tube, 26 mm x 590 mm cap to cap, G13 bi-pins (2.47 mm, 12.7 mm apart, 7.5 mm out of each end), 0.13 kg. "
              "Free root; four visual strips on prismatic joints show when it is lit. Origin: the tube's middle, y along the tube, pins along x at 0 degrees.",
              m - 4e-4, inertia=(it, ia, it))
    a.articulation_root()
    f = (0.60, 0.50)
    a.cyl((0, 0, 0), ro, TUBE_L - 2 * 6.0 * MM, "Y", PHOS, f, key="glass", name="Body")
    for s in (1.0, -1.0):
        a.cyl((0, s * (TUBE_L / 2 - 6.0 * MM), 0), ro - 0.3 * MM, 12.0 * MM, "Y", ALU, f, key="glass", name="Cap")
        for sx in (1.0, -1.0):
            a.capsule((sx * PIN_SP / 2, s * (TUBE_L / 2 - 1.0 * MM + (PIN_OUT + 1.0 * MM) / 2), 0), PIN_R, PIN_OUT + 1.0 * MM - 2 * PIN_R, "Y", BRASS, (0.30, 0.25), key="brass", name="Pin")
    for s in (1.0, -1.0):  # visual only: dark groove ring where cap meets glass
        a.cyl((0, s * (TUBE_L / 2 - 12.5 * MM), 0), ro + 0.05 * MM, 1.2 * MM, "Y", (0.30, 0.31, 0.33), None, name="CapRing")
    a.cyl((0, -0.13, 0), ro + 0.15 * MM, 0.16, "Y", LABEL, None, name="Label")  # printed rating label band
    for yy in (-0.13 + 0.078, -0.13 - 0.078):
        a.cyl((0, yy, 0), ro + 0.2 * MM, 3.0 * MM, "Y", LABEL2, None, name="LabelRing")
    root = a.base_prim
    h = 0.0035
    for i, (axis, lo, hi, c, sz) in enumerate((("Z", 0.0, h, (0, 0, 0.0110), (8 * MM, 0.56, 1.6 * MM)), ("Z", -h, 0.0, (0, 0, -0.0110), (8 * MM, 0.56, 1.6 * MM)),
                                                ("X", 0.0, h, (0.0110, 0, 0), (1.6 * MM, 0.56, 8 * MM)), ("X", -h, 0.0, (-0.0110, 0, 0), (1.6 * MM, 0.56, 8 * MM)))):
        lk = a.link(f"Glow{i}", 1e-4, com=(0, 0, 0), inertia=(1e-8, 1e-8, 1e-8), pos=c)
        a.box((0, 0, 0), sz, BRIGHT, None, name="Strip")
        a.cur = a.base_prim.GetPath().pathString
        _slide(a, f"Glow{i}", root, lk, axis, lo, hi, c)
    a.save()


def cradles(fname):
    a = Asset(fname, "Cradles", "Two fixed bench cradles (y +-0.10 m): each a flat tray 40 mm wide and 20 mm long between two 8 mm side walls, on which the tube lies with its axis at 0.115 m, and two end stops 1.5 mm outside its caps. "
              "Origin: table level at the tube's axis x, y 0.", 0.5, com=(0, 0, 0.05), inertia=(1e-3, 1e-3, 1e-3))
    f = (0.45, 0.40)
    fl = AX - TUBE_D / 2  # the tube's underside
    for yc in (0.10, -0.10):  # a flat tray 40 mm wide between two 8 mm side walls reaching the tube's axis height
        a.box((0, yc, fl / 2), (0.056, 20.0 * MM, fl), (0.55, 0.40, 0.25), f, key="wood", name="Tray")
        for sx in (1.0, -1.0):
            a.box((sx * 24.0 * MM, yc, AX / 2), (8.0 * MM, 20.0 * MM, AX), (0.55, 0.40, 0.25), f, key="wood", name="Side")
    for sy in (1.0, -1.0):  # end stops 1.5 mm outside the tube's caps, below the pins' height (the pins pass over them)
        a.box((0, sy * (TUBE_L / 2 + 1.5 * MM + 3.0 * MM), (AX - 7.0 * MM) / 2), (40.0 * MM, 6.0 * MM, AX - 7.0 * MM), (0.55, 0.40, 0.25), f, key="wood", name="Stop")
    a.save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    batten("batten.usda")
    tube("t8_tube.usda")
    cradles("cradles.usda")
