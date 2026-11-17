"""Writes this task's USD assets with pxr.

Run:  python build_assets.py        (writes ./assets/*.usda)

Frames: metres, kilograms, Z up.  The pan's origin is the centre of its rim plane (the plane that rests on the mat),
so the pan's base face is at local z = 0.064.  The pad holder's origin is the middle of its white pad's contact
face, and the residue cells' origin is the middle of the 80 x 40 mm patch.

Real products (see REFERENCE.md):
  Calphalon Premier Hard-Anodized Nonstick 12-Inch Frying Pan, RBT1392: 30.5 cm across, 1.63 kg.
  Chemtronics Chempad CP400: 76 x 102 mm, 91% IPA.
  Avery 5163 TrueBlock label: a 80 x 40 mm piece is cut from one 2 x 4 in label.
The pad holder is a bench applicator that holds the real Chempad so the parallel-jaw gripper can press its flat
face on the pan.  The pad's 76 x 102 mm contact face, its solvent load and the white-pad check are unchanged; the
holder only replaces the hand's grip (see task.json fidelity).
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"
MM = 0.001
TOP = 0.7475  # the station table top (yam_arena TABLE_TOP["station"])

# -- real dimensions ------------------------------------------------------------------------------------------------
PAN_R, PAN_BASE_R, PAN_H = 0.1525, 0.120, 0.064  # 30.5 cm across, 6.4 cm tall (EST height/base)
PAN_MASS = 1.63  # Calphalon published net weight, 3.6 lb
PAN_HANDLE_L, PAN_HANDLE_W, PAN_HANDLE_T = 0.200, 0.025, 0.015  # handle 20 cm (EST), 25 x 15 mm (EST)
PAD_W, PAD_L, PAD_T = 0.076, 0.102, 0.003  # Chemtronics published face; 3 mm felt (EST)
PAD_MASS = 0.020  # EST; a Chempad plus 1.5 mL IPA
HOLDER_FOOT = (0.080, 0.106, 0.004)  # the applicator's backing plate
HOLDER_HANDLE = (0.030, 0.030, 0.030)  # 30 mm round grip bar
HOLDER_MASS = 0.045  # EST; the holder and its pad

N_X, N_Y, CELL = 8, 16, 0.005  # 40 mm across x, 80 mm along y: 128 cells of 5 mm
PATCH_X, PATCH_Y = N_X * CELL, N_Y * CELL

# -- colours --------------------------------------------------------------------------------------------------------
ANODIZED = (0.050, 0.052, 0.058)
ANODIZED_EDGE = (0.085, 0.087, 0.095)
STEEL = (0.70, 0.71, 0.73)
RIVET = (0.78, 0.79, 0.81)
NYLON = (0.065, 0.067, 0.072)
NYLON_EDGE = (0.23, 0.23, 0.24)
PAD_WHITE = (0.955, 0.950, 0.930)
FOIL = (0.78, 0.79, 0.81)
MAT = (0.090, 0.095, 0.105)
TRAY = (0.56, 0.62, 0.68)
TRAY_IN = (0.68, 0.73, 0.78)
LABEL = (0.965, 0.962, 0.945)
BLUE = (0.20, 0.47, 0.78)
LAMP = (0.20, 0.21, 0.23)
LENS = (0.55, 0.68, 0.80)
RESIDUE = (0.895, 0.875, 0.820)
GRID = (0.12, 0.12, 0.12)


class Asset:
    def __init__(self, fname, name, doc, mass=0.0, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), body=True,
                 no_gravity=False):
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
        self.cur = f"/{name}"

    def link(self, lname, mass, com=(0, 0, 0), inertia=(1e-6, 1e-6, 1e-6), pos=(0, 0, 0), no_gravity=False):
        path = f"/{self.name}/{lname}"
        x = UsdGeom.Xform.Define(self.stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        p = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(p)
        m = UsdPhysics.MassAPI.Apply(p)
        m.CreateMassAttr(mass)
        m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        if no_gravity:
            p.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        self.cur = path
        return p

    def color(self, key, rgb, rough=0.6, metal=0.0, opacity=1.0):
        k = f"m{key}"
        if k in self.mats:
            return self.mats[k]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{k}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{k}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        if opacity < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[k] = mat
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
        """Soft (compliant) contact on a physics material (N/m, N s/m): the felt gives under the press."""
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
            look = self.color("c%02d%02d%02d" % tuple(int(c * 99) for c in rgb), rgb)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)

    def box(self, center, size, rgb=None, collide=None, key="c", name=None, rot=(0, 0, 0)):
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
        self.n += 1
        g = UsdGeom.Capsule.Define(self.stage, f"{self.cur}/{name or 'Capsule'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        self._xf(g, center, *rot)
        self._finish(g, rgb, collide, key)

    def mesh(self, pts, counts, idx, rgb=None, collide=None, key="c", name="Mesh", mesh=True):
        self.n += 1
        g = UsdGeom.Mesh.Define(self.stage, f"{self.cur}/{name}{self.n}")
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        g.CreateFaceVertexCountsAttr([int(c) for c in counts])
        g.CreateFaceVertexIndicesAttr([int(i) for i in idx])
        g.CreateSubdivisionSchemeAttr("none")
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        self._finish(g, rgb, collide, key, mesh=mesh)

    def lathe(self, profile, rgb, n=96, center=(0, 0, 0), name="Lathe"):
        """A surface of revolution about z: (radius, z) pairs, bottom to top.  A radius of 0 closes the end."""
        pts, counts, idx = [], [], []
        cx, cy, cz = center
        for rad, z in profile:
            for k in range(n):
                a = 2 * math.pi * k / n
                pts.append((cx + rad * math.cos(a), cy + rad * math.sin(a), cz + z))
        for i in range(len(profile) - 1):
            for k in range(n):
                a, b = i * n + k, i * n + (k + 1) % n
                counts.append(4)
                idx += [a, b, b + n, a + n]
        for end, flip in ((0, True), (len(profile) - 1, False)):
            if profile[end][0] > 0:
                ring = [end * n + k for k in range(n)]
                counts.append(n)
                idx += ring[::-1] if flip else ring
        self.mesh(pts, counts, idx, rgb=rgb, name=name, mesh=False)

    def prism_y(self, poly_xz, y0, y1, rgb=None, collide=None, key="c", name="Prism"):
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
        self.mesh(pts, counts, faces, rgb=rgb, collide=collide, key=key, name=name)

    def prism_z(self, poly_xy, z0, z1, rgb=None, collide=None, key="c", name="Prism"):
        pts = [(x, y, z0) for (x, y) in poly_xy] + [(x, y, z1) for (x, y) in poly_xy]
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
        self.mesh(pts, counts, faces, rgb=rgb, collide=collide, key=key, name=name)

    def articulation_root(self):
        UsdPhysics.ArticulationRootAPI.Apply(self.root.GetPrim())

    def prismatic(self, name, b0, b1, axis="Z", lo=-0.0003, hi=0.0, stiffness=50.0, damping=1.0, target=0.0,
                  p0=(0, 0, 0), p1=(0, 0, 0)):
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

    def save(self):
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


# -- helpers --------------------------------------------------------------------------------------------------------

def rounded_rect(w, h, r, n=4):
    """A rounded rectangle in x/y, counter-clockwise, approximated with n segments per corner."""
    out = []
    corners = [(w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180),
               (w / 2 - r, -h / 2 + r, 270)]
    for cx, cy, a0 in corners:
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            out.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return out


def rounded_bar(hx, hz, r, n=3):
    """A rounded rectangle in x/z (for the handle cross-section)."""
    return rounded_rect(2 * hx, 2 * hz, r, n)


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


def pan(fname):
    a = Asset(fname, "Pan", "Calphalon Premier hard-anodized 12-inch frying pan, 30.5 cm, 1.63 kg, upside down: dark "
              "hard-anodized exterior, flared rim, stainless handle with two rivets and a hanging hole.  Origin: the "
              "centre of the rim plane; the base face is at z = 0.064.", PAN_MASS, com=(0, 0.004, 0.032),
              inertia=(0.0105, 0.0118, 0.0188))
    a.phys_mat("anodized", 1.2, 1.0)
    a.phys_mat("steel", 0.8, 0.6)
    # outer wall and base top (open at the rim; the rim ring is the contact)
    a.lathe([(PAN_R, 0.0006), (PAN_R + 0.0004, 0.0020), (PAN_R - 0.0012, 0.0042), (0.1470, 0.0090),
             (0.1390, 0.0190), (0.1310, 0.0320), (0.1250, 0.0450), (0.1206, 0.0570), (0.1200, 0.0620),
             (0.1200, 0.0640), (0.0, 0.0640)], ANODIZED, n=96, name="Bowl")
    # a rim lip: a thin ring standing 1.2 mm proud of the wall
    a.lathe([(PAN_R - 0.0018, 0.0004), (PAN_R + 0.0006, 0.0004), (PAN_R + 0.0006, 0.0022),
             (PAN_R - 0.0018, 0.0022), (PAN_R - 0.0018, 0.0004)], ANODIZED_EDGE, n=96, name="RimLip")
    # the base's shallow step ring
    a.lathe([(0.0995, 0.0640), (0.1005, 0.0640), (0.1005, 0.06425), (0.0995, 0.06425), (0.0995, 0.0640)],
            ANODIZED_EDGE, n=96, name="BaseStep")
    # the base's model plate: a small etched rectangle, no text
    a.box((0.072, 0.028, 0.06408), (0.034, 0.012, 0.00016), ANODIZED_EDGE, None, key="edge", name="ModelPlate")
    # the rim ring: 32 small pads carry the pan on the mat, leaving the bowl open underneath
    for k in range(32):
        phi = 360.0 * k / 32
        rr = PAN_R - 0.004
        a.box((rr * math.cos(math.radians(phi)), rr * math.sin(math.radians(phi)), 0.0014),
              (0.026, 0.006, 0.0028), None, (1.2, 1.0), key="anodized", name="RimPad", rot=(0, 0, phi))
    # the base plate collider: the wiped face
    a.cyl((0, 0, 0.0620), PAN_BASE_R, 0.0040, "Z", None, (1.2, 1.0), key="anodized", name="Base")
    # the handle: a rounded stainless strap from the rim to +y, with a slight upward set
    hx, hz = PAN_HANDLE_W / 2, PAN_HANDLE_T / 2
    y0, y1 = PAN_R - 0.002, PAN_R + PAN_HANDLE_L
    prof = []
    for x, z in rounded_bar(hx, hz, 0.0055, n=4):
        prof.append((x, z + 0.0355))
    a.prism_y(prof, y0, y1, STEEL, (0.8, 0.6), key="steel", name="Handle")
    # the mounting collar and its two rivets
    a.box((0, y0 + 0.010, 0.0355), (0.032, 0.022, 0.019), STEEL, None, key="steel", name="Collar")
    for sx in (-1, 1):
        a.cyl((sx * 0.0085, y0 + 0.012, 0.0455), 0.0042, 0.0020, "Z", RIVET, None, key="rivet", name="Rivet")
    # the hanging hole, drawn as a dark disc through the end
    a.cyl((0, y1 - 0.014, 0.0355), 0.0048, 0.0165, "Z", (0.02, 0.02, 0.02), None, key="hole", name="Hole")
    a.save()


def pad_holder(fname):
    a = Asset(fname, "PadHolder", "Bench pad applicator: a 76 x 102 x 3 mm Chemtronics Chempad on an 80 x 106 x 4 mm "
              "backing foot with a 30 x 18 x 30 mm vertical grip bar.  The white pad is the contact face.  Origin: the "
              "middle of the pad's contact face.", HOLDER_MASS, com=(0, 0, 0.012),
              inertia=(5.2e-5, 4.2e-5, 3.1e-5))
    a.phys_mat("felt", 0.55, 0.45)  # wet felt on anodized aluminium: low friction (EST; measure)
    a.set_compliant("felt", 900.0, 30.0)  # a 3 mm felt pad: about 1 mm of give at 0.9 N
    a.phys_mat("plastic", 0.6, 0.5)
    # the white Chempad, 76 x 102 x 3 mm
    a.prism_z(rounded_rect(PAD_W, PAD_L, 0.0040, n=4), 0.0, PAD_T, PAD_WHITE, (1.2, 1.0), key="felt", name="Chempad")
    # a thin blue edge band, the pad's wrapper colour
    a.prism_z(rounded_rect(PAD_W + 0.0016, PAD_L + 0.0016, 0.0048, n=4), 0.0002, 0.0009, BLUE, None,
              key="band", name="PadBand")
    # the backing foot
    a.prism_z(rounded_rect(HOLDER_FOOT[0], HOLDER_FOOT[1], 0.006, n=4), PAD_T, PAD_T + HOLDER_FOOT[2], TRAY,
              (0.6, 0.5), key="plastic", name="Foot")
    a.box((0, 0, PAD_T + HOLDER_FOOT[2] / 2 + 0.0002), (HOLDER_FOOT[0] - 0.004, HOLDER_FOOT[1] - 0.004, 0.0004),
          TRAY_IN, None, key="foot_in", name="FootTop")
    # the grip bar
    a.cyl((0.0, 0.0, PAD_T + HOLDER_FOOT[2] + HOLDER_HANDLE[2] / 2), 0.015, HOLDER_HANDLE[2], "Z",
          TRAY, (0.6, 0.5), key="plastic", name="GripBar")
    for z in (0.018, 0.022, 0.026):
        a.box((0, 0, z), (HOLDER_HANDLE[0] + 0.001, HOLDER_HANDLE[1] + 0.001, 0.0012), TRAY_IN, None,
              key="rib", name="GripRib")
    a.save()


def bench(fname):
    a = Asset(fname, "Bench", "The returns bench fixtures: a 350 x 350 x 3 mm high-friction silicone mat, a 300 x 200 x "
              "20 mm tool/pad tray, a 150 x 100 x 20 mm waste tray with the peeled Avery label, three opened Chempad "
              "foil packets, a 5 mm grid overlay, a 10x loupe and a 30-degree raking LED.  Origin: table top, x/y "
              "world.", 1.0, com=(0, 0, 0.02), inertia=(1, 1, 1))
    a.phys_mat("mat", 1.2, 1.0)
    a.phys_mat("tray", 0.7, 0.6)
    # the mat under the pan
    a.box((0.270, 0.000, 0.0015), (0.310, 0.310, 0.003), MAT, (1.2, 1.0), key="mat", name="Mat")
    # the tool and pad tray
    a.box((0.280, -0.290, 0.010), (0.260, 0.220, 0.020), TRAY, (0.7, 0.6), key="tray", name="ToolTray")
    a.box((0.280, -0.290, 0.0202), (0.252, 0.212, 0.0004), TRAY_IN, None, key="tray_in", name="ToolTrayFloor")
    # three opened foil packets under the pad holders
    for i, x in enumerate((0.180, 0.260, 0.340)):
        a.box((x, -0.300, 0.0203 + 0.00025), (0.090, 0.116, 0.0005), FOIL, None, key="foil", name=f"Foil{i}")
        a.box((x, -0.300, 0.0203 + 0.00055), (0.082, 0.108, 0.0002), (0.92, 0.92, 0.90), None, key="foil_in",
              name=f"FoilInner{i}")
    # the waste tray and the peeled label
    a.box((0.480, -0.220, 0.010), (0.140, 0.120, 0.020), TRAY, (0.7, 0.6), key="tray", name="WasteTray")
    a.box((0.480, -0.220, 0.0202), (0.132, 0.112, 0.0004), TRAY_IN, None, key="tray_in", name="WasteTrayFloor")
    a.box((0.460, -0.180, 0.0207), (0.080, 0.040, 0.0004), LABEL, None, key="label", name="PeeledLabel")
    a.box((0.460, -0.180, 0.02092), (0.080, 0.006, 0.0002), BLUE, None, key="label_blue", name="LabelBlock")
    # a curled corner of the peeled label
    a.box((0.498, -0.162, 0.0211), (0.008, 0.010, 0.0012), LABEL, None, key="label_curl", name="LabelCurl")
    # the 5 mm grid overlay (clear acrylic with dark lines)
    a.box((0.470, 0.330, 0.0010), (0.120, 0.120, 0.0020), (0.72, 0.80, 0.86), None, key="grid_plate", name="GridPlate")
    for k in range(-11, 12, 2):
        a.box((0.470 + k * 0.005, 0.330, 0.0021), (0.0004, 0.120, 0.0002), GRID, None, key="grid", name="GridX")
        a.box((0.470, 0.330 + k * 0.005, 0.0021), (0.120, 0.0004, 0.0002), GRID, None, key="grid", name="GridY")
    # a 10x loupe
    a.cyl((0.520, 0.270, 0.0180), 0.018, 0.030, "Z", LAMP, None, key="lamp", name="LoupeBody")
    a.cyl((0.520, 0.270, 0.0331), 0.014, 0.0010, "Z", LENS, None, key="lens", name="LoupeLens")
    a.cyl((0.520, 0.270, 0.0360), 0.010, 0.0050, "Z", LAMP, None, key="lamp", name="LoupeEye")
    # the raking LED: base, pole, arm, head aimed at the pan
    a.cyl((0.520, -0.060, 0.005), 0.050, 0.010, "Z", LAMP, (0.7, 0.6), key="lamp", name="LampBase")
    a.cyl((0.520, -0.060, 0.180), 0.008, 0.350, "Z", LAMP, (0.7, 0.6), key="lamp", name="LampPole")
    a.box((0.492, -0.045, 0.360), (0.070, 0.016, 0.012), LAMP, None, key="lamp", name="LampArm")
    a.box((0.455, -0.030, 1.000), (0.130, 0.070, 0.030), LAMP, None, key="lamp", name="LampHead",
          rot=(0, 0, 0))
    a.box((0.398, -0.030, 0.995), (0.008, 0.060, 0.020), (0.98, 0.94, 0.82), None, key="led", name="LedFace")
    a.save()


def residue(fname):
    """The 128-cell residue patch, visual only: each 5 mm cell is a thin plate on a stiff slide.  The free root follows
    the pan each control step; a cell slides 0.3 mm into the dark base when it is clean.  No colliders."""
    a = Asset(fname, "Residue", "A declared material-removal layer: 128 cells of 5 mm on the pan's 80 x 40 mm patch, "
              "each a 4.8 x 4.8 x 0.05 mm pale film on a slide.  The free root follows the pan; a clean cell slides "
              "0.3 mm down into the base and disappears.  Visual only, no collider.", 1e-3,
              inertia=(1e-7, 1e-7, 1e-7), no_gravity=True)
    a.articulation_root()
    root = a.root.GetPrim()
    for j in range(N_Y):
        for i in range(N_X):
            n = j * N_X + i
            x = (i + 0.5) * CELL - PATCH_X / 2
            y = (j + 0.5) * CELL - PATCH_Y / 2
            lk = a.link(f"C{n}", 1e-6, inertia=(1e-12, 1e-12, 1e-12), pos=(x, y, 0.00004), no_gravity=True)
            a.box((0, 0, 0), (CELL - 0.0002, CELL - 0.0002, 0.00005), RESIDUE, None, key="residue", name="Cell")
            a.prismatic(f"C{n}", root, lk, axis="Z", lo=-0.0003, hi=0.0, stiffness=80.0, damping=2.0,
                        target=0.0, p0=(x, y, 0.00004), p1=(0, 0, 0))
    a.save()




if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    pan("pan_calphalon.usda")
    pad_holder("pad_holder.usda")
    bench("bench.usda")
    residue("residue.usda")
