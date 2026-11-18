"""Writes this task's USD assets with pxr (boxes, cylinders and generated meshes; no outside meshes).

Run:  ../local_sim/.venv/bin/python build_assets.py        (writes ./assets/*.usda)

A real CAT6 RJ45 (8P8C) termination on a ratcheting crimper.  Four USD bodies:
  * crimper.usda  - the fixed, visual-only ratcheting crimp tool: base, plug-seat rail, eight contact teeth,
                    strain-relief jaw, ratchet wheel and handle stops (no colliders).
  * handles.usda  - the two crimper handles plus the ratchet pawl, posed by the task as squeeze closes them.
  * plug.usda     - a real-size modular plug (11.6 x 9.5 x 6.5 mm, 8 contacts on 1.27 mm pitch, pin-1 index notch).
  * cable.usda    - a stripped CAT6 cable (6.2 mm jacket) with eight straightened, colour-coded conductors on the
                    same 1.27 mm pitch and a red index band on wire 1 (orange/white, T568B pin 1).

All sizes are true (not scaled down); sources and the measured/estimated split are in REFERENCE.md.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT = Path(__file__).resolve().parent / "assets"

# -- real product numbers (metres) -------------------------------------------------------------------------------------
PITCH = 0.00102            # 8P8C contact pitch = 0.040 in (1.02 mm), IEC 60603-7 / L-Com drawing
PLUG_L = 0.0116            # plug body length (front-to-back)
PLUG_W = 0.01168           # 8P8C body width (0.460 in, L-Com drawing)
PLUG_H = 0.0066            # 8P8C insertion-area height (0.260 in)
CABLE_OD = 0.0062          # CAT6 UTP jacket outside diameter (0.24 in)
WIRE_OD = 0.00105          # max insulated conductor diameter (0.042 in, L-Com)
STRIP_LEN = 0.0100         # stripped/straightened wire length beyond the jacket
N_WIRES = 8

# -- colours ------------------------------------------------------------------------------------------------------------
BODY_C = (0.18, 0.20, 0.24)     # crimper die-cast body
STEEL = (0.62, 0.65, 0.69)
DARK = (0.09, 0.10, 0.12)
GOLD = (0.86, 0.68, 0.18)
HANDLE_C = (0.12, 0.14, 0.18)
RUBBER = (0.16, 0.34, 0.52)
PLUG_C = (0.88, 0.88, 0.84)
INDEX_RED = (0.85, 0.12, 0.10)
JACKET_C = (0.16, 0.17, 0.19)
# T568B wire order, pin 1 -> 8 (wire 1 = orange/white carries the red index band)
WIRE_COLORS = [
    (0.95, 0.55, 0.10),  # 1 orange/white
    (0.95, 0.45, 0.08),  # 2 orange
    (0.20, 0.70, 0.25),  # 3 green/white
    (0.25, 0.30, 0.80),  # 4 blue
    (0.45, 0.65, 0.95),  # 5 blue/white
    (0.12, 0.55, 0.18),  # 6 green
    (0.55, 0.38, 0.16),  # 7 brown/white
    (0.40, 0.24, 0.10),  # 8 brown
]


class Asset:
    def __init__(self, fname, name, doc, mass=None, kinematic=False, no_gravity=False):
        self.path = OUT / fname
        s = self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(s, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(s, 1.0)
        self.name = name
        root = UsdGeom.Xform.Define(s, f"/{name}")
        s.SetDefaultPrim(root.GetPrim())
        self.root = root.GetPrim()
        self.root.SetMetadata("documentation", doc)
        UsdPhysics.RigidBodyAPI.Apply(self.root)
        if kinematic:
            self.root.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        if no_gravity:
            self.root.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        if mass is not None:
            m = UsdPhysics.MassAPI.Apply(self.root)
            m.CreateMassAttr(mass)
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-6, 1e-6, 1e-6))
        UsdGeom.Scope.Define(s, f"/{name}/Looks")
        self.mats = {}
        self.n = 0

    def color(self, key, rgb, metal=0.0, rough=0.6):
        if key in self.mats:
            return self.mats[key]
        mat = UsdShade.Material.Define(self.stage, f"/{self.name}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"/{self.name}/Looks/{key}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def _finish(self, g, rgb, collide, metal, rough):
        prim = g.GetPrim()
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0005)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
        if rgb is not None:
            mat = self.color("m%02d%02d%02d_%d%d" % (*tuple(int(c * 99) for c in rgb), int(metal * 9), int(rough * 9)),
                             rgb, metal, rough)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def box(self, center, size, rgb, collide=False, metal=0.0, rough=0.6, name=None):
        self.n += 1
        g = UsdGeom.Cube.Define(self.stage, f"/{self.name}/{name or 'Box'}{self.n}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        self._finish(g, rgb, collide, metal, rough)

    def cyl(self, center, radius, height, rgb, collide=False, axis="Z", metal=0.0, rough=0.6, name=None):
        self.n += 1
        g = UsdGeom.Cylinder.Define(self.stage, f"/{self.name}/{name or 'Cyl'}{self.n}")
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr(axis)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        self._finish(g, rgb, collide, metal, rough)

    def save(self):
        OUT.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(self.path))
        print("wrote", self.path.name, self.path.stat().st_size, "bytes")


def _wire_y(i):
    """Transverse position of contact/wire i; pin 1 (i=0) is at +y."""
    return (3.5 - i) * PITCH


def build_crimper():
    """Ratcheting crimp frame: base, plug-seat rail, eight contact teeth, strain-relief jaw, ratchet stops."""
    a = Asset("crimper.usda", "Crimper",
              "ratcheting RJ45 crimper: base, plug seat, eight contact teeth, strain-relief jaw, ratchet stops",
              kinematic=True)
    # bench base 100 x 74 x 8 mm (local origin at the base centre, bench top z 0)
    a.box((0, -0.020, 0.004), (0.110, 0.140, 0.008), BODY_C, metal=0.25, rough=0.55, name="CrimperBase")
    a.box((0, -0.020, 0.0085), (0.112, 0.142, 0.001), DARK, name="CrimperTop")
    # die-cast head that carries the plug seat; seat centre is at local (0.020, 0, 0.018)
    a.box((0.020, 0, 0.012), (0.038, 0.030, 0.008), BODY_C, metal=0.3, rough=0.5, name="CrimperHead")
    # plug-seat rail: a shallow U channel along x, sized to the real plug (9.5 mm wide, 6.5 mm tall)
    a.box((0.020, 0, 0.0125), (0.026, 0.0118, 0.002), DARK, name="SeatFloor")
    for dy in (-1, 1):
        a.box((0.020, dy * (PLUG_W / 2 + 0.0006), 0.016), (0.026, 0.0012, 0.007), STEEL,
              metal=0.7, name="SeatRail")
    # rear stop the plug butts against when fully seated
    a.box((0.033, 0, 0.016), (0.0010, PLUG_W + 0.002, 0.008), STEEL, metal=0.7, name="SeatBackStop")
    # eight contact teeth above the seat at the plug's 1.27 mm pitch (they pierce the wire insulation on crimp)
    for i in range(N_WIRES):
        a.cyl((0.014, _wire_y(i), 0.0230), 0.00042, 0.0055, GOLD, metal=0.85, rough=0.28,
              name=f"ContactTooth{i + 1}")
    # strain-relief jaw behind the plug (local x behind the seat), grips the cable jacket on crimp
    a.box((0.004, 0, 0.016), (0.004, 0.012, 0.006), STEEL, metal=0.6, name="StrainReliefJaw")
    a.box((0.004, 0, 0.0195), (0.004, 0.012, 0.0015), HANDLE_C, name="StrainReliefPad")
    # ratchet wheel / pivot boss (the handles body rotates about this)
    a.cyl((-0.040, -0.070, 0.020), 0.006, 0.010, STEEL, axis="Y", metal=0.7, name="RatchetWheel")
    a.cyl((-0.040, -0.070, 0.020), 0.0024, 0.012, DARK, axis="Y", name="RatchetAxle")
    a.save()


def build_handles():
    """The two crimper handles + ratchet pawl. Root prim is at the hinge pivot, so the task poses it by rotation."""
    a = Asset("handles.usda", "Handles",
              "ratcheting crimper handles: two grips and a pawl; root at the hinge pivot, posed by the task",
              mass=0.120)
    # local frame: pivot at origin. Two handle bars sweep toward -y and +z to the rubber grips.
    for lbl, zoff in (("Lower", -0.004), ("Upper", 0.004)):
        a.box((-0.016, -0.022, 0.011 + zoff), (0.032, 0.046, 0.006), HANDLE_C, metal=0.25, rough=0.55,
              name=f"HandleBar{lbl}")
        a.box((-0.030, -0.040, 0.020 + zoff), (0.014, 0.030, 0.010), RUBBER, rough=0.85,
              name=f"Grip{lbl}")
    # the pawl that catches the ratchet wheel
    a.box((0.0, -0.010, 0.010), (0.004, 0.012, 0.004), GOLD, metal=0.8, name="RatchetPawl")
    a.cyl((0, 0, 0), 0.003, 0.014, STEEL, axis="Y", metal=0.7, name="Hinge")
    a.save()


def build_plug():
    """A real-size 8P8C modular plug: body, latch, 8 front contacts, wire-entry rear and a pin-1 index notch."""
    a = Asset("plug.usda", "Plug",
              "CAT6 RJ45 8P8C plug: 11.6x11.7x6.6 mm, 8 contacts at 1.02 mm pitch, pin-1 index notch",
              mass=0.002, no_gravity=True)
    a.box((0, 0, 0), (PLUG_L, PLUG_W, PLUG_H), PLUG_C, metal=0.05, rough=0.5, name="PlugBody")
    # front (+x) gold contacts, one per position at the real pitch
    for i in range(N_WIRES):
        a.box((PLUG_L / 2 + 0.0002, _wire_y(i), 0.0015), (0.0006, 0.0009, 0.0024), GOLD,
              metal=0.9, rough=0.25, name=f"Pin{i + 1}")
    # locking latch on top near the front
    a.box((PLUG_L / 2 - 0.0025, 0, PLUG_H / 2 + 0.0012), (0.0045, PLUG_W - 0.002, 0.0024), PLUG_C,
          metal=0.05, name="Latch")
    # wire-entry funnel at the rear (-x) with 8 channels
    a.box((-PLUG_L / 2 - 0.0004, 0, 0), (0.0012, PLUG_W - 0.0004, PLUG_H - 0.0012), DARK,
          name="WireEntry")
    for i in range(N_WIRES):
        a.box((-PLUG_L / 2 - 0.0010, _wire_y(i), 0), (0.0014, 0.0006, PLUG_H - 0.0022), DARK,
              name=f"Channel{i + 1}")
    # pin-1 index notch (bright red) on the +y side: wire 1 must align here
    a.box((-PLUG_L / 2 + 0.002, PLUG_W / 2 + 0.0002, 0), (0.0022, 0.0006, 0.0016), INDEX_RED,
          name="Pin1Index")
    a.save()


def build_cable():
    """A stripped CAT6 cable with eight straightened, colour-coded conductors on 1.27 mm pitch and a wire-1 index band."""
    a = Asset("cable.usda", "Cable",
              "stripped CAT6 UTP: 6.2 mm jacket, eight 23 AWG conductors on 1.02 mm pitch, red index on wire 1",
              mass=0.010, no_gravity=True)
    # local frame: +x = the direction the stripped wire tips point; wire tips at x = +STRIP_LEN
    # jacket from x = -0.030 to x = 0
    a.cyl((-0.015, 0, 0), CABLE_OD / 2, 0.030, JACKET_C, axis="X", rough=0.8, name="Jacket")
    a.cyl((-0.0005, 0, 0), CABLE_OD / 2 + 0.0002, 0.0016, DARK, axis="X", name="JacketCut")
    # eight straightened conductors from x = 0 to x = STRIP_LEN, at the real pitch
    for i in range(N_WIRES):
        a.cyl((STRIP_LEN / 2, _wire_y(i), 0), WIRE_OD / 2, STRIP_LEN, WIRE_COLORS[i], axis="X",
              rough=0.6, name=f"Wire{i + 1}")
    # red index band on wire 1 (orange/white) right behind its tip
    a.cyl((STRIP_LEN - 0.0018, _wire_y(0), 0), WIRE_OD / 2 + 0.00025, 0.0016, INDEX_RED, axis="X",
          name="IndexBand")
    a.save()


if __name__ == "__main__":
    build_crimper()
    build_handles()
    build_plug()
    build_cable()
