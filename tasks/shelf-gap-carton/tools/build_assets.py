"""Writes assets/*.usda (simple primitives, plain text). Run: python build_assets.py

Cartons: origin at the middle of the base, +x depth (front to back), +y width, z up. Shelf: origin at the middle of
the board's top-face footprint on the table, +x toward the back panel, +y the robot's left."""
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "assets"

# name: (file, prim, width y, depth x, height z, mass kg, board colour, print colour, doc)
CARTONS = {
    "tea100": ("carton_tea100", "CartonTea100", 0.080, 0.060, 0.130, 0.24, (0.93, 0.78, 0.15), (0.55, 0.12, 0.10),
               "Tea-bag carton for 100 bags, 80 x 60 x 130 mm (typical value for retail tea cartons), 240 g."),
    "tea40": ("carton_tea40", "CartonTea40", 0.062, 0.070, 0.105, 0.095, (0.15, 0.35, 0.72), (0.95, 0.95, 0.92),
              "Herbal tea carton for 40 bags, 62 x 70 x 105 mm (typical value), 95 g."),
    "soup": ("carton_soup", "CartonSoup", 0.078, 0.052, 0.120, 0.18, (0.78, 0.15, 0.13), (0.96, 0.90, 0.70),
             "Instant soup sachet carton, 78 x 52 x 120 mm (typical value), 180 g."),
    "tablets": ("carton_tablets", "CartonTablets", 0.060, 0.045, 0.110, 0.06, (0.95, 0.95, 0.94), (0.15, 0.55, 0.75),
                "Over-the-counter tablet carton, 60 x 45 x 110 mm (typical value), 60 g."),
    "tea80": ("carton_tea80", "CartonTea80", 0.074, 0.062, 0.125, 0.20, (0.18, 0.60, 0.28), (0.95, 0.95, 0.90),
              "Tea-bag carton for 80 bags, 74 x 62 x 125 mm (typical value), 200 g."),
}

# shelf (metres)
BOARD_Y, BOARD_X, BOARD_T = 0.436, 0.22, 0.018  # 18 mm melamine-faced chipboard board, 400 mm between the dividers
DIV_T, DIV_H = 0.018, 0.14  # 18 mm end dividers, 140 mm tall from the table (122 mm above the board)
PANEL_T, PANEL_H = 0.003, 0.14  # 3 mm hardboard back panel


def mat(name, rgb, rough=0.7, fric=None):
    return f'''        def Material "{name}"
        {{
            token outputs:surface.connect = </ROOT/Looks/{name}/PreviewSurface.outputs:surface>

            def Shader "PreviewSurface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = ({rgb[0]}, {rgb[1]}, {rgb[2]})
                float inputs:metallic = 0
                float inputs:roughness = {rough}
                token outputs:surface
            }}
        }}
'''


def phys_mat(name, static, dynamic):
    return f'''        def Material "{name}" (
            prepend apiSchemas = ["PhysicsMaterialAPI"]
        )
        {{
            float physics:dynamicFriction = {dynamic}
            float physics:restitution = 0
            float physics:staticFriction = {static}
        }}
'''


def vis_cube(name, mat_name, size, centre):
    return f'''    def Cube "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        rel material:binding = </ROOT/Looks/{mat_name}>
        double size = 1
        float3 xformOp:scale = ({size[0]}, {size[1]}, {size[2]})
        double3 xformOp:translate = ({centre[0]}, {centre[1]}, {centre[2]})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
'''


def col_cube(name, pm, size, centre):
    return f'''        def Cube "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = </ROOT/Looks/{pm}> (
                bindMaterialAs = "weakerThanDescendants"
            )
            custom float physxCollision:contactOffset = 0.002
            custom float physxCollision:restOffset = 0
            uniform token purpose = "guide"
            double size = 1
            float3 xformOp:scale = ({size[0]}, {size[1]}, {size[2]})
            double3 xformOp:translate = ({centre[0]}, {centre[1]}, {centre[2]})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}
'''


def doc(root, text, body, mass=None, kinematic=False):
    ms = f"\n    float physics:mass = {mass}\n" if mass else ""
    kin = "\n    bool physics:kinematicEnabled = true" if kinematic else ""
    api = '["PhysicsRigidBodyAPI", "PhysicsMassAPI"]'
    return f'''#usda 1.0
(
    defaultPrim = "ROOT"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "ROOT" (
    prepend apiSchemas = {api}
    doc = "{text}"
)
{{{kin}{ms}
{body}}}
'''


def carton(key):
    f, prim, w, d, h, m, col, pr, text = CARTONS[key]
    looks = '    def Scope "Looks"\n    {\n' + mat("Board", col, 0.8) + mat("Print", pr, 0.6) + \
        mat("Edge", tuple(c * 0.85 for c in col), 0.8) + phys_mat("Phys_Board", 0.55, 0.40) + "    }\n\n"
    eps = 0.0004
    body = looks
    body += vis_cube("Body", "Board", (d, w, h), (0, 0, h / 2))
    # printed panel on the front face (toward -x) and a band near the top, thin visual-only slabs
    body += vis_cube("FrontPrint", "Print", (eps * 2, w * 0.72, h * 0.34), (-d / 2 - eps, 0, h * 0.62))
    body += vis_cube("TopBand", "Edge", (d + eps * 2, w + eps * 2, 0.010), (0, 0, h - 0.005))
    body += '\n    def Scope "Colliders"\n    {\n' + col_cube("Box", "Phys_Board", (d, w, h), (0, 0, h / 2)) + "    }\n"
    (OUT / f"{f}.usda").write_text(doc("ROOT", text, body, mass=m))


def shelf():
    looks = '    def Scope "Looks"\n    {\n' + mat("Melamine", (0.80, 0.68, 0.50), 0.5) + \
        mat("Panel", (0.62, 0.50, 0.36), 0.8) + phys_mat("Phys_Melamine", 0.5, 0.4) + "    }\n\n"
    zb = BOARD_T / 2
    yd = BOARD_Y / 2 - DIV_T / 2
    # the board's top face is at z = BOARD_T; the dividers stand beside it; the cartons' zone is 400 mm wide
    body = looks
    body += vis_cube("Board", "Melamine", (BOARD_X, BOARD_Y, BOARD_T), (0, 0, zb))
    body += vis_cube("DividerL", "Melamine", (BOARD_X, DIV_T, DIV_H), (0, yd, DIV_H / 2))
    body += vis_cube("DividerR", "Melamine", (BOARD_X, DIV_T, DIV_H), (0, -yd, DIV_H / 2))
    body += vis_cube("Back", "Panel", (PANEL_T, BOARD_Y, PANEL_H), (BOARD_X / 2 + PANEL_T / 2, 0, PANEL_H / 2))
    body += '\n    def Scope "Colliders"\n    {\n'
    body += col_cube("Board", "Phys_Melamine", (BOARD_X, BOARD_Y, BOARD_T), (0, 0, zb))
    body += col_cube("DividerL", "Phys_Melamine", (BOARD_X, DIV_T, DIV_H), (0, yd, DIV_H / 2))
    body += col_cube("DividerR", "Phys_Melamine", (BOARD_X, DIV_T, DIV_H), (0, -yd, DIV_H / 2))
    body += col_cube("Back", "Phys_Melamine", (PANEL_T, BOARD_Y, PANEL_H),
                     (BOARD_X / 2 + PANEL_T / 2, 0, PANEL_H / 2))
    body += "    }\n"
    text = ("Open shelf bay: 18 mm melamine-faced chipboard board 436 x 220 mm, two 18 mm end dividers 140 mm tall "
            "(400 mm clear between them), 3 mm hardboard back panel. Fixed. Origin: middle of the board on the table.")
    (OUT / "shelf_bay.usda").write_text(doc("ROOT", text, body, mass=6.0, kinematic=True))


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for k in CARTONS:
        carton(k)
    shelf()
    print("ok")
