from meshkit_inline import _orient, _weld, check_closed, lathe  # noqa
# ---- meshkit2 (2026-11-15): more closed visual-mesh helpers (rounded boxes, lofts, star-shaped lathes, ear-clipped extrusions,
# helical threads, knurls). Needs _orient, _weld, check_closed, lathe from meshkit.py (the task build scripts embed a copy of both).
# Every function returns (pts, tris): a closed, outward-wound mesh (check_closed True).
import math
import numpy as np


def rr_loop(w, h, r, nc=8):
    """Rounded rectangle (width w along u, height h along v, corner radius r), counter-clockwise, 4*(nc+1) points."""
    r = max(min(r, w / 2 - 1e-9, h / 2 - 1e-9), 1e-6)
    out = []
    for (cx, cy, a0) in ((w / 2 - r, h / 2 - r, 0.0), (-w / 2 + r, h / 2 - r, 90.0), (-w / 2 + r, -h / 2 + r, 180.0), (w / 2 - r, -h / 2 + r, 270.0)):
        for i in range(nc + 1):
            a = math.radians(a0 + 90.0 * i / nc)
            out.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return out


def loft(loops, cap0=True, cap1=True):
    """Join equal-length 3D loops (consecutive ones lie in order along an axis) with quads; the end loops are capped by a centroid fan."""
    L = [np.asarray(l, float) for l in loops]
    n = len(L[0])
    pts = [tuple(p) for l in L for p in l]
    tris = []
    for k in range(len(L) - 1):
        for i in range(n):
            j = (i + 1) % n
            a, b, c, d = k * n + i, k * n + j, (k + 1) * n + j, (k + 1) * n + i
            tris += [(a, b, c), (a, c, d)]
    for flag, k, rev in ((cap0, 0, True), (cap1, len(L) - 1, False)):
        if flag:
            c = L[k].mean(axis=0)
            pts.append(tuple(c))
            ci = len(pts) - 1
            for i in range(n):
                a, b = k * n + i, k * n + (i + 1) % n
                tris.append((b, a, ci) if rev else (a, b, ci))
    return pts, _orient(pts, tris)


def rounded_box(sx, sy, sz, r, nc=6, centre=(0, 0, 0), steps=5):
    """A box sx x sy x sz with all edges rounded (radius r): built as a stack of rounded-rectangle loops (fillet top and bottom)."""
    return fillet_slab(lambda d: rr_loop(max(sx - 2 * d, 1e-6), max(sy - 2 * d, 1e-6), max(r - d, 1e-6), nc), -sz / 2, sz / 2, r, steps, centre)


def fillet_slab(outline, z0, z1, f, steps=4, centre=(0, 0, 0), f_top=None, f_bot=None):
    """Extrude the outline function ``outline(inset) -> loop2d`` from z0 to z1 with a fillet of radius f_bot at the bottom edge and f_top at
    the top edge (default f). The outline is used at inset 0 on the straight wall and at the inset the fillet circle needs elsewhere."""
    f_top = f if f_top is None else f_top
    f_bot = f if f_bot is None else f_bot
    seq = []
    for k in range(steps + 1):  # bottom fillet: from the face (inset f) up to the wall (inset 0)
        ph = math.pi / 2 * k / steps
        seq.append((f_bot * (1 - math.sin(ph)), z0 + f_bot * (1 - math.cos(ph))))
    for k in range(steps + 1):
        ph = math.pi / 2 * k / steps
        seq.append((f_top * (1 - math.cos(ph)), z1 - f_top * (1 - math.sin(ph))))
    loops = [[(x + centre[0], y + centre[1], z + centre[2]) for (x, y) in outline(d)] for d, z in seq]
    return loft(loops)


def poly_area(p):
    a = 0.0
    for i in range(len(p)):
        x0, y0 = p[i]
        x1, y1 = p[(i + 1) % len(p)]
        a += x0 * y1 - x1 * y0
    return a / 2


def triangulate(poly):
    """Ear clipping of a simple polygon (list of (x, y)); returns index triples, counter-clockwise."""
    idx = list(range(len(poly)))
    if poly_area(poly) < 0:
        idx.reverse()
    out = []

    def inside(p, a, b, c):
        def s(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
        d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
        return not ((d1 < -1e-15 or d2 < -1e-15 or d3 < -1e-15) and (d1 > 1e-15 or d2 > 1e-15 or d3 > 1e-15))

    guard = 0
    while len(idx) > 3 and guard < 100000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-15:
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2) and poly[j] not in (a, b, c)):
                continue
            out.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            idx.pop(0)  # degenerate sliver: drop a vertex
    if len(idx) == 3:
        out.append(tuple(idx))
    return out


def extrude(poly, z0, z1):
    """A simple polygon (any winding, may be concave) extruded between z0 and z1: closed."""
    p = list(poly)
    if poly_area(p) < 0:
        p.reverse()
    k = len(p)
    pts = [(x, y, z0) for x, y in p] + [(x, y, z1) for x, y in p]
    tris = []
    for (a, b, c) in triangulate(p):
        tris.append((a, c, b))  # bottom faces down
        tris.append((k + a, k + b, k + c))
    for i in range(k):
        j = (i + 1) % k
        tris += [(i, j, k + j), (i, k + j, k + i)]
    return pts, _orient(pts, tris)


def ray_radius(poly, ang):
    """Distance from the origin to a polygon boundary along direction ang (the polygon must be star-shaped about the origin)."""
    dx, dy = math.cos(ang), math.sin(ang)
    best = None
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        if abs(den) < 1e-14:
            continue
        t = (x0 * ey - y0 * ex) / den
        s = (x0 * dy - y0 * dx) / den
        if t > 1e-12 and -1e-9 <= s <= 1 + 1e-9 and (best is None or t > best):
            best = t
    return best


def lathe_var(profile, outline, n=96, centre=(0.0, 0.0)):
    """Like lathe, but radius = a + b*R(theta): profile points are (a, b, z), R(theta) the distance to ``outline`` (a star-shaped polygon
    about the origin) -- use for plates with a round hole and an odd outer outline, with chamfers. Returns (pts, tris), welded."""
    R = [ray_radius(outline, 2 * math.pi * i / n) for i in range(n)]
    k = len(profile)
    pts = []
    for i in range(n):
        c, s = math.cos(2 * math.pi * i / n), math.sin(2 * math.pi * i / n)
        for (a, b, z) in profile:
            r = a + b * R[i]
            pts.append((centre[0] + r * c, centre[1] + r * s, z))
    tris = []
    for i in range(n):
        i2 = (i + 1) % n
        for j in range(k):
            j2 = (j + 1) % k
            a_, b_, c_, d_ = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            tris += [(a_, b_, c_), (a_, c_, d_)]
    p, t = _weld(pts, tris)
    return p, _orient(p, t)


def stadium_poly(r, x_back, n=48, tip_dir=-1):
    """Outline: a round end of radius r about the origin facing ``tip_dir`` (x), straight sides back to x = x_back (a square back end)."""
    pts = []
    for i in range(n + 1):
        a = math.pi / 2 + math.pi * i / n if tip_dir < 0 else -math.pi / 2 + math.pi * i / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    pts.append((x_back, pts[-1][1]))
    pts.append((x_back, pts[0][1]))
    return pts


def thread_rod(r_major, r_minor, z0, z1, pitch, n=64, ramp=1.2e-3, tip=None):
    """A closed cylinder rod with a helical thread (right-handed screw surface, radius r_minor..r_major) between z0 and z1; the thread fades
    out over ``ramp`` at both ends (plain core). ``tip`` = (length, radius) cones the z0 end to that radius."""
    m = max(int((z1 - z0) / (pitch / 7.0)), 12)
    zs = [z0 + (z1 - z0) * j / m for j in range(m + 1)]
    pts = []
    for z in zs:
        env = min(1.0, (z - z0) / ramp, (z1 - z) / ramp) if ramp else 1.0
        env = max(0.0, env)
        for i in range(n):
            th = 2 * math.pi * i / n
            ph = ((z - pitch * th / (2 * math.pi)) / pitch) % 1.0
            tooth = 1.0 - abs(2 * ph - 1.0)  # triangular 0..1..0
            r = r_minor + (r_major - r_minor) * (0.15 + 0.85 * tooth) * env if env > 0 else r_minor
            if tip and z - z0 < tip[0]:
                r = min(r, tip[1] + (r_minor - tip[1]) * (z - z0) / tip[0])
            pts.append((r * math.cos(th), r * math.sin(th), z))
    tris = []
    for j in range(m):
        for i in range(n):
            i2 = (i + 1) % n
            a, b, c, d = j * n + i, j * n + i2, (j + 1) * n + i2, (j + 1) * n + i
            tris += [(a, b, c), (a, c, d)]
    for j, flag in ((0, 0), (m, 1)):  # end caps (centre points)
        r_end = math.hypot(pts[j * n][0], pts[j * n][1])
        pts.append((0.0, 0.0, zs[j]))
        ci = len(pts) - 1
        for i in range(n):
            i2 = (i + 1) % n
            tris.append((j * n + i2, j * n + i, ci) if j == 0 else (j * n + i, j * n + i2, ci))
    return pts, _orient(pts, tris)


def knurl_outline(r_out, depth, teeth, flat_half=None, n_per=6):
    """Outline of a knurled disc (triangular teeth of height ``depth`` below r_out), optionally cut by two flats at y = +-flat_half. CCW."""
    N = teeth * n_per
    p = []
    for i in range(N):
        th = 2 * math.pi * i / N
        ph = (i % n_per) / n_per
        tri = 1.0 - abs(2 * ph - 1.0)
        r = r_out - depth * (1.0 - tri)
        p.append((r * math.cos(th), r * math.sin(th)))
    if flat_half is None:
        return p
    q = [(x, max(-flat_half, min(flat_half, y))) for x, y in p]
    on = [abs(abs(y) - flat_half) < 1e-12 for x, y in q]
    out = []
    for i in range(len(q)):  # a run of points on a flat: keep only its two ends
        if on[i] and on[i - 1] and on[(i + 1) % len(q)] and (q[i][1] == q[i - 1][1] == q[(i + 1) % len(q)][1]):
            continue
        out.append(q[i])
    return out


def xform(mesh, R=None, t=(0, 0, 0)):
    """Apply a 3x3 rotation R (rows) and translation t to a mesh; keeps it outward-wound (R must be a proper rotation)."""
    pts, tris = mesh
    P = np.asarray(pts, float)
    if R is not None:
        P = P @ np.asarray(R, float).T
    P = P + np.asarray(t, float)
    pts = [tuple(p) for p in P]
    return pts, _orient(pts, tris)  # (re-oriented: R may be a reflection or a permutation)


def merge(*meshes):
    pts, tris = [], []
    for p, t in meshes:
        o = len(pts)
        pts += list(p)
        tris += [(a + o, b + o, c + o) for a, b, c in t]
    return pts, tris
