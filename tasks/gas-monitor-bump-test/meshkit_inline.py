"""Shared visual-mesh helpers (2026-11-15): closed, outward-wound meshes for round parts (discs, rings, cones, square plates with a
round hole). Built-in USD Cylinder prims render as a coarse prism in Isaac and rings made of boxes leave gaps; use these for the
VISIBLE shape and keep the collider (invisible, purpose guide) unchanged.  `check_closed(pts, tris)` verifies a mesh."""
import math
import numpy as np


def _orient(pts, tris):
    """Flip all triangles if the signed volume is negative (so the normals point outward)."""
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[a], np.cross(p[b], p[c])) for a, b, c in tris) / 6.0
    return tris if vol >= 0 else [(a, c, b) for a, b, c in tris]


def check_closed(pts, tris):
    """True if every directed edge has exactly one opposite twin (watertight, consistent winding) and the volume is positive."""
    from collections import Counter
    e = Counter()
    for a, b, c in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            e[(u, v)] += 1
    ok = all(n == 1 and e.get((v, u)) == 1 for (u, v), n in e.items())
    p = np.asarray(pts, float)
    vol = sum(np.dot(p[a], np.cross(p[b], p[c])) for a, b, c in tris) / 6.0
    return ok and vol > 0, vol


def lathe(profile, n=96):
    """Revolve a closed (r, z) profile about z. Returns (pts, tris), outward-wound."""
    k = len(profile)
    pts = [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z) for i in range(n) for (r, z) in profile]
    tris = []
    for i in range(n):
        i2 = (i + 1) % n
        for j in range(k):
            j2 = (j + 1) % k
            a, b, c, d = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            tris += [(a, b, c), (a, c, d)]
    return pts, _orient(pts, tris)


def disc(r, z0, z1, n=96, axis="Z"):
    """A solid cylinder of radius r between z0 and z1 along ``axis`` (Z, Y or X), centred on the axis."""
    pts, tris = lathe([(0.0, z0), (r, z0), (r, z1), (0.0, z1)], n)
    # the axis points (r=0) repeat n times: weld them
    return _weld(_to_axis(pts, axis), tris)


def _to_axis(pts, axis):
    if axis == "Z":
        return pts
    if axis == "Y":
        return [(x, z, -y) for x, y, z in pts]
    return [(z, y, -x) for x, y, z in pts]  # X


def _weld(pts, tris, tol=1e-9):
    m, out, new = {}, [], []
    for p in pts:
        key = tuple(round(v / tol) for v in p)
        if key not in m:
            m[key] = len(out)
            out.append(p)
        new.append(m[key])
    t = [(new[a], new[b], new[c]) for a, b, c in tris]
    t = [x for x in t if len(set(x)) == 3]  # drop the degenerate triangles at the axis
    return out, t


def square_plate_round_hole(half, r_in, z0, z1, n=96):
    """A square plate (+-half in x and y) between z0 and z1 with a round hole of radius r_in. n is a multiple of 8."""
    ti, to, bi, bo = [], [], [], []
    for i in range(n):
        a = 2 * math.pi * i / n
        c, s = math.cos(a), math.sin(a)
        m = half / max(abs(c), abs(s))  # where the ray meets the square: corners lie on the 45-degree rays
        ti.append((r_in * c, r_in * s, z1)); to.append((m * c, m * s, z1))
        bi.append((r_in * c, r_in * s, z0)); bo.append((m * c, m * s, z0))
    pts = ti + to + bi + bo
    T, O, B, P = 0, n, 2 * n, 3 * n
    tris = []
    for i in range(n):
        j = (i + 1) % n
        for quad in ((T + i, O + i, O + j, T + j),   # top
                     (B + i, B + j, P + j, P + i),   # bottom
                     (O + i, P + i, P + j, O + j),   # outer wall
                     (T + i, T + j, B + j, B + i)):  # hole wall
            a, b, c, d = quad
            tris += [(a, b, c), (a, c, d)]
    return pts, _orient(pts, tris)
