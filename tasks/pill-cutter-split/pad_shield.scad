// Pad shield: a 0.6 mm PETG U-clip for one YAM finger pad, used in the real trial (not a scored object).
// It covers the pad's outer and two side faces near the tip and leaves the inner grip face open, so the fin 8 mm
// squeeze and the caplet 1 mm squeeze do not change. Print in Prusament PETG, 0.2 mm layers, 3 perimeters, 40% infill.
// Units: mm. The pad's inner face is at z = 0 (open side); the outer face is at z = depth; the clip wraps the sides.

wall       = 0.6;   // PETG wall
length     = 24.0;  // along the pad, near the tip
width      = 14.0;  // across the pad
depth      = 6.0;   // pad thickness covered (outer face at z = depth)
lip        = 1.0;   // small returns on the grip side, so it clips on
$fn        = 32;

module u_clip() {
    difference() {
        // outer block: two side walls and the outer face
        cube([length, width, depth + wall]);
        // the pad seat: open on the grip side (z < 0), wall at each side and the outer face left
        translate([-0.05, wall + 0.05, -0.05])
            cube([length + 0.1, width - 2 * wall - 0.1, depth + 0.05]);
    }
    // two return lips at the open side, 1 mm deep, on the side walls only (the inner face stays exposed)
    translate([0, wall, -lip]) cube([length, wall, lip]);
    translate([0, width - 2 * wall, -lip]) cube([length, wall, lip]);
}

u_clip();
