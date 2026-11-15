// Rocinante-Pico breadboard enclosure
// Two parts: a flat base (breadboards get taped to it) and a lid that carries
// the RCA jacks, DE-9, reset button and a micro-USB access slot.
// Four M3 screws go up through the base into bosses in the lid corners.
//
// All dimensions in mm.  MEASURE YOUR BUILD and adjust the "board" and
// "port" sections before printing; the defaults were estimated from a photo.
//
//   openscad -D 'part="lid"'    -o lid.stl    rocinante_pico_enclosure.scad
//   openscad -D 'part="base"'   -o base.stl   rocinante_pico_enclosure.scad
//   openscad -D 'part="coupon"' -o coupon.stl rocinante_pico_enclosure.scad

part = "assembly";   // "assembly", "lid", "base", "coupon"

/* [Board footprint (everything that sits on the base)] */
board_w   = 180;  // left-right, measured
board_d   = 85;   // front-back, measured
margin    = 6;    // free space around the footprint (>= 6 so corner bosses clear)
board_t   = 10;   // breadboard thickness
inner_h   = 62;   // floor to underside of lid top: 10 board + 50 wires + 2 spare

/* [Shell] */
wall      = 2.4;
top_t     = 2.4;
base_t    = 3.0;
corner_r  = 3;
fit       = 0.3;  // clearance between base lip and lid walls

/* [Micro-USB slot, back wall] */
usb_x     = 95;   // from LEFT edge of board footprint to centre of the Pico's USB jack
usb_z     = 15;   // floor to centre of USB jack (breadboard 10 + header 2.5 + PCB 1 + 1.5)
usb_w     = 14;   // slot must pass the cable's overmold, not just the metal plug
usb_h     = 9.5;

/* [RCA jacks, left wall] */
rca_d       = 9.7;  // 9.32 mm measured thread + clearance
rca_spacing = 22;   // centre to centre (video / audio); nut/flange is 12.1 mm
rca_y_off   = 0;    // shift the pair toward the back (+) or front (-)
rca_z       = 26;   // floor to centre

/* [DE-9, right wall] */
de9_y_off   = 0;     // shift toward back (+) or front (-)
de9_z       = 26;    // floor to centre
de9_cut_w   = 20.5;  // D opening, wide (top) edge
de9_cut_h   = 11.4;
de9_hole_sp = 24.99; // standard D-sub size-E mounting hole spacing
de9_hole_d  = 3.2;   // clearance for 4-40 / M3

/* [Reset button, lid top] */
// Both sit over the empty right-hand end of the breadboard: the button reaches
// 16.5 mm and the LED 15 mm below the lid, which is lower than the tallest wires.
reset_d   = 7.2;  // 6.82 mm measured thread + clearance
reset_x   = 28;   // from the RIGHT outer edge
reset_y   = 25;   // from the FRONT outer edge

/* [Panel LED, lid top] */
led_d     = 6.3;  // 5.89 mm measured thread + clearance
led_x     = 28;   // from the RIGHT outer edge
led_y     = 50;   // from the FRONT outer edge

/* [Screws] */
boss_d     = 7.5;
pilot_d    = 2.5;  // M3 self-tapping into plastic
pilot_len  = 14;
screw_d    = 3.4;  // clearance in base
head_d     = 6.4;
head_depth = 1.6;

/* [Base lip] */
lip_t = 1.6;
lip_h = 3;

$fn = 48;
eps = 0.01;

ow = board_w + 2*margin + 2*wall;   // outer width  (X)
od = board_d + 2*margin + 2*wall;   // outer depth  (Y)
lid_h = inner_h + top_t;
boss_c = wall + boss_d/2 - 0.75;    // boss centre inset from outer faces

function boss_pts() = [[boss_c, boss_c], [ow-boss_c, boss_c],
                       [boss_c, od-boss_c], [ow-boss_c, od-boss_c]];

module rrect(w, d, r) {
    hull() for (x = [r, w-r], y = [r, d-r]) translate([x, y]) circle(r);
}

// D-sub opening, centred, wide edge up
module de9_cut2d() {
    r = 1.5; w = de9_cut_w; h = de9_cut_h;
    inset = h * tan(10);
    offset(r) offset(delta = -r)
        polygon([[-w/2, h/2], [w/2, h/2], [w/2-inset, -h/2], [-w/2+inset, -h/2]]);
}
module de9_2d() {
    de9_cut2d();
    for (s = [-1, 1]) translate([s*de9_hole_sp/2, 0]) circle(d = de9_hole_d);
}
module rca_2d() {
    for (s = [-1, 1]) translate([s*rca_spacing/2, 0]) circle(d = rca_d);
}

module lid() {
    difference() {
        union() {
            difference() {
                linear_extrude(lid_h) rrect(ow, od, corner_r);
                translate([wall, wall, -eps]) cube([ow-2*wall, od-2*wall, inner_h+eps]);
            }
            for (p = boss_pts()) translate([p[0], p[1], 0]) cylinder(d = boss_d, h = inner_h+eps);
        }
        // screw pilots
        for (p = boss_pts()) translate([p[0], p[1], -eps]) cylinder(d = pilot_d, h = pilot_len);
        // micro-USB slot, back wall
        translate([wall + margin + usb_x - usb_w/2, od - wall - 1, usb_z - usb_h/2])
            cube([usb_w, wall + 2, usb_h]);
        // RCA, left wall (as seen from outside, width runs along -Y; symmetric so no matter)
        translate([-1, od/2 + rca_y_off, rca_z]) rotate([90, 0, 90])
            linear_extrude(wall + 2) rca_2d();
        // DE-9, right wall
        translate([ow - wall - 1, od/2 + de9_y_off, de9_z]) rotate([90, 0, 90])
            linear_extrude(wall + 2) de9_2d();
        // reset button, top
        translate([ow - reset_x, reset_y, inner_h - 1]) cylinder(d = reset_d, h = top_t + 2);
        // panel LED, top
        translate([ow - led_x, led_y, inner_h - 1]) cylinder(d = led_d, h = top_t + 2);
    }
}

module base() {
    difference() {
        union() {
            translate([0, 0, -base_t]) linear_extrude(base_t) rrect(ow, od, corner_r);
            // locating lip, interrupted at the corners for the lid bosses
            difference() {
                linear_extrude(lip_h) difference() {
                    translate([wall+fit, wall+fit]) square([ow-2*(wall+fit), od-2*(wall+fit)]);
                    translate([wall+fit+lip_t, wall+fit+lip_t])
                        square([ow-2*(wall+fit+lip_t), od-2*(wall+fit+lip_t)]);
                }
                for (p = boss_pts()) translate([p[0], p[1], -eps])
                    cylinder(d = boss_d + 6, h = lip_h + 1);
            }
        }
        for (p = boss_pts()) {
            translate([p[0], p[1], -base_t-eps]) cylinder(d = screw_d, h = base_t + 1);
            translate([p[0], p[1], -base_t-eps]) cylinder(d = head_d, h = head_depth);
        }
    }
}

// Small flat test piece with every cutout, to check fit before the long print
module coupon() {
    linear_extrude(wall) difference() {
        rrect(100, 44, 3);
        translate([22, 28]) de9_2d();
        translate([22, 10]) square([usb_w, usb_h], center = true);
        translate([62, 12]) rca_2d();
        translate([62, 33]) circle(d = reset_d);
        translate([80, 33]) circle(d = led_d);
        translate([92, 12]) circle(d = pilot_d);
    }
}

if (part == "lid")         translate([0, od, lid_h]) rotate([180, 0, 0]) lid();  // top face on the bed
else if (part == "base")   translate([0, 0, base_t]) base();
else if (part == "coupon") coupon();
else {
    color("SteelBlue") base();
    color("Gainsboro", 0.6) lid();
    // ghost of the board footprint
    %translate([wall+margin, wall+margin, 0]) cube([board_w, board_d, board_t]);
}
