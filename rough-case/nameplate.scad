// Rosa / Pico nameplate for the rough case lid.
// A flat base with raised letters; glue or tape it to the lid top.
// Sized for the clear area toward the rear, between the d-pad and fire button.
//
//   openscad -D 'part="all"'     -o nameplate.stl         nameplate.scad
//   openscad -D 'part="base"'    -o nameplate_base.stl    nameplate.scad
//   openscad -D 'part="letters"' -o nameplate_letters.stl nameplate.scad
//
// One-nozzle printers: print nameplate.stl and change filament at z = base_t.
// Multi-colour printers: load the base and letters STLs together; they share an origin.

part = "all";   // "all", "base", "letters"

plate_w   = 60;
plate_d   = 36;
corner_r  = 3;
base_t    = 1.6;
letter_h  = 1.0;   // height of the letters above the base

line1     = "Rosa";
line2     = "Pico";
font      = "DejaVu Sans:style=Bold";
text_size = 11;
line_gap  = 15.5;  // baseline to baseline

$fn = 48;

module base() {
    linear_extrude(base_t) hull()
        for (x = [corner_r, plate_w-corner_r], y = [corner_r, plate_d-corner_r])
            translate([x, y]) circle(corner_r);
}

module letters() {
    translate([plate_w/2, plate_d/2, base_t]) linear_extrude(letter_h) {
        translate([0,  line_gap/2 - text_size/2]) text(line1, size = text_size, font = font, halign = "center");
        translate([0, -line_gap/2 - text_size/2]) text(line2, size = text_size, font = font, halign = "center");
    }
}

if (part == "base") base();
else if (part == "letters") letters();
else { color("DimGray") base(); color("Gold") letters(); }
