// SSD1306 oled module cover
// design by Marius Gheorghescu, September 2020

// offset between the OLED PCB and CurrentRanger PCB
pcb2pcb_offset = 12.60;

// pcb width&height + some clearance
oled_pcb_x = 25.3 + 0.3;
oled_pcb_y = 26.1 + 0.4;

// assuming square pattern
oled_screw_spacing = 21;

// screen dimensions
oled_screen_x = 24;
oled_screen_y = 12;
oled_screen_thickness = 2.4;

// oled offset from center (should be closer to the pins)
oled_screen_y_offset = 1.15;


oled_pcb_thickness = 1.6;

// connector + some clearance
connector_x = 11.3;
connector_y = 2.8;

// ideally multiple of extrusion width to minimize thin walls
wall_thickness = 1.6;
top_wall_thickness = 0.6;

// leave opening to see the logo / access serial 
open_bottom = 1;


// internal and computed values (LCD module + 2-3 layers of print)
epsilon = 0.001;
total_case_height = pcb2pcb_offset + oled_screen_thickness + top_wall_thickness;
retainer_clip_cutout = connector_y + wall_thickness;


module case()
{    
    difference() {

        translate([0, 0, total_case_height/2])
        hull() {
            cube([2*wall_thickness + oled_pcb_x-1, 2*wall_thickness + oled_pcb_y-1, total_case_height], center=true);
            cube([2*wall_thickness + oled_pcb_x, 2*wall_thickness + oled_pcb_y-1, total_case_height-1], center=true);    
            cube([2*wall_thickness + oled_pcb_x-1, 2*wall_thickness + oled_pcb_y, total_case_height-1], center=true);    
        }

        // PCB corner supports (look ma, no screws!)
        translate([0,0, (oled_screen_thickness + oled_pcb_thickness)/2 + top_wall_thickness])
            hull() {
                cube([oled_pcb_x-5, oled_pcb_y, oled_screen_thickness + oled_pcb_thickness], center=true);
                cube([oled_pcb_x, oled_pcb_y-5, oled_screen_thickness + oled_pcb_thickness], center=true);
                translate([0, 0, total_case_height])
                    cube([oled_pcb_x, epsilon, epsilon], center=true);
            }

        // pcb load path
        translate([0,0, total_case_height/2 + oled_screen_thickness])
        {
            difference() {
                
                union() {
                    cube([oled_pcb_x, oled_pcb_y, total_case_height], center=true);                
                }
                
                translate([0, oled_pcb_y/2 - connector_y*0.75, 0])
                    cube([connector_x + 4*wall_thickness, connector_y + + 2*wall_thickness, total_case_height], center=true);                
            }
        }

        // LCD cutout
        translate([0, oled_screen_y_offset, 0])
            cube([oled_screen_x, oled_screen_y, total_case_height], center=true);
        
        // connector    
        taper_amount = 0.08;
        translate([0, oled_pcb_y/2 - connector_y*0.75, total_case_height*3/2 - pcb2pcb_offset])
        {
            hull() {
                translate([0, 0, total_case_height/2])
                    cube([connector_x + taper_amount, connector_y + taper_amount, epsilon], center=true);
                translate([0, 0, -total_case_height/2])
                    cube([connector_x - taper_amount, connector_y - taper_amount, epsilon], center=true);
            }
            

            // connector load clearance
            hull() {
                translate([0, connector_y/8, -total_case_height/3])
                    cube([connector_x, connector_y*1.25, 5], center=true);

                translate([0, -connector_y, 0])
                    cube([epsilon, 4+epsilon, epsilon], center=true);
            }
           
            translate([0,-wall_thickness/2 - epsilon, 0])
                cube([connector_x - connector_y, connector_y+wall_thickness, total_case_height], center=true);
        }
        
    
            
        // retaining clip
        translate([0, -retainer_clip_cutout/2, oled_screen_thickness + oled_pcb_thickness + 1])
        hull() {
            cube([oled_pcb_x + 2, oled_pcb_y - retainer_clip_cutout, epsilon], center=true);
            cube([oled_pcb_x, oled_pcb_y - retainer_clip_cutout, 2], center=true);
        }
            
                
        if (open_bottom) {            
            // opening to see the logo / access the BT/serial connector
            rotate([50,0,0])
            translate([0,0, total_case_height + 100/2])
                cube([100,100,100], center=true);
        }
        
    }

}

module retainer_clip()
{

    // springiness - how much to oversize it so it stays put
    oversize = 0.5;
    
    //clip_height = oled_pcb_y - retainer_clip_cutout - wall_thickness;
    clip_height = oled_pcb_y/2;
    
    difference() {
        
        union() {
            translate([0, total_case_height/4, clip_height/2])
                cube([oled_pcb_x, total_case_height/2, clip_height], center=true);
            
            // wings
            translate([0,0, clip_height/2])
            hull() {
                cube([oled_pcb_x + 2 + oversize, epsilon, clip_height], center=true);
            
                    cube([oled_pcb_x + oversize, 2, clip_height], center=true);
            }
        }

        // make it elastic
        translate([0, total_case_height/4 - wall_thickness, clip_height/2])
            cube([oled_pcb_x - 2*wall_thickness, total_case_height/2, clip_height + epsilon], center=true);
        
        // clearance for the resistors on the PCB, just in case
        hull() {
            translate([0, 0, clip_height/2])
                cube([oled_pcb_x - 2*wall_thickness, wall_thickness, clip_height + epsilon], center=true);
            translate([0, -wall_thickness, clip_height/2])
                cube([oled_pcb_x - 2*wall_thickness + wall_thickness*1.5, wall_thickness, clip_height + epsilon], center=true);
        }

    }
}

case();

translate([2*oled_screen_x, 0, 0])
    retainer_clip();
