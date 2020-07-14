// box for remote HF mk3 home device

module grid() {
    for (i = [0:4]) {
        translate([i*5,0,0])cylinder(d=3, h= 4, $fn=90);
    }
    for (i = [0:3]) {
        translate([2.5 + (i*5),5,0])cylinder(d=3, h= 4, $fn=90);
        translate([2.5 + (i*5),-5,0])cylinder(d=3, h= 4, $fn=90);
    }
    for (i = [0:2]) {
        translate([5 + (i*5),10,0])cylinder(d=3, h= 4, $fn=90);
        translate([5 + (i*5),-10,0])cylinder(d=3, h= 4, $fn=90);
    }
}
// bottom part
    translate([0,55,0])difference() {
        union() {
        // bottom
        translate([-50,-25,0])cube([100,50,1.5]);
        // walls & rims
        translate([-50,-25,0])cube([3,50,15]);
        translate([-50,-25,15])cube([1.4,50,2.5]);
        translate([47,-25,0])cube([3,50,15]);
        translate([48.6,-25,15])cube([1.4,50,2.5]);
        translate([-50,-25,0])cube([100,3,15]);
        translate([-50,-25,15])cube([100,1.4,2.5]);
        translate([-50,22,0])cube([100,3,15]);
        translate([-50,23.6,15])cube([100,1.4,2.5]);
    }
      // usb
      translate([27,-26,4])cube([10,6,5]);
      // sound
      translate([-27,-21,7])rotate([90,0,0])cylinder(d=6, h=6, $fn=90);
}
     
// top part
difference () {
    union() {
      // bottom
      translate([-50,-25,0])cube([100,50,1.5]);
      // walls & rims
      translate([-50,-25,0])cube([3,50,15]);
      translate([-48.4,-23.4,15])cube([1.4,46.8,2.5]);
      translate([47,-25,0])cube([3,50,15]);
      translate([47,-23.4,15])cube([1.4,46.8,2.5]);
      translate([-50,-25,0])cube([100,3,15]);
      translate([-47,-23.4,15])cube([94,1.4,2.5]);
      translate([-50,22,0])cube([100,3,15]);
      translate([-47,22,15])cube([94,1.4,2.5]);
      // OLED display attachments
      translate([10,15,0])cylinder(d=2.6, h= 7.5, $fn=90);
      translate([-10,15,0])cylinder(d=2.6, h= 7.5, $fn=90);
      translate([10,-8,0])cylinder(d=2.6, h= 7.5, $fn=90);
      translate([-10,-8,0])cylinder(d=2.6, h= 7.5, $fn=90); 
      translate([10,15,0])cylinder(d=4.8, h= 4.5, $fn=90);
      translate([-10,15,0])cylinder(d=4.8, h= 4.5, $fn=90);
      translate([10,-8,0])cylinder(d=4.8, h= 4.5, $fn=90);
      translate([-10,-8,0])cylinder(d=4.8, h= 4.5, $fn=90);  
    }    
     // OLED display
    translate([-12,-4,-1])cube([24,12,4]);      
    // loudspeaker
    translate([-40,0,-1])grid();
    // LED
    translate([30,0,-1])cylinder(d=5, h= 4, $fn=90);
    // connect swirch
    translate([30,-21,8.5])rotate([90,0,0])cylinder(d=6.2, h= 6, $fn=90);
    // connect switch
    translate([0,-21,8.5])rotate([90,0,0])cylinder(d=7, h= 6, $fn=90);  
    // volume potentiometer
    translate([-30,-21,8.5])rotate([90,0,0])cylinder(d=10.2, h= 6, $fn=90);
  }
 
  // adesion helpers (optional)
  translate([50,25,0])cylinder(d=8, h=1, $fn=60);
  translate([50,-25,0])cylinder(d=8, h=1, $fn=60);
  translate([-50,25,0])cylinder(d=8, h=1, $fn=60);
  translate([-50,-25,0])cylinder(d=8, h=1, $fn=60); 
  translate([0,55,0]) {
  translate([50,25,0])cylinder(d=8, h=1, $fn=60);
  translate([50,-25,0])cylinder(d=8, h=1, $fn=60);
  translate([-50,25,0])cylinder(d=8, h=1, $fn=60);
  translate([-50,-25,0])cylinder(d=8, h=1, $fn=60); } 