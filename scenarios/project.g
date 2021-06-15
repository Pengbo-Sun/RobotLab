world {}

table1 (world){
    shape:ssBox, Q:<t(1 0. .6)>, size:[2. 2. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

table2 (world){
    shape:ssBox, Q:<t(-2. 0. .6)>, size:[2. 2. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

tables3(world){
  shape:ssBox, Q:<t(-1.8 0.6 1.0) d(90 0 0 1)>, size:[.2 .8 .05 .01], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

#L_lift (table){ joint:transZ, limits:[0 .5] }

Prefix: "L_"
Include: 'panda_moveGripper.g'

Prefix: "R_"
Include: 'panda_moveGripper.g'

Prefix!

        
Edit L_panda_link0 (table1) { Q:<t(0 0 0) d(180 0 0 1)> }
Edit R_panda_link0 (table2)  { Q:<t( 0 0 0) d(0 0 0 1)> }

Delete L_finger1
Delete L_finger2


camera(R_gripper){
    Q:<t(0 0.1 0) d(90 1 0 0 )>,
    shape:marker, size:[.3],
    focalLength:0.895, width:640, height:360, zRange:[.1 100]
}

boardgreen(L_gripper) { Q:<t(0 0 -.155) d(0 0 1 0)> shape:cylinder, size:[0.01 0.15], color:[0 255 0] }
boardred(boardgreen) { Q:<t(0 0 0) d(0 0 1 0)> shape:cylinder, size:[0.02 0.1], color:[255 0 0] }
boardframe(boardred){ Q:<t(0 0 0) d(0 0 1 0)> shape:marker, size:[0.3 0.3 0.3], color:[255 0 0] }


dart1{ shape:cylinder, size:[0.15 0.01], color:[0 0 255],mass:0.2,X:<[-1.5,0.6, 1.07, 0, 0, 1, 0]>,contact=1}

dart3{ shape:cylinder, size:[0.15 0.01], color:[0 0 255],mass:0.2,X:<[-1.6,0.6, 1.07, 0, 0, 1, 0]>,contact=1}


Edit L_finger1{ joint:rigid }
Edit L_finger2{ joint:rigid }
Edit R_finger1{ joint:rigid }
Edit R_finger2{ joint:rigid }

        
