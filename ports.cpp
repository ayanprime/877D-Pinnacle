#include "main.h"

//Main Devices
brain Brain;
controller ControllerDriver = controller(primary);
controller ControllerAssistant = controller(partner);

//Drivetrain
motor lfm = motor(PORT10, ratio6_1, true);
motor lmm = motor(PORT9, ratio18_1, true);
motor lbm = motor(PORT13, ratio6_1, true);

motor rfm = motor(PORT1, ratio6_1, false);
motor rmm = motor(PORT2, ratio18_1, false);
motor rbm = motor(PORT3, ratio6_1, false);

//Lift
motor blueCont = motor(PORT17, ratio18_1, true);
motor pinkCont = motor(PORT15, ratio18_1, false);

//Extra Motors
motor flipper = motor(PORT5, ratio18_1, false);
motor intake = motor(PORT7, ratio36_1, true);

//Pneumatics
digital_out outriggers = digital_out(Brain.ThreeWirePort.C);
digital_out clutch = digital_out(Brain.ThreeWirePort.B);
digital_out claw = digital_out(Brain.ThreeWirePort.A);

//Sensors
inertial inrtl = inertial(PORT16);
gps GPS = gps(PORT20, 3.846278, -6.0891625, inches);
rotation fwdTrack  = rotation(PORT18, true);   // forward/back pod
rotation sideTrack = rotation(PORT12, true);   // side-to-side pod
