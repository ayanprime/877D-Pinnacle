#pragma once
#include "main.h"

//Main Devices
extern brain Brain;
extern controller ControllerDriver;
extern controller ControllerAssistant;

//Drivetrain
extern motor lfm;
extern motor lmm;
extern motor lbm;

extern motor rfm;
extern motor rmm;
extern motor rbm;

//Lift
extern motor blueCont;
extern motor pinkCont;

//Extra Motors
extern motor flipper;
extern motor intake;

//Pneumatics
extern digital_out outriggers;
extern digital_out clutch;

//Sensors
extern inertial inrtl;
extern gps GPS;
