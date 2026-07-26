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

//Extra Motors

extern motor flipper;
extern motor intake;

//Sensors

extern inertial inrtl;
extern gps GPS;