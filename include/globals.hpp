#pragma once
#include "main.h"
//Controller Definition
extern pros::Controller master;

//Motor Declarations
extern pros::Motor intake;
extern pros::Motor lift;
extern pros::Motor LF, LM, LB, RF, RM ,RB;


//Sensor Declarations
// extern pros::Optical topOptical, botOptical;

// extern pros::Distance goalDist, ballIndexer;

extern pros::Imu M_IMU, L_IMU, R_IMU;

// //Motor Encoders
extern pros::Rotation LOdometer, ROdometer;

