#pragma once
#include "main.h"

//Controller Declaration
extern pros::Controller master;

//Motor Declarations
extern pros::Motor MG;
extern pros::Motor Intake;
extern pros::Motor LF, /*LM,*/ LB, RF, /*RM,*/ RB;

//Inertial Declarations
extern pros::Imu M_IMU, L_IMU, R_IMU;

//Tracking Wheel Declarations
extern pros::Rotation OdomL, OdomS;

