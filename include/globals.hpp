#pragma once
#include "main.h"
#include "control/units.hpp"

using namespace okapi;

#define HOLD pros::E_MOTOR_BRAKE_HOLD
#define BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define COAST pros::E_MOTOR_BRAKE_COAST

//Controller Declaration
extern pros::Controller master;

//Motor Declarations
extern pros::Motor arm;

extern pros::Motor intake;

extern pros::Motor LF, LM, LB, RF, RM, RB;

//Inertial Declarations
extern pros::Imu L_Imu, R_Imu;

extern pros::ADIDigitalOut frontClamp, backClamp;

extern pros::Gps gps;

extern pros::Distance LB_dist, RB_dist;