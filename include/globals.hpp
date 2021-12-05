#pragma once
#include "main.h"
#include "pros/distance.hpp"

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
extern pros::Imu lf_Imu, lb_Imu, rf_Imu, rb_Imu;

extern pros::ADIDigitalOut clamp, mg;

extern pros::Gps gps;