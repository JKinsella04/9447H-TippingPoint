#pragma once
#include "main.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"

#define HOLD pros::E_MOTOR_BRAKE_HOLD
#define BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define COAST pros::E_MOTOR_BRAKE_COAST

//Controller Declaration
extern pros::Controller master;

//Motor Declarations
extern pros::Motor MG;
extern pros::Motor leftArm, rightArm;
extern pros::Motor LF, /*LM,*/ LB, RF, /*RM,*/ RB;

//Inertial Declarations
extern pros::Imu M_IMU, L_IMU, R_IMU;

//Tracking Wheel Declarations
extern pros::Rotation OdomL, OdomS;

extern pros::ADIPotentiometer liftPos;

extern pros::ADIDigitalOut clamp, dragger;

extern pros::Gps gps;