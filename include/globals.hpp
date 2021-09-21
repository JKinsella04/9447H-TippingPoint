#pragma once
#include "main.h"

#define HOLD pros::E_MOTOR_BRAKE_HOLD
#define BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define COAST pros::E_MOTOR_BRAKE_COAST

//Controller Declaration
extern pros::Controller master;

//Motor Declarations
extern pros::Motor leftArm, rightArm;

extern pros::Motor leftMobileGoal, rightMobileGoal;

extern pros::Motor LF, /*LM,*/ LB, RF, /*RM,*/ RB;

//Inertial Declarations
extern pros::Imu lf_Imu, lb_Imu, rf_Imu, rb_Imu;

//Tracking Wheel Declarations
extern pros::Rotation OdomL, OdomS;

extern pros::ADIPotentiometer liftPos, mobileGoalPos;

extern pros::ADIDigitalOut clamp, dragger;

extern pros::Gps gps;