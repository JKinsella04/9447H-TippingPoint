#pragma once
#include "main.h"
#include "pros/distance.hpp"

#define HOLD pros::E_MOTOR_BRAKE_HOLD
#define BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define COAST pros::E_MOTOR_BRAKE_COAST

//Controller Declaration
extern pros::Controller master, partner;

//Motor Declarations
extern pros::Motor leftArm, rightArm;

extern pros::Motor leftMobileGoal, rightMobileGoal;

extern pros::Motor LF, /*LM,*/ LB, RF, /*RM,*/ RB;

//Inertial Declarations
extern pros::Imu lf_Imu, lb_Imu, rf_Imu, rb_Imu;

extern pros::Distance platform;

//Tracking Wheel Declarations
extern pros::Rotation OdomL, OdomS;

extern pros::ADIPotentiometer mobileGoalPos;

extern pros::ADIDigitalOut clamp, dragger;

extern pros::Gps gps;