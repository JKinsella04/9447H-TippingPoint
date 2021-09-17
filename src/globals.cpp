#include "main.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/motors.h"

//Controller Definition
pros::Controller master(CONTROLLER_MASTER);

//Motor Definitions

// Intake Definitions
pros::Motor leftArm(17, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS), //
            rightArm(14, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS); //1

//Mobile goal Definition
pros::Motor leftMobileGoal(4, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            rightMobileGoal(5, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS); //1

//Drive Base Definitions
pros::Motor LF(20, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS), //19,9,13,
            LB(10, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS),
            RF(11, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            RB(1, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);

/*
Inertial sensor Definitions
*/
pros::Imu M_IMU(13), L_IMU(7), R_IMU(5);

/*
Tracking Wheels Definitions
*/
pros::Rotation OdomL(12),
               OdomS(14);

pros::ADIPotentiometer liftPos('A'),
                        mobileGoalPos('F');

pros::ADIDigitalOut clamp ('H'),
                    leftDragger('D'),
                    rightDragger('C');


pros::Gps gps(5, 15, 15, 0); //port, X offset, Y offset, heading offset
