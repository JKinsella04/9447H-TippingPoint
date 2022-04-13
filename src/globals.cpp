#include "main.h"

//Controller Definition
pros::Controller master(CONTROLLER_MASTER);

// Motor Definitions //
// Lift Definition
pros::Motor arm(13, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);

// Intake Definition
pros::Motor intake(15, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS);

//Drive Base Definitions
pros::Motor LF(6, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS), 
            LM(8, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            LB(10, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            RF(1, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS),
            RM(3, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS),
            RB(5, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS);

// Inertial sensor Definitions //
pros::Imu L_Imu(17), R_Imu(18);

// Pneumatic Definitions //
pros::ADIDigitalOut frontClamp ('H'), //H
                    backClamp('D'); //D

// GPS Definition // 
pros::Gps gps(4,0.1651,0.0762); 

// Distance Sensor Definitions //
pros::Distance LB_dist(20), RB_dist(19);