#include "main.h"
#include "pros/adi.hpp"

//Controller Definition
pros::Controller master(CONTROLLER_MASTER);

//Motor Definitions

// Intake Definitions
pros::Motor Intake(1, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_ROTATIONS); //1

//Mobile goal Definition
pros::Motor MG(4, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_ROTATIONS); //1

//Drive Base Definitions
pros::Motor LF(13, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS), //19,9,13,
            // LM(9, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            LB(9, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            RF(17, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            // RM(17, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            RB(6, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS);

/*
Inertial sensor Definitions
*/
pros::Imu M_IMU(8), L_IMU(11), R_IMU(20);

/*
Tracking Wheels Definitions
*/
pros::Rotation OdomL(12),
               OdomS(14);

pros::ADIPotentiometer liftPos('A');

pros::ADIDigitalOut piston ('B');
