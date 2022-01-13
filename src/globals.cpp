#include "main.h"
#include "pros/distance.hpp"
#include "pros/misc.h"

//Controller Definition
pros::Controller master(CONTROLLER_MASTER);

// Motor Definitions //
// Lift Definition
pros::Motor arm(1, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);

// Intake Definition
pros::Motor intake(9, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS);

//Drive Base Definitions
pros::Motor LF(20, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS), 
            LM(8, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            LB(10, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS),
            RF(11, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS),
            RM(3, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS),
            RB(2, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);

// Inertial sensor Definitions //
pros::Imu L_Imu(18), R_Imu(13);

// Pneumatic Definitions //
pros::ADIDigitalOut frontClamp ('H'),
                    backArm('D'),
                    backClamp('F');

// GPS Definition // 
pros::Gps gps(5, 0, -0.3048); 
