#include "main.h"
#include "pros/adi.hpp"
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
            RB(5, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS); // 2

// Inertial sensor Definitions //
pros::Imu L_Imu(18), R_Imu(13);

// Pneumatic Definitions //
pros::ADIDigitalOut frontClamp ('H'),
                    backClamp('D');

pros::ADIDigitalIn backLimit('B'),
                   frontLImit('C');

// GPS Definition // 
pros::Gps gps(4,0.1651,0.0762); 

// Distance Sensor Definitions //
pros::Distance backDist(2);