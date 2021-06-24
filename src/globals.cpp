#include "main.h"

//Controller Definition
pros::Controller master(CONTROLLER_MASTER);

//Motor Definitions

//Intakes
pros::Motor Intake(4, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_ROTATIONS); //1

//Drive Base
pros::Motor LF(13, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS), //19,9,13,
            LM(9, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            LB(9, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            RF(17, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            RM(17, MOTOR_GEARSET_6, 1, MOTOR_ENCODER_COUNTS),
            RB(6, MOTOR_GEARSET_6, 0, MOTOR_ENCODER_COUNTS);

/*
Optical Sensors
ROptical Optical Sensor on the right of the robot.
LOptical Optical Sensor on the left of the robot.
*/
// pros::Optical topOptical(2), botOptical(10);

/*
Distance Sensor
*/
// pros::Distance goalDist(3), ballIndexer(5);

/*
Inertial sensor
L_IMU  Left facing inertial sensor.
M_IMU  Front facing inertial sensor.
R_IMU  Right facing inertial sensor.
*/
pros::Imu M_IMU(8), L_IMU(11), R_IMU(20);

//Free Spinning Wheel Encoders
// pros::ADIEncoder LEncoder('E', 'F', true),
//                  REncoder('A', 'B', false);
pros::Rotation LOdometer(12),
               ROdometer(14);
//Line Sensors
// pros::ADIAnalogIn topLight('H');
