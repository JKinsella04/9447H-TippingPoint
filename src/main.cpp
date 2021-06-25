#include "main.h"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/mobileGoal.hpp"
#include "control/intake.hpp"

// Class Initialization
Chassis chassis;

void initialize() {
	//Sensors
	L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
	while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(10); }

	LOdometer.reset();
	ROdometer.reset();

	//Threads
	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	ROdometer.set_reversed(true);
}


void disabled() {}

void competition_initialize() {}

void autonomous() {
	chassis.drive(20).withGains(.3, 0.01, .15).withTol(100).waitUntilSettled();
	chassis.turn(90).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();
}

void opcontrol() {

	while (true) {
		
		pros::delay(5);
	}
}
