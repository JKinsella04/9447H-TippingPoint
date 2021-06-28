#include "main.h"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/mobileGoal.hpp"
#include "control/intake.hpp"
#include "control/gui.hpp"



void initialize() {
	// Class Init
    Chassis chassis;
	Display display;

	// Sensors
	L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
	while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(10); }

	LOdometer.reset();
	ROdometer.reset();
    ROdometer.set_reversed(true);

    // Threads
	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	pros::Task DisplayController(display.start, NULL, "Display Controller");
	DisplayController.set_priority(TASK_PRIORITY_MIN);
	

}


void disabled() {}

void competition_initialize() {}

void autonomous() {
	// chassis.drive(20).withGains(.3, 0.01, .15).withTol(100).waitUntilSettled();
	// chassis.turn(90).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();
}

void opcontrol() {
	while (true) {
		pros::delay(5);
	}
}
