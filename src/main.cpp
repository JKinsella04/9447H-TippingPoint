#include "main.h"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/mobileGoal.hpp"
#include "control/intake.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"

// Class Init


void initialize() {
	Chassis Chassis;
 	Display Display;
    // Sensors
	L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
	while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(10); }

	LOdometer.reset();
	ROdometer.reset();
    ROdometer.set_reversed(true);

    // Threads
	pros::Task ChassisController(Chassis.start, NULL, "Chassis Controller");
	Chassis.setBrakeType(HOLD);

	pros::Task DisplayController(Display.start, NULL, "Display Controller");
	DisplayController.set_priority(TASK_PRIORITY_MIN);
	

}


void disabled() {}

void competition_initialize() {}

void autonomous() {
	Autonomous Auton;
	Auton.runAuton();
}

void opcontrol() {
	Chassis Chassis;
	Chassis.setState(ChassisState::OPCONTROL);
	Chassis.setBrakeType(COAST);
	while (true) {

	  pros::delay(5);
	}
}
