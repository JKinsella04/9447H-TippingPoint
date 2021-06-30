#include "main.h"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/mobileGoal.hpp"
#include "control/intake.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/odometry.hpp"
#include "control/purePursuit.hpp"
#include <iostream>

// Class Init


void initialize() {
	Chassis Chassis;
 	Display Display;
	Odom Odom;

    // Sensors
	L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
	while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(10); }

	LOdometer.reset();
	ROdometer.reset();
    // ROdometer.set_reversed(true);

    // Threads
	pros::Task ChassisController(Chassis.start, NULL, "Chassis Controller");
	Chassis.setBrakeType(HOLD);

	pros::Task DisplayController(Display.start, NULL, "Display Controller");
	DisplayController.set_priority(TASK_PRIORITY_MIN);

	pros::Task OdometryController(Odom.start, NULL, "Odom Controller");

}


void disabled() {}

void competition_initialize() {}

void autonomous() {
	Autonomous Auton;
	Auton.runAuton();
}

void opcontrol() {
  PurePursuit PurePursuit;
  Chassis Chassis;
  Chassis.setState(ChassisState::OPCONTROL);
  Chassis.setBrakeType(COAST);

  while (true) {
    PurePursuit.goToPoint(10, 10, 200);
    pros::delay(5);
	}
}
