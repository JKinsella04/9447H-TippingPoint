#include "main.h"

#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/odometry.hpp"
#include "control/purePursuit.hpp"

void initialize() {
	Odometry odom;
	Chassis chassis;
 	Display display;

	// Sensor Init
	L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
  	while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(20); }

    // Threads
	pros::Task OdometryController(odom.start, NULL, "Odom Controller");

	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	pros::Task DisplayController(display.start, NULL, "Display Controller");
	DisplayController.set_priority(TASK_PRIORITY_MIN);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	Autonomous auton;
	auton.runAuton();
}

void opcontrol() {
  Chassis chassis;
  chassis.setState(ChassisState::OPCONTROL);
  chassis.setBrakeType(COAST);

  while (true) {
    pros::delay(5);
	}
}