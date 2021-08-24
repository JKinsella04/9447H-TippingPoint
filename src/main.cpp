#include "main.h"

#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/odometry.hpp"
#include "control/lift.hpp"

void initialize() {
	// Class Init
	Odometry odom;	 
	Chassis chassis(odom.getL(), odom.getThetaDeg(), odom.getX(), odom.getY());
 	Display display;
	Lift lift;

	// Sensor Init
	liftPos.calibrate();
	OdomL.reset_position();
	OdomS.reset_position();
	OdomL.set_reversed(false);
	L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
  while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(20); }

  // Threads
	pros::Task OdometryController(odom.start, NULL, "Odom Controller");

	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	pros::Task LiftController(lift.start, NULL, "Lift Controller");

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
  chassis.setState(ChassisState::OPCONTROL); // Runs Tank Control 
  chassis.setBrakeType(COAST);
	
  while (true) {
    pros::delay(5);
	}
}