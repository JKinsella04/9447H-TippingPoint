#include "main.h"

#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/mobileGoal.hpp"
#include "control/odometry.hpp"
#include "control/lift.hpp"
#include "control/mobileGoal.hpp"
#include "globals.hpp"
#include "pros/misc.h"


void initialize() {

	// Class Init
	Odom odom;	 
	// Chassis chassis(odom.getL(), odom.getThetaDeg(), odom.getX(), odom.getY());
	Chassis chassis(odom.getL(), odom.getYaw(), odom.getX(), odom.getY());
 	Display display;
	Lift lift;
	MobileGoal mobileGoal;
	
	// Rotation Sensor calibration
	// OdomL.reset_position();
	// OdomS.reset_position();
	// OdomL.set_reversed(false);

	//Motor encoder calibration
	LF.tare_position();
	LB.tare_position();
	RF.tare_position();
	RB.tare_position();

	// chassis.reset();
	lift.reset();	
	mobileGoal.reset();	

	// IMU calibration
	odom.calibrateGyro();
	odom.zero();

  // Threads
	pros::Task OdometryController(odom.start, NULL, "Odom Controller");

	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	pros::Task LiftController(lift.start, NULL, "Lift Controller");
	lift.setBrakeType(HOLD);
	
	pros::Task MobileGoalController(mobileGoal.start, NULL, "MobileGoal Controller");
	// mobileGoal.setBrakeType(HOLD);

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
  chassis.setState(ChassisState::OPCONTROL); // Runs Tank Control.
  chassis.setBrakeType(COAST);

	Lift lift;
	lift.setState(LiftState::OPCONTROL); // Controls Lift + Pneumatic Clamp.

	MobileGoal mobileGoal;
	mobileGoal.setup();
	mobileGoal.setState(MobileGoalState::OPCONTROL); // Controls MobileGoal grabber.

  while (true) {
		
		// Controls Draggers.
		if(master.get_digital(DIGITAL_LEFT)) { Dragger::setState(true); }
		else if(master.get_digital(DIGITAL_RIGHT)) { Dragger::setState(false); }

	  pros::delay(5);
 	}
}