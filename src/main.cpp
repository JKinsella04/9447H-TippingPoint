#include "main.h"

#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/mobileGoal.hpp"
#include "control/positionTracking.hpp"
#include "control/lift.hpp"
#include "control/mobileGoal.hpp"
#include "globals.hpp"

void initialize() {

	// Class Init
	Position robotPos;	 
	Chassis chassis(robotPos.getRotation(), robotPos.getThetaDeg(), robotPos.getX(), robotPos.getY());
	// Chassis chassis(robotPos.getL(), robotPos.getYaw(), robotPos.getX(), robotPos.getY());
 	Display display;
	Lift lift;
	MobileGoal mobileGoal;
	
	// Sensor and Motor  
	robotPos.resetDriveBase().calibrateGyro().getGPSHeading(false);
	lift.reset();	
	mobileGoal.reset();

	// Threads
	pros::Task PositionController(robotPos.start, NULL, "Position Controller");

	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	pros::Task LiftController(lift.start, NULL, "Lift Controller");
	lift.setBrakeType(HOLD);
	
	pros::Task MobileGoalController(mobileGoal.start, NULL, "MobileGoal Controller");
	mobileGoal.setBrakeType(HOLD);

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
	mobileGoal.setBrakeType(HOLD);
	mobileGoal.setState(MobileGoalState::OPCONTROL); // Controls MobileGoal grabber.

  while (true) {

		if(master.get_digital(DIGITAL_DOWN)) mobileGoal.setup();
		
		// Controls Draggers.
		if(master.get_digital(DIGITAL_LEFT)) { Dragger::setState(true); }
		else if(master.get_digital(DIGITAL_RIGHT)) { Dragger::setState(false); }

	  pros::delay(5);
 	}
}