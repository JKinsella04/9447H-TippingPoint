#include "main.h"

#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/backLift.hpp"
#include "control/frontLift.hpp"
#include "control/positionTracking.hpp"
#include "globals.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/impl/util/timer.hpp"
#include "pros/rtos.h"

void initialize() {

	// Class Init
	Position robotPos;	 
	Chassis chassis(robotPos.getRotation(), robotPos.getThetaDeg(), robotPos.getX(), robotPos.getY());
 	Display display;
	FrontLift frontLift;
	BackLift backLift;
	
	// Sensor and Motor  
	robotPos.resetDriveBase().calibrateGyro().setState(PositionTracker::RELATIVE);
	frontLift.reset();	

	// Threads
	pros::Task PositionController(robotPos.start, NULL, "Position Controller");

	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	pros::Task LiftController(frontLift.start, NULL, "FrontLift Controller");
	frontLift.setBrakeType(HOLD);
	
	pros::Task backLiftController(backLift.start, NULL, "BackLift Controller");

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
	chassis.reset();
  chassis.setState(ChassisState::OPCONTROL); // Runs Tank Control.
  chassis.setBrakeType(COAST);

	FrontLift frontLift;
	frontLift.setState(FrontLiftState::OPCONTROL); // Controls Lift + Pneumatic Clamp.

	BackLift backLift;
	backLift.setState(BackLiftState::OPCONTROL); // Controls Mobile Goal.

	double offset = pros::c::millis();

  while (true) {
		double time = pros::c::millis() - offset;
		double delay = ( 75000 - time ) / 7.5;
		macro::print("DELAY ", delay);
		pros::delay(delay);
		if(delay > 0) master.rumble("..");
 	}
}