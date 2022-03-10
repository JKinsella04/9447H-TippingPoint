#include "globals.hpp"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/backLift.hpp"
#include "control/frontLift.hpp"
#include "control/positionTracking.hpp"

void initialize() {

	// Class Init
	Position robotPos(4.15, 200, {5,7});	 
	Chassis chassis(robotPos.getRotation(), robotPos.getTheta(), robotPos.getX(), robotPos.getY());
 	Display display;
	FrontLift frontLift;
	BackLift backLift;
	
	// Sensor and Motor  
	robotPos.resetDriveBase().calibrateGyro().zero().setState(PositionTracker::RELATIVE);
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
	Autonomous auton;
  Position robotPos;
  robotPos.resetTime("Opcontrol");
	
	Chassis chassis;
	chassis.reset();
  chassis.setState(ChassisState::OPCONTROL); // Runs Tank Control.
  chassis.setBrakeType(COAST);

  FrontLift frontLift;
  frontLift.setState(FrontLiftState::OPCONTROL); // Controls Lift + Pneumatic Clamp.
	if ( !frontLift.getClampState() && auton.getAuton() == "Skills") frontLift.toggleClamp(); // If clamped Unclamp.

  BackLift backLift;
  backLift.setState(BackLiftState::OPCONTROL); // Controls Mobile Goal.

  while (true) {
    double delay = (75000 - *robotPos.getTime()) / 7.5;
    pros::delay(delay);
    if (delay > 0)
      master.rumble("..");
  }
}