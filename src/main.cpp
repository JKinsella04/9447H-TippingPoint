#include "globals.hpp"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/backLift.hpp"
#include "control/frontLift.hpp"
#include "control/positionTracking.hpp"
#include "okapi/api/units/QTime.hpp"

void initialize() {

	// Class Init
	static PositionTracker *robot;
	Chassis chassis;
 	Display display;
	FrontLift frontLift;
	BackLift backLift;
	
	// Sensor and Motor Reset  
	frontLift.reset();
	// robot->Odom::tarePosition()->Odom::calibrateGyro()->Odom::zero();

	// Threads
	pros::Task PositionController(robot->start, NULL, "Position Controller");

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
  static PositionTracker * robot;
  robot->resetTime();
	
	Chassis chassis;
	chassis.reset();
  chassis.setState(ChassisState::OPCONTROL); // Runs Tank Control.
  chassis.setBrakeType(COAST);

  FrontLift frontLift;
  frontLift.setState(FrontLiftState::OPCONTROL); // Controls Lift + Pneumatic Clamp.
	if ( !frontLift.getClampState() && auton.getAuton() == "Skills") frontLift.toggleClamp(); // If clamped Unclamp.

  BackLift backLift;
  backLift.setState(BackLiftState::OPCONTROL); // Controls Mobile Goal.
	// Clamp front('H', false);
	// Clamp back('D', false);

  while (true) {
		// if(master.get_digital(DIGITAL_A))front.toggle();
		// if(master.get_digital(DIGITAL_B))back.toggle();
	  double delay = (75000 - robot->getTime().convert(millisecond)) / 7.5;
    pros::delay(delay);
    if (delay > 0)
      master.rumble("..");
	}
}