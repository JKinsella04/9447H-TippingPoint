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


void initialize() {
	// Class Init
	Odometry odom;	 
	Chassis chassis(odom.getEncoderCount(), odom.getThetaDeg(), odom.getX(), odom.getY()); // Replace odom.getEncoderCount() with odom.getL() to use rotaiton sensor.
 	Display display;
	Lift lift;
	MobileGoal mobileGoal;

	// Sensor Init
	
	// Potentiometer calibration
	liftPos.calibrate();
	mobileGoalPos.calibrate();
	
	// Rotation Sensor calibration
	OdomL.reset_position();
	OdomS.reset_position();
	OdomL.set_reversed(false);

	// IMU calibration
	// L_IMU.reset();
	// M_IMU.reset(); 
	// R_IMU.reset();	
  // while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(20); }

  // Threads
	pros::Task OdometryController(odom.start, NULL, "Odom Controller");

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
	mobileGoal.setState(MobileGoalState::OPCONTROL); // Controls MobileGoal grabber.


  while (true) {
		
		// Controls Draggers.
		if(master.get_digital(DIGITAL_UP)) { draggers::setState(BOTH, true); }
		else if(master.get_digital(DIGITAL_RIGHT)) { draggers::setState(BOTH, false); }

	  pros::delay(5);
 	}
}