#include "main.h"

#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/odometry.hpp"
// #include "control/lift.hpp"
#include <stdint.h>

void initialize() {
	// Class Init
	Odometry odom;	 
	Chassis chassis(odom.getL(), odom.getThetaDeg(), odom.getX(), odom.getY());
 	Display display;
	// Lift lift;

	// Sensor Init
	// liftPos.calibrate();
	OdomL.reset_position();
	OdomS.reset_position();
	OdomL.set_reversed(false);

	// L_IMU.reset();
	M_IMU.reset(); 
	// R_IMU.reset();	
  	// while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(20); } // Comment out if no IMUs on robot! Will block rest of code!

  // Threads
	pros::Task OdometryController(odom.start, NULL, "Odom Controller");

	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
	chassis.setBrakeType(HOLD);

	// pros::Task LiftController(lift.start, NULL, "Lift Controller");

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

	double lastAccel = 0;
	double lastTime = pros::c::millis();
	double lastVelocity = 0;
	double lastPosition = 0;
	double velocity = 0;
	double position = 0;


  while (true) {
	pros::c::imu_accel_s_t accel = M_IMU.get_accel();


	if(accel.x >= 0.02){
	velocity = (lastVelocity + ( ( lastAccel + accel.x) /2 ) *(lastTime - pros::c::millis()));
	position = (lastPosition + ( ( lastVelocity + velocity) /2 ) *(lastTime - pros::c::millis()));
	lastPosition = position;
	lastAccel = accel.x;
	lastTime = pros::c::millis();
	lastVelocity = velocity;
	}
	// if(accel.y <= 0.05) accel.y = 0;
	std::cout << "Pos:" << position << std::endl;
    pros::delay(5);
 	}
}