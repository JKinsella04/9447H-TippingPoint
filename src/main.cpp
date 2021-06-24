#include "main.h"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/mobileGoal.hpp"
#include "control/intake.hpp"


void initialize() {
	// Class Initialization
	Chassis chassis;

	//Threads
	pros::Task ChassisController(chassis.start, NULL, "Chassis Controller");
}


void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

	while (true) {
		
		pros::delay(5);
	}
}
