#include "main.h"
#include "control/misc.hpp"
#include "control/chassis.hpp"
#include "control/mobileGoal.hpp"
#include "control/intake.hpp"
#include "control/gui.hpp"
#include "control/auton.hpp"
#include "control/odometry.hpp"
#include "control/purePursuit.hpp"
#include <iostream>

// Class Init
Odometry Odom;

void initialize() {
	Chassis Chassis;
 	Display Display;

	// Sensor Init

    // Threads
	pros::Task OdometryController(Odom.start, NULL, "Odom Controller");

	pros::Task ChassisController(Chassis.start, NULL, "Chassis Controller");
	Chassis.setBrakeType(HOLD);

	pros::Task DisplayController(Display.start, NULL, "Display Controller");
	DisplayController.set_priority(TASK_PRIORITY_MIN);


}


void disabled() {}

void competition_initialize() {}

void autonomous() {
	Autonomous Auton;
	Auton.runAuton();
}

void opcontrol() {
  PurePursuit PurePursuit;
  Chassis Chassis;
  Chassis.setState(ChassisState::OPCONTROL);
  Chassis.setBrakeType(COAST);

  while (true) {
    PurePursuit.goToPoint(1000, 1000, 200);
    pros::delay(5);
	}
}


double PurePursuit::distToTarget, PurePursuit::absAngleToTarget, PurePursuit::relAngleToTarget; 
double PurePursuit::relXToPoint, PurePursuit::relYToPoint;
double PurePursuit::mvmtXPower, PurePursuit::mvmtYPower; 

void PurePursuit::goToPoint(double target_x, double target_y, double speed){
    distToTarget = hypot(target_x - Odom.getX(), target_y- Odom.getX());

    absAngleToTarget = atan2(target_y-Odom.getY(), target_x-Odom.getX());

    relAngleToTarget = absAngleToTarget - Odom.getThetaRad();

    relXToPoint = cos(relAngleToTarget) * distToTarget;
    relYToPoint = sin(relAngleToTarget) * distToTarget;

    mvmtXPower = relXToPoint / ( fabs(relXToPoint) + fabs(relYToPoint));
    mvmtYPower = relYToPoint / ( fabs(relXToPoint) + fabs(relYToPoint));


	double leftPower = mvmtYPower + mvmtXPower;
	double rightPower = mvmtYPower - mvmtXPower;

    std::cout << "LPow:" << leftPower << "RPow:" << rightPower << std::endl;

	// LF.move(leftPower * 1000);
	// LB.move(leftPower * 1000);
	// RF.move(rightPower * 1000);
	// RB.move(rightPower * 1000);

}