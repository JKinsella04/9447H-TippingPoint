#include "auton.hpp"
#include "chassis.hpp"
#include "mobileGoal.hpp"
#include "positionTracking.hpp"
#include "lift.hpp"
#include "misc.hpp"

// Class Init
Position robotPos;
static Chassis chassis;
MobileGoal mobileGoal;
Lift lift;

std::string Autonomous::name;

int Autonomous::id;

Autonomous::Autonomous() {}

std::string Autonomous::getAuton() { return name; }

void Autonomous::setId(int id_) {
  id = id_;
  switch (id) {
  case 1:
    name = "AWP";
    break;
  case 2:
    name = "Two Goal";
    break;
  case 3:
    name = "Skills";
    break;
  default:
    name = "ERROR INVALID AUTON";
    break;
  }
}

void Autonomous::runAuton() {
  switch (id) {
  case 1: {
    awp();
    break;
  }
  case 2: {
    twoGoal();
    break;
  }
  case 3: {
    skills();
    break;
  }

  default: {
    break;
  }
  }
}

// Match Autons
void awp() {
  // gps.initialize_full(intX, intX, intTheta, offX, offY);
  chassis.setBrakeType(COAST);
  mobileGoal.setup().waitUntilSettled();
  chassis.eDrive(-20).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP);
  chassis.eDrive(155,75,450).withGains(15, 0, 6.25).withAngles(90, 180).withTurnGains(266, 1, 133).withTol(50, 20).waitUntilSettled();
  pros::delay(250);
  clamp.set_value(true);
  pros::delay(250);
  chassis.eDrive(-30,900).withGains(30,0,10).withTol(20).waitUntilSettled();
  clamp.set_value(false);
  chassis.eDrive(-25,900).withGains(30,0,10).withAngle(120).withTurnGains(266,0,133).withTol(25,10).waitUntilSettled();
  chassis.eDrive(55,900).withGains(30,0,10).withAngle(120).withTurnGains(266,0,133).withTol(20,10).waitUntilSettled();
  pros::delay(250);
  clamp.set_value(true);
  chassis.eDrive(-65,300).withGains(30,0,10).withAngle(180).withTurnGains(266,0,133).withTol(30,10).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::DOWN);
  chassis.eDrive(30,900).withGains(30,0,10).withTol(10).waitUntilSettled();
  
}

void oneGoal() {
  chassis.setBrakeType(COAST);
  // chassis.eDrive(-20).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  // chassis.eDrive(65).withGains(30, 0, 10).withAngles(90, 0).withTurnGains(133, 0.01, 66).withTol(20, 20).waitUntilSettled();
  // chassis.turn(0).withTurnGains(133, 0.1, 66).withTol(1).waitUntilSettled();
  // chassis.drive(-250, 0, 345, 900, 9000, 900, 9000).withGains(9000,0,5).withTurnGains(10,0,5).withTol(50).waitUntilSettled();
  // chassis.drive(800, -940, 0, 900, 9000, 900, 9000).withGains(9000,0,3000).withTurnGains(12000,0,6000).withTol(50).waitUntilSettled();
  // chassis.drive(0, 0, 900, 9000, 900, 9000).withGains(20,0,10).withTurnGains(10,0,5).withTol(20).waitUntilSettled();
  // chassis.drive(1000, 250, 900, 9000, 900, 9000).withGains(20,0,10).withTurnGains(10,0,5).withTol(20).waitUntilSettled();
}

void twoGoal() {
  chassis.setBrakeType(COAST);
  mobileGoal.setup();
  chassis.eDrive(-85,1500, 900, 12000).withGains(30,0,10).withAngle(0).withTurnGains(133,0,66).withTol(100,10).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP).waitUntilSettled();
  // pros::delay(200);
  chassis.eDrive(80,900).withGains(30,0,10).withAngle(135, 322.5).withTurnGains(133,0,66).withTol(20,10).waitUntilSettled();
  clamp.set_value(true);
  lift.setState(LiftState::MIDDLE);
  chassis.eDrive(-95,750).withGains(30,0,10).withAngle(180, 900).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::DOWN).waitUntilSettled();
  chassis.eDrive(-10,900).withGains(30,0,10).withAngle(180, 450).withTurnGains(133, 0, 66).withTol(20,30).waitUntilSettled();
  chassis.eDrive(15,1000, 12000).withGains(40,0,10).withAngle(270, 450).withTurnGains(133, 0, 66).withTol(40,30).waitUntilSettled();
  lift.setState(LiftState::DOWN).waitUntilSettled();
  pros::delay(100);
  clamp.set_value(false);
  chassis.eDrive(-130, 100, 900, 12000).withGains(30,0,10).withAngle(300).withTurnGains(266,1,133).withTol(40,20).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP).waitUntilSettled();
  chassis.eDrive(60,600).withGains(30,0,10).withAngle(90).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
}

// Skills
void skills(){ // No mobileGoal since it blocks GPS sensor!
  // gps.initialize_full(0, 0, 180, 0, -0.3048);
  robotPos.setState(PositionTracker::GPS);
  chassis.setBrakeType(COAST);
  chassis.drive({0, 0}).withGains(6,0,0).withTurnGains(400,0,0).withTol(300).waitUntilSettled();
// chassis.eDrive(-20).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  // clamp.set_value(true);
  // chassis.eDrive(20).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  // chassis.drive({0,900});
  // chassis.drive({1200, 0});
  // chassis.drive({1200, -900});
  // chassis.drive({0, -900});
  // chassis.drive({-1200, 0});
  // chassis.drive({-900,-1500});
  // chassis.drive({0,0});
  // chassis.drive({1500, 900});
  // chassis.drive({900, 1500});
  // chassis.drive({-1500, 1500});
}

// Testing
void test() {
  //   Chassis.drive(20).withGains(.3, 0.01,
  //   .15).withTol(100).waitUntilSettled(); Chassis.turn(90).withGains(133,
  //   0.01, 66).withTol(1).waitUntilSettled();
}