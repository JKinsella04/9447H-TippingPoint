#include "auton.hpp"
#include "chassis.hpp"
#include "mobileGoal.hpp"
#include "positionTracking.hpp"
#include "lift.hpp"
#include "misc.hpp"
#include "pros/rtos.hpp"

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
  robotPos.setState(PositionTracker::RELATIVE);
  chassis.setBrakeType(COAST);
  mobileGoal.setup().waitUntilSettled();
  chassis.eDrive(-20).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP);
  chassis.eDrive(160,75,450).withGains(15, 0, 6.25).withAngles(90, 180).withTurnGains(266, 1, 133).withTol(50, 20).waitUntilSettled();
  pros::delay(250);
  clamp.set_value(true);
  pros::delay(250);
  chassis.eDrive(-30,900).withGains(30,0,10).withTol(20).waitUntilSettled();
  clamp.set_value(false);
  chassis.eDrive(-22.5,900).withGains(30,0,10).withAngle(120).withTurnGains(266,0,133).withTol(50,20).waitUntilSettled();
  chassis.eDrive(55,900).withGains(30,0,10).withAngle(120).withTurnGains(266,0,133).withTol(20,10).waitUntilSettled();
  pros::delay(250);
  clamp.set_value(true);
  chassis.eDrive(-70,300).withGains(30,0,10).withAngle(180).withTurnGains(266,0,133).withTol(30,10).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::DOWN);
  chassis.eDrive(30,900).withGains(30,0,10).withTol(10).waitUntilSettled();
  
}

void oneGoal() { }

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
  chassis.eDrive(-10,900).withGains(30,0,10).withAngle(180, 450).withTurnGains(133, 0, 66).withTol(40,30).waitUntilSettled();
  chassis.eDrive(15,1000, 12000).withGains(40,0,10).withAngle(270, 450).withTurnGains(133, 0, 66).withTol(40,30).waitUntilSettled();
  lift.setState(LiftState::DOWN).waitUntilSettled();
  pros::delay(100);
  clamp.set_value(false);
  chassis.eDrive(-130, 100, 900, 12000).withGains(30,0,10).withAngle(295).withTurnGains(266,1,133).withTol(40,20).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP).waitUntilSettled();
  chassis.eDrive(60,600).withGains(30,0,10).withAngle(90).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
}

// Skills
void skills(){ // No mobileGoal since it blocks GPS sensor!
  chassis.setBrakeType(COAST);
  robotPos.setState(PositionTracker::RELATIVE);
  mobileGoal.setup();
  chassis.eDrive(10).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  clamp.set_value(true);
  lift.setState(LiftState::UP);
  chassis.eDrive(-10).withGains(30,0,15).withTol(40).waitUntilSettled();
  chassis.eDrive(-95, 60, 450).withGains(15, 0, 6.25).withAngle(94, 1000, 12000).withTurnGains(266,0.1,133).withTol(70, 10).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::MIDDLE);
  chassis.eDrive(-65, 350).withGains(30, 0, 6.25).withAngle(177.5, 900, 9000).withTurnGains(266,0,133).withTol(100, 5).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::DOWN);
  chassis.eDrive(-20).withGains(60, .5, 10).withAngle(180).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
  chassis.eDrive(3.75,900).withGains(30,.75,10).withAngle(180).withTurnGains(266,0,133).withTol(30,10).waitUntilSettled();
  chassis.turn(270).withTurnGains(266,0,133).withTol(5,10).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP);
  chassis.eDrive(15,900).withGains(30,0,10).withAngle(270).withTurnGains(266,0,133).withTol(160,15).waitUntilSettled();
  pros::delay(1000);
  clamp.set_value(false);
  chassis.eDrive(-7,900).withGains(30,0,10).withAngle(360, 350).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
  lift.setState(LiftState::DOWN).waitUntilSettled();
  chassis.eDrive(40,900).withGains(30,0,10).withTol(40).waitUntilSettled();
  clamp.set_value(true);
  lift.setState(LiftState::UP).waitUntilSettled();
  chassis.eDrive(-29,900).withGains(30,0,10).withAngle(360, 350).withTurnGains(133,0,66).withTol(30,10).waitUntilSettled();
  chassis.turn(270).withTurnGains(266,0,133).withTol(5,5).waitUntilSettled();
  chassis.eDrive(17.5,900).withGains(30,0,10).withAngle(270).withTurnGains(266,0,133).withTol(150,15).waitUntilSettled();
  pros::delay(1000);
  clamp.set_value(false);
  chassis.eDrive(-4,900, 12000).withGains(30,0,10).withTol(10,15).waitUntilSettled();
  chassis.turn(180).withTurnGains(266, 0, 133).withTol(5, 5).waitUntilSettled();
  lift.setState(LiftState::DOWN).waitUntilSettled();
  chassis.eDrive(90,900).withGains(30,0,10).withAngle(182.5, 350).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  pros::delay(500);
  clamp.set_value(true);
}

// Testing
void test() {
  //   Chassis.drive(20).withGains(.3, 0.01,
  //   .15).withTol(100).waitUntilSettled(); Chassis.turn(90).withGains(133,
  //   0.01, 66).withTol(1).waitUntilSettled();
}