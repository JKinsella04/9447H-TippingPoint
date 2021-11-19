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
    name = "Elim";
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
    elim();
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
  robotPos.setState(PositionTracker::RELATIVE);
  chassis.setBrakeType(COAST);
  mobileGoal.setState(MobileGoalState::SETUP).waitUntilSettled();
  pros::delay(500);
  chassis.eDrive(-20).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP);
  chassis.eDrive(160,75,450).withGains(15, 0, 6.25).withAngles(90, 178).withTurnGains(266, 1, 133).withTol(50, 20).waitUntilSettled();
  pros::delay(250);
  lift.setClamp(true);
  pros::delay(250);
  chassis.eDrive(-30,900).withGains(30,0,10).withTol(20).waitUntilSettled();
  lift.setClamp(false);
  chassis.eDrive(-22.5,900).withGains(30,0,10).withAngle(120).withTurnGains(266,0,133).withTol(70,25).waitUntilSettled();
  chassis.eDrive(55,900).withGains(30,0,10).withAngle(120).withTurnGains(266,0,133).withTol(20,10).waitUntilSettled();
  pros::delay(250);
  lift.setClamp(true);
  chassis.eDrive(-70,300).withGains(30,0,10).withAngle(180).withTurnGains(266,0,133).withTol(30,10).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::DOWN);
  chassis.eDrive(30,900).withGains(30,0,10).withTol(10).waitUntilSettled(); 
}

void elim() {
  robotPos.setState(PositionTracker::RELATIVE);
  chassis.setBrakeType(COAST);
  mobileGoal.setState(MobileGoalState::SETUP);
  chassis.eDrive(-85,1500, 900, 12000).withGains(30,0,10).withAngle(0).withTurnGains(133,0,66).withTol(100,10).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP).waitUntilSettled();
  chassis.eDrive(80,900).withGains(30,0,10).withAngle(135, 322.5).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
  lift.setClamp(true);
  lift.setState(LiftState::MIDDLE);
  chassis.eDrive(-95,750).withGains(30,0,10).withAngle(180, 900).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  // mobileGoal.setState(MobileGoalState::DOWN).waitUntilSettled();
  // chassis.eDrive(-10,900).withGains(30,0,10).withAngle(180, 450).withTurnGains(133, 0, 66).withTol(40,30).waitUntilSettled();
  // chassis.eDrive(15,1000, 12000).withGains(40,0,10).withAngle(270, 450).withTurnGains(133, 0, 66).withTol(40,30).waitUntilSettled();
  // lift.setState(LiftState::DOWN).waitUntilSettled();
  // pros::delay(100);
  // lift.setClamp(false);
  // chassis.eDrive(-130, 100, 900, 12000).withGains(30,0,10).withAngle(295).withTurnGains(266,1,133).withTol(40,20).waitUntilSettled();
  // mobileGoal.setState(MobileGoalState::UP).waitUntilSettled();
  // chassis.eDrive(60,600).withGains(30,0,10).withAngle(90).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
}

// Skills
void skills(){ // No mobileGoal since it blocks GPS sensor!
  robotPos.setState(PositionTracker::RELATIVE);
  chassis.setBrakeType(BRAKE);
  mobileGoal.setState(MobileGoalState::SETUP).waitUntilSettled();
  pros::delay(500);
  chassis.eDrive(-20).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::UP);
  chassis.eDrive(9).withGains(30, 0.1, 10).withTol(20).waitUntilSettled();
  pros::delay(100);
  chassis.turn(90).withGains(266, 0.5, 133).withTol(0,5).waitUntilSettled();
  pros::delay(100);
  chassis.eDrive(85).withGains(30, 0.1, 10).withAngle(90).withTol(60, 10).waitUntilSettled();
  lift.setClamp(true).setState(LiftState::UP);
  chassis.eDrive(60).withGains(30, 0.1, 10).withAngle(175, 100).withTol(80, 5).waitUntilSettled();
  mobileGoal.setState(MobileGoalState::DOWN).waitUntilSettled();
  chassis.eDrive(30).withGains(30, 0.1, 10).withAngle(175).withTol(50, 10).waitUntilSettled();
  pros::delay(500);
  chassis.turn(90).withGains(133, 1, 66).withTol(0,12).waitUntilSettled();
  chassis.eDrive(20).withGains(30, 0.1, 10).withAngle(88).withTol(120, 50, true).waitUntilSettled();
  pros::delay(2000);
  lift.setState(LiftState::DOWN).setClamp(false);
  // First Goal //


  chassis.eDrive(-7).withGains(30, 0.1, 10).withTol(40).waitUntilSettled();
  chassis.turn(5).withGains(266, 0.5, 133).withTol(0,5).waitUntilSettled();
  lift.setState(LiftState::DOWN).waitUntilSettled();
  chassis.eDrive(30).withGains(30, 0.1, 10).withTol(40).waitUntilSettled();
  lift.setClamp(true);
  pros::delay(500);
  lift.setState(LiftState::UP);
  chassis.eDrive(-55).withGains(30, 0.1, 10).withTol(40).waitUntilSettled();
  pros::delay(200);
  chassis.turn(90).withGains(133, 1, 66).withTol(0,15).waitUntilSettled();
  chassis.eDrive(25).withGains(30, 0.1, 10).withAngle(90).withTol(120, 50, true).waitUntilSettled();
  pros::delay(750);
  lift.setClamp(false);
  // Second goal //
  
  
  chassis.eDrive(-32).withGains(20, 0.1, 10).withAngle(90).withTol(40, 20).waitUntilSettled();
  lift.setState(LiftState::DOWN);
  chassis.turn(330).withGains(133, 2, 66).withTol(0,5).waitUntilSettled();
  pros::delay(200);
  chassis.eDrive(40).withGains(30, 0.1, 10).withTol(40).waitUntilSettled();
  lift.setClamp(true);
  pros::delay(200);
  lift.setState(LiftState::UP).waitUntilSettled();  
  chassis.turn(235).withGains(133, 1, 66).withTol(0,12).waitUntilSettled();
  chassis.eDrive(-110).withGains(30, 0.1, 10).withAngle(235, 175).withTol(40, 20).waitUntilSettled();
  chassis.turn(170).withGains(133, 1, 66).withTol(0,12).waitUntilSettled();
  lift.delayClamp(false);
  chassis.eDrive(20, 900, 900, 12000).withGains(30, 0.1, 10).withTol(80).waitUntilSettled();
  pros::delay(500);
  chassis.eDrive(-15).withGains(30, 0.1, 10).withTol(50).waitUntilSettled();
  // THIRD GOAL //


}

// Testing
void test() {
  //   Chassis.drive(20).withGains(.3, 0.01,
  //   .15).withTol(100).waitUntilSettled(); Chassis.turn(90).withGains(133,
  //   0.01, 66).withTol(1).waitUntilSettled();
}