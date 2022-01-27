#include "auton.hpp"
#include "chassis.hpp"
#include "globals.hpp"
#include "backLift.hpp"
#include "positionTracking.hpp"
#include "frontLift.hpp"
#include "misc.hpp"

// Class Init
Position robotPos;
static Chassis chassis;
BackLift backLift;
FrontLift frontLift;

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
  backLift.setState(BackLiftState::UP);
  pros::delay(1000);
  backLift.setState(BackLiftState::DOWN);
  chassis.drive(4700,85,450).withGains(15, 0, 6.25).withAngles(270,179).withTurnGains(133, 0, 66).withTol(50, 20).waitUntilSettled();
  chassis.turn(0).withTurnGains(133,0,66).withTol(0,5).waitUntilSettled();
  chassis.drive(-750).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();  
  backLift.setState(BackLiftState::UP);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(275).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();  
  chassis.turn(270).withTurnGains(133,0,33).withTol(0,5).waitUntilSettled();
  chassis.drive(1500, 450, 450, 6000).withAngle(270).withGains(15, 0, 6.25).withTol(50, 10).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  chassis.drive(-1000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  backLift.setState(BackLiftState::DOWN);
  chassis.turn(100).withTurnGains(133, 0, 66).withTol(0, 5).waitUntilSettled();
  chassis.drive(-1000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  backLift.setState(BackLiftState::UP, 0);
  pros::delay(100);
  chassis.turn(320, 900, 12000).withTurnGains(133,.25,66).swingTurn(1).withTol(0,10).waitUntilSettled();
  chassis.drive(1050).withGains(15, 0, 6.25).withAngle(320).withTol(50,10).waitUntilSettled();
  frontLift.setClamp(true).setState(FrontLiftState::MIDDLE).waitUntilSettled();
  chassis.drive(-2500).withGains(15, 0, 6.25).withAngle(270, 200).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  // chassis.drive(2000).withGains(15, 0, 6.25).withAngle(270).withTurnGains(133,0,66).withTol(50,5).waitUntilSettled();
}

void elim() {
  chassis.setBrakeType(COAST);
  chassis.drive(-2300,1500, 3000, 12000).withGains(15, 0, 6.25).withAngle(359).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  backLift.setState(BackLiftState::UP, 0);
  pros::delay(100);
  chassis.turn(225, 900, 12000).withTurnGains(133,.25,66).swingTurn(1).withTol(0,10).waitUntilSettled();
  chassis.drive(1050).withGains(15, 0, 6.25).withAngle(225).withTol(50,10).waitUntilSettled();
  frontLift.setClamp(true).setState(FrontLiftState::MIDDLE).waitUntilSettled();
  chassis.drive(-2500).withGains(15, 0, 6.25).withAngle(180, 200).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  }

// Skills
void skills(){
robotPos.setState(PositionTracker::GPS);
}

// Testing
void test() {
}