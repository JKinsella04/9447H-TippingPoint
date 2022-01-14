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
  pros::delay(750);
  backLift.setState(BackLiftState::DOWN);
  chassis.drive(1050,75,450).withGains(15, 0, 6.25).withAngle(270).withTurnGains(133, 0, 66).withTol(50, 40).waitUntilSettled();
  chassis.reset();
  chassis.turn(0).withTurnGains(133,0,66).withTol(0,5).waitUntilSettled();
  chassis.reset();
  chassis.drive(-4950, 900, 450, 9000).withGains(15, 0, 6.25).withAngle(1).withTurnGains(266, 1, 133).withTol(50, 10).waitUntilSettled();
  backLift.setState(BackLiftState::UP);
  frontLift.setState(FrontLiftState::UP);
  chassis.reset();
  chassis.drive(400).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();  
  chassis.reset();
  chassis.turn(275).withTurnGains(133,0,33).withTol(0,5).waitUntilSettled();
  chassis.reset();
  chassis.drive(1500).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  chassis.reset();
  chassis.drive(-1000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  backLift.setState(BackLiftState::DOWN);
  chassis.reset();
  chassis.turn(290).withTurnGains(266, 0, 66).withTol(0, 5).waitUntilSettled();
  chassis.reset();
  chassis.drive(1750).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  frontLift.setClamp(true).setState(FrontLiftState::MIDDLE).waitUntilSettled();
  // chassis.drive(-2000).withGains(30,0,10).withAngle(205, 322.5).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
  // backLift.setState(BackLiftState::UP);
  // chassis.drive(2000).withGains(15, 0, 6.25).withAngle(270).withTurnGains(133,0,66).withTol(50,5).waitUntilSettled();
}

void elim() {
}

// Skills
void skills(){
}

// Testing
void test() {
  chassis.turn(90).withTurnGains(66,0,33).withTol(0,5).waitUntilSettled();
  chassis.turn(0).withTurnGains(66,0,33).withTol(0,5).waitUntilSettled();
  chassis.turn(180).withTurnGains(66,0,33).withTol(0,5).waitUntilSettled();
  chassis.turn(270).withTurnGains(66,0,33).withTol(0,5).waitUntilSettled();
}