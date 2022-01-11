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
  pros::delay(200);
  backLift.setState(BackLiftState::DOWN);
  chassis.drive(1050,75,450).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133, 0, 66).withTol(50, 5).waitUntilSettled();
  chassis.turn(0).withTurnGains(133,0,66).withTol(0,5).waitUntilSettled();
  chassis.drive(-5050, 900, 450, 12000).withGains(15, 0, 6.25).withAngle(0).withTurnGains(266, 1, 133).withTol(50, 10).waitUntilSettled();
  backLift.setState(BackLiftState::UP);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(400).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();  
  chassis.turn(90).withTurnGains(133,0,66).withTol(0,5).waitUntilSettled();
  chassis.drive(1500).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  chassis.drive(-1000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  backLift.setState(BackLiftState::DOWN);
  // backLift.setState(BackLiftState::DOWN);
  // chassis.drive(3000,75,450).withAngle(100).withGains(30,0,15).withTurnGains(166,0,66).withTol(45,5).waitUntilSettled();
  // frontLift.setClamp(true).setState(FrontLiftState::MIDDLE).waitUntilSettled();
  // chassis.drive(-2350, 800).withGains(30,0,10).withAngle(330, 1000, 12000).withTurnGains(133,0,66).withTol(45,10).waitUntilSettled();
  // backLift.setState(BackLiftState::UP);
  // chassis.drive(2000, 800).withGains(30,0,10).withAngle(180, 900, 12000).withTurnGains(133,0,66).withTol(45,10).waitUntilSettled();
  // backLift.setState(BackLiftState::DOWN);
  // chassis.drive(-200).withGains(30,0,7.5).withTol(45).waitUntilSettled();
  // chassis.turn(0).withGains(133,0,66).waitUntilSettled();
  // chassis.drive(-1500).withGains(30,0,7.5).withTol(45).waitUntilSettled();
  // backLift.setState(BackLiftState::UP);
  // chassis.drive(2000, 450).withGains(30,0,15).withAngle(270).withTurnGains(133,0,66).waitUntilSettled();
}

void elim() {
}

// Skills
void skills(){
}

// Testing
void test() {
  robotPos.setState(PositionTracker::ODOM);
  chassis.drive({500,500}, {250,250}).waitUntilSettled();
}