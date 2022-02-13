#include "auton.hpp"
#include "chassis.hpp"
#include "globals.hpp"
#include "backLift.hpp"
#include "positionTracking.hpp"
#include "frontLift.hpp"
#include "misc.hpp"
#include "pros/rtos.hpp"

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
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::UP);
  pros::delay(1000);
  backLift.setState(BackLiftState::DOWN);
  chassis.drive(4750,80,450).withGains(15, 0, 6.25).withAngles(270,179).withTurnGains(133, 0, 66).withTol(50, 20).waitUntilSettled();
  chassis.turn(0,450).withTurnGains(66,0.2,33).withTol(0,2.5).waitUntilSettled();
  chassis.drive(-1000).withGains(15, 0, 6.25).withTol(50, 10, true).waitUntilSettled();  
  backLift.setState(BackLiftState::UP);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(275).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();  
  chassis.turn(270).withTurnGains(133,0,33).withTol(0,5).waitUntilSettled();
  chassis.drive(1250, 450, 450, 7500).withAngle(270).withGains(15, 0, 6.25).withTol(50, 10).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  chassis.drive(-1000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  backLift.setState(BackLiftState::DOWN);
  // AWP BONUS
  chassis.turn(285).withTurnGains(300, 0, 150).withTol(0, 5).waitUntilSettled();
  chassis.drive(1300,1500,4000,12000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  frontLift.setClamp(true);
  pros::delay(100);
  frontLift.setState(FrontLiftState::UP);
  chassis.turn(125, 900, 12000).withTurnGains(133,.25,66).swingTurn(RIGHT).withTol(0,10).waitUntilSettled();
  chassis.drive(-1500).withGains(15, 0, 6.25).withAngle(125).withTol(50,10).waitUntilSettled();
  backLift.setState(BackLiftState::UP, 0);
  chassis.drive(2000).withGains(15, 0, 6.25).withAngle(180, 50).withTurnGains(50,0,25).withTol(40,20).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  // YELLOW
}

void elim() {
  frontLift.withTol(10).setState(FrontLiftState::DOWN);
  chassis.setBrakeType(COAST);
  chassis.drive(2275,1500, 4000, 12000).withGains(15, 0, 6.25).withAngle(359).withTurnGains(133,0,66).withTol(50,20).waitUntilSettled();
  frontLift.setClamp(true);
  pros::delay(100);
  frontLift.withTol(75).setState(FrontLiftState::UP);
  chassis.turn(225, 900, 12000).withTurnGains(133,.25,66).swingTurn(RIGHT).withTol(0,10).waitUntilSettled();
  chassis.drive(-1050).withGains(15, 0, 6.25).withAngle(225).withTol(50,10).waitUntilSettled();
  backLift.setState(BackLiftState::UP, 0);
  chassis.drive(2000).withGains(15, 0, 6.25).withAngle(225, 50).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  chassis.turn(40, 450).withTurnGains(133,0,133).withTol(0,2).waitUntilSettled();
  chassis.drive(-1750).withGains(15, 0, 6.25).withAngle(45).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  backLift.setState(BackLiftState::DOWN, 0);
  chassis.drive(1000).withGains(15, 0, 6.25).withAngle(45).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  chassis.turn(90).withTurnGains(133,0,66).withTol(0,10).waitUntilSettled();
  chassis.drive(-500).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  backLift.setState(BackLiftState::UP, 0);
  frontLift.setState(FrontLiftState::DOWN);
  }

// Skills
void skills(){
  chassis.setBrakeType(COAST);
  frontLift.setState(FrontLiftState::DOWN);
  backLift.setState(BackLiftState::UP);
  chassis.turn(250, 900, 12000).withTurnGains(133,.25,66).swingTurn(1).withTol(0,10).waitUntilSettled();
  chassis.drive(1950,1125,2250,9000).withGains(15, 0, 6.25).withAngle(250).withTurnGains(133,0,66).withTol(50, 5).waitUntilSettled();  
  frontLift.setClamp(true);
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP).waitUntilSettled();
  chassis.drive(2900).withGains(15, 0, 6.25).withAngle(235).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();  
  chassis.drive(-800).withGains(15, 0, 6.25).withAngle(180).withTurnGains(66,0.25,33).withTol(50,2.5).waitUntilSettled();
  chassis.drive(1800,450, 6000).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,5).waitUntilSettled();
  chassis.drive(-500).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,5).waitUntilSettled();
  chassis.turn(270,450).withTurnGains(133,0.2,33).withTol(0,2.5).waitUntilSettled();
  chassis.drive(700).withGains(15, 0, 6.25).withAngle(270).withTurnGains(133,0,66).withTol(50,20).waitUntilSettled();
  frontLift.setClamp(false);
  // FIRST GOAL
  chassis.drive(-500).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  pros::delay(200);
  chassis.turn(180,450).withTurnGains(133,0.2,66).withTol(0,2.5).waitUntilSettled();
  backLift.setState(BackLiftState::DOWN);
  frontLift.setState(FrontLiftState::DOWN);
  chassis.drive(-300).withGains(15, 0, 6.25).withTol(50);
  chassis.drive(500).withGains(15, 0, 6.25).withTol(50);
  pros::delay(250);
  chassis.turn(0,450).withTurnGains(133,0,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(1200).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  frontLift.setClamp(true);
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP).waitUntilSettled();
  chassis.turn(270,450).withTurnGains(133,0.2,33).withTol(0,2.5).waitUntilSettled();
  frontLift.delayClamp(false);
  chassis.drive(850).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  pros::delay(250);
  chassis.drive(-425).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  // SECOND GOAL
  chassis.turn(180,450).withTurnGains(133,0.2,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(-2100).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50, 2.5, true).waitUntilSettled();
  pros::delay(250);
  backLift.setState(BackLiftState::UP, 100);
  pros::delay(250);
  chassis.drive(2700).withGains(15, 0, 6.25).withAngle(135).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.drive(-250).withGains(15, 0, 6.25).withAngle(135).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN).waitUntilSettled();
  chassis.drive(700).withGains(15, 0, 6.25).withAngle(135).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setClamp(true);
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(3100).withGains(15, 0, 6.25).withAngle(120).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.turn(90,900).withTurnGains(166,0.25,66).withTol(0,3).waitUntilSettled();
  pros::delay(750);

  chassis.setBrakeType(HOLD);
  chassis.drive(800, 450,450, 3000).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.drive(-1100, 450,450, -3000).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  pros::delay(750);
  chassis.drive(1000, 450,450, 3000).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.drive(-1200, 450,450, -3000).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  pros::delay(750);
  chassis.setBrakeType(COAST);
  chassis.drive(900).withGains(15, 0, 6.25).withAngle(120).withTurnGains(133,0,66).withTol(50,20).waitUntilSettled();
  
  chassis.turn(15,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  frontLift.delayClamp(false);
  chassis.drive(950, 1500,900, 12000).withGains(15, 0, 6.25).withAngle(15).withTurnGains(133,0,66).withTol(100,10);
  pros::delay(1500);
  // THIRD GOAL
  chassis.drive(-1000).withGains(15, 0, 6.25).withAngle(0).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.turn(315,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(3000).withGains(15, 0, 6.25).withAngle(0, 100).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  backLift.setState(BackLiftState::DOWN);
  frontLift.setState(FrontLiftState::DOWN);
  pros::delay(500);
  chassis.drive(400).withGains(15, 0, 6.25).withAngle(0, 100).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.turn(180,450).withTurnGains(66,0.25,33).withTol(0,2.5).waitUntilSettled();
  chassis.drive(1000).withGains(15, 0, 6.25).withAngle(178, 100).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setClamp(true);
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(-800).withGains(15, 0, 6.25).withAngle(180, 100).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.turn(90,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  frontLift.delayClamp(false);
  chassis.drive(400).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10);
  pros::delay(1500);
  // FOURTH GOAL
  chassis.drive(-1500).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  backLift.setState(BackLiftState::UP);
  chassis.turn(180,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(1800).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.drive(-250).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN).waitUntilSettled();
  chassis.drive(700).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setClamp(true);
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP);
  chassis.turn(50,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(3000).withGains(15, 0, 6.25).withAngle(50).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setClamp(false);
  pros::delay(500);
  chassis.drive(-750).withGains(15, 0, 6.25).withAngle(50).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  // FIFTH GOAL
}

// Testing
void test() {
}