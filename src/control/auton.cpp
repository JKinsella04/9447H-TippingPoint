#include "auton.hpp"
#include "chassis.hpp"
#include "globals.hpp"
#include "backLift.hpp"
#include "positionTracking.hpp"
#include "frontLift.hpp"
#include "misc.hpp"

// Class Init
static Position robotPos;
static Chassis chassis;
static BackLift backLift;
static FrontLift frontLift;

std::string Autonomous::name;

int Autonomous::id;

Autonomous::Autonomous() {}

std::string Autonomous::getAuton() { return name; }

void Autonomous::setId(int id_) {
  id = id_;
  switch (id) {
  case 1:
    name = "Left AWP";
    break;
  case 2:
    name = "Right AWP";
    break;
  case 3:
    name = "Full AWP";
    break;
  case 4:
    name = "Middle Goal";
    break;
  case 5:
    name = "Two Goal";
    break;
  case 6:
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
    leftAWP();
    break;
  }
  case 2: {
    rightAWP();
    break;
  }
  case 3: {
    fullAwp();
    break;
  }

  case 4: {
    middleGoal();
    break;
  }
  
  case 5: {
    elim();
    break;
  }

  case 6: {
    skills();
    break;
  }

  default: {
    test();
    break;
  }
  }
}

// Autons
void leftAWP() { // One yellow + left AWP
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.withTol(10).setState(FrontLiftState::DOWN);
  chassis.drive(2200, 1500, 450, 12000).withGains(15, 0, 6.25).withAngle(355, 900, 12000).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100);
  pros::delay(50);
  chassis.drive(-2400, 1500, 900, -12000).withGains(15, 0, 6.25).withAngle(15).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.turn(90,450, -4500).withTurnGains(133,0.2,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(-1500, 900, 900, -5500).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(40,5, true).waitUntilSettled();
  backLift.toggleClamp();
  chassis.drive(1000, 900, 900, 3000).withGains(15, 0, 6.25).withAngle(105).withTurnGains(133,0,66).withTol(40,10).waitUntilSettled();
  chassis.drive(-1100, 900, 900, -3000).withGains(15, 0, 6.25).withAngle(105).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.drive(900, 900, 900, 3000).withGains(15, 0, 6.25).withAngle(105).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.drive(-900, 900, 900, -3000).withGains(15, 0, 6.25).withAngle(105).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.drive(900, 900, 900, 3000).withGains(15, 0, 6.25).withAngle(105).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.drive(-900).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
}

void rightAWP(){ // One yellow + right AWP
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.withTol(10).setState(FrontLiftState::DOWN);
  chassis.drive(2000, 1500, 450, 12000).withGains(15, 0, 6.25).withAngle(358).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100);
  pros::delay(50);
  chassis.drive(-1600, 1500, 900, -12000).withGains(15, 0, 6.25).withAngle(358).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.turn(90,450, -6000).withTurnGains(133,0.2,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(-1300, 900, 900, -6000).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(40,5, true).waitUntilSettled();
  backLift.toggleClamp();
  pros::delay(150);
  chassis.drive(300).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.turn(0,450).withTurnGains(133,0.2,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(2000).withGains(15, 0, 6.25).withAngle(0).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  chassis.drive(-2300, 1500, 3000, -12000).withGains(15, 0, 6.25).withAngle(0).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
}

void fullAwp() { // FULL AWP
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  backLift.toggleClamp();
  pros::delay(1000);
  backLift.delayClamp(1.5);
  chassis.drive(4750,125,450).withGains(15, 0, 6.25).withAngles(270,178).withTurnGains(133, 0, 66).withTol(50, 20).waitUntilSettled();
  chassis.turn(0,450).withTurnGains(66,0.2,33).withTol(0,2.5).waitUntilSettled();
  chassis.drive(-1000, 450, 450, -5500).withGains(15, 0, 6.25).withTol(50, 10, true).waitUntilSettled();  
  backLift.toggleClamp();
  pros::delay(100);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(275).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();  
  chassis.turn(270).withTurnGains(133,0,33).withTol(0,5).waitUntilSettled();
  chassis.drive(1250, 450, 450, 7500).withAngle(270).withGains(15, 0, 6.25).withTol(50, 10).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  chassis.drive(-1000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  // AWP BONUS
  chassis.turn(285).withTurnGains(300, 0, 150).withTol(0, 5).waitUntilSettled();
  chassis.drive(1300,1500,4000,12000).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100);
  chassis.turn(250).withTurnGains(300, 0, 150).withTol(0, 5).waitUntilSettled();
  chassis.drive(-1500).withGains(15, 0, 6.25).withAngle(250).withTol(50,20).waitUntilSettled();
  // YELLOW
}

void middleGoal(){ // Gets Middle Neutral Goal.
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.toggleClamp().setState(FrontLiftState::DOWN);
  chassis.drive(2800, 1500, 450, 12000).withGains(15, 0, 6.25).withAngle(359).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::MIDDLE, 150);
  chassis.drive(-2400, 1500, 3000, -12000).withGains(15, 0, 6.25).withAngle(0).withTurnGains(133,0,66).withTol(40,5).waitUntilSettled();
}

void elim() { // Right Side Neutral Goals.
  frontLift.toggleClamp();
  frontLift.withTol(10).setState(FrontLiftState::DOWN);
  chassis.setBrakeType(COAST);
  chassis.drive(2100,1500, 4000, 12000).withGains(15, 0, 6.25).withAngle(359).withTurnGains(133,0,66).withTol(50,20).waitUntilSettled();
  frontLift.toggleClamp();
  frontLift.withTol(75).setState(FrontLiftState::UP);
  chassis.turn(225, 900, 12000).withTurnGains(133,.25,66).swingTurn(RIGHT).withTol(0,10).waitUntilSettled();
  chassis.drive(-1050).withGains(15, 0, 6.25).withAngle(225).withTol(50,10).waitUntilSettled();
  backLift.toggleClamp();
  chassis.drive(2000).withGains(15, 0, 6.25).withAngle(225, 50).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  chassis.turn(40, 450).withTurnGains(133,0,133).withTol(0,2).waitUntilSettled();
  chassis.drive(-1750).withGains(15, 0, 6.25).withAngle(45).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  backLift.toggleClamp();
  chassis.drive(1000).withGains(15, 0, 6.25).withAngle(45).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  chassis.turn(90).withTurnGains(133,0,66).withTol(0,10).waitUntilSettled();
  chassis.drive(-500).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(40,20).waitUntilSettled();
  backLift.toggleClamp();
  frontLift.setState(FrontLiftState::DOWN);
}

void skills(){ // Skills
  chassis.setBrakeType(COAST);
  frontLift.setState(FrontLiftState::DOWN);
  backLift.toggleClamp();
  chassis.turn(250, 900, 12000).withTurnGains(133,.25,66).swingTurn(1).withTol(0,10).waitUntilSettled();
  chassis.drive(1950,1125,2250,9000).withGains(15, 0, 6.25).withAngle(250).withTurnGains(133,0,66).withTol(50, 5).waitUntilSettled();  
  frontLift.toggleClamp();
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP).waitUntilSettled();
  chassis.drive(2900).withGains(15, 0, 6.25).withAngle(235).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();  
  chassis.drive(-800).withGains(15, 0, 6.25).withAngle(180).withTurnGains(66,0.25,33).withTol(50,2.5).waitUntilSettled();
  chassis.drive(1800,450, 6000).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,5).waitUntilSettled();
  chassis.drive(-500).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,5).waitUntilSettled();
  chassis.turn(270,450).withTurnGains(133,0.2,33).withTol(0,2.5).waitUntilSettled();
  chassis.drive(700).withGains(15, 0, 6.25).withAngle(270).withTurnGains(133,0,66).withTol(50,20).waitUntilSettled();
  frontLift.toggleClamp();
  // FIRST GOAL
  chassis.drive(-500).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  pros::delay(200);
  chassis.turn(180,450).withTurnGains(133,0.2,66).withTol(0,2.5).waitUntilSettled();
  backLift.toggleClamp();
  frontLift.setState(FrontLiftState::DOWN);
  chassis.drive(-300).withGains(15, 0, 6.25).withTol(50);
  chassis.drive(500).withGains(15, 0, 6.25).withTol(50);
  pros::delay(250);
  chassis.turn(0,450).withTurnGains(133,0,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(1200).withGains(15, 0, 6.25).withTol(50).waitUntilSettled();
  frontLift.toggleClamp();
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
  backLift.toggleClamp();
  pros::delay(250);
  chassis.drive(2700).withGains(15, 0, 6.25).withAngle(135).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.drive(-250).withGains(15, 0, 6.25).withAngle(135).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN).waitUntilSettled();
  chassis.drive(700).withGains(15, 0, 6.25).withAngle(135).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.toggleClamp();
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
  backLift.toggleClamp();
  frontLift.setState(FrontLiftState::DOWN);
  pros::delay(500);
  chassis.drive(400).withGains(15, 0, 6.25).withAngle(0, 100).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.turn(180,450).withTurnGains(66,0.25,33).withTol(0,2.5).waitUntilSettled();
  chassis.drive(1000).withGains(15, 0, 6.25).withAngle(178, 100).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.toggleClamp();
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(-800).withGains(15, 0, 6.25).withAngle(180, 100).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.turn(90,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  frontLift.delayClamp(false);
  chassis.drive(400).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10);
  pros::delay(1500);
  // FOURTH GOAL
  chassis.drive(-1500).withGains(15, 0, 6.25).withAngle(90).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  backLift.toggleClamp();
  chassis.turn(180,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(1800).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  chassis.drive(-250).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN).waitUntilSettled();
  chassis.drive(700).withGains(15, 0, 6.25).withAngle(180).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.toggleClamp();
  pros::delay(250);
  frontLift.setState(FrontLiftState::UP);
  chassis.turn(50,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  chassis.drive(3000).withGains(15, 0, 6.25).withAngle(50).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  frontLift.toggleClamp();
  pros::delay(500);
  chassis.drive(-750).withGains(15, 0, 6.25).withAngle(50).withTurnGains(133,0,66).withTol(50,10).waitUntilSettled();
  // FIFTH GOAL
}

void test() { // Testing
}