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
  // frontLift.toggleClamp();
  // chassis.setBrakeType(COAST);
  // backLift.setState(BackLiftState::AUTON);
  // frontLift.withTol(10).setState(FrontLiftState::DOWN);
  // chassis.drive(2200, 1500, 450, 12000).withAngle(355_deg, 900, 12000).withTol(40,5).waitUntilSettled();
  // frontLift.toggleClamp().setState(FrontLiftState::UP, 100).waitUntilClamped();
  // chassis.drive(-2500, 1500, 900, -12000).withAngle(15).withTol(40,5).waitUntilSettled();
  // chassis.turn(95,450, -4500).withTurnGains(100,0.2,50).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(-1500, 900, 900, -5500).withAngle(95).withTol(40,5, true).waitUntilSettled();
  // backLift.toggleClamp();
  // chassis.drive(1150, 900, 900, 3000).withAngle(105).withTol(40,10).waitUntilSettled();
  // chassis.drive(-900, 900, 900, -3000).withAngle(105).withTol(40,5).waitUntilSettled();
  // chassis.drive(1000, 900, 900, 3000).withAngle(105).withTol(40,5).waitUntilSettled();
  // chassis.drive(-1000, 900, 900, -3000).withAngle(105).withTol(40,5).waitUntilSettled();
  // chassis.drive(1000, 900, 900, 3000).withAngle(105).withTol(40,5).waitUntilSettled();
  // chassis.drive(-1000).withAngle(90).withTol(40,5).waitUntilSettled();
}

void rightAWP(){ // One yellow + right AWP
  // frontLift.toggleClamp();
  // chassis.setBrakeType(COAST);
  // backLift.setState(BackLiftState::AUTON);
  // frontLift.withTol(10).setState(FrontLiftState::DOWN);
  // chassis.drive(2000, 1500, 450, 12000).withAngle(350, 1500, 12000).withTol(40,5).waitUntilSettled();
  // frontLift.toggleClamp().setState(FrontLiftState::UP, 100).waitUntilClamped();
  // chassis.drive(-1600, 1500, 900, -12000).withAngle(358).withTol(40,5).waitUntilSettled();
  // chassis.turn(90,450, -4500).withTurnGains(100,0.5,50).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(-1300, 900, 900, -6000).withAngle(90).withTol(40,5, true).waitUntilSettled();
  // backLift.toggleClamp();
  // pros::delay(150);
  // chassis.drive(300).withAngle(90).withTol(40,5).waitUntilSettled();
  // chassis.turn(0,450).withTurnGains(133,0.2,66).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(2000).withAngle(0).withTol(40,5).waitUntilSettled();
  // chassis.drive(-2300, 1500, 3000, -12000).withAngle(0).withTol(40,5).waitUntilSettled();
}

void fullAwp() { // FULL AWP
  // frontLift.toggleClamp();
  // chassis.setBrakeType(COAST);
  // backLift.setState(BackLiftState::AUTON);
  // backLift.toggleClamp();
  // pros::delay(250);
  // backLift.delayClamp(1.5);
  // chassis.drive(4750,105,450).withAngles(270,178).withTol(50, 20).waitUntilSettled();
  // chassis.turn(0,450).withTurnGains(66,0.2,33).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(-1000, 450, 450, -5500).withTol(50, 10, true).waitUntilSettled();  
  // backLift.toggleClamp();
  // backClamp.set_value(true);
  // pros::delay(100);
  // frontLift.setState(FrontLiftState::UP);
  // chassis.drive(275).withTol(50).waitUntilSettled();  
  // chassis.turn(270).withTurnGains(133,0,33).withTol(0,5).waitUntilSettled();
  // chassis.drive(1250, 450, 450, 7500).withAngle(270).withTol(50, 10).waitUntilSettled();
  // frontLift.setState(FrontLiftState::DOWN);
  // chassis.drive(-1000).withTol(50).waitUntilSettled();
  // // AWP BONUS
  // chassis.turn(285).withTurnGains(300, 0, 150).withTol(0, 5).waitUntilSettled();
  // chassis.drive(1300,1500,4000,12000).withTol(50).waitUntilSettled();
  // frontLift.toggleClamp().setState(FrontLiftState::UP, 100).waitUntilClamped();
  // chassis.turn(270).withTurnGains(300, 0, 150).withTol(0, 5).waitUntilSettled();
  // chassis.drive(-1500).withAngle(270).withTol(50,20).waitUntilSettled();
  // // YELLOW
}

void middleGoal(){ // Gets Middle Neutral Goal.
  // chassis.setBrakeType(COAST);
  // backLift.setState(BackLiftState::AUTON);
  // frontLift.toggleClamp().setState(FrontLiftState::DOWN);
  // chassis.drive(2800, 1500, 450, 12000).withAngle(358).withTol(40,5).waitUntilSettled();
  // frontLift.toggleClamp().setState(FrontLiftState::MIDDLE, 100).waitUntilClamped();
  // chassis.drive(-2400, 1500, 3000, -12000).withAngle(0).withTol(40,5).waitUntilSettled();
}

void elim() { // Right Side Neutral Goals.
  // frontLift.toggleClamp();
  // chassis.setBrakeType(COAST);
  // backLift.setState(BackLiftState::AUTON);
  // frontLift.withTol(10).setState(FrontLiftState::DOWN);
  // chassis.drive(2000, 1500, 450, 12000).withAngle(350, 1500, 12000).withTol(40,5).waitUntilSettled();
  // frontLift.toggleClamp().setState(FrontLiftState::UP, 100).waitUntilClamped();
  // chassis.turn(225, 900, -9000).withTurnGains(133,.25,66).swingTurn(RIGHT).withTol(0,10).waitUntilSettled();
  // chassis.drive(-1050).withAngle(225).withTol(50,10, true).waitUntilSettled();
  // backLift.toggleClamp();
  // pros::delay(100);
  // chassis.drive(2000).withAngle(225, 50).withTol(40,20).waitUntilSettled();
  // chassis.turn(40, 450).withTurnGains(133,0,133).withTol(0,2).waitUntilSettled();
  // chassis.drive(-1750).withAngle(45).withTol(40,20).waitUntilSettled();
  // backLift.toggleClamp();
  // chassis.drive(1000).withAngle(45).withTol(40,20).waitUntilSettled();
  // chassis.turn(90).withTol(0,10).waitUntilSettled();
  // chassis.drive(-500).withAngle(90).withTol(40,20).waitUntilSettled();
  // backLift.toggleClamp();
  // frontLift.setState(FrontLiftState::DOWN);
}

void skills(){ // Skills
  // chassis.setBrakeType(COAST);
  // backLift.setState(BackLiftState::AUTON);
  // frontLift.toggleClamp().withTol(10).setState(FrontLiftState::DOWN);
  // backLift.toggleClamp();
  // chassis.turn(245, 900, 12000).withTurnGains(133,.25,66).swingTurn(LEFT).withTol(0,10).waitUntilSettled();
  // chassis.drive(1950,1125,2250,9000).withAngle(245).withTol(50, 5).waitUntilSettled();  
  // frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100).waitUntilClamped();
  // chassis.drive(1650).withAngle(250).withTol(50,10).waitUntilSettled();  
  // chassis.turn(180, 900, 12000).withTurnGains(133,.25,66).withTol(0,10).waitUntilSettled();
  // chassis.drive(4000, 450, 450, 6000).withAngle(180).withTurnGains(66,0.25,33).withTol(50,5).waitUntilSettled();
  // chassis.drive(-2275, 450, 450, -9000).withAngle(180).withTol(50,5).waitUntilSettled();
  // chassis.turn(270, 900, 12000).withTurnGains(133,.25,66).withTol(0,10).waitUntilSettled();
  // chassis.drive(800).withAngle(270).withTol(50,20).waitUntilSettled();
  // pros::delay(750);
  // frontLift.toggleClamp().waitUntilClamped();
  // // FIRST GOAL

  // chassis.drive(-500).withTol(50).waitUntilSettled();
  // pros::delay(200);
  // chassis.turn(0,450).withTurnGains(100,0.2,50).withTol(0,2.5).waitUntilSettled();
  // backLift.toggleClamp();
  // frontLift.setState(FrontLiftState::DOWN);
  // pros::delay(650);
  // chassis.drive(500).withTol(50);
  // pros::delay(250);
  // chassis.turn(180,450).withTurnGains(100,0.2,50).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(1100).withAngle(178).withTurnGains(100,0.2, 50).withTol(50, 5).waitUntilSettled();
  // frontLift.toggleClamp().setState(FrontLiftState::UP, 100).waitUntilSettled();
  // chassis.turn(270,450).withTurnGains(133,0.2,33).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(850).withAngle(270).withTurnGains(100, 0.2, 50).withTol(50, 5).waitUntilSettled();
  // frontLift.toggleClamp();
  // pros::delay(250);
  // chassis.drive(-300).withTol(50).waitUntilSettled();
  // // SECOND GOAL
  
  // chassis.turn(180,450).withTurnGains(100,0.2,50).withTol(0,2.5).waitUntilSettled();
  // backLift.setState(BackLiftState::IDLE); conveyer::spin(-600);
  // chassis.drive(-2850, 900, 450, -6000).withAngle(180, 900, 7000).withTurnGains(100, 0.2, 50).withTol(50, 2.5, true).waitUntilSettled();
  // backLift.setState(BackLiftState::AUTON).toggleClamp();
  // pros::delay(500);
  // chassis.drive(2500).withAngle(140).withTol(50,10).waitUntilSettled();
  // chassis.drive(-250).withAngle(140).withTol(50,10).waitUntilSettled();
  // frontLift.withTol(100).setState(FrontLiftState::DOWN, 0).waitUntilSettled();
  // pros::delay(700);
  // chassis.drive(700).withAngle(140).withTol(50,10).waitUntilSettled();
  // pros::delay(400);
  // frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100).waitUntilClamped();
  // pros::delay(250);
  // chassis.drive(3100).withAngle(135).withTol(50,10).waitUntilSettled();
  // chassis.turn(90,900).withTurnGains(166,0.25,66).withTol(0,3).waitUntilSettled();
  // pros::delay(750);
  // // MATCH LOADS  
  // chassis.drive(1000, 450,450, 3000).withAngle(90).withTol(50,10).waitUntilSettled();
  // chassis.drive(-1000, 450,450, -3000).withAngle(90).withTol(50,10).waitUntilSettled();
  // chassis.drive(1000, 450,450, 3000).withAngle(90).withTol(50,10).waitUntilSettled();
  // chassis.drive(-1000, 450,450, -3000).withAngle(90).withTol(50,10).waitUntilSettled();
  // // MATCH LOADS  
  // chassis.drive(900).withAngle(120).withTol(50,20).waitUntilSettled();
  // chassis.turn(15,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  // frontLift.delayClamp(2.75);
  // chassis.drive(900, 900, 450, 12000).withGains(40, 1, 20).withAngle(15).withTol(100,10).waitUntilSettled();
  // // THIRD GOAL

  // chassis.drive(-1000).withAngle(0).withTol(50,10).waitUntilSettled();
  // chassis.turn(315,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(2900).withAngle(0, 175).withTol(50,10).waitUntilSettled();
  // backLift.toggleClamp();
  // frontLift.setState(FrontLiftState::DOWN);
  // pros::delay(500);
  // chassis.drive(400).withAngle(0, 100).withTol(50,10).waitUntilSettled();
  // chassis.turn(180,450).withTurnGains(100,0.2,50).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(1000).withAngle(183).withTurnGains(100,0.2, 50).withTol(50, 10).waitUntilSettled();
  // frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100).waitUntilClamped();
  // chassis.drive(-500).withAngle(180, 100).withTol(50,10).waitUntilSettled();
  // chassis.turn(90,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(700).withAngle(90).withTol(50,10).waitUntilSettled();
  // frontLift.toggleClamp().waitUntilClamped();
  // pros::delay(250);
  // // FOURTH GOAL

  // chassis.drive(-1700, 900, 900, -9000).withAngle(90).withTol(50,10).waitUntilSettled();
  // backLift.toggleClamp();
  // chassis.turn(180,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(1800).withAngle(180).withTol(50,10).waitUntilSettled();
  // chassis.drive(-300).withAngle(180).withTol(50,10).waitUntilSettled();
  // frontLift.withTol(75).setState(FrontLiftState::DOWN, 0).waitUntilSettled();
  // pros::delay(700);
  // chassis.drive(700).withAngle(180).withTol(50,15).waitUntilSettled();
  // frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100).waitUntilClamped();
  // chassis.turn(55,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  // chassis.drive(3000, 1500, 450, 12000).withGains(30, 0, 6.25).withAngle(55).withTol(50,10).waitUntilSettled();
  // frontLift.toggleClamp().waitUntilClamped();
  // pros::delay(250);
  // chassis.drive(-750, 1500, 450. -12000).withGains(50, 0, 6.25).withTol(50).waitUntilSettled();
  // // FIFTH GOAL
}

void test() { // Testing
chassis.setBrakeType(COAST);
chassis.drive(24_in, 0.37_ftps2, 0.37_ftps2, 4.78_ftps).waitUntilSettled();
chassis.turn(90_deg).withTurnGains(18).waitUntilSettled();
}