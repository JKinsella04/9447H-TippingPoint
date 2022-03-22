#include "auton.hpp"
#include "chassis.hpp"
#include "globals.hpp"
#include "backLift.hpp"
#include "positionTracking.hpp"
#include "frontLift.hpp"
#include "misc.hpp"
#include "pros/rtos.hpp"

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
  chassis.drive(42_in, 2.5_ftps2).withAngle(355_deg, 28_radps2).withGains(3.5, 0, .5).withTurnGains(30).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 50).waitUntilClamped();
  chassis.drive(-53_in, 2.5_ftps2, .37_ftps2, -4.87_ftps).withAngle(355_deg).withTol(4_in).withTurnTol(10_deg).withTurnGains(30).waitUntilSettled();
  chassis.turn(90_deg, 1.1_radps2, 14_radps).waitUntilSettled();
  chassis.drive(-30_in, .2_ftps2, .2_ftps2, -2_ftps).withAngle(90_deg).withTol(1_in, true).waitUntilSettled();
  backLift.toggleClamp();
  pros::delay(250);
  chassis.drive(27_in, 0.05_ftps2, 0.05_ftps2, 2_ftps).withGains(3).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-20_in, 0.05_ftps2, 0.05_ftps2, -2_ftps).withGains(3).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(19_in, 0.05_ftps2, 0.05_ftps2, 2_ftps).withGains(3).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-20_in, 0.05_ftps2, 0.05_ftps2, -2_ftps).withGains(3).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(19_in, 0.05_ftps2, 0.05_ftps2, 2_ftps).withGains(3).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-20_in, 0.05_ftps2, 0.05_ftps2, -2_ftps).withGains(3).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
}

void rightAWP(){ // One yellow + right AWP
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.withTol(1).setState(FrontLiftState::DOWN);
  chassis.drive(42.5_in, 2.5_ftps2).withAngle(355_deg, 28_radps2).withGains(3.5, 0, .5).withTurnGains(30).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 50).waitUntilClamped();
  chassis.drive(-30_in, 0.37_ftps2, 0.37_ftps2, -4.87_ftps).withAngle(360_deg).waitUntilSettled();
  chassis.turn(85_deg, 1.1_radps2, 14_radps).withTurnGains(20,0,10).waitUntilSettled();
  chassis.drive(-29_in, .37_ftps2, .37_ftps2, -2_ftps).withGains(3,0,0.75).withAngle(90_deg).withTol(1_in, true).waitUntilSettled();
  pros::delay(150);
  backLift.toggleClamp();
  pros::delay(150);
  chassis.drive(3_in).withGains(10).waitUntilSettled();
  chassis.turn(0_deg).waitUntilSettled();
  chassis.drive(40_in, 0.135_ftps2, 0.135_ftps2, 2_ftps).withGains(3).withAngle(0_deg).withTol(5_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-45_in).withAngle(0_deg).waitUntilSettled();
}

void fullAwp() { // FULL AWP
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  backLift.toggleClamp();
  backClamp.set_value(true);
  pros::delay(250);
  backLift.delayClamp(0.5); //0.05576_ftps2
  chassis.drive(4_tile, 0.09_ftps2, 0.239_ftps2, 3_ftps).withAngles(270_deg, 180_deg, 1_radps2).withTurnGains(28).withTol(3_in).withTurnTol(20_deg).waitUntilSettled();
  chassis.turn(0_deg).withTurnGains(14).waitUntilSettled();
  chassis.drive(-30_in, .37_ftps2, .37_ftps2, -2_ftps).withTol(2_in, true).waitUntilSettled();  
  backLift.toggleClamp();
  backClamp.set_value(true);
  pros::delay(100);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(5_in).withGains(10).waitUntilSettled();
  chassis.turn(270_deg).waitUntilSettled();
  chassis.drive(35_in, 0.15_ftps2, 0.15_ftps2, 3_ftps).withGains(10).withAngle(270_deg).withTol(5_in).withTurnTol(5_deg).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  chassis.drive(-30_in, 0.15_ftps2, 0.15_ftps2, -4.87_ftps).withGains(3).withAngle(270_deg).withTol(5_in).withTurnTol(2_deg).waitUntilSettled();
  // AWP BONUS
  chassis.turn(280_deg).withTurnGains(36).waitUntilSettled();
  chassis.drive(32_in).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100).waitUntilClamped();
  chassis.turn(270_deg).withTurnGains(36).withTurnTol(10_deg).waitUntilSettled();
  chassis.drive(-35_in).withAngle(270_deg).waitUntilSettled();
  // YELLOW
}

void middleGoal(){ // Gets Middle Neutral Goal.
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.toggleClamp().setState(FrontLiftState::DOWN);
  chassis.drive(57_in, 2.5_ftps2).withAngle(355_deg, 28_radps2).withGains(3.5, 0, .5).withTurnGains(30).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::MIDDLE, 100).waitUntilClamped();
  chassis.drive(-40_in, 2.5_ftps2, 0.37_ftps2, -4.78_ftps).withAngle(45_deg).withTol(5_in).withTurnTol(10_deg).waitUntilSettled();
  chassis.drive(-30_in, .37_ftps2, .37_ftps2, -1.5_ftps).withGains(3,0,0.75).withAngle(45_deg).withTol(1_in, true).waitUntilSettled();
  pros::delay(150);
  backLift.toggleClamp();
  chassis.drive(1_tile).waitUntilSettled();
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
  frontLift.toggleClamp().withTol(1).setState(FrontLiftState::DOWN);
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  backLift.toggleClamp();
  backClamp.set_value(true);
  pros::delay(250);
  chassis.turn(250_deg).withTurnGains(36).swingTurn(LEFT).waitUntilSettled();
  chassis.drive(40_in).withAngle(250_deg).withTurnGains(36,0,10).waitUntilSettled();  
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100).waitUntilClamped();
  chassis.drive(30_in).withAngle(250_deg).withTurnGains(36,0,10).waitUntilSettled();  
  chassis.turn(180_deg).waitUntilSettled();
  chassis.drive(70_in, .15_ftps2, .37_ftps2, 1.75_ftps).withGains(5,0,1).withAngle(180_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.drive(-37_in, .15_ftps2, 45.15_ftps2, -4_ftps).withAngle(180_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.turn(270_deg).waitUntilSettled();
  chassis.drive(17_in).withGains(5,0,1).withAngle(270_deg).withTurnGains(36,0,10).waitUntilSettled();
  pros::delay(750);
  frontLift.toggleClamp().waitUntilClamped();
  // FIRST GOAL ( ( tick ) / 900 ) * 13.031 = target(inches) 

  chassis.drive(-8_in).withGains(5,0,1).waitUntilSettled();
  pros::delay(200);
  chassis.turn(0_deg).waitUntilSettled();
  backLift.toggleClamp();
  frontLift.setState(FrontLiftState::DOWN);
  pros::delay(650);
  chassis.drive(7_in).withGains(5,0,1).withAngle(0_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.turn(190_deg).withTurnGains(14).waitUntilSettled();
  chassis.drive(27_in).withGains(5,0,1).withAngle(190_deg).withTurnGains(36,0,10).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100).waitUntilSettled();
  chassis.turn(270_deg,1.1_radps2).waitUntilSettled();
  chassis.drive(17_in).withGains(5,0,1).withAngle(270_deg).withTurnGains(36,0,10).waitUntilSettled();
  frontLift.toggleClamp();
  pros::delay(250);
  chassis.drive(-5_in).waitUntilSettled();
  // SECOND GOAL
  
  // chassis.turn(180_deg, 1.1_radps2).waitUntilSettled();
  // backLift.setState(BackLiftState::IDLE); conveyer::spin(-600);
  // chassis.drive(-41.26_in, .37_ftps2, .15_ftps2, -2.43_ftps).withAngle(180_deg, 2.2_radps2, 18_radps).withTol(1.5_in, true).waitUntilSettled();
  // backLift.setState(BackLiftState::AUTON).toggleClamp();
  // pros::delay(500);
  // chassis.drive(36.197_in).withAngle(140_deg).waitUntilSettled();
  // chassis.drive(-3.619_in).withAngle(140_deg).waitUntilSettled();
  // frontLift.withTol(100).setState(FrontLiftState::DOWN, 0).waitUntilSettled();
  // pros::delay(700);
  // chassis.drive(10.135_in).withAngle(140_deg).waitUntilSettled();
  // pros::delay(400);
  // frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100).waitUntilClamped();
  // pros::delay(250);
  // chassis.drive(44.884_in).withAngle(135_deg).waitUntilSettled();
  // chassis.turn(90_deg).waitUntilSettled();
  // pros::delay(750);
  // // MATCH LOADS  
  // chassis.drive(14.478_in, .15_ftps2, .15_ftps2, 1.5_ftps).withAngle(90_deg).waitUntilSettled();
  // chassis.drive(-14.478_in, .15_ftps2, .15_ftps2, 1.5_ftps).withAngle(90_deg).waitUntilSettled();
  // chassis.drive(14.478_in, .15_ftps2, .15_ftps2, 1.5_ftps).withAngle(90_deg).waitUntilSettled();
  // chassis.drive(-14.478_in, .15_ftps2, .15_ftps2, 1.5_ftps).withAngle(90_deg).waitUntilSettled();
  // MATCH LOADS  (target / 900) * 13.031 = target(inches)
  // chassis.drive(900).withAngle(120).withTol(50,20).waitUntilSettled();
  // chassis.turn(15,900).withTurnGains(133,0.25,66).withTol(0,2.5).waitUntilSettled();
  // frontLift.delayClamp(2.75);
  // chassis.drive(900, 900, 450, 12000).withGains(40, 1, 20).withAngle(15).withTol(100,10).waitUntilSettled();
  // THIRD GOAL

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
chassis.drive(24_in).withGains(2.75, 0, 0.5).waitUntilSettled(); // test
chassis.turn(90_deg).withTurnGains(18).waitUntilSettled();
}