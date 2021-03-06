#include "auton.hpp"
#include "chassis.hpp"
#include "globals.hpp"
#include "backLift.hpp"
#include "positionTracking.hpp"
#include "frontLift.hpp"
#include "misc.hpp"
#include <string>

// Class Init
static Chassis chassis;
static BackLift backLift;
static FrontLift frontLift;
static macro::GoalCover goalCover;

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
  backLift.setState(BackLiftState::AUTON);
  goalCover.toggle();
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  frontLift.withTol(10).setState(FrontLiftState::DOWN);
  chassis.drive(42.5_in, 1.44_ftps2).withAngle(0_deg, 28_radps2).withTurnGains(36,0,10).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 50_ms).waitUntilClamped();
  chassis.drive(-53_in, 4.87_ftps2, .37_ftps2, -4.87_ftps).withAngle(355_deg).withTol(4_in).withTurnTol(10_deg).withTurnGains(30).waitUntilSettled();
  chassis.turn(87_deg, 1.1_radps2, 14_radps).waitUntilSettled();
  chassis.drive(-30_in, .2_ftps2, .2_ftps2, -3_ftps).withAngle(87_deg).withTol(1_in, true).waitUntilSettled();
  backLift.toggleClamp(100_ms);
  pros::delay(750);
  chassis.drive(30_in, 0.05_ftps2, 0.05_ftps2, 3_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-20_in, 0.05_ftps2, 0.05_ftps2, -3_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  pros::delay(500);
  chassis.drive(20_in, 0.05_ftps2, 0.05_ftps2, 3_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-15_in, 0.05_ftps2, 0.05_ftps2, -3_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  pros::delay(500);
  chassis.drive(19_in, 0.05_ftps2, 0.05_ftps2, 3_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  winPointSafety();
}

void rightAWP(){ // One yellow + right AWP
  backLift.setState(BackLiftState::AUTON).toggleClamp();
  goalCover.toggle();
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  frontLift.withTol(75).setState(FrontLiftState::DOWN);
  chassis.drive(42.5_in, 1.44_ftps2).withAngle(0_deg).withTurnGains(36,0,10).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::MIDDLE, 250_ms).waitUntilClamped();
  chassis.drive(-28_in, 4.87_ftps2, 0.37_ftps2, -8.51_ftps).withAngles(0_deg, 0_deg).withTol(2_in).withTurnTol(15_deg).waitUntilSettled();
  chassis.turn(85_deg, 1.1_radps2, 14_radps).withTurnGains(20,0,10).waitUntilSettled();
  chassis.drive(-29_in, .37_ftps2, .37_ftps2, -2_ftps).withAngle(90_deg).withTol(1_in, true).waitUntilSettled();
  pros::delay(150);
  backLift.toggleClamp();
  backClamp.set_value(true);
  frontLift.setState(FrontLiftState::UP);
  pros::delay(150);
  chassis.drive(2_in).withGains(10).waitUntilSettled();
  chassis.turn(0_deg).waitUntilSettled();
  chassis.drive(40_in, 0.135_ftps2, 0.135_ftps2, 3.75_ftps).withGains(10,1,5).withAngle(0_deg).withTol(5_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-45_in).withAngle(0_deg).waitUntilSettled();
  winPointSafety();
}


void fullAwp() { // FULL AWP
  backLift.setState(BackLiftState::AUTON).toggleClamp();
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  chassis.drive(-12_in).withTol(1.5_in, true).waitUntilSettled();
  backLift.toggleClamp();
  backClamp.set_value(true);
  pros::delay(250);
  backLift.delayClamp(0.5_s); //0.05576_ftps2
  chassis.drive(3.25_tile, 0.09_ftps2, 0.239_ftps2, 8.51_ftps).withAngles(270_deg, 180_deg, 1_radps2).withTurnGains(28).withTol(3_in).withTurnTol(20_deg).waitUntilSettled();
  chassis.turn(0_deg).withTurnGains(14).waitUntilSettled();
  chassis.drive(-30_in, .37_ftps2, .37_ftps2, -2_ftps).withTol(2_in, true).waitUntilSettled();  
  backLift.toggleClamp();
  backClamp.set_value(true);
  pros::delay(100);
  frontLift.setState(FrontLiftState::UP);
  chassis.drive(5_in).withGains(10).waitUntilSettled();
  chassis.turn(270_deg).waitUntilSettled();
  chassis.drive(35_in, 0.15_ftps2, 0.15_ftps2, 4.87_ftps).withGains(10).withAngle(270_deg).withTol(5_in).withTurnTol(5_deg).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  chassis.drive(-30_in, 0.15_ftps2, 0.15_ftps2, -8.51_ftps).withAngle(270_deg).withTol(5_in).withTurnTol(2_deg).waitUntilSettled();
  // AWP BONUS
  chassis.turn(280_deg).withTurnGains(36).waitUntilSettled();
  goalCover.toggle();
  chassis.drive(32_in).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100_ms).waitUntilClamped();
  chassis.turn(270_deg).withTurnGains(36).withTurnTol(10_deg).waitUntilSettled();
  chassis.drive(-40_in).withAngle(270_deg).waitUntilSettled();
  backLift.toggleClamp();
  chassis.drive(5_in).withAngle(270_deg).waitUntilSettled();
  // YELLOW
}

void middleGoal(){ // Gets Middle Neutral Goal.
  backLift.setState(BackLiftState::AUTON).toggleClamp();
  goalCover.toggle();
  chassis.setBrakeType(COAST);
  frontLift.toggleClamp().setState(FrontLiftState::DOWN);
  chassis.drive(52_in, 2.5_ftps2, .1_ftps2).withAngle(0_deg, 28_radps2).withTurnGains(30).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp();
  chassis.drive(-60_in, 4.78_ftps2, 0.37_ftps2, -8.51_ftps).withAngles(45_deg, 60_deg).withTol(5_in, true).withTurnTol(10_deg).waitUntilSettled();
  pros::delay(150);
  backLift.toggleClamp();
  backClamp.set_value(true);
  frontLift.setState(FrontLiftState::UP);
  pros::delay(500);
  chassis.drive(1_tile).withAngle(60_deg).waitUntilSettled();
  winPointSafety();
}

void elim() { // Right Side Neutral Goals.
  backLift.setState(BackLiftState::AUTON).toggleClamp();
  goalCover.toggle();
  chassis.setBrakeType(COAST);
  frontLift.toggleClamp().setState(FrontLiftState::DOWN);
  chassis.drive(55_in, 2.5_ftps2, .1_ftps2).withAngle(50_deg, 28_radps2).withTurnGains(30).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp();
  chassis.drive(-60_in, 4.78_ftps2, 0.37_ftps2, -8.51_ftps).withAngle(90_deg).withTurnGains(10).withTol(5_in, true).withTurnTol(10_deg).waitUntilSettled();
  pros::delay(150);
  backLift.toggleClamp();
  backClamp.set_value(true);
  frontLift.setState(FrontLiftState::UP);
  pros::delay(500);
  chassis.drive(1.5_tile).withAngle(90_deg).waitUntilSettled();
  winPointSafety();
}

void skills(){ // Skills
  backLift.setState(BackLiftState::AUTON);  
  backLift.toggleClamp();
  frontLift.toggleClamp().withTol(1).setState(FrontLiftState::DOWN);
  chassis.setBrakeType(COAST);
  chassis.turn(250_deg).swingTurn(LEFT).waitUntilSettled();
  chassis.drive(40_in).withAngle(250_deg).withTurnGains(36,0,10).withTurnTol(2.1_deg).withTurnTol(5_deg).waitUntilSettled();  
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100_ms).waitUntilClamped();
  chassis.drive(27_in).withAngle(250_deg).withTurnGains(36,0,10).waitUntilSettled();  
  chassis.turn(180_deg).waitUntilSettled();
  chassis.drive(75_in, .15_ftps2, .37_ftps2, 3.5_ftps).withGains(15,0,2).withAngle(180_deg).withTurnGains(36,0,10).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-42_in, .15_ftps2, 45.15_ftps2, -7.2_ftps).withAngle(180_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.turn(270_deg).withTurnGains(26,0,9).waitUntilSettled();
  chassis.drive(20_in, 1.44_ftps2).withGains(30,0,2).withAngle(270_deg).withTurnGains(36,0,10).withTol(3_in).withTurnTol(3_deg).waitUntilSettled();
  pros::delay(750);
  frontLift.toggleClamp().waitUntilClamped();
  // FIRST GOAL 
  chassis.drive(-8_in).withGains(10,0,2).waitUntilSettled();
  pros::delay(200);
  chassis.turn(0_deg).waitUntilSettled();
  backLift.toggleClamp();
  frontLift.setState(FrontLiftState::DOWN);
  pros::delay(650);
  chassis.drive(7_in).withGains(10,0,2).withAngle(0_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.turn(190_deg).withTurnGains(14).waitUntilSettled();
  chassis.drive(23_in).withGains(7,0,1).withAngle(190_deg).withTurnGains(36,0,10).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100_ms).waitUntilSettled();
  chassis.turn(270_deg,1.1_radps2).waitUntilSettled();
  chassis.drive(18.5_in).withGains(10,0,2).withAngle(270_deg).withTurnGains(36,0,10).withTol(3_in).withTurnTol(3_deg).waitUntilSettled();
  frontLift.toggleClamp();
  pros::delay(250);
  chassis.drive(-5_in).withGains(10,0,2).waitUntilSettled();
  // SECOND GOAL
  
  chassis.turn(180_deg).waitUntilSettled();
  frontLift.setState(FrontLiftState::DOWN);
  backLift.setState(BackLiftState::IDLE); conveyer::spin(-12000);
  chassis.drive(35_in).withAngle(180_deg).withTurnGains(36,0,10).withTol(2_in).withTurnTol(5_deg).waitUntilSettled();
  backLift.setState(BackLiftState::AUTON);
  chassis.turn(90_deg).waitUntilSettled();
  pros::delay(500);
  chassis.drive(30_in).withAngle(90_deg).withTurnGains(36,0,10).withTol(2_in).withTurnTol(5_deg).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::MIDDLE).waitUntilClamped();
  chassis.drive(25_in).withAngle(90_deg).withTurnGains(36,0,10).withTol(2_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.turn(0_deg).waitUntilSettled();
  chassis.drive(-20_in).withAngle(0_deg).withTurnGains(36,0,10).withTol(2_in, true).withTurnTol(5_deg).waitUntilSettled();
  backLift.toggleClamp();
  chassis.drive(2_in).withGains(10).waitUntilSettled();
  chassis.turn(270_deg).waitUntilSettled();
  chassis.drive(60_in).withAngle(270_deg).withTurnGains(36,0,10).withTol(2_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.turn(0_deg).waitUntilSettled();
  chassis.drive(24_in).withAngle(0_deg).withTurnGains(36,0,10).withTol(2_in).withTurnTol(5_deg).waitUntilSettled();
  frontLift.setState(FrontLiftState::UP).waitUntilSettled();
  chassis.turn(270_deg).waitUntilSettled();
  chassis.drive(18.5_in).withGains(10,0,2).withAngle(270_deg).withTurnGains(36,0,10).withTol(3_in).withTurnTol(3_deg).waitUntilSettled();
  frontLift.toggleClamp();
  pros::delay(250);
  chassis.drive(-5_in).withGains(10,0,2).waitUntilSettled();

  /*
  chassis.drive(-20_in).withAngle(0_deg).withTurnTol(5_deg).waitUntilSettled();
  chassis.turn(315_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.drive(40_in).withAngle(0_deg, .5_radps2).withTurnGains(36,0,10).waitUntilSettled();
  backLift.toggleClamp();
  frontLift.setState(FrontLiftState::DOWN);
  pros::delay(500);
  chassis.drive(8_in).withAngle(0_deg).withTurnGains(36,0,1).waitUntilSettled();
  chassis.turn(180_deg).waitUntilSettled();
  chassis.drive(15_in).withAngle(185_deg).withTurnGains(36,0,1).waitUntilSettled();
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100_ms).waitUntilClamped();
  chassis.drive(-25_in).withAngle(180_deg).withTurnGains(36,0,1).waitUntilSettled();
  chassis.turn(90_deg).waitUntilSettled();
  chassis.drive(15_in).withAngle(90_deg).withTurnGains(36,0,1).withTol(4_in).waitUntilSettled();
  frontLift.toggleClamp().waitUntilClamped();
  pros::delay(250);
  // FOURTH GOAL

  chassis.drive(-38_in,.37_ftps2, .37_ftps2, -4.87_ftps).withAngle(90_deg).withTurnGains(36,0,10).waitUntilSettled();
  backLift.toggleClamp();
  chassis.turn(180_deg).waitUntilSettled();
  chassis.drive(30_in).withAngle(180_deg).waitUntilSettled();
  frontLift.withTol(75).setState(FrontLiftState::DOWN, 0_ms).waitUntilSettled();
  chassis.drive(-8_in).withAngle(180_deg).withTurnGains(36,0,10).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  pros::delay(700);
  chassis.drive(15_in).withAngle(180_deg).withTurnGains(36,0,10).withTol(5_in).withTurnTol(5_deg).waitUntilSettled();
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100_ms).waitUntilClamped();
  chassis.turn(50_deg).waitUntilSettled();
  chassis.drive(3_tile).withAngle(50_deg).withTurnGains(36,0,10).withTol(5_in).withTurnTol(5_deg).waitUntilSettled();
  frontLift.toggleClamp().waitUntilClamped();
  pros::delay(250);
  chassis.drive(-1_tile).waitUntilSettled();
  backLift.toggleClamp();
  // FIFTH GOAL
  */
}

void test() { // Testing
}

void winPointSafety(){
  chassis.drive(-10_in).waitUntilSettled();
  backLift.toggleClamp();
  chassis.drive(-10_in).waitUntilSettled();
}