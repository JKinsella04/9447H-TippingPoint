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
  chassis.drive(42.5_in, 4_ftps2, .1_ftps2).withAngle(355_deg, 28_radps2).withTurnGains(36,0,10).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 50_ms).waitUntilClamped();
  chassis.drive(-53_in, 4.87_ftps2, .37_ftps2, -4.87_ftps).withAngle(355_deg).withTol(4_in).withTurnTol(10_deg).withTurnGains(30).waitUntilSettled();
  chassis.turn(90_deg, 1.1_radps2, 14_radps).waitUntilSettled();
  chassis.drive(-30_in, .2_ftps2, .2_ftps2, -2_ftps).withAngle(90_deg).withTol(1_in, true).waitUntilSettled();
  backLift.toggleClamp();
  pros::delay(250);
  chassis.drive(27_in, 0.05_ftps2, 0.05_ftps2, 2_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-20_in, 0.05_ftps2, 0.05_ftps2, -2_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(19_in, 0.05_ftps2, 0.05_ftps2, 2_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-20_in, 0.05_ftps2, 0.05_ftps2, -2_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(19_in, 0.05_ftps2, 0.05_ftps2, 2_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-20_in, 0.05_ftps2, 0.05_ftps2, -2_ftps).withAngle(105_deg).withTol(3_in).withTurnTol(5_deg).waitUntilSettled();
}

void rightAWP(){ // One yellow + right AWP
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.withTol(75).setState(FrontLiftState::DOWN);
  chassis.drive(42.5_in, 4_ftps2).withAngle(355_deg, 28_radps2).withTurnGains(36,0,10).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::MIDDLE, 250_ms).waitUntilClamped();
  chassis.drive(-35_in, 4.87_ftps2, 0.37_ftps2, -4.87_ftps).withAngle(315_deg).withTurnGains(100,0,10).withTol(2_in).withTurnTol(2_deg).waitUntilSettled();
  chassis.turn(85_deg, 1.1_radps2, 14_radps).withTurnGains(20,0,10).waitUntilSettled();
  chassis.drive(-29_in, .37_ftps2, .37_ftps2, -2_ftps).withAngle(90_deg).withTol(1_in, true).waitUntilSettled();
  pros::delay(150);
  backLift.toggleClamp();
  pros::delay(150);
  chassis.drive(3_in).withGains(10).waitUntilSettled();
  chassis.turn(0_deg).waitUntilSettled();
  chassis.drive(40_in, 0.135_ftps2, 0.135_ftps2, 2_ftps).withAngle(0_deg).withTol(5_in).withTurnTol(5_deg).waitUntilSettled();
  chassis.drive(-45_in).withAngle(0_deg).waitUntilSettled();
}

void fullAwp() { // FULL AWP
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  backLift.toggleClamp();
  backClamp.set_value(true);
  pros::delay(250);
  backLift.delayClamp(0.5_s); //0.05576_ftps2
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
  chassis.drive(-30_in, 0.15_ftps2, 0.15_ftps2, -4.87_ftps).withAngle(270_deg).withTol(5_in).withTurnTol(2_deg).waitUntilSettled();
  // AWP BONUS
  chassis.turn(280_deg).withTurnGains(36).waitUntilSettled();
  chassis.drive(32_in).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100_ms).waitUntilClamped();
  chassis.turn(270_deg).withTurnGains(36).withTurnTol(10_deg).waitUntilSettled();
  chassis.drive(-40_in).withAngle(270_deg).waitUntilSettled();
  backLift.toggleClamp();
  chassis.drive(5_in).withAngle(270_deg).waitUntilSettled();
  // YELLOW
}

void middleGoal(){ // Gets Middle Neutral Goal.
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.toggleClamp().setState(FrontLiftState::DOWN);
  chassis.drive(57_in, 2.5_ftps2, .1_ftps2).withAngle(355_deg, 28_radps2).withTurnGains(30).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::MIDDLE, 100_ms).waitUntilClamped();
  chassis.drive(-40_in, 4.78_ftps2, 0.37_ftps2, -4.78_ftps).withAngle(45_deg).withTol(5_in).withTurnTol(10_deg).waitUntilSettled();
  chassis.drive(-30_in, .37_ftps2, .37_ftps2, -1.5_ftps).withAngle(45_deg).withTol(1_in, true).waitUntilSettled();
  pros::delay(150);
  backLift.toggleClamp();
  chassis.drive(1_tile).waitUntilSettled();
}

void elim() { // Right Side Neutral Goals.
  frontLift.toggleClamp();
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  frontLift.withTol(75).setState(FrontLiftState::DOWN);
  chassis.drive(42.5_in, 4_ftps2, .1_ftps2).withAngle(355_deg, 28_radps2).withTurnGains(36,0,10).withTol(2_in).withTurnTol(10_deg).waitUntilSettled();
  chassis.turn(235_deg).swingTurn(RIGHT).waitUntilSettled();
  chassis.drive(-20_in).withAngle(235_deg).withTol(1.5_in, true).waitUntilSettled();
}

void skills(){ // Skills
  frontLift.toggleClamp().withTol(1).setState(FrontLiftState::DOWN);
  chassis.setBrakeType(COAST);
  backLift.setState(BackLiftState::AUTON);
  backLift.toggleClamp();
  backClamp.set_value(true);
  pros::delay(250);
  chassis.turn(255_deg).swingTurn(LEFT).waitUntilSettled();
  chassis.drive(40_in).withAngle(255_deg).withTurnGains(36,0,10).withTurnTol(2.1_deg).waitUntilSettled();  
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100_ms).waitUntilClamped();
  chassis.drive(28_in).withAngle(250_deg).withTurnGains(36,0,10).waitUntilSettled();  
  chassis.turn(180_deg).waitUntilSettled();
  chassis.drive(70_in, .15_ftps2, .37_ftps2, 4_ftps).withGains(15,0,2).withAngle(180_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.drive(-36.5_in, .15_ftps2, 45.15_ftps2, -7.2_ftps).withAngle(180_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.turn(270_deg).withTurnGains(26,0,9).waitUntilSettled();
  chassis.drive(17_in, 4_ftps2).withAngle(270_deg).withTurnGains(36,0,10).withTol(3_in).withTurnTol(3_deg).waitUntilSettled();
  pros::delay(750);
  frontLift.toggleClamp().waitUntilClamped();
  /*
  // FIRST GOAL 
  chassis.drive(-8_in).waitUntilSettled();
  pros::delay(200);
  chassis.turn(0_deg).waitUntilSettled();
  backLift.toggleClamp();
  frontLift.setState(FrontLiftState::DOWN);
  pros::delay(650);
  chassis.drive(7_in).withAngle(0_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.turn(190_deg).withTurnGains(14).waitUntilSettled();
  chassis.drive(23_in, .37_ftps2, .37_ftps2, 2.43_ftps).withGains(7,0,1).withAngle(190_deg).withTurnGains(36,0,10).waitUntilSettled();
  frontLift.toggleClamp().setState(FrontLiftState::UP, 100_ms).waitUntilSettled();
  chassis.turn(270_deg,1.1_radps2).waitUntilSettled();
  chassis.drive(18.5_in).withAngle(270_deg).withTurnGains(36,0,10).withTol(3_in).withTurnTol(3_deg).waitUntilSettled();
  frontLift.toggleClamp();
  pros::delay(250);
  chassis.drive(-5_in).withGains(7,0,1).waitUntilSettled();
  // SECOND GOAL
  
  chassis.turn(180_deg).waitUntilSettled();
  backLift.setState(BackLiftState::IDLE); conveyer::spin(-12000);
  chassis.drive(-59.75_in, .37_ftps2, .15_ftps2, -2.43_ftps).withAngle(180_deg).withTurnGains(36,0,10).withTol(2_in, true).withTurnTol(5_deg).waitUntilSettled();
  backLift.setState(BackLiftState::AUTON).toggleClamp();
  pros::delay(500);
  chassis.drive(49_in).withAngle(137_deg).withTurnGains(36,0,10).waitUntilSettled();
  chassis.drive(-5_in).withAngle(137_deg).withTurnGains(36,0,10).waitUntilSettled();
  frontLift.withTol(100).setState(FrontLiftState::DOWN, 0_ms).waitUntilSettled();
  pros::delay(700);
  chassis.drive(18_in).withAngle(137_deg).withTurnGains(36,0,10).waitUntilSettled();
  pros::delay(400);
  frontLift.toggleClamp().withTol(75).setState(FrontLiftState::UP, 100_ms).waitUntilClamped();
  pros::delay(250);
  chassis.drive(50_in).withAngle(130_deg).withTurnGains(36,0,10).withTurnTol(5.5_deg).waitUntilSettled();
  chassis.turn(90_deg).withTurnGains(36,0,10).waitUntilSettled();
  pros::delay(750);
  // MATCH LOADS  
  chassis.drive(30_in, .15_ftps2, .15_ftps2, 1.5_ftps).withGains(10,0,1).withAngle(90_deg).withTurnGains(36,0,1).withTurnTol(2_deg).waitUntilSettled();
  chassis.drive(-20_in, .15_ftps2, .15_ftps2, -1.5_ftps).withGains(10,0,1).withAngle(90_deg).withTurnGains(36,0,1).waitUntilSettled();
  chassis.drive(20_in, .15_ftps2, .15_ftps2, 1.5_ftps).withGains(10,0,1).withAngle(90_deg).withTurnGains(36,0,1).waitUntilSettled();
  chassis.drive(-20_in, .15_ftps2, .15_ftps2, -1.5_ftps).withGains(10,0,1).withAngle(90_deg).withTurnGains(36,0,1).waitUntilSettled();
  // MATCH LOADS 
  chassis.drive(18_in).withAngle(120_deg).withTurnGains(36,0,10).withTurnTol(5_deg).waitUntilSettled();
  chassis.turn(15_deg).withTurnGains(20,0,10).waitUntilSettled();
  frontLift.delayClamp(1_s);
  chassis.drive(20_in).withAngle(10_deg).withTol(4_in).withTurnTol(5_deg).waitUntilSettled();
  // THIRD GOAL

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