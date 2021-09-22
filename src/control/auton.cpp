#include "auton.hpp"
#include "chassis.hpp"
#include "misc.hpp"

// Class Init
static Chassis chassis;

std::string Autonomous::name;

int Autonomous::id;

Autonomous::Autonomous() {}

std::string Autonomous::getAuton() { return name; }

void Autonomous::setId(int id_) {
  id = id_;
  switch (id) {
  case 1:
    name = "Home Row";
    break;
  case 2:
    name = "One Goal";
    break;
  case 3:
    name = "Two Goal";
    break;
  default:
    name = "ERROR INVALID AUTON";
    break;
  }
}

void Autonomous::runAuton() {
  switch (id) {
  case 1: {
    homeRow();
    break;
  }
  case 2: {
    oneGoal();
    break;
  }
  case 3: {
    twoGoal();
    break;
  }

  default: {
    break;
  }
  }
}

// Match Autons
void homeRow() {
  // chassis.drive(1000, 0, 900, 9000, 900, 12000).withGains(9,0,0).withTol(10).waitUntilSettled();
  // chassis.drive(0, 0, 900, 9000, 900, 12000).withTol(10).waitUntilSettled();

  chassis.eDrive(50).withTol(40).waitUntilSettled();
}

void oneGoal() {
  chassis.turn(-90, 900, 9000).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();
  // chassis.turn(0, 900, 9000).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();
  // chassis.turn(90, 900, 9000).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();
  
}

void twoGoal() {}

// Skills
void skills();

// Testing
void test() {
  //   Chassis.drive(20).withGains(.3, 0.01,
  //   .15).withTol(100).waitUntilSettled(); Chassis.turn(90).withGains(133,
  //   0.01, 66).withTol(1).waitUntilSettled();
}