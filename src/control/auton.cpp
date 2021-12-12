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
  frontLift.setState(FrontLiftState::UP).setClamp(true).waitUntilSettled();
  backLift.setState(BackLiftState::UP);
  conveyer::spin(127);  
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