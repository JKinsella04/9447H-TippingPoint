#include "mobileGoal.hpp"
#include "misc.hpp"

MobileGoalState MobileGoalMode = MobileGoalState::UP;

bool MobileGoal::isRunning = false;

MobileGoalState MobileGoal::getState(){
  return MobileGoalMode;
}

MobileGoal& MobileGoal::setState(MobileGoalState s){
  MobileGoalMode = s;
  return *this;
}

void MobileGoal::start(void * ignore) {
  if (!isRunning) {
    pros::delay(500);
    MobileGoal *that = static_cast<MobileGoal *>(ignore);
    that->run();
  }
}

void MobileGoal::run() {
  isRunning = true;

  while (isRunning) {

    if(pros::competition::is_disabled()) goto end;

    switch (MobileGoalMode) {
    case MobileGoalState::DOWN: {
      mg.set_value(true);
      break;
    }
    case MobileGoalState::UP: {
      mg.set_value(false);
      break;
    }
    case MobileGoalState::OPCONTROL: {
      if (master.get_digital(DIGITAL_UP)) {
        mg.set_value(false);
      } else if (master.get_digital(DIGITAL_X)) {
        mg.set_value(true);
      }
      break;
    }
    }

    end:

    pros::delay(10);
  }
}