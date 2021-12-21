#include "backLift.hpp"
#include "misc.hpp"

BackLiftState BackLiftMode = BackLiftState::UP;

bool BackLift::isRunning = false;

BackLiftState BackLift::getState(){
  return BackLiftMode;
}

BackLift& BackLift::setState(BackLiftState s){
  BackLiftMode = s;
  return *this;
}

void BackLift::start(void * ignore) {
  if (!isRunning) {
    pros::delay(500);
    BackLift *that = static_cast<BackLift *>(ignore);
    that->run();
  }
}

void BackLift::run() {
  isRunning = true;

  while (isRunning) {

    if(pros::competition::is_disabled()) goto end;

    switch (BackLiftMode) {
    case BackLiftState::DOWN: {
      backArm.set_value(true);
      backClamp.set_value(false);
      conveyer::spin(0);
      break;
    }
    case BackLiftState::UP: {
      backArm.set_value(false);
      backClamp.set_value(true);
      conveyer::spin(127);
      break;
    }
    case BackLiftState::OPCONTROL: {
      if (master.get_digital(DIGITAL_UP)) {
        backArm.set_value(false);
        backClamp.set_value(true);
        conveyer::spin(127);
      } else if (master.get_digital(DIGITAL_X)) {
        backArm.set_value(true);
        backClamp.set_value(false);
        conveyer::spin(0);
      }
      break;
    }
    }
    
    if(master.get_digital(DIGITAL_DOWN)) conveyer::spin(-127);
    end:

    pros::delay(10);
  }
}
