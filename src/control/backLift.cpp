#include "backLift.hpp"
#include "misc.hpp"

BackLiftState BackLiftMode = BackLiftState::DOWN;

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
      backArm.set_value(false);
      backClamp.set_value(false);
      conveyer::spin(0);
      break;
    }
    case BackLiftState::UP: {
      backClamp.set_value(true);
      pros::delay(100);
      backArm.set_value(true);
      conveyer::spin(600);
      macro::print("RINGS SCORED ", 1);
      break;
    }
    case BackLiftState::OPCONTROL: {
      if (master.get_digital(DIGITAL_UP)) {
        backClamp.set_value(true);
        pros::delay(100);
        backArm.set_value(true);
        conveyer::spin(600);
      } else if (master.get_digital(DIGITAL_X)) {
        backArm.set_value(false);
        backClamp.set_value(false);
        conveyer::spin(0);
      }
      if(master.get_digital(DIGITAL_LEFT)) conveyer::spin(-600);
      break;

      if(intake.get_actual_velocity() <= 150 && intake.get_target_velocity() == 600) {
        conveyer::spin(-600);
        pros::delay(300);
        conveyer::spin(600);
      }
    }
    }
    end:

    pros::delay(10);
  }
}