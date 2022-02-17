#include "backLift.hpp"
#include "globals.hpp"
#include "misc.hpp"

BackLiftState BackLiftMode = BackLiftState::DOWN;

bool BackLift::isRunning = false;
double BackLift::delay = 0;

BackLiftState BackLift::getState(){
  return BackLiftMode;
}

BackLift& BackLift::setState(BackLiftState s){
  BackLiftMode = s;
  return *this;
}

BackLift& BackLift::setState(BackLiftState s, double delay_){
  BackLiftMode = s;
  delay = delay_;
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
      backClamp.set_value(false);
      conveyer::spin(0);
      break;
    }
    case BackLiftState::UP: {
      backClamp.set_value(true);
      pros::delay(delay);
      conveyer::spin(600);
      macro::print("RINGS SCORED ", 1);
      break;
    }
    case BackLiftState::OPCONTROL: {
      if (master.get_digital(DIGITAL_X)) { // Drop Goal
        backClamp.set_value(false);
        conveyer::spin(0);
      } else if (master.get_digital(DIGITAL_LEFT)){ // Reverse Intake
        conveyer::spin(-600);
      } else if (master.get_digital(DIGITAL_UP) || backLimit.get_new_press()) { // Grab Goal
        backClamp.set_value(true);
        pros::delay(100);
        conveyer::spin(600);
      } else if (intake.get_efficiency() <= 10 && intake.get_target_velocity() == 600) { // Jam Detection
        conveyer::spin(-600);
        pros::delay(300);
        conveyer::spin(600);
      }
      break;
    }
    }
    end:

    pros::delay(10);
  }
}
