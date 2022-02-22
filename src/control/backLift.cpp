#include "backLift.hpp"
#include "globals.hpp"
#include "misc.hpp"
#include "pros/rtos.hpp"

BackLiftState BackLiftMode = BackLiftState::AUTON;

bool BackLift::isRunning = false, BackLift::clampState = false, BackLift::lastClampState = !clampState, BackLift::checkDist = true;
double BackLift::delay = 0;

bool checkJam = false;

BackLiftState BackLift::getState(){
  return BackLiftMode;
}

bool BackLift::getClampState(){
  return clampState;
}

BackLift& BackLift::setState(BackLiftState s){
  BackLiftMode = s;
  return *this;
}

BackLift& BackLift::toggleClamp(){
  clampState = !clampState;
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
    case BackLiftState::AUTON: {
      updateClamp();
      break;
    }
    case BackLiftState::OPCONTROL: {
      if( backDist.get() >= 45 || backDist.get() == 0){
        checkDist = true;
      }
      
      if ( master.get_digital(DIGITAL_LEFT) ){ // Reverse Intake
        conveyer::spin(-600);
      } else if ( master.get_digital_new_press(DIGITAL_R2) ) { // Grab Goal
        toggleClamp().updateClamp();
      } else if( checkDist && backDist.get() <= 30 && backDist.get() != 0 ){
        clampState = true;
        updateClamp();
        checkDist = false;
      }
          if(checkJam && intake.get_efficiency() == 0){
            conveyer::spin(-600);
            pros::delay(300);
            conveyer::spin(600);
        }
      if(intake.get_actual_velocity() == 600) checkJam = clampState;
      if(!clampState) checkJam = false;
      break;
    }
    }
    end:

    pros::delay(10);
  }
}

void BackLift::updateClamp() {
    backClamp.set_value(clampState);
    pros::delay(100);
    clampState ? conveyer::spin(600) : conveyer::spin(0);
    lastClampState = clampState;
}
