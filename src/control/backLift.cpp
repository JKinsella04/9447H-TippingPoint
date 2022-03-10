#include "backLift.hpp"
#include "globals.hpp"
#include "misc.hpp"
#include "positionTracking.hpp"

static Position robotPos;

BackLiftState BackLiftMode = BackLiftState::AUTON;

bool BackLift::isRunning = false, BackLift::clampState = false, 
BackLift::lastClampState = !clampState, BackLift::checkDist = true, BackLift::isDelayingClamp = false;
double BackLift::delay = 150;

double lastTimeCheck = 0;
bool checkJam = false;

double intakeSpeed = 600;

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
  delay = 0;
  isDelayingClamp = false;
  return *this;
}

BackLift& BackLift::delayClamp(double delay_){
  clampState = !clampState;
  isDelayingClamp = true;
  delay = delay_ * 1000;
  lastTimeCheck = *robotPos.getTime();
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
      if (checkJam && intake.get_efficiency() == 0) {
        conveyer::spin(-intakeSpeed);
        pros::delay(300);
        conveyer::spin(intakeSpeed);
        checkJam = false;
      }
      if (intake.get_actual_velocity() == intakeSpeed)
        checkJam = clampState;
      updateClamp();
      break;
    }
    case BackLiftState::OPCONTROL: {
      delay = 1000;
      if( backDist.get() >= 45 || backDist.get() == 0){
        checkDist = true;
      }
      
      if ( master.get_digital_new_press(DIGITAL_R2) ) { // Grab Goal
        toggleClamp().updateClamp();
      } else if( checkDist && backDist.get() <= 30 && backDist.get() != 0 ){
        clampState = true;
        updateClamp();
        checkDist = false;
      }
          if(checkJam && intake.get_efficiency() == 0){
            conveyer::spin(-intakeSpeed);
            pros::delay(300);
            conveyer::spin(intakeSpeed);
            checkJam = false;
        }
      if(intake.get_actual_velocity() == intakeSpeed) checkJam = clampState;
      if(!clampState) checkJam = false;
      break;
    }
    case BackLiftState::IDLE:{
      break;
    }
    }
    // updateClamp();
    end:

    pros::delay(10);
  }
}

void BackLift::updateClamp() {
  if (isDelayingClamp) {
    if (*robotPos.getTime() - lastTimeCheck >= delay) {
      backClamp.set_value(clampState);
      pros::delay(delay);
      clampState ? conveyer::spin(intakeSpeed): conveyer::spin(0);
      isDelayingClamp = false;
    }
  } else {
    backClamp.set_value(clampState);
    pros::delay(250);
    clampState ? conveyer::spin(intakeSpeed) : conveyer::spin(0);
    if ( master.get_digital(DIGITAL_LEFT) ) conveyer::spin(-intakeSpeed);
  }
}