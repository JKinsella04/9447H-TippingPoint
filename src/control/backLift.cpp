#include "backLift.hpp"
#include "globals.hpp"
#include "misc.hpp"
#include "positionTracking.hpp"

BackLiftState BackLiftMode = BackLiftState::AUTON;

PositionTracker *BackLift::robot;

bool BackLift::isRunning = false, BackLift::clampState = false, 
BackLift::lastClampState = true, BackLift::checkDist = true, BackLift::isDelayingClamp = false;

QTime BackLift::lastTimeCheck, BackLift::delay = 250 * millisecond;
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

BackLift& BackLift::toggleClamp(QTime delay_){
  clampState = !clampState;
  delay = delay_;
  isDelayingClamp = false;
  return *this;
}

BackLift& BackLift::delayClamp(QTime delay_){
  clampState = !clampState;
  isDelayingClamp = true;
  delay = delay_ * 1000;
  lastTimeCheck = robot->getTime();
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
      if (!clampState)
        checkJam = false;
      if(clampState != lastClampState) pros::delay(delay.convert(millisecond));
      backClamp.set_value(clampState);
      clampState ? conveyer::spin(600) : conveyer::spin(0);
      lastClampState = clampState;
      break;
    }
    case BackLiftState::OPCONTROL: {
      if( backDist.get() >= 45 || backDist.get() == 0){
        checkDist = true;
      }
      
      if ( master.get_digital_new_press(DIGITAL_R2) ) { // Grab Goal
        toggleClamp(250_ms).updateClamp();
      } else if( checkDist && backDist.get() <= 30 && backDist.get() != 0 ){
        clampState = true;
        updateClamp();
        checkDist = false;
      }
      if (checkJam && intake.get_efficiency() == 0) {
        conveyer::spin(-intakeSpeed);
        pros::delay(300);
        conveyer::spin(intakeSpeed);
        checkJam = false;
      }
      if(intake.get_actual_velocity() == intakeSpeed) checkJam = clampState;
      if(!clampState) checkJam = false;
      if ( master.get_digital(DIGITAL_LEFT) ) conveyer::spin(-intakeSpeed);
      else{ clampState ? conveyer::spin(intakeSpeed) : conveyer::spin(0); }
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
    if (robot->getTime() - lastTimeCheck >= delay) {
      backClamp.set_value(clampState);
      pros::delay(delay.convert(millisecond));
      clampState ? conveyer::spin(intakeSpeed): conveyer::spin(0);
      isDelayingClamp = false;
      lastClampState = clampState;
    }
  } else {
    backClamp.set_value(clampState);
    pros::delay(delay.convert(millisecond));
    clampState ? conveyer::spin(intakeSpeed) : conveyer::spin(0);
    lastClampState = clampState;
 }
}