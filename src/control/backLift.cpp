#include "backLift.hpp"
#include "globals.hpp"
#include "misc.hpp"
#include "positionTracking.hpp"

BackLiftState BackLiftMode = BackLiftState::AUTON;

PositionTracker *BackLift::robot;

bool BackLift::isRunning = false, BackLift::clampState = false, 
BackLift::lastClampState = true, BackLift::checkDist = true, BackLift::isDelayingClamp = false;

QTime BackLift::lastTimeCheck, BackLift::delay = 250_ms;
bool checkJam = false;
double BackLift::goalDist;

double intakeSpeed = 12000;

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
  delay = delay_;
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
    goalDist = (LB_dist.get() + RB_dist.get()) / 2.0;

    if(pros::competition::is_disabled()) goto end;

    switch (BackLiftMode) {
    case BackLiftState::AUTON: {
      if (clampState && intake.get_efficiency() == 0) {
        conveyer::spin(-intakeSpeed);
        pros::delay(100);
        conveyer::spin(intakeSpeed);
        checkJam = false;
      }
      if(clampState != lastClampState) pros::delay(delay.convert(millisecond));
      backClamp.set_value(clampState);
      clampState ? conveyer::spin(intakeSpeed) : conveyer::spin(0);
      lastClampState = clampState;
      break;
    }
    case BackLiftState::OPCONTROL: {
      if( goalDist >= 45 || goalDist == 0){
        checkDist = true;
      }
      
      if ( master.get_digital_new_press(DIGITAL_R2) ) { // Grab Goal
        toggleClamp(250_ms).updateClamp();
      } else if( checkDist && goalDist <= 25 && goalDist != 0 ){
        clampState = true;
        updateClamp();
        checkDist = false;
      }
      if (clampState && intake.get_efficiency() == 0) {
        conveyer::spin(-intakeSpeed);
        pros::delay(200);
        conveyer::spin(intakeSpeed);
        checkJam = false;
      }
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