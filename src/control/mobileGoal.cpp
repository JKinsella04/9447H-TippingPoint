#include "mobileGoal.hpp"
#include "misc.hpp"

MobileGoalState MobileGoalMode = MobileGoalState::IDLE;

macro::PID MobileGoal_PID(5, 0.01, 3.75);
macro::Slew MobileGoal_Slew(600);

double MobileGoal::output = 0, MobileGoal::target = 0, MobileGoal::tol = 30, MobileGoal::lastTarget = 0,
       MobileGoal::slewOutput = 0, MobileGoal::current = mobileGoalPos.get_value();

bool MobileGoal::isRunning = false, MobileGoal::isSettled = true;

MobileGoalState MobileGoal::getState(){
  return MobileGoalMode;
}

MobileGoal& MobileGoal::setState(MobileGoalState s){
  MobileGoalMode = s;
  return *this;
}

MobileGoal& MobileGoal::setup(){
  isSettled = false;
  // leftMobileGoal.move_absolute(-800, 127);
  // rightMobileGoal.move_absolute(-800, 127);
  leftMobileGoal.move(-100);
  rightMobileGoal.move(-100);
  macro::print("SETUP", 0);
  pros::delay(250);
  setState(MobileGoalState::DOWN);
  return *this;
}

void MobileGoal::setBrakeType(pros::motor_brake_mode_e_t state){
  leftMobileGoal.set_brake_mode(state);
  rightMobileGoal.set_brake_mode(state);
}

void MobileGoal::reset(){
  leftMobileGoal.tare_position();
  rightMobileGoal.tare_position();
}


void MobileGoal::waitUntilSettled(){
  while(!isSettled){ pros::delay(20);}
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
      move(3900);
      break;
    }
    case MobileGoalState::UP: {
      move(500);
      break;
    }
    case MobileGoalState::OPCONTROL: {
      if (master.get_digital(DIGITAL_UP))
      {
        isSettled = false;
        move(500);
      }
      else if (master.get_digital(DIGITAL_X))
      {
        isSettled = false;
        move(3900);
      }else{
      if(!isSettled){
        isSettled = true;
        leftMobileGoal.move(0);
        rightMobileGoal.move(0);
        lastTarget = mobileGoalPos.get_value();
      }
      move(lastTarget); // Hold current Position.
      }
      break;
    }
    case MobileGoalState::IDLE: {
      // macro::print("IDLE", 0);
      // leftMobileGoal.move(0);
      // rightMobileGoal.move(0);
      // MobileGoal motor to zero;
    }
    }

    end:

    pros::delay(10);
  }
}

void MobileGoal::move(double target){
  current = mobileGoalPos.get_value();

  output = MobileGoal_PID.calculate(target, current);

  slewOutput = MobileGoal_Slew.calculate(output);

  leftMobileGoal.move_voltage(-slewOutput);
  rightMobileGoal.move_voltage(-slewOutput);

  if(fabs(MobileGoal_PID.getError()) < tol){ 
    if (!pros::competition::is_autonomous()) {
      MobileGoalMode = MobileGoalState::OPCONTROL;
    } else {
      // MobileGoalMode = MobileGoalState::IDLE;
    }
    isSettled = true;
  }
}
