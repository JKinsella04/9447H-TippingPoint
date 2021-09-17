#include "MobileGoal.hpp"
#include "misc.hpp"

MobileGoalState MobileGoalMode = MobileGoalState::IDLE;

macro::PID MobileGoal_PID(0.1, 0.01, 0.05);
macro::Slew MobileGoal_Slew(900, 900, true);

double MobileGoal::output = 0, MobileGoal::target = 0, MobileGoal::current = 0, MobileGoal::tol = 10;

bool MobileGoal::isRunning = false, MobileGoal::isSettled = true;

MobileGoalState MobileGoal::getState(){
  return MobileGoalMode;
}

MobileGoal& MobileGoal::setState(MobileGoalState s){
  MobileGoalMode = s;
  return *this;
}

void MobileGoal::setBrakeType(pros::motor_brake_mode_e_t state){
  leftMobileGoal.set_brake_mode(state);
  rightMobileGoal.set_brake_mode(state);
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
    case MobileGoalState::ZERO: {
      move(0);
      break;
    }
    case MobileGoalState::UP: {
      move(100);
      break;
    }
    case MobileGoalState::OPCONTROL: {
      if (master.get_digital(DIGITAL_A)) {
        move(2000);
      } else if (master.get_digital(DIGITAL_B)) {
        move(0);
      } else {
        leftMobileGoal.move(0);
        rightMobileGoal.move(0);
      }
      break;
    }
    case MobileGoalState::IDLE: {
      leftMobileGoal.move(0);
      rightMobileGoal.move(0);
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

  MobileGoal_Slew.calculate(output);

  leftMobileGoal.move_voltage(output);
  rightMobileGoal.move_voltage(output);

  if(fabs(MobileGoal_PID.getError()) < tol){ 
    MobileGoalMode = MobileGoalState::IDLE;
    isSettled = true;
  }
}
