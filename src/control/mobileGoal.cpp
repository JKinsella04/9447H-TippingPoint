#include "mobileGoal.hpp"
#include "misc.hpp"

MobileGoalState MobileGoalMode = MobileGoalState::IDLE;

macro::PID MobileGoal_PID(5, 0, 2.5);
macro::Slew MobileGoal_Slew(600);

double MobileGoal::output = 0, MobileGoal::target = 0, MobileGoal::current = 0, MobileGoal::tol = 30, MobileGoal::slewOutput = 0, MobileGoal::lastTarget = 0;

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
    case MobileGoalState::ZERO: {
      move(2120);
      break;
    }
    case MobileGoalState::UP: {
      move(100);
      break;
    }
    case MobileGoalState::OPCONTROL: {
      if (master.get_digital(DIGITAL_UP))
      {
        double temp = (leftMobileGoal.get_position() + rightMobileGoal.get_position())/2;
        std::cout << temp << std::endl;
        lastTarget = -750;
      }
      else if (master.get_digital(DIGITAL_X))
      {
        lastTarget = -1520;
      }
      move(lastTarget);
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
  // current = mobileGoalPos.get_value();
  current = ( leftMobileGoal.get_position() + rightMobileGoal.get_position() )/2;

  output = MobileGoal_PID.calculate(target, current);

  slewOutput = MobileGoal_Slew.calculate(output);

  leftMobileGoal.move_voltage(slewOutput);
  rightMobileGoal.move_voltage(slewOutput);

  if(fabs(MobileGoal_PID.getError()) < tol){ 
    if (!pros::competition::is_autonomous()) {
      MobileGoalMode = MobileGoalState::OPCONTROL;
    } else {
      MobileGoalMode = MobileGoalState::IDLE;
    }
    isSettled = true;
  }
}
