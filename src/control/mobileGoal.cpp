#include "mobileGoal.hpp"
#include "misc.hpp"

MobileGoalState MobileGoalMode = MobileGoalState::IDLE;

macro::PID MobileGoal_PID(5, 0.01, 3.75);
macro::Slew MobileGoal_Slew(600);

double MobileGoal::output = 0, MobileGoal::target = 0, MobileGoal::current = 0, MobileGoal::tol = 30, MobileGoal::slewOutput = 0, MobileGoal::lastTarget = 4000;

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

void MobileGoal::setup(){
  leftMobileGoal.move_absolute(-800, 127);
  rightMobileGoal.move_absolute(-800, 127);
  std::cout << "SETUP" << std::endl;
  pros::delay(2000);
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
      move(4000);
      std::cout << "DOWN" << std::endl;
      break;
    }
    case MobileGoalState::UP: {
      move(500);
      break;
    }
    case MobileGoalState::OPCONTROL: {
      if (master.get_digital(DIGITAL_UP))
      {
        lastTarget = 500;
      }
      else if (master.get_digital(DIGITAL_X))
      {
        lastTarget = 4000;
      }
      move(lastTarget);
      break;
    }
    case MobileGoalState::IDLE: {
      std::cout << "IDLE" << std::endl;
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
      MobileGoalMode = MobileGoalState::IDLE;
    }
    isSettled = true;
  }
}
