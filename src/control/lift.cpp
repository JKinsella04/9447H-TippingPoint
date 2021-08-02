#include "lift.hpp"
#include "misc.hpp"

LiftState liftMode = LiftState::IDLE;

macro::PID lift_PID;
macro::Slew lift_Slew(900, 900, true);

double Lift::output = 0, Lift::target = 0, Lift::current = 0, Lift::tol = 10;

LiftState Lift::getState(){
  return liftMode;
}

Lift& Lift::setState(LiftState s){
  liftMode = s;
  return *this;
}

void Lift::waitUntilSettled(){
  while(!isSettled){ pros::delay(20);}
}

void Lift::start(void * ignore) {
  if (!isRunning) {
    pros::delay(500);
    Lift *that = static_cast<Lift *>(ignore);
    that->run();
  }
}

void Lift::run() {
  isRunning = true;

  while (isRunning) {

    if(pros::competition::is_disabled()) goto end;

    switch (liftMode) {
    case LiftState::ZERO: {
      move(0);
      break;
    }
    case LiftState::SMALL: {
      move(100);
      break;
    }
    case LiftState::MIDDLE: {
      move(180);
      break;
    }
    case LiftState::TALL: {
      move(270);
      break;
    }
    case LiftState::IDLE: {
      // Lift motor to zero;
    }
    }
    
    end:

    pros::delay(10);
  }
}

void Lift::move(double target){
  current = liftPos.get_value_calibrated();

  output = lift_PID.calculate(target, current);

  lift_Slew.calculate(output);

  // lift.move_voltage(output);

  if(fabs(lift_PID.getError()) < tol){ 
    liftMode = LiftState::IDLE;
    isSettled = true;
  }
}
