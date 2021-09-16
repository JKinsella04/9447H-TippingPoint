#include "lift.hpp"
#include "misc.hpp"

LiftState liftMode = LiftState::IDLE;

macro::PID lift_PID(0.1, 0.01, 0.05);
macro::Slew lift_Slew(900, 900, true);

double Lift::output = 0, Lift::target = 0, Lift::current = 0, Lift::tol = 10;

bool Lift::isRunning = false, Lift::isSettled = true;

LiftState Lift::getState(){
  return liftMode;
}

Lift& Lift::setState(LiftState s){
  liftMode = s;
  return *this;
}

void Lift::setBrakeType(pros::motor_brake_mode_e_t state){
  leftArm.set_brake_mode(state);
  rightArm.set_brake_mode(state);
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
    case LiftState::UP: {
      move(100);
      break;
    }
    case LiftState::OPCONTROL: {
      if (master.get_digital(DIGITAL_L1)) {
        leftArm.move(127);
        rightArm.move(127);
      } else if (master.get_digital(DIGITAL_L2)) {
        leftArm.move(-127);
        rightArm.move(-127);
      } else {
        leftArm.move(0);
        rightArm.move(0);
      }
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
