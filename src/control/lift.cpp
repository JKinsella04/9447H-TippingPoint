#include "lift.hpp"
#include "misc.hpp"

LiftState liftMode = LiftState::IDLE;

macro::PID lift_PID(0.1, 0.01, 0.05);
macro::Slew lift_Slew(900, 900, true);

double Lift::output = 0, Lift::target = 0, Lift::current = 0, Lift::tol = 10;

double *Lift::liftPos;

bool Lift::isRunning = false, Lift::isSettled = true;

Lift::Lift(){ }

Lift::Lift(double liftPos_){
  liftPos = &liftPos_;
}

LiftState Lift::getState(){
  return liftMode;
}

Lift& Lift::setState(LiftState s){
  liftMode = s;
  return *this;
}

Lift& Lift::setClamp(bool state_){
  clamp.set_value(state_);
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
      // Lift Control
      if (master.get_digital(DIGITAL_L1)) {
        double output = lift_Slew.calculate(12000);
        leftArm.move_voltage(output);
        rightArm.move_voltage(output);
        
      } else if (master.get_digital(DIGITAL_L2)) {
        double output = lift_Slew.calculate(-12000);
        leftArm.move_voltage(output);
        rightArm.move_voltage(output);

      } else {
        leftArm.move(0);
        rightArm.move(0);
      }

      // Clamp Control
      if (master.get_digital(DIGITAL_R1)) {
        setClamp(true);
      } else if (master.get_digital(DIGITAL_R2)) {
        setClamp(false);
      }
      break;
    }
    case LiftState::IDLE: {
      leftArm.move(0);
      rightArm.move(0);
      // Lift motor to zero;
    }
    }

    end:

    pros::delay(10);
  }
}

void Lift::move(double target){
  output = lift_PID.calculate(target, *liftPos);

  lift_Slew.calculate(output);

  leftArm.move_voltage(output);
  rightArm.move_voltage(output);

  if(fabs(lift_PID.getError()) < tol){ 
    liftMode = LiftState::IDLE;
    isSettled = true;
  }
}
