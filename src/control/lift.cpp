#include "lift.hpp"
#include "misc.hpp"
#include "pros/misc.hpp"

LiftState liftMode = LiftState::IDLE;

macro::PID lift_PID(20, 0.1, 5);
macro::Slew lift_Slew(600);

double Lift::output = 0, Lift::target = 0, Lift::tol = 40, Lift::slewOutput = 0, Lift::lastTarget = 0,
      Lift::current = (leftArm.get_position() + rightArm.get_position() )/2;

double tempLiftPos;

bool Lift::isRunning = false, Lift::isSettled = true;

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

void Lift::reset(){
  leftArm.tare_position();
  rightArm.tare_position();
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
    case LiftState::DOWN: {
      move(0);
      break;
    }
    case LiftState::UP: {
      move(1900);
      break;
    }
    case LiftState::OPCONTROL: {
      // Lift Control
      if (master.get_digital(DIGITAL_L1)) {
        lift_PID.set(20, 0.1, 5);
        move(1900);
      } else if (master.get_digital(DIGITAL_L2)) {
        lift_PID.set(10,0.01,5);
        move(0);
      }else{
          leftArm.move(0);
          rightArm.move(0); //Hold current Position.
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
  current = ( leftArm.get_position() + rightArm.get_position() )/2;

  output = lift_PID.calculate(target, current);

  slewOutput = lift_Slew.calculate(output);

  leftArm.move_voltage(slewOutput);
  rightArm.move_voltage(slewOutput);

  if (fabs(lift_PID.getError()) < tol) {
    if (!pros::competition::is_autonomous()) {
      liftMode = LiftState::OPCONTROL;
    } else {
      // liftMode = LiftState::IDLE;
    }
    isSettled = true;
  }
}