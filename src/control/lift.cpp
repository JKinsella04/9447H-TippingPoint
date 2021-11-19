#include "lift.hpp"
#include "misc.hpp"
#include "chassis.hpp"

Chassis chassis;

LiftState liftMode = LiftState::IDLE;

macro::PID lift_PID(20, 0.1, 5);
macro::Slew lift_Slew(600);

double Lift::output = 0, Lift::target = 0, Lift::tol = 50, Lift::slewOutput = 0, Lift::lastTarget = -100,
      Lift::current = (leftArm.get_position() + rightArm.get_position() )/2;

double tempLiftPos;

bool Lift::isRunning = false, Lift::isSettled = true, Lift::isDelayingClamp = false;

LiftState Lift::getState(){
  return liftMode;
}

Lift& Lift::setState(LiftState s){
  isSettled = false;
  liftMode = s;
  return *this;
}

Lift& Lift::setClamp(bool state_){
  clamp.set_value(state_);
  return *this;
}

Lift& Lift::delayClamp(bool state_){
  isDelayingClamp = true;
  clampState = state_;
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
      lift_PID.set(10,0.01,5);
      move(0);
      break;
    }
    case LiftState::MIDDLE: {
      lift_PID.set(21, 0.2, 7.5);
      move(500);
      break;
    }
    case LiftState::UP: {
      lift_PID.set(21, 0.2, 7.5);
      move(2000);
      break;
    }
    case LiftState::OPCONTROL: {
      current = ( leftArm.get_position() + rightArm.get_position() )/2;
      // Lift Control
      if (master.get_digital(DIGITAL_L1)) {
        lastTarget = -100;
        lift_PID.set(20, 0.1, 5);
        move(2000);
      } else if (master.get_digital(DIGITAL_L2)) {
        lastTarget = -100;
        lift_PID.set(10,0.01,5);
        move(0);
      } else if(current <= 500 && lf_Imu.get_roll() >= 5 || lf_Imu.get_roll() <= -5 ){
        lastTarget = -100;
        lift_PID.set(21, 0.2, 7.5);
        move(500);
      } else{
          if(lastTarget == -100) lastTarget = (leftArm.get_position() + rightArm.get_position() )/2;
          lift_Slew.reset();
          move(lastTarget);
         //Hold current Position.
      }


      // Clamp Control
      if (master.get_digital(DIGITAL_R1)) {
        clamp.set_value(true);
      } else if (master.get_digital(DIGITAL_R2)) {
        clamp.set_value(false);
      }
      break;
    }
    case LiftState::IDLE: {
      leftArm.move(0);
      rightArm.move(0);
      // Lift motor to zero;
    }
    }

    if(isDelayingClamp){
      if(abs( chassis.getDriveError()) < chassis.getTol() * 2) setClamp(clampState);
    }else{
      clamp.set_value(clampState);
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