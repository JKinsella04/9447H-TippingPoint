#include "lift.hpp"
#include "misc.hpp"
#include "chassis.hpp"

Chassis chassis;

LiftState liftMode = LiftState::DOWN;

macro::PID lift_PID(20, 0.1, 5);
macro::Slew lift_Slew(600);

double Lift::output = 0, Lift::target = 0, Lift::tol = 75, Lift::slewOutput = 0,
      Lift::current = arm.get_position();

double tempLiftPos;

bool Lift::isRunning = false, Lift::isSettled = true, Lift::isDelayingClamp = false, Lift::clampState = false, Lift::lastClampState = !clampState, Lift::checkLift = false;

LiftState Lift::getState(){
  return liftMode;
}

Lift& Lift::setState(LiftState s){
  isSettled = false;
  liftMode = s;
  return *this;
}

Lift& Lift::setClamp(bool state_){
  clampState = state_;
  return *this;
}

Lift& Lift::delayClamp(bool state_){
  isDelayingClamp = true;
  clampState = state_;
  return *this;
}

void Lift::setBrakeType(pros::motor_brake_mode_e_t state){
  arm.set_brake_mode(state);
}

void Lift::waitUntilSettled(){
  while(!isSettled){ pros::delay(20);}
}

void Lift::reset(){
  arm.tare_position();
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
      lift_PID.set(22, 0.2, 7.5);
      move(2000);
      break;
    }
    case LiftState::OPCONTROL: {
      current = arm.get_position();
      // Lift Control
      if (master.get_digital(DIGITAL_L1)) {
        checkLift = true;
        lift_PID.set(20, 0.1, 5);
        move(2000);
      } else if (master.get_digital(DIGITAL_L2)) {
        checkLift = true;
        lift_PID.set(10,0.01,5);
        move(0);
      } else if(current <= 500 && lf_Imu.get_roll() >= 10 || lf_Imu.get_roll() <= -10 ){
        checkLift = true;
        lift_PID.set(21, 0.2, 7.5);
        move(500);
      } else{
          if(checkLift) current = arm.get_position();
          lift_Slew.reset();
          move(current);
          checkLift = false;
      }

      // Clamp Control
      if (master.get_digital(DIGITAL_R1)) {
        clamp.set_value(true);
      } else if (master.get_digital(DIGITAL_R2)) {
        clamp.set_value(false);
      }
      break;
    }
    }

    if (clampState != lastClampState && isDelayingClamp) {
      if (abs(chassis.getDriveError()) < chassis.getTol() * 2) {
        clamp.set_value(clampState);
        lastClampState = clampState;
      }
    } else if (clampState != lastClampState) {
      clamp.set_value(clampState);
      lastClampState = clampState;
    }

    end:

    pros::delay(10);
  }
}

void Lift::move(double target){
  current = arm.get_position();

  output = lift_PID.calculate(target, current);

  slewOutput = lift_Slew.calculate(output);

  arm.move_voltage(slewOutput);

  if (fabs(lift_PID.getError()) < tol) {
    if (!pros::competition::is_autonomous()) {
      liftMode = LiftState::OPCONTROL;
    } else {
      // liftMode = LiftState::IDLE;
    }
    isSettled = true;
  }
}