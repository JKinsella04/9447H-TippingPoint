#include "frontLift.hpp"
#include "chassis.hpp"
#include "display/lv_misc/lv_symbol_def.h"
#include "misc.hpp"

static Chassis chassis;

FrontLiftState FrontLiftMode = FrontLiftState::DOWN;

macro::PID FrontLift_PID(30, 1, 12.5);
macro::Slew FrontLift_Slew(1100,600, true);

double FrontLift::output = 0, FrontLift::target = 0, FrontLift::tol = 75,
       FrontLift::slewOutput = 0, FrontLift::current = arm.get_position();

bool FrontLift::isRunning = false, FrontLift::isSettled = true,
     FrontLift::isDelayingClamp = false, FrontLift::clampState = false,
     FrontLift::lastClampState = !clampState, FrontLift::checkFrontLift = true;

PID_constants up{70, 1, 6}, mid{30, 0.01, 12.5}, down{15, 0.01, 5};
double FrontLift::downPos = 200, FrontLift::midPos = 750, FrontLift::upPos = 2000, FrontLift::delay = 100; 

FrontLiftState FrontLift::getState() { return FrontLiftMode; }

FrontLift &FrontLift::setState(FrontLiftState s) {
  isSettled = false;
  FrontLiftMode = s;
  return *this;
}

FrontLift &FrontLift::setState(FrontLiftState s, double delay_) {
  isSettled = false;
  FrontLiftMode = s;
  delay = delay_;
  return *this;
}

FrontLift &FrontLift::toggleClamp() {
  clampState = !clampState;
  return *this;
}

FrontLift &FrontLift::delayClamp(bool state_) {
  isDelayingClamp = true;
  clampState = state_;
  return *this;
}

FrontLift &FrontLift::withTol(double tol_){
  tol = tol_;
  return *this;
}

void FrontLift::setBrakeType(pros::motor_brake_mode_e_t state) {
  arm.set_brake_mode(state);
}

void FrontLift::waitUntilSettled() {
  while (!isSettled) {
    pros::delay(20);
  }
}

void FrontLift::reset() { arm.tare_position(); }

void FrontLift::start(void *ignore) {
  if (!isRunning) {
    pros::delay(500);
    FrontLift *that = static_cast<FrontLift *>(ignore);
    that->run();
  }
}

void FrontLift::run() {
  isRunning = true;

  while (isRunning) {

    if (pros::competition::is_disabled())
      goto end;

    switch (FrontLiftMode) {
    case FrontLiftState::DOWN: {
      FrontLift_PID.set(down.kP, down.kI, down.kD);
      move(downPos);
      break;
    }
    case FrontLiftState::MIDDLE: {
      FrontLift_PID.set(mid.kP, mid.kI, mid.kD);
      move(midPos);
      break;
    }
    case FrontLiftState::UP: {
      if(!clampState) pros::delay(delay); // If grabbing goal delay moving to ensure goal is grabbed.
      FrontLift_PID.set(up.kP, up.kI, up.kD);
      move(upPos - 100);
      break;
    }
    case FrontLiftState::OPCONTROL: {
      // FrontLift Control
      if (master.get_digital(DIGITAL_L1)) {
        checkFrontLift = true;
        FrontLift_PID.set(up.kP, up.kI, up.kD);
        target = upPos;
      } else if (master.get_digital(DIGITAL_L2)) {
        checkFrontLift = true;
        FrontLift_PID.set(down.kP, down.kI, down.kD);
        target = downPos;
      } else if (arm.get_position() <= 500 && L_Imu.get_roll() >= 15 || L_Imu.get_roll() <= -15) {
        FrontLift_PID.set(mid.kP, mid.kI, mid.kD);
        target = midPos;
      } else {
        if (checkFrontLift) {
          FrontLift_Slew.reset();
          target = arm.get_position();
          checkFrontLift = false;
        }
      }
      move(target);

      // Clamp Control
      if (master.get_digital_new_press(DIGITAL_R1)) {
        toggleClamp().updateClamp();
      }
      break;
    }
    }
    updateClamp();
  end:

    pros::delay(10);
  }
}

void FrontLift::move(double target) {
  current = arm.get_position();

  output = FrontLift_PID.calculate(target, current);

  slewOutput = FrontLift_Slew.withLimit(12000).calculate(output);

  // macro::print("Output: ", slewOutput);

  arm.move_voltage(slewOutput);

  if (fabs(FrontLift_PID.getError()) < tol) {
    if (!pros::competition::is_autonomous()) {
      FrontLiftMode = FrontLiftState::OPCONTROL;
    }
    isSettled = true;
  }
  
}

void FrontLift::updateClamp() {
  if (clampState != lastClampState && isDelayingClamp) {
    if (abs(chassis.getDriveError()) < chassis.getTol() * 2) {
      frontClamp.set_value(clampState);
      lastClampState = clampState;
    }
  } else if (clampState != lastClampState) {
    frontClamp.set_value(clampState);
    lastClampState = clampState;
  }
}