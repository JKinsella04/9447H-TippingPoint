#include "frontLift.hpp"
#include "chassis.hpp"
#include "misc.hpp"


Chassis chassis;

FrontLiftState FrontLiftMode = FrontLiftState::DOWN;

macro::PID FrontLift_PID(20, 0.1, 5);
macro::Slew FrontLift_Slew(600);

double FrontLift::output = 0, FrontLift::target = 0, FrontLift::tol = 75,
       FrontLift::slewOutput = 0, FrontLift::current = arm.get_position();

double tempFrontLiftPos;

bool FrontLift::isRunning = false, FrontLift::isSettled = true,
     FrontLift::isDelayingClamp = false, FrontLift::clampState = false,
     FrontLift::lastClampState = !clampState, FrontLift::checkFrontLift = false;

FrontLiftState FrontLift::getState() { return FrontLiftMode; }

FrontLift &FrontLift::setState(FrontLiftState s) {
  isSettled = false;
  FrontLiftMode = s;
  return *this;
}

FrontLift &FrontLift::setClamp(bool state_) {
  clampState = state_;
  return *this;
}

FrontLift &FrontLift::delayClamp(bool state_) {
  isDelayingClamp = true;
  clampState = state_;
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
      FrontLift_PID.set(10, 0.01, 5);
      move(0);
      break;
    }
    case FrontLiftState::MIDDLE: {
      FrontLift_PID.set(21, 0.2, 7.5);
      move(500);
      break;
    }
    case FrontLiftState::UP: {
      FrontLift_PID.set(22, 0.2, 7.5);
      move(2000);
      break;
    }
    case FrontLiftState::OPCONTROL: {
      // FrontLift Control
      current = arm.get_position();
      if (master.get_digital(DIGITAL_L1)) {
        checkFrontLift = true;
        FrontLift_PID.set(20, 0.1, 5);
        move(2000);
      } else if (master.get_digital(DIGITAL_L2)) {
        checkFrontLift = true;
        FrontLift_PID.set(10, 0.01, 5);
        move(0);
      } else if (current <= 500 && L_Imu.get_roll() >= 10 || L_Imu.get_roll() <= -10) {
        checkFrontLift = true;
        FrontLift_PID.set(21, 0.2, 7.5);
        move(500);
      } else {
        if (checkFrontLift)
          current = arm.get_position();
        FrontLift_Slew.reset();
        move(current);
        checkFrontLift = false;
      }

      // Clamp Control
      if (master.get_digital(DIGITAL_R1)) {
        frontClamp.set_value(true);
      } else if (master.get_digital(DIGITAL_R2)) {
        frontClamp.set_value(false);
      }
      break;
    }
    }

    // Delayed Clamp Control
    if (clampState != lastClampState && isDelayingClamp) {
      if (abs(chassis.getDriveError()) < chassis.getTol() * 2) {
        frontClamp.set_value(clampState);
        lastClampState = clampState;
      }
    } else if (clampState != lastClampState) {
      frontClamp.set_value(clampState);
      lastClampState = clampState;
    }

  end:

    pros::delay(10);
  }
}

void FrontLift::move(double target) {
  current = arm.get_position();

  output = FrontLift_PID.calculate(target, current);

  slewOutput = FrontLift_Slew.calculate(output);

  arm.move_voltage(slewOutput);

  if (fabs(FrontLift_PID.getError()) < tol) {
    if (!pros::competition::is_autonomous()) {
      FrontLiftMode = FrontLiftState::OPCONTROL;
    }
    isSettled = true;
  }
}