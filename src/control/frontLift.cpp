#include "frontLift.hpp"
#include "chassis.hpp"
#include "misc.hpp"
#include "positionTracking.hpp"

static Chassis chassis;
static macro::GoalCover goalCover;
PositionTracker *FrontLift::robot;

FrontLiftState FrontLiftMode = FrontLiftState::DOWN;

macro::PID FrontLift_PID(70, 1.5, 45);
macro::Slew FrontLift_Slew(1100,600, true);

double FrontLift::output = 0, FrontLift::target = 0, FrontLift::tol = 75,
       FrontLift::slewOutput = 0, FrontLift::current = arm.get_position();

bool FrontLift::isRunning = false, FrontLift::isSettled = true,
     FrontLift::isDelayingClamp = false, FrontLift::clampState = false,
     FrontLift::lastClampState = !clampState, FrontLift::checkFrontLift = true;

PID_constants up{40, 1, 20}, mid{30, 0.01, 12.5}, down{20, 0.01, 5};
double FrontLift::downPos = 350, FrontLift::midPos = 2000, FrontLift::upPos = 4100;
QTime FrontLift::delay = 100_ms, FrontLift::lastTimeCheck; 

FrontLiftState FrontLift::getState() { return FrontLiftMode; }

bool FrontLift::getClampState(){ return clampState; }

FrontLift &FrontLift::setState(FrontLiftState s) {
  isSettled = false;
  FrontLiftMode = s;
  return *this;
}

FrontLift &FrontLift::setState(FrontLiftState s, QTime delay_) {
  isSettled = false;
  FrontLiftMode = s;
  delay = delay_;
  lastTimeCheck = robot->getTime();
  return *this;
}

FrontLift &FrontLift::toggleClamp() {
  clampState = !clampState;
  return *this;
}

FrontLift &FrontLift::delayClamp(QTime delay_) {
  isDelayingClamp = true;
  clampState = !clampState;
  delay = delay_;
  lastTimeCheck = robot->getTime();
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

void FrontLift::waitUntilClamped() {
  while (lastClampState != clampState) { // Delay until clamp is toggled.
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
      if(clampState) pros::delay(delay.convert(millisecond));  
      FrontLift_PID.set(mid.kP, mid.kI, mid.kD);
      move(midPos);
      break;
    }
    case FrontLiftState::UP: {
      if(clampState) pros::delay(delay.convert(millisecond));  
      FrontLift_PID.set(up.kP, up.kI, up.kD);
      move(upPos);
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
      if(master.get_digital_new_press(DIGITAL_A)){
        goalCover.toggle();
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
  if (isDelayingClamp) {
    if (robot->getTime() - lastTimeCheck >= delay) {
      frontClamp.set_value(clampState);
      lastClampState = clampState;
      delay = 75_ms;
    }
  } else {
    frontClamp.set_value(clampState);
    lastClampState = clampState;
  }
}
