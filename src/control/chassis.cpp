#include "control/chassis.hpp"
#include "control/misc.hpp"
#include "pros/misc.hpp"

macro::PID drive_PID;
macro::PID turn_PID;

ChassisState chassis_mode = ChassisState::OFF;

bool Chassis::isRunning = false;

// Chassis::Chassis() { }

Chassis &Chassis::withGains(double kP_, double kI_, double kD_) {
  switch (chassis_mode) {
  case ChassisState::DRIVE:
    drive_PID.set(kP_, kD_, kI_);
    break;

  case ChassisState::TURN:
    turn_PID.set(kP_, kI_, kD_);
    break;
  
  default:
    break;
  }
  return *this;
}

Chassis &Chassis::withTol(int tol_) {
  tol = tol_;
  return *this;
}

Chassis &Chassis::drive(double target_) {
  target = target_;
  chassis_mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::turn(double theta_) {
  theta = theta_;
  chassis_mode = ChassisState::TURN;
  return *this;
}

void Chassis::start(void *ignore) {
  if (!isRunning) {
    pros::delay(500);
    Chassis *that = static_cast<Chassis *>(ignore);
    that->run();
  }
}

Chassis &Chassis::run() {
  isRunning = true;

  while (isRunning) {

    if(!pros::competition::is_autonomous()) goto end;

    switch (chassis_mode) {
    case ChassisState::DRIVE:
      current = ( LOdometer.get_position() + ROdometer.get_position() ) /2.0;
      drive_PID.calculate(target,  current);
      break;

    case ChassisState::TURN:
      current = ( L_IMU.get_heading() + M_IMU.get_heading() + R_IMU.get_heading() )/3;
      if( current >= 358) current = 0;
      turn_PID.calculate(target, current);
      break;

    case ChassisState::OPCONTROL:
      break;

    case ChassisState::OFF:
      break;
    }

    end:
    pros::delay(10);
  }
  return *this;
}
