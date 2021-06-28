#include "control/chassis.hpp"
#include "control/misc.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"

macro::PID drive_PID;
macro::PID turn_PID;

macro::Slew left(900, 900, true);
macro::Slew right(900, 900, true);

ChassisState chassis_mode = ChassisState::IDLE;

bool Chassis::isRunning = false;
bool Chassis::isSettled = true;
int Chassis::tol = 0;
double Chassis::target = 0, Chassis::theta = 0, Chassis::current = 0, Chassis::output = 0;

Chassis::Chassis() { }

Chassis::~Chassis() { }

void Chassis::setState(ChassisState s){
  chassis_mode = s;
}

void Chassis::setBrakeType(pros::motor_brake_mode_e_t state){
  LF.set_brake_mode(state);
  // LM.set_brake_mode(state);
  LB.set_brake_mode(state);
  RF.set_brake_mode(state);
  // RM.set_brake_mode(state);
  RB.set_brake_mode(state);
}
void Chassis::waitUntilSettled() {
  while(!isSettled) pros::delay(20);
}

void Chassis::reset(){
  LF.tare_position();
  // LM.tare_position();
  LB.tare_position();
  RF.tare_position();
  // RM.tare_position();
  RB.tare_position();
}

Chassis &Chassis::withGains(double kP_, double kI_, double kD_) {
  switch (chassis_mode) {
  case ChassisState::DRIVE:
    drive_PID.set(kP_, kI_, kD_);
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
  isSettled = false;
  target = target_ * CONVERSION;
  chassis_mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::turn(double theta_) {
  isSettled = false;
  theta = theta_;
  chassis_mode = ChassisState::TURN;
  return *this;
}

void Chassis::start(void* ignore) {
  if(!isRunning) {
    pros::delay(500);
    Chassis *that = static_cast<Chassis*>(ignore);
    that -> run();
  }
}

void Chassis::run() {
  isRunning = true;

  while (isRunning) {

    if(pros::competition::is_disabled()) goto end;

    switch (chassis_mode) {
    case ChassisState::DRIVE: {
      current = ( LOdometer.get_position() + ROdometer.get_position() ) /2.0;
      output = drive_PID.calculate(target, current);

      std::cout << "Error:" << drive_PID.getError() <<std::endl; //Debug

      LF.move_voltage(output);
      // LM.move_voltage(output);
      LB.move_voltage(output);
      RF.move_voltage(output);
      // RM.move_voltage(output);
      RB.move_voltage(output);

      if(fabs(drive_PID.getError()) < tol){
        LF.move(0);
        // LM.move(0);
        LB.move(0);
        RF.move(0);
        // RM.move(0);
        RB.move(0);
        isSettled = true;
        chassis_mode = ChassisState::IDLE;
        goto end;
      }
      break;
    }

    case ChassisState::TURN: {
      current = ( L_IMU.get_yaw() + M_IMU.get_yaw() + R_IMU.get_yaw() )/3;
      
      output = turn_PID.calculate(theta, current);
      
      //Find quickest turn.
      if (fabs(turn_PID.getError()) > 180) { 
        turn_PID.setError( turn_PID.getError() - 360); 
        output = turn_PID.calculate(); //recalculate output
      }
    
      std::cout << "Error:" << turn_PID.getError() << std::endl; //Debug

      LF.move_voltage(output);
      // LM.move_voltage(output);
      LB.move_voltage(output);
      RF.move_voltage(-output);
      // RM.move_voltage(output);
      RB.move_voltage(-output);

      if(fabs(turn_PID.getError()) < tol){
        LF.move(0);
        // LM.move(0);
        LB.move(0);
        RF.move(0);
        // RM.move(0);
        RB.move(0);
        isSettled = true;
        chassis_mode = ChassisState::IDLE;
        goto end;
      }

      break;
    }

    case ChassisState::OPCONTROL: {
      double leftJoystick = ( master.get_analog(ANALOG_LEFT_Y) * DRIVE_CONVERSION );
      double rightJoystick = ( master.get_analog(ANALOG_RIGHT_Y) * DRIVE_CONVERSION );
      
      double leftOutput = left.calculate(leftJoystick);
      double rightOutput = right.calculate(rightJoystick);

      LF.move_voltage(leftOutput);
      // LM.move_voltage(leftOutput);
      LB.move_voltage(leftOutput);
      RF.move_voltage(rightOutput);
      // RM.move_voltage(rightOutput);
      RB.move_voltage(rightOutput);
      break;
    }

    case ChassisState::IDLE: {
      LF.move(0);
      // LM.move(0);
      LB.move(0);
      RF.move(0);
      // RM.move(0);
      RB.move(0);
      // std::cout << "State: IDLE " << std::endl;
      break;
    }

    default: {
      break;
    }
  }

    // std::cout << "Error: " << std::endl;

    end:
    pros::delay(2);
  }
}
