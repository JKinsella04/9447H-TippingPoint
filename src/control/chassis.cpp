#include "control/chassis.hpp"
#include "control/misc.hpp"

// PID Init
macro::PID drive_PID;
macro::PID turn_PID(133,0,66);

// Slew Init
macro::Slew leftSlew(900, 900, true);
macro::Slew rightSlew(900, 900, true);
macro::Slew turnSlew(900, 900, true);

// Vector of Struct Init
std::vector<ChassisTarget> Chassis::target;
// int Chassis::;

// State Machine Init
ChassisState mode = ChassisState::IDLE;

bool Chassis::isRunning = false, Chassis::isSettled = true;

int Chassis::tol = 0;

double Chassis::current = 0,
       Chassis::output = 0, Chassis::turn_output = 0,
       Chassis::LslewOutput = 0, RslewOutput = 0, TslewOutput = 0;

bool Chassis::adjustAngle = false;

Chassis::Chassis() { }

Chassis::~Chassis() {
  reset();
 }

void Chassis::setState(ChassisState s){
  mode = s;
}

ChassisState Chassis::getState(){
  return mode;
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
  // PID values reset
  drive_PID.reset();
  turn_PID.reset();
  // Slew values reset
  leftSlew.reset();
  rightSlew.reset();

  mode = ChassisState::IDLE;

  left(0);
  right(0);

  // Motor values reset
  LF.tare_position();
  // LM.tare_position();
  LB.tare_position();
  RF.tare_position();
  // RM.tare_position();
  RB.tare_position();
}

Chassis &Chassis::withGains(double kP_, double kI_, double kD_) {
  drive_PID.set(kP_, kI_, kD_);
  return *this;
}

Chassis &Chassis::withTurnGains(double kP_, double kI_, double kD_){
  turn_PID.set(kP_, kI_, kD_);
  return *this;
}

Chassis &Chassis::withTol(int tol_) {
  tol = tol_;
  return *this;
}

Chassis &Chassis::withAngle(double theta, double rate, double speed){
  adjustAngle = true;
  if(target.size() != 1) target.resize(1);
  target[0].theta = theta;
  target[0].rateTurn = rate;
  target[0].speedTurn = speed;
  return *this;
}

Chassis &Chassis::drive(double target_, double rate, double speed) {
  if(target.size() != 1) target.resize(1);
  target[0].x = target_ * CONVERSION;
  target[0].rateDrive = rate;
  target[0].speedDrive = speed;
  reset();
  isSettled = false;
  mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::turn(double theta, double rate, double speed) {
  if(target.size() != 1) target.resize(1);
  target[0].theta = theta;
  target[0].rateTurn = rate;
  target[0].speedTurn = speed;
  reset();
  isSettled = false;
  mode = ChassisState::TURN;
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

    switch (mode) {
    case ChassisState::DRIVE: {
      // Drive PID calc
      current = (LOdometer.get_position()); // + ROdometer.get_position()) / 2.0;
      output = drive_PID.calculate(target[0].x, current);

      if (!adjustAngle) goto skip; // Skip turn calculation if withAngle wasn't called.

      // Turn PID calc
      current = (L_IMU.get_yaw() + M_IMU.get_yaw() + R_IMU.get_yaw()) / 3;
      turn_output = turn_PID.calculate(target[0].theta, current);

      // Find quickest turn.
      if (fabs(turn_PID.getError()) > 180) {
        turn_PID.setError(turn_PID.getError() - 360);
        turn_output = turn_PID.calculate(); // recalculate output
      }

      TslewOutput = turnSlew.withGains(target[0].rateTurn, target[0].rateTurn, true).withLimit(target[0].speedTurn).calculate(turn_output);
      
      skip:

      LslewOutput = leftSlew.withGains(target[0].rateDrive, target[0].rateDrive, true).withLimit(target[0].speedDrive).calculate(output);
      RslewOutput = rightSlew.withGains(target[0].rateDrive, target[0].rateDrive, true).withLimit(target[0].speedDrive).calculate(output);

      std::cout << "Output:" << LslewOutput << std::endl; // Debug

      if(!adjustAngle){
        left(LslewOutput);
        right(RslewOutput);
      }else{
        left(LslewOutput + TslewOutput);
        right(RslewOutput - TslewOutput);
      }

      if (fabs(drive_PID.getError()) < tol) {
        left(0);
        right(0);
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }
      break;
    }

    case ChassisState::TURN: {
      // Turn PID calc
      current = (L_IMU.get_yaw() + M_IMU.get_yaw() + R_IMU.get_yaw()) / 3;
      turn_output = turn_PID.calculate(target[0].theta, current);

      // Find quickest turn.
      if (fabs(turn_PID.getError()) > 180) {
        turn_PID.setError(turn_PID.getError() - 360);
        turn_output = turn_PID.calculate(); // recalculate output
      }

      double TslewOutput = turnSlew.withGains(target[0].rateTurn, target[0].rateTurn, true).withLimit(target[0].speedTurn).calculate(turn_output);
    
      std::cout << "Error:" << turn_PID.getError() << std::endl; //Debug

      left(TslewOutput);
      right(-TslewOutput);

      if (fabs(turn_PID.getError()) < tol) {
        left(0);
        right(0);
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }

      break;
    }

    case ChassisState::OPCONTROL: {
      double leftJoystick = ( master.get_analog(ANALOG_LEFT_Y) * DRIVE_CONVERSION );
      double rightJoystick = ( master.get_analog(ANALOG_RIGHT_Y) * DRIVE_CONVERSION );
      
      double leftOutput = leftSlew.calculate(leftJoystick);
      double rightOutput = rightSlew.calculate(rightJoystick);

      left(leftOutput);
      right(rightOutput);
      break;
    }

    case ChassisState::IDLE: {
      left(0);
      right(0);
      reset();
      break;
    }
  }

  end:
    pros::delay(10);
  }
}

void Chassis::left(double input){
  LF.move_voltage(input);
  // LM.move_voltage(input);
  LB.move_voltage(input);
}

void Chassis::right(double input){
  RF.move_voltage(input);
  // RM.move_voltage(input);
  RB.move_voltage(input);
}

void Chassis::stop() { isRunning = false; }