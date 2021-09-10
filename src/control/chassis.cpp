#include "control/chassis.hpp"
#include "control/misc.hpp"

// PID Init
macro::PID drive_PID(0.5, 0.01, 0.25);
macro::PID turn_PID(133, 0, 66);

// Slew Init
macro::Slew leftSlew(900, 900, true);
macro::Slew rightSlew(900, 900, true);
macro::Slew turnSlew(900, 900, true);

// ChassisTarget Struct Init
ChassisTarget target;

// State Machine Init
ChassisState mode = ChassisState::IDLE;


// Variable Init
bool Chassis::isRunning = false, Chassis::isSettled = true;

double *Chassis::odomSide = 0, *Chassis::theta, *Chassis::posX, *Chassis::posY;

int Chassis::tol = 0;

double Chassis::current = 0, Chassis::drive_output = 0,
       Chassis::turn_output = 0, Chassis::LslewOutput = 0,
       Chassis::RslewOutput = 0, Chassis::TslewOutput = 0;

bool Chassis::adjustAngle = false;

double Chassis::distToTarget, Chassis::absAngleToTarget, Chassis::relAngleToTarget; 
double Chassis::relXToPoint, Chassis::relYToPoint;
double Chassis::mvmtXPower, Chassis::mvmtYPower; 

Chassis::Chassis() { }

Chassis::Chassis(double *odomSide_, double *theta_, double *posX_, double *posY_){
  odomSide = odomSide_;
  theta = theta_;
  posX = posX_;
  posY = posY_;
}

Chassis::~Chassis() {
  reset();
 }

void Chassis::setState(ChassisState s){
  mode = s;
}

void Chassis::setBrakeType(pros::motor_brake_mode_e_t state){
  LF.set_brake_mode(state);
  // LM.set_brake_mode(state);
  LB.set_brake_mode(state);
  RF.set_brake_mode(state);
  // RM.set_brake_mode(state);
  RB.set_brake_mode(state);
}

ChassisState Chassis::getState(){
  return mode;
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

  LF.tare_position();
  LB.tare_position();
  RF.tare_position();
  RB.tare_position();

  adjustAngle = false;

  mode = ChassisState::IDLE;
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
  target.theta = theta;
  target.rateTurn = rate;
  target.speedTurn = speed;
  return *this;
}

Chassis &Chassis::drive(double target_, double rate, double speed) {
  
  target.x = target_ * CONVERSION;
  target.rateDrive = rate;
  target.speedDrive = speed;
  reset();
  isSettled = false;
  mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::eDrive(double e_target_, double rate, double speed) {
  
  target.x = e_target_ * BASE_CONVERSION;
  target.rateDrive = rate;
  target.speedDrive = speed;
  reset();
  isSettled = false;
  mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::drive(double x, double y, double driveRate, double driveSpeed, double turnRate, double turnSpeed){
  
  target.x = x;
  target.y = y;
  target.rateDrive = driveRate;
  target.speedDrive = driveSpeed;
  target.rateTurn = turnRate;
  target.speedTurn = turnSpeed;
  reset();
  isSettled = false;
  mode = ChassisState::POINT;
  return *this;
}

Chassis &Chassis::turn(double theta, double rate, double speed) {
  
  target.theta = theta;
  target.rateTurn = rate;
  target.speedTurn = speed;
  reset();
  isSettled = false;
  mode = ChassisState::TURN;
  return *this;
}

Chassis &Chassis::balance(double rate, double speed){
  
  target.rateDrive = rate;
  target.speedDrive = speed;
  reset();
  isSettled = false;
  mode = ChassisState::BALANCE;
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
      // Drive PID calc.
      drive_output = ( drive_PID.calculate(target.x, *odomSide) ) * 20;

      if (!adjustAngle) goto skip; // Skip turn calculation if withAngle wasn't called.

      // Turn PID calc.
      turn_output = turn_PID.calculate(target.theta, *theta);

      // Find quickest turn.
      calcDir();

      // Turn slew calc.
      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);
      
      skip:

      // Drive slew calc.
      LslewOutput = leftSlew.withGains(target.rateDrive, target.rateDrive, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.rateDrive, target.rateDrive, true).withLimit(target.speedDrive).calculate(drive_output);

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
        withGains().withTurnGains().withTol();
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }
      break;
    }

    case ChassisState::POINT: {
      distToTarget = hypot(target.x - *posX, target.y - *posY);

      absAngleToTarget = atan2(target.y - *posY, target.x - *posX);

      double thetaRad = *theta * PI / 180;

      relAngleToTarget = absAngleToTarget - thetaRad;

      double relAngleDeg = relAngleToTarget * 180 / PI;

      // Drive PID calc
      drive_output = drive_PID.calculate(distToTarget, *posX);

      // Turn PID calc.
      turn_output = turn_PID.calculate(relAngleDeg, *theta);

      // Find quickest turn.
      calcDir();
      
      // Slew Calcs.
      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);
      LslewOutput = leftSlew.withGains(target.rateDrive, target.rateDrive, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.rateDrive, target.rateDrive, true).withLimit(target.speedDrive).calculate(drive_output);

      std::cout << "Angle:" << drive_output << "AngleDeg:" << turn_output << std::endl;

      left(LslewOutput + TslewOutput);
      right(RslewOutput - TslewOutput);

      if(fabs(drive_PID.getError()) <= tol){
        left(0);
        right(0);
        withGains().withTurnGains().withTol();
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }

      break;
    }

    case ChassisState::TURN: {
      // Turn PID calc
      turn_output = turn_PID.calculate(target.theta, *theta);

      // Find quickest turn.
      calcDir();

      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);
    
      std::cout << "Error:" << turn_PID.getError() << std::endl; //Debug

      left(TslewOutput);
      right(-TslewOutput);

      if (fabs(turn_PID.getError()) < tol) {
        left(0);
        right(0);
        withTurnGains().withTol();
        reset();
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

    case ChassisState::BALANCE: {
      current = (L_IMU.get_pitch() + M_IMU.get_pitch() + R_IMU.get_pitch()) / 3;
      drive_output = drive_PID.calculate(0, current);

      LslewOutput = leftSlew.withGains(target.rateDrive, target.rateDrive, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.rateDrive, target.rateDrive, true).withLimit(target.speedDrive).calculate(drive_output);

      left(LslewOutput);
      right(RslewOutput);

      if (fabs(drive_PID.getError()) < tol) {
        left(0);
        right(0);
        withGains().withTol();
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }
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

// Macros

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

void Chassis::calcDir(){ // Find Quickest turn.
  if (fabs(turn_PID.getError()) > 180) {
    turn_PID.setError(turn_PID.getError() - 360);
    turn_output = turn_PID.calculate(); // recalculate output
  }
}

void Chassis::stop() { isRunning = false; }