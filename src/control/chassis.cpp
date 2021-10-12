#include "control/chassis.hpp"
#include "control/misc.hpp"
#include "positionTracking.hpp"

static Position robotPos;

// PID Init
macro::PID drive_PID(0.5, 0.01, 0.25);
macro::PID turn_PID(133, 0, 66);
macro::PID theta_PID(133,0,66);

// Slew Init
macro::Slew leftSlew(900, 900, true);
macro::Slew rightSlew(900, 900, true);
macro::Slew turnSlew(900, 900, true);
macro::Slew thetaSlew(900,900, true);

// ChassisTarget Struct Init
ChassisTarget target;

// State Machine Init
ChassisState mode = ChassisState::IDLE;

// Variable Init
bool Chassis::isRunning = false, Chassis::isSettled = true;

double *Chassis::theta, *Chassis::posX, *Chassis::posY, *Chassis::rotation;

double Chassis::drive_tol = 0, Chassis::turn_tol = 0;

double Chassis::current = 0, Chassis::drive_output = 0,
       Chassis::turn_output = 0, Chassis::LslewOutput = 0,
       Chassis::RslewOutput = 0, Chassis::TslewOutput = 0;

bool Chassis::adjustAngle = false, Chassis::turnComplete = false;

double Chassis::distToTarget, Chassis::absAngleToTarget, Chassis::relAngleToTarget; 
double Chassis::relXToPoint, Chassis::relYToPoint;
double Chassis::mvmtXPower, Chassis::mvmtYPower; 

double Chassis::tempTarget = 0, Chassis::tempTheta = 0;

double debugSpeed = 3000;

Chassis::Chassis() { }

Chassis::Chassis(double *rotation_, double *theta_, double *posX_, double *posY_){
  rotation = rotation_;
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
  turnComplete = false;

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

Chassis &Chassis::withTol(double drive_tol_, double turn_tol_) {
  drive_tol = drive_tol_;
  turn_tol = turn_tol_;
  return *this;
}

Chassis &Chassis::withAngle(double theta, double rate, double speed){
  adjustAngle = true;
  target.theta = theta;
  target.thetaTwo = theta;
  target.rateTurn = rate;
  target.speedTurn = speed;
  return *this;
}

Chassis &Chassis::withAngles(double theta, double thetaTwo, double rate, double speed){
  adjustAngle = true;
  target.theta = theta;
  target.thetaTwo = thetaTwo;
  target.rateTurn = rate;
  target.speedTurn = speed;
  return *this;
}

Chassis &Chassis::drive(double target_, double rate, double speed) {
  
  target.x = target_ * CONVERSION;
  target.accel_rate = rate;
  target.speedDrive = speed;
  reset();
  isSettled = false;
  mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::eDrive(double e_target_, double accel_rate, double decel_rate, double speed) {
  
  target.x = e_target_ * BASE_CONVERSION;
  target.accel_rate = accel_rate;
  target.decel_rate =  decel_rate; 
  target.speedDrive = speed;
  reset();
  isSettled = false;
  mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::drive(double x, double y, double theta, double rate, double driveSpeed, double turnRate, double turnSpeed){
  
  target.x = x;
  target.y = y;
  target.theta = theta;
  target.accel_rate = rate;
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
  
  target.accel_rate = rate;
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
      drive_output = drive_PID.calculate(target.x, *rotation);

      if (!adjustAngle) goto skip; // Skip turn calculation if withAngle wasn't called.

      // Turn PID calc.
      // If two turns during movement check firt turn's completion then turn to second angle if first turn is finished.
      if(turnComplete){
        turn_output = turn_PID.calculate(target.thetaTwo, *theta);
      }else{
        turn_output = turn_PID.calculate(target.theta, *theta);
      }
      if(fabs(turn_PID.getError()) <= turn_tol && !turnComplete){ 
        turnComplete = true;
        turnSlew.reset();
        turn_PID.set(133,0,66);
      }
      // Find quickest turn.
      calcDir();
      
      // Turn slew calc.
      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);
      
      skip:

      // Drive slew calc.
      LslewOutput = leftSlew.withGains(target.accel_rate, target.decel_rate, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.accel_rate, target.decel_rate, true).withLimit(target.speedDrive).calculate(drive_output);

      // macro::print("Error: ", turn_PID.getError());

      if(!adjustAngle){
        left(LslewOutput);
        right(RslewOutput);
      }else{
        left(LslewOutput + TslewOutput);
        right(RslewOutput - TslewOutput);
      }

      if (fabs(drive_PID.getError()) < drive_tol && fabs(turn_PID.getError()) < turn_tol) {
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

      double thetaRad = macro::toRad(*theta);

      relAngleToTarget = macro::angleWrap(absAngleToTarget - (thetaRad - macro::toRad(90)));

      double relAngleDeg = macro::toDeg(relAngleToTarget);

      relXToPoint = cos(relAngleToTarget) * distToTarget;
      relYToPoint = sin(relAngleToTarget) * distToTarget;

      mvmtXPower = relXToPoint / (fabs(relXToPoint) + fabs(relYToPoint));
      mvmtYPower = relYToPoint / (fabs(relXToPoint) + fabs(relYToPoint));

      // Drive PID calc
      // drive_PID.setError(relXToPoint);
      drive_PID.setError(mvmtXPower);
      drive_output = drive_PID.calculate();
  
      // Turn PID calc.
      // turn_PID.setError(relYToPoint);
      turn_PID.setError(mvmtYPower);
      turn_output = turn_PID.calculate();

      // Find quickest turn.
      // calcDir();
      
      // Slew Calcs.
      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);
      LslewOutput = leftSlew.withGains(target.accel_rate, target.accel_rate, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.accel_rate, target.accel_rate, true).withLimit(target.speedDrive).calculate(drive_output);

      // macro::print("Angle: ", drive_PID.getError());
      // macro::print("Turn: ", turn_output);
      macro::print("relX: ", relXToPoint);
      // macro::print("xPower:", xPower);
      // macro::print("yPower:", yPower);
      // macro::print("Drive:", drive_output);
      // macro::print("Turn:", turn_output);

      double relTurnAngle = relAngleToTarget - macro::toRad(180) + target.theta;
      double turnPower = macro::clip(relTurnAngle/macro::toRad(30), -1, 1);
      theta_PID.setError(turnPower);
      double thetaOutput = theta_PID.calculate();       
      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(thetaOutput);

      left(LslewOutput + TslewOutput + thetaOutput);
      right(RslewOutput - TslewOutput - thetaOutput);



      // macro::print("turn: ", turnPower");
      // macro::print("rel: ", relTurnAngle");


      if(fabs(drive_output) <= drive_tol && fabs(turn_output) <= turn_tol){ //drive_PID.getError()) <= tol && fabs(turn_PID.getError()) <= 0.75
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
    
      macro::print("Error: ", turn_PID.getError()); //Debug

      left(TslewOutput);
      right(-TslewOutput);

      if (fabs(turn_PID.getError()) < turn_tol) {
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
      
      double leftOutput = leftSlew.withGains(900,900,true).calculate(leftJoystick);
      double rightOutput = rightSlew.withGains(900,900,true).calculate(rightJoystick);

      left(leftOutput);
      right(rightOutput);
      break;
    }

    case ChassisState::BALANCE: {
      current = (lf_Imu.get_pitch() + lb_Imu.get_pitch() + rf_Imu.get_pitch() + rb_Imu.get_pitch()) / 4;
      drive_output = drive_PID.calculate(0, current);

      LslewOutput = leftSlew.withGains(target.accel_rate, target.accel_rate, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.accel_rate, target.accel_rate, true).withLimit(target.speedDrive).calculate(drive_output);

      left(LslewOutput);
      right(RslewOutput);

      if (fabs(drive_PID.getError()) < drive_tol) {
        left(0);
        right(0);
        withGains().withTol();
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }
    }

    case ChassisState::DEBUG: {
      debugSpeed += 3000;
      if (debugSpeed > 12000)
        debugSpeed = 0;
      left(debugSpeed);
      right(debugSpeed);
      pros::delay(2000);
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
    if(*theta < 180){
      tempTheta = *theta + 360;
    }else{
      if(!turnComplete){
        tempTarget = target.theta + 360;
      }else{
        tempTarget = target.thetaTwo + 360;
      }
    }
    turn_output = turn_PID.calculate(tempTarget, tempTheta); // recalculate output
  }
}

void Chassis::stop() { isRunning = false; }