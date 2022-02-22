#include "control/chassis.hpp"
#include "control/misc.hpp"
#include "globals.hpp"
#include "positionTracking.hpp"
#include "auton.hpp"
#include "pros/rtos.h"

static Position robotPos;
static Autonomous auton;

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
bool Chassis::isRunning = false, Chassis::isSettled = true, Chassis::checkBack = false, Chassis::justTurn = false,
     Chassis::checkErr = false;

double *Chassis::theta, *Chassis::posX, *Chassis::posY, *Chassis::rotation;

double Chassis::drive_tol = 0, Chassis::turn_tol = 0;

double Chassis::current = 0, Chassis::drive_output = 0,
       Chassis::turn_output = 0, Chassis::LslewOutput = 0,
       Chassis::RslewOutput = 0, Chassis::TslewOutput = 0;

bool Chassis::adjustAngle = false, Chassis::turnComplete = false, Chassis::twoAngles = false;

double Chassis::driveError, Chassis::turnError; 

double debugSpeed = 3000;

double Chassis::lastRot, Chassis::brakeTime;
bool Chassis::isBraking = false, Chassis::gotTime = true;

int Chassis::oneSide = 0;
bool isParking = false;

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
  LM.set_brake_mode(state);
  LB.set_brake_mode(state);
  RF.set_brake_mode(state);
  RM.set_brake_mode(state);
  RB.set_brake_mode(state);
}

ChassisState Chassis::getState(){
  return mode;
}

double Chassis::getDriveError(){
  return drive_PID.getError();
}

double Chassis::getTurnError(){
  return turn_PID.getError();
}

double Chassis::getTol(){
  return drive_tol;
}

void Chassis::waitUntilSettled() {
  while(!isSettled) {
    if( checkBack == true && backDist.get() <= 30 && backDist.get() != 0 )isSettled = true;
    pros::delay(20);
  }
}

void Chassis::reset(){
  macro::print("RESET: ", 0);
  // PID values reset
  drive_PID.reset();
  turn_PID.reset();

  // Slew values reset
  leftSlew.reset();
  rightSlew.reset();

  LF.tare_position();
  LM.tare_position();
  LB.tare_position();
  RF.tare_position();
  RM.tare_position();
  RB.tare_position();

  adjustAngle = turnComplete = justTurn = checkBack = twoAngles = false;

  oneSide = 0;

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

Chassis &Chassis::withTol(double drive_tol_, double turn_tol_, bool checkBack_) {
  drive_tol = drive_tol_;
  turn_tol = turn_tol_;
  checkBack = checkBack_;
  return *this;
}

Chassis &Chassis::withAngle(double theta, double rate, double speed){
  adjustAngle = true;
  target.theta = theta;
  target.rateTurn = rate;
  target.speedTurn = speed;
  return *this;
}

Chassis &Chassis::withAngles(double theta, double thetaTwo, double rate, double speed){
  adjustAngle = true;
  twoAngles = true;
  target.theta = theta;
  target.thetaTwo = thetaTwo;
  target.rateTurn = rate;
  target.speedTurn = speed;
  return *this;
}

Chassis &Chassis::swingTurn(int oneSide_){
  oneSide = oneSide_;
  return *this;
}

Chassis &Chassis::drive(double target_,  double accel_rate, double decel_rate, double speed) {
  
  macro::print("DRIVE ", 0);
  target.x = target_;
  target.accel_rate = accel_rate;
  target.decel_rate = decel_rate;
  target.speedDrive = speed;
  reset();
  isSettled = false;
  mode = ChassisState::DRIVE;
  return *this;
}

Chassis &Chassis::drive(coords endPoint, coords controlPoint ,bool reverse, double rate, double driveSpeed, double turnRate, double turnSpeed){
  
  macro::print("POINT ", 0);
  target.x = endPoint.x;
  target.y = endPoint.y;
  target.controlX = controlPoint.x;
  target.controlY = controlPoint.y;
  target.accel_rate = rate;
  target.speedDrive = driveSpeed;
  target.rateTurn = turnRate;
  target.speedTurn = turnSpeed;
  target.reverse = reverse;
  reset();
  isSettled = false;
  mode = ChassisState::POINT;
  return *this;
}

Chassis &Chassis::turn(double theta, double rate, double speed) {
  
  macro::print("TURN ", 0);
  target.theta = theta;
  target.rateTurn = rate;
  target.speedTurn = speed;
  reset();
  isSettled = false;
  mode = ChassisState::TURN;
  return *this;
}

Chassis &Chassis::balance(double rate, double speed){
  
  macro::print("BALANCE ", 0);
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
      if (!twoAngles) {
        turnError = macro::toRad(target.theta - *theta);
      } else {
        if (turnComplete) {
          turnError = (target.thetaTwo - *theta) * PI / 180;
        } else {
          turnError = (target.theta - *theta) * PI / 180;
        }
      }

      turnError = atan2(sin(turnError), cos(turnError));
      turnError = turnError * 180 / PI;
      turn_output = turn_PID.calculate(turnError);

      if(fabs(turn_PID.getError()) <= turn_tol && !turnComplete && twoAngles){ 
        turnComplete = true;
        turnSlew.reset();
        turn_PID.set(133,0,66);
      }

      // Turn slew calc.
      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);
      
      skip:

      // Drive slew calc.
      LslewOutput = leftSlew.withGains(target.accel_rate, target.decel_rate, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.accel_rate, target.decel_rate, true).withLimit(target.speedDrive).calculate(drive_output);

      macro::print("DRIVE ERR", drive_PID.getError());
      macro::print("TURN ERR", turn_PID.getError());

      if(!adjustAngle){
        left(LslewOutput);
        right(RslewOutput);
      }else{
        left(LslewOutput - TslewOutput);
        right(RslewOutput + TslewOutput);
      }

      if ( fabs(drive_PID.getError()) < drive_tol && fabs(turn_PID.getError()) < turn_tol  && checkErr) { 
        left(0);
        right(0);
        withGains().withTurnGains().withTol();
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }
      if( turnComplete && justTurn && fabs ( turn_PID.getError() < turn_tol) ){
        left(0);
        right(0);
        withGains().withTurnGains().withTol();
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }
      checkErr = true;

      break;
    }

    case ChassisState::POINT: {
      moveToPoint(target);
      // for ( float t = 0; t <= 1; t += 0.01){
      // target.x = macro::quadracticBezier({*posX, *posY}, {target.controlX, target.controlY}, {target.x, target.y}, t).x;
      // target.y = macro::quadracticBezier({*posX, *posY}, {target.controlX, target.controlY}, {target.x, target.y}, t).y;

      // moveToPoint(target);
      // }
      break;
    }

    case ChassisState::TURN: {
      // Turn PID calc
      turnError = (target.theta - *theta) * PI / 180;
      turnError = atan2(sin(turnError), cos(turnError));
      turnError = turnError * 180 / PI;
      turn_output = turn_PID.calculate(turnError);

      TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);

      macro::print("Turn Error: ", turn_PID.getError());

      if (oneSide == 1) {
        left(-TslewOutput);
      } else if (oneSide == 2) {
        right(TslewOutput);
      } else {
        left(-TslewOutput);
        right(TslewOutput);
      }

      if ( fabs( turn_PID.getError() ) <= turn_tol ) {
        macro::print("TURN FINISHED: ", 0);
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

      if(!gotTime && fabs( leftJoystick ) < 50 && fabs ( rightJoystick ) < 50 ){
        brakeTime = *robotPos.getTime();
        gotTime = true;
      }else if ( fabs( leftJoystick ) > 50 || fabs ( rightJoystick ) > 50 && gotTime){
        gotTime = isBraking = false;
      }

      if (gotTime && !isBraking && *robotPos.getTime() - brakeTime >= 1500 || gotTime && !isBraking && isParking) {
        lastRot = *rotation;
        isBraking = true;
      }
      if (isBraking) {
        leftJoystick = drive_PID.set(30, 0, 0).calculate(lastRot, *rotation);
        rightJoystick = drive_PID.set(30, 0, 0).calculate(lastRot, *rotation);
      }

      if (arm.get_position() >= 500) { // Slow accel when holding a goal.
        LslewOutput = leftSlew.withGains(450, 900, true).withLimit(12000).calculate(leftJoystick);
        RslewOutput = rightSlew.withGains(450, 900, true).withLimit(12000).calculate(rightJoystick);
      } else {
        LslewOutput = leftSlew.withGains(900, 900, true).withLimit(12000).calculate(leftJoystick);
        RslewOutput = rightSlew.withGains(900, 900, true).withLimit(12000).calculate(rightJoystick);
      }

      macro::print("Speed: ", (LslewOutput + RslewOutput)/2);

      left(LslewOutput);
      right(RslewOutput);

      if( *robotPos.getTime() > 60000 || auton.getAuton() == "Skills" && *robotPos.getTime() > 40000){
        if(L_Imu.get_roll() >= 10 || L_Imu.get_roll() <= -10) isParking = true;
        if(isParking) setBrakeType(HOLD);
        master.print(2, 0, "Parking Time");
      }else{
        setBrakeType(COAST);
      }
      break;
    }

    case ChassisState::BALANCE: {
      current = L_Imu.get_roll();
      drive_output = drive_PID.calculate(0, current);

      LslewOutput = leftSlew.withGains(target.accel_rate, target.accel_rate, true).withLimit(target.speedDrive).calculate(drive_output);
      RslewOutput = rightSlew.withGains(target.accel_rate, target.accel_rate, true).withLimit(target.speedDrive).calculate(drive_output);

      left(LslewOutput);
      right(RslewOutput);
      macro::print("Speed: ", LslewOutput);

      if (fabs(drive_PID.getError()) < drive_tol) {
        left(0);
        right(0);
        withGains().withTol();
        reset();
        isSettled = true;
        mode = ChassisState::IDLE;
        goto end;
      }
    break;
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
  LM.move_voltage(input);
  LB.move_voltage(input);
}

void Chassis::right(double input){
  RF.move_voltage(input);
  RM.move_voltage(input);
  RB.move_voltage(input);
}

void Chassis::moveToPoint(ChassisTarget target) {
  do{
  // Drive part.
  driveError = hypot(target.x - *posX, target.y - *posY);

  // Turn part.
  target.theta = atan2(target.y - *posY, target.x - *posX);

  turnError = (target.theta - macro::toRad(*theta));
  turnError = atan2(sin(turnError), cos(turnError));
  turnError = macro::toDeg(turnError) + 90; // GPS
  // if(target.reverse) turnError -= 180; // ODOM

  // PID Calcs.
  drive_output = drive_PID.calculate(driveError);
  turn_output = turn_PID.calculate(turnError);

  // Slew Calcs.
  LslewOutput = leftSlew.withGains(target.accel_rate, target.accel_rate, true).withLimit(target.speedDrive).calculate(drive_output);
  TslewOutput = turnSlew.withGains(target.rateTurn, target.rateTurn, true).withLimit(target.speedTurn).calculate(turn_output);

  macro::print("Drive: ", drive_PID.getError());
  macro::print("Turn: ", turnError);

  // if (target.reverse) {
  //   left(-LslewOutput - TslewOutput);
  //   right(-LslewOutput + TslewOutput);
  // } else {
  //   left(LslewOutput - TslewOutput);
  //   right(LslewOutput + TslewOutput);
  // }
  }while( fabs(drive_PID.getError()) <= drive_tol && fabs(turn_PID.getError()) <=turn_tol );
}

void Chassis::stop() { isRunning = false; }