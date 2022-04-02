#include "positionTracking.hpp"
#include "misc.hpp"
#include "units.hpp"

  QLength Odom::currentL, Odom::currentR, Odom::deltaL, Odom::deltaR, Odom::lastL, Odom::lastR;
  QLength Odom::rotation, Odom::posX, Odom::posY;
  QAngle Odom::theta, Odom::offset;
  bool Odom::isRunning;

  const double Odom::diameter = 3.25, Odom::ratio = 1.0 / 1.0,
               Odom::ticks = 300;

  Odom::Odom(){}

  QLength &Odom::getRotation() { return rotation; }

  QLength &Odom::getX() { return posX; }

  QLength &Odom::getY() { return posY; }
  
  QAngle &Odom::getTheta() { return theta; }
  
  Odom *Odom::zero() {
    float left = abs(L_Imu.get_heading() - 360) * PI / 180;
    float right = abs(R_Imu.get_heading() - 360) * PI / 180;

    float x = (cos(left + PI) + cos(right + PI)) / 2;
    float y = (sin(left + PI) + sin(right + PI)) / 2;

    offset = abs(atan2f(y, x) + PI) * radian;
    return this;
}

Odom *Odom::calibrateGyro(){
  L_Imu.reset();
  R_Imu.reset();
  
  while(L_Imu.is_calibrating() || R_Imu.is_calibrating() ) { pros::delay(5); }
  return this;
}

Odom *Odom::tarePosition() {
  LF.tare_position();
  LM.tare_position();
  LB.tare_position();
  RF.tare_position();
  RM.tare_position();
  RB.tare_position();
  return this;
}

void Odom::run(){
  double inertL = abs(L_Imu.get_heading() - 360) * PI / 180;
  double inertR = abs(R_Imu.get_heading() - 360) * PI / 180;

  float x = (cos(inertL - offset.convert(radian) + PI) + cos(inertR - offset.convert(radian) + PI)) / 2;
  float y = (sin(inertL - offset.convert(radian) + PI) + sin(inertR - offset.convert(radian) + PI)) / 2;

  theta = abs(atan2f(y, x) + PI) * radian;

  currentL = ((LF.get_position() + LM.get_position() + LB.get_position()) / 3) * tick;
  currentR = ((RF.get_position() + RM.get_position() + RB.get_position()) / 3) * tick;

  deltaL = currentL - lastL;
  deltaR = currentR - lastR;

  // Odom Calc (x,y) in inches.
  posX += (((deltaL.convert(inch) + deltaR.convert(inch)) / 2) * cos(theta.convert(radian))) * inch;
  posY += (((deltaL.convert(inch) + deltaR.convert(inch)) / 2) * sin(theta.convert(radian))) * inch;

  lastL = ((LF.get_position() + LM.get_position() + LB.get_position()) / 3) * tick;
  lastR = ((RF.get_position() + RM.get_position() + RB.get_position()) / 3) * tick;

  // Convert drive base ticks to inches traveled.
  rotation = (currentL + currentR)/2;
  rotation = ( (rotation.convert(revolution) / ratio) * (diameter * PI)) * inch; 
}

void Odom::reset(){
  posX = posY = 0_in;
}

void Odom::stop() {
  isRunning = false;
}

QLength GPS::posX, GPS::posY;
QAngle GPS::theta;
bool GPS::isRunning;
pros::c::gps_status_s_t GPS::gpsData;
double GPS::error;

GPS::GPS(){}

QLength &GPS::getX() { return posX; }

QLength &GPS::getY() { return posY; }

QAngle &GPS::getTheta() { return theta; }

double &GPS::getError() { return error; }

void GPS::run(){
  gpsData = gps.get_status();

  theta = abs(gps.get_heading() - 360) * degree;

  posX = (gpsData.x * 1000) * millimeter;
  posY = (gpsData.y * 1000) * millimeter;

  // GPS Error
  error = gps.get_error();
}

void GPS::stop() {
  isRunning = false;
}


bool PositionTracker::isRunning = false;
QTime PositionTracker::time, PositionTracker::prevTime;

QTime PositionTracker::getTime(){
  return time;
}

void PositionTracker::resetTime(){
  prevTime = pros::c::millis() * millisecond; 
}

void PositionTracker::start(void* ignore) {
  if(!isRunning) {
    pros::delay(500);
    PositionTracker *that = static_cast<PositionTracker*>(ignore);
    that -> run();
  }
}

void PositionTracker::run(){
  isRunning = true;
  while(isRunning){
    Odom::run();
    GPS::run();
    time = (pros::c::millis() - prevTime.convert(millisecond)) * millisecond;
    pros::delay(10);
  }
}