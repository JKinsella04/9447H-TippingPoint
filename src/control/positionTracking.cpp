#include "positionTracking.hpp"
#include "misc.hpp"
#include "pros/rtos.h"


bool Position::isRunning = false;

pros::c::gps_status_s_t Position::gpsData;

double Position::posX = 0, Position::posY = 0, Position::thetaRad = 0, Position::thetaDeg = 0, Position::error = 0, Position::rotation;

double Position::currentL = 0, Position::currentR = 0, Position::deltaL = 0, Position::deltaR = 0, Position::lastL = 0, Position::lastR = 0, Position::offset = 0;

double Position::time, Position::time_offset;

std::string Position::stamp;

PositionTracker PositionTrackerState = PositionTracker::RELATIVE;

double * Position::getX() {
  return &posX;
}

double * Position::getY() {
  return &posY;
}

double * Position::getThetaRad() {
  return &thetaRad;
}

double * Position::getThetaDeg() {
  return &thetaDeg;
}

double * Position::getError() {
  return &error;
}

double * Position::getRotation() {
  return &rotation;
}

double * Position::getTime() {
  return &time;
}

void Position::resetTime(std::string stamp_) {
  time_offset = pros::c::millis();
  stamp = stamp_;
}

void Position::setThetaOffset(double offset_){
  offset = offset_;
}

Position& Position::setState(PositionTracker s){
  PositionTrackerState = s;
  return *this;
}

Position& Position::resetDriveBase() {
  LF.tare_position();
  LM.tare_position();
  LB.tare_position();
  RF.tare_position();
  RM.tare_position();
  RB.tare_position();
  return *this;
}

Position& Position::calibrateGyro() {
  L_Imu.reset();
  R_Imu.reset();
  
  while(L_Imu.is_calibrating() || R_Imu.is_calibrating() ) { pros::delay(5); }
  return *this;
}

Position& Position::zero(){
  float left = abs( L_Imu.get_heading() - 360 ) * PI / 180;
  float right = abs( R_Imu.get_heading() - 360 ) * PI / 180;

  float x = ( cos( left + PI ) + cos( right + PI ) ) / 2;
  float y = ( sin( left + PI ) + sin( right + PI ) ) / 2;

  offset = abs( atan2f(y, x) + PI );
  return *this;
}

void Position::start(void *ignore) {
  if(!isRunning) {
    pros::delay(500);
    Position *that = static_cast<Position*>(ignore);
    that -> run();
  }
}

void Position::run() {
  isRunning = true;

  while(isRunning) {

    switch (PositionTrackerState) {
    case PositionTracker::RELATIVE: {
      double inertL = abs(L_Imu.get_heading() - 360) * PI / 180;
      double inertR = abs(R_Imu.get_heading() - 360) * PI / 180;

      float x = (cos(inertL - offset + PI) + cos(inertR - offset + PI)) / 2;
      float y = (sin(inertL - offset + PI) + sin(inertR - offset + PI)) / 2;

      thetaRad = abs(atan2f(y, x) + PI);
      thetaDeg = macro::toDeg(thetaRad);
      // thetaRad = macro::toRad(thetaDeg);

      rotation = ( LF.get_position() + LM.get_position() + LB.get_position() + RF.get_position() + RM.get_position() + RB.get_position() ) /6;
      
      // macro::print("Theta: ", thetaDeg); // Debug
      break;
    }
    case PositionTracker::GPS: {
      gpsData = gps.get_status();
      
      thetaDeg = abs(gps.get_heading() - 360);
      thetaRad = macro::toRad(thetaDeg);
      
      posX = gpsData.x * 1000;
      posY = gpsData.y * 1000;

      // GPS Error
      error = gps.get_error();
      break;
    }
    case PositionTracker::ODOM: {
      double inertL = abs(L_Imu.get_heading() - 360) * PI / 180;
      double inertR = abs(R_Imu.get_heading() - 360) * PI / 180;

      float x = (cos(inertL - offset + PI) + cos(inertR - offset + PI)) / 2;
      float y = (sin(inertL - offset + PI) + sin(inertR - offset + PI)) / 2;

      thetaRad = abs(atan2f(y, x) + PI);

      currentL = ( LF.get_position() + LM.get_position()+ LB.get_position() ) / 3;
      currentR = ( RF.get_position() + RM.get_position()+ RB.get_position() ) / 3;

      deltaL = currentL - lastL;
      deltaR = currentR - lastR;

      posX += ((deltaL + deltaR) / 2) * cos(thetaRad);
      posY += ((deltaL + deltaR) / 2) * sin(thetaRad);

      lastL = ( LF.get_position() + LM.get_position()+ LB.get_position() ) / 3;
      lastR = ( RF.get_position() + RM.get_position()+ RB.get_position() ) / 3;
      
      break;
    }
    }

    // Timer Tracking
    time = pros::c::millis() - time_offset;
    macro::print(stamp, time);
    pros::delay(10);
  }
}

void Position::reset(){
  posX = posY = 0;
}

void Position::stop() {
  isRunning = false;
}