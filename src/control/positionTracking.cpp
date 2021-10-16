#include "positionTracking.hpp"
#include "misc.hpp"


bool Position::isRunning = false;

double Position::posX = 0, Position::posY = 0, Position::thetaRad = 0, Position::thetaDeg = 0, Position::error = 0, Position::rotation;

pros::c::gps_status_s_t Position::gpsData;


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

Position& Position::resetDriveBase() {
  LF.tare_position();
  LB.tare_position();
  RF.tare_position();
  RB.tare_position();
  return *this;
}

Position& Position::calibrateGyro() {
  lf_Imu.reset();
  lb_Imu.reset();
  rf_Imu.reset();
  rb_Imu.reset();
  
  while(lf_Imu.is_calibrating() || lb_Imu.is_calibrating() || rf_Imu.is_calibrating() || rb_Imu.is_calibrating()) {pros::delay(5);}

  return *this;
}

Position& Position::reset() {
  posX = posY = 0;
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

    gpsData = gps.get_status();

    posX = gpsData.x * 1000;
    posY = gpsData.y * 1000;
    
    thetaDeg = gps.get_heading();
    // thetaDeg = (lf_Imu.get_heading() + lb_Imu.get_heading() + rf_Imu.get_heading() + rb_Imu.get_heading() )/4; /*+ gps.get_heading() )/5;*/

    thetaRad = macro::toRad(thetaDeg);

    error = gps.get_error();

    rotation = ( LF.get_position() + LB.get_position() + RF.get_position() + RB.get_position() )/4;

    pros::delay(10);
  }
}

void Position::stop() {
  isRunning = false;
}
