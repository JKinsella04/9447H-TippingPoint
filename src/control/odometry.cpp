#include "control/odometry.hpp"
#include "control/chassis.hpp"

bool Odom::isRunning = false;

int Odom::currentL = 0, Odom::currentR = 0;
int Odom::deltaL = 0, Odom::deltaR = 0, Odom::lastDeltaL = 0, Odom::lastDeltaR = 0;

double Odom::inertL = 0, Odom::inertR = 0, Odom::inertT = 0;
double Odom::thetaRad = 0, Odom::thetaDeg = 0, Odom::offset = 0, Odom::posX = 0, Odom::posY = 0;

double Odom::output = 0, Odom::Desiredtheta = 0, Odom::DesiredX = 0, Odom::DesiredY = 0;

int Odom::getL() {
  return currentL;
}

int Odom::getR() {
  return currentR;
}

int Odom::getDL() {
  return deltaL;
}

int Odom::getDR() {
  return deltaR;
}

double Odom::getThetaRad() {
  return thetaRad;
}

double Odom::getThetaDeg() {
  return thetaDeg;
}

double Odom::getX() {
  return posX;
}

double Odom::getY() {
  return posY;
}

Odom& Odom::calibrateGyro() {
  M_IMU.reset();
  L_IMU.reset();
  R_IMU.reset();

  while( M_IMU.is_calibrating() || L_IMU.is_calibrating() || R_IMU.is_calibrating() ) { pros::delay(20); }
  // io::master.rumble(" . .");
  return *this;
}

Odom& Odom::zero() {
  float left = fabs( L_IMU.get_heading() - 360 ) * M_PI / 180;
  float right = fabs( R_IMU.get_heading() - 360 ) * M_PI / 180;

  float x = ( cos( left + M_PI ) + cos( right + M_PI ) ) / 2;
  float y = ( sin( left + M_PI ) + sin( right + M_PI ) ) / 2;

  offset = fabs( atan2f(y, x) + M_PI );
  return *this;
}

Odom& Odom::reset() {
  posX = posY = 0;
  return *this;
}

void Odom::start(void *ignore) {
  if(!isRunning) {
    pros::delay(500);
    Odom *that = static_cast<Odom*>(ignore);
    that -> run();
  }
}

void Odom::run() {
  isRunning = true;

  while(isRunning) {
    inertL = fabs( L_IMU.get_heading() - 360 ) * M_PI / 180;
    inertR = fabs( R_IMU.get_heading() - 360 ) * M_PI / 180;

    float x = ( cos( inertL - offset + M_PI ) + cos( inertR - offset + M_PI ) ) / 2;
    float y = ( sin( inertL - offset + M_PI ) + sin( inertR - offset + M_PI ) ) / 2;

    thetaRad = fabs( atan2f(y, x) + M_PI );
    thetaDeg = thetaRad * 180 / M_PI;

    currentL = LOdometer.get_position();
    currentR = ROdometer.get_position();

    deltaL = currentL - lastDeltaL;
    deltaR = currentR - lastDeltaR;

    posX = posX + (( deltaL + deltaR ) / 2.0) * cos( thetaRad );
    posY = posY + (( deltaL + deltaR ) / 2.0) * sin( thetaRad );

    lastDeltaL = LOdometer.get_position();
    lastDeltaR = ROdometer.get_position();

    std::cout << "(" << getX() << "," << getY() << "," << getThetaDeg() << ")" <<std::endl;
    
    pros::delay(10);
  }
}

void Odom::stop() {
  isRunning = false;
}
