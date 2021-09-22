#include "odometry.hpp"

bool Odom::isRunning = false;

int Odom::currentL = 0, Odom::currentR = 0;
int Odom::deltaL = 0, Odom::deltaR = 0, Odom::lastDeltaL = 0, Odom::lastDeltaR = 0;

double Odom::yaw = 0;

double Odom::inertL = 0, Odom::inertR = 0, Odom::inertT = 0;
double Odom::thetaRad = 0, Odom::thetaDeg = 0, Odom::offset = 0, Odom::posX = 0, Odom::posY = 0;

double Odom::output = 0, Odom::Desiredtheta = 0, Odom::DesiredX = 0, Odom::DesiredY = 0;

int * Odom::getL() {
  return &currentL;
}

int * Odom::getR() {
  return &currentR;
}

int * Odom::getDL() {
  return &deltaL;
}

int * Odom::getDR() {
  return &deltaR;
}

double * Odom::getThetaRad() {
  return &thetaRad;
}

double * Odom::getThetaDeg() {
  return &thetaDeg;
}

double * Odom::getYaw(){
  return &yaw;
}

double * Odom::getX() {
  return &posX;
}

double * Odom::getY() {
  return &posY;
}

double Odom::returnX() {
  return posX;
}

double Odom::returnY(){
  return posY;
}


Odom& Odom::calibrateGyro() {
  lf_Imu.reset();
  lb_Imu.reset();
  rf_Imu.reset();
  rb_Imu.reset();

  while( lf_Imu.is_calibrating() || lb_Imu.is_calibrating() || rf_Imu.is_calibrating() || rb_Imu.is_calibrating()) { pros::delay(20); }
  return *this;
}

Odom& Odom::zero() {
  float left = abs( lf_Imu.get_heading() - 360 ) * PI / 180;
  float right = abs( rf_Imu.get_heading() - 360 ) * PI / 180;

  float x = ( cos( left + PI ) + cos( right + PI ) ) / 2;
  float y = ( sin( left + PI ) + sin( right + PI ) ) / 2;

  offset = abs( atan2f(y, x) + PI );
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
    inertL = abs( lb_Imu.get_heading() - 360 ) * PI / 180;
    inertR = abs( rf_Imu.get_heading() - 360 ) * PI / 180;

    float x = ( cos( inertL - offset + PI ) + cos( inertR - offset + PI ) ) / 2;
    float y = ( sin( inertL - offset + PI ) + sin( inertR - offset + PI ) ) / 2;

    thetaRad = abs( atan2f(y, x) + PI );
    thetaDeg = thetaRad * 180 / PI;
    yaw = ( lf_Imu.get_yaw() + lb_Imu.get_yaw() + rf_Imu.get_yaw() + rb_Imu.get_yaw()) / 4;

    currentL = ( LF.get_position() + LB.get_position() )/2;
    currentR = ( RF.get_position() + RB.get_position() )/2;

    deltaL = currentL - lastDeltaL;
    deltaR = currentR - lastDeltaR;

    posX = posX + (( deltaL + deltaR ) / 2) * cos( thetaRad );
    posY = posY + (( deltaL + deltaR ) / 2) * sin( thetaRad );

    lastDeltaL = ( LF.get_position() + LB.get_position() )/2;
    lastDeltaR = ( RF.get_position() + RB.get_position() )/2;

    pros::delay(10);
  }
}

void Odom::stop() {
  isRunning = false;
}

/*
    // Chassis Sensor Readings
    odomL = OdomL.get_position();
    thetaDeg = ( lb_Imu.get_yaw() + lf_Imu.get_yaw() + rf_Imu.get_yaw() ) / 3;
    encoderCount = ( LF.get_position() + LB.get_position() + RF.get_position() + RB.get_position()) /4;

    GPS Sensor
    Struct for GPS coord values.
    
    pros::c::gps_status_s_t gpsData;
    
    global_x = gpsData.x;
    global_y = gpsData.y;

    IMU POSITION Tracking
  double lastAccel = 0;
	double lastTime = pros::c::millis();
	double lastVelocity = 0;
	double lastPosition = 0;
	double velocity = 0;
	double position = 0;
	pros::c::imu_accel_s_t accel = lf_Imu.get_accel();
	if(accel.x >= 0.02){
	velocity = (lastVelocity + ( ( lastAccel + accel.x) /2 ) *(lastTime - pros::c::millis()));
	position = (lastPosition + ( ( lastVelocity + velocity) /2 ) *(lastTime - pros::c::millis()));
	lastPosition = position;
	lastAccel = accel.x;
	lastTime = pros::c::millis();
	lastVelocity = velocity;
	}
	// if(accel.y <= 0.05) accel.y = 0;
	std::cout << "Pos:" << position << std::endl;

    pros::delay(10);
  }
}

*/
