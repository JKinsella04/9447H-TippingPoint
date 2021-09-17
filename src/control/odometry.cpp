#include "odometry.hpp"

bool Odometry::isRunning = false;

double  Odometry::sideDistance = -3,  // Distance in inches from tracking center to center of side tracker
        Odometry::backDistance = -8, // Distance in inches from tracking center to center of back tracker
        Odometry::sideDiameter = 2.75,   // Side tracker diameter in inches
        Odometry::backDiameter = 2.75;   // Back tracker diameter in inches

double Odometry::x, Odometry::y, Odometry::angle, Odometry::diff = 0, Odometry::thetaDeg = 0, Odometry::odomL = 0, Odometry::encoderCount = 0,
       Odometry::global_x, Odometry::global_y;

Odometry::Odometry() { }

void Odometry::calibrateGyro(){
  L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
  while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(20); }
  OdomL.reset_position();
  OdomS.reset_position();
  
}

double * Odometry::getX(){
  return &global_x;
}

double * Odometry::getY(){
  return &global_y;
}

double * Odometry::getL(){
  return &odomL;
}

double * Odometry::getEncoderCount(){
  return &encoderCount;
}

double * Odometry::getThetaDeg(){
  return &thetaDeg;
}

double * Odometry::getThetaRad(){
  return &angle;
}

double Odometry::returnX(){
  return global_x;
}

double Odometry::returnY(){
  return global_y;
}

//Math
double Odometry::radToDeg(double rad){
  return rad/M_PI * 180.0;
}

double Odometry::degToRad(double deg){
  return deg * M_PI / 180.0;
}

//L_IMU Filter
double Odometry::filter(const double& currentVal, const double& lastVal){
  double filteredVal = currentVal - lastVal;
  if(fabs(filteredVal) < 0.01){
    filteredVal = 0;
  }
  return filteredVal;
}

void Odometry::start(void* ignore) {
  if(!isRunning) {
    pros::delay(500);
    Odometry *that = static_cast<Odometry*>(ignore);
    that -> track();
  }
}

void Odometry::reset(){
  x = y = angle = diff = 0;
}

//Tracking
void Odometry::track(){

  isRunning = true;

  reset();

  float Ss = sideDistance,
        Sb = backDistance,
        wheelDiameter = sideDiameter,
        trackingDiameter = backDiameter;

  float deltaTheta = 0,
        thetaFiltered = 0,
        thetaNew = 0,
        thetaAvg = 0,
        curRotation = 0,
        lastRotation = 0,

        curSide = 0,
        curBack = 0,
        lastSide = 0,
        lastBack = 0,
        deltaSide = 0,
        deltaBack = 0,
        sideChord = 0,
        backChord = 0,
        i, Ri = L_IMU.get_rotation();

  while(isRunning){

    i = (L_IMU.get_rotation() - Ri - diff);

    curRotation = i;

    thetaFiltered += filter(curRotation, lastRotation);
    lastRotation = curRotation;
    thetaNew = degToRad(thetaFiltered);
    deltaTheta = thetaNew - angle;

    curSide = ( OdomS.get_position() * M_PI/360);
    curBack = ( OdomL.get_position() * M_PI/360);
    deltaSide = (curSide - lastSide)*(wheelDiameter);
    deltaBack = (curBack - lastBack)*(trackingDiameter);
    lastSide = curSide;
    lastBack = curBack;

    if(deltaTheta != 0){
      backChord = (2*sin(deltaTheta/2))*(deltaBack/deltaTheta + Sb);
      sideChord = (2*sin(deltaTheta/2))*(deltaSide/deltaTheta + Ss);
    }
    else{
       sideChord = deltaSide;
       backChord = deltaBack;
    }

    thetaAvg = angle + deltaTheta/2;

    x += sideChord * sin(thetaAvg);
    y += sideChord * cos(thetaAvg);
    x += backChord * -cos(thetaAvg);
    y += backChord *  sin(thetaAvg);
    
    angle += deltaTheta;

    // Chassis Sensor Readings
    odomL = OdomL.get_position();
    thetaDeg = ( L_IMU.get_yaw() + M_IMU.get_yaw() + R_IMU.get_yaw() ) / 3;
    encoderCount = ( LF.get_position() + LB.get_position() + RF.get_position() + RB.get_position()) /4;

    /*
    GPS Sensor
    Struct for GPS coord values.
    */
    pros::c::gps_status_s_t gpsData;
    
    global_x = gpsData.x;
    global_y = gpsData.y;

  /*  IMU POSITION Tracking
  double lastAccel = 0;
	double lastTime = pros::c::millis();
	double lastVelocity = 0;
	double lastPosition = 0;
	double velocity = 0;
	double position = 0;
	pros::c::imu_accel_s_t accel = M_IMU.get_accel();
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
  */

    pros::delay(10);
  }
}

//Tasks
void Odometry::stopTracking(){
  isRunning = false;
}