#include "odometry.hpp"

bool Odometry::isRunning = false;

double  Odometry::sideDistance = -3,  // Distance in inches from tracking center to center of side tracker
        Odometry::backDistance = -8, // Distance in inches from tracking center to center of back tracker
        Odometry::sideDiameter = 2.75,   // Side tracker diameter in inches
        Odometry::backDiameter = 2.75;   // Back tracker diameter in inches

double Odometry::x, Odometry::y, Odometry::angle, Odometry::diff = 0;

Odometry::Odometry() { }

void Odometry::calibrateGyro(){
  L_IMU.reset(); M_IMU.reset(); R_IMU.reset();
  while(L_IMU.is_calibrating() || M_IMU.is_calibrating() || R_IMU.is_calibrating()){ pros::delay(20); }
  OdomL.reset();
  OdomS.reset();
}

double Odometry::getX(){
  return x;
}

double Odometry::getY(){
  return y;
}

double Odometry::getThetaDeg(){
  return radToDeg(angle);
}

double Odometry::getThetaRad(){
  return angle;
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
  x = 0;
  y = 0;
  angle = 0;
  diff = 0;
}

//Tracking
void Odometry::track(){

  isRunning = true;

  calibrateGyro();

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
    
    // std::cout << "(" << x << "," << y << ")" << std::endl;

    pros::delay(10);
  }
}

//Tasks
void Odometry::stopTracking(){
  isRunning = false;
}