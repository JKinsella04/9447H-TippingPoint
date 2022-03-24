#pragma once
#include "main.h"
#include "globals.hpp"

class Odom{
  public:
  Odom();
  /*
  Return current X coord.
  */
  QLength& getRotation();
  QLength& getX();
  QLength& getY();
  QAngle& getTheta();

  Odom *zero();
  Odom *calibrateGyro();
  Odom *tarePosition();
  
  void run();
  void reset();
  void stop();
  
  private:
  static QLength currentL, currentR, deltaL, deltaR, lastL, lastR;
  static QLength rotation, posX, posY;
  static QAngle theta, offset;
  static bool isRunning;
  static const double diameter, ratio, ticks;

};


class GPS{
  public:
  GPS();
  /*
  Return current GPS X coord.
  */
  QLength& getX();
  QLength& getY();
  QAngle& getTheta();
  double& getError();  
  
  void run();
  void stop();
  
  private:
  static QLength posX, posY;
  static QAngle theta;
  static bool isRunning;
  static pros::c::gps_status_s_t gpsData;
  static double error;
};

class PositionTracker : public Odom, public GPS {
  public:

  QTime getTime();

  void resetTime();
  
  static void start(void* ignore);
  void run();
  
  private:
  static std::string stamp;
  static QTime time, prevTime;
  static bool isRunning;
};
