#pragma once
#include "main.h"
#include "globals.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

enum class PositionTracker{
  RELATIVE, GPS, ODOM
};

class Position {
  public:

    /*
    Return current X coord.
    */
    double * getX();

    /*
    Return current Y coord.
    */
    double * getY();

    /*
    Return current heading in Degrees.
    */
    QAngle * getTheta();

    /*
    Return current heading in Radians.
    */
    double * getThetaRad();
    
    /*
    Return GPS sensor error.
    */
    double * getError();

    /*
    Return current average drive base position.
    */
    QLength * getRotation();

    /*
    Get current time im MS since last reset.
    */
    double * getTime();

    /*
    Reset timer.
    @param string stamp
    */
    void resetTime(std::string stamp_);

    /*
    Sets offset value for theta in Degrees.
    */
    void setThetaOffset(double offset_);

    /*
    Return current state.
    */
    PositionTracker getState();

    /*
    Set state between RELATIVE, GPS, and ODOM.
    */
    Position& setState(PositionTracker s);

    /*
    Reset drive base encoders.
    */
    Position& resetDriveBase();

    /*
    Reset IMUs.
    */
    Position& calibrateGyro();  

    /*
    Set IMU offset.
    */
    Position& zero();

    static void start(void * ignore);

    void run();

    void reset();

    void stop();

    private:
      static bool isRunning;
      
      static pros::c::gps_status_s_t gpsData;

      static double posX, posY, thetaDeg, thetaRad, error;

      static QAngle theta;

      static QLength rotation;

      static double currentL, currentR, deltaL, deltaR, lastL, lastR, offset;

      static double time, time_offset;
      static std::string stamp;
};
