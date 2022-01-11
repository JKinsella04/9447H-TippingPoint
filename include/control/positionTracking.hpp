#pragma once
#include "main.h"
#include "globals.hpp"

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
    double * getThetaDeg();

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
    double * getRotation();

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

    static void start(void * ignore);

    void run();

    void reset();

    void stop();

    private:
      static bool isRunning;
      
      static pros::c::gps_status_s_t gpsData;

      static double posX, posY, thetaDeg, thetaRad, error, rotation;

      static double currentL, currentR, deltaL, deltaR, lastL, lastR, offset;
};
