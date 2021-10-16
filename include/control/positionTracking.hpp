#pragma once
#include "main.h"
#include "globals.hpp"

class Position {
  public:

    /*
    Getters
    */    
    double * getX();

    double * getY();

    double * getThetaDeg();

    double * getThetaRad();
    
    double * getError();

    double * getRotation();

    /*
    Obtain heading from GPS instead of IMU.
    */
    Position& getGPSHeading(bool gpsHeading_ = true);

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

    void stop();

    private:
      static bool isRunning;

      static double posX, posY, thetaDeg, thetaRad, error, rotation;

      static bool gpsHeading;

      static pros::c::gps_status_s_t gpsData;
};
