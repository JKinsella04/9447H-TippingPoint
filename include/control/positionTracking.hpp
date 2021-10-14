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

    Position& resetDriveBase();

    Position& calibrateGyro();

    Position& reset();

    static void start(void * ignore);

    void run();

    void stop();

    private:
      static bool isRunning;

      static double posX, posY, thetaDeg, thetaRad, error, rotation;

      static pros::c::gps_status_s_t gpsData;
};
