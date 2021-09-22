#pragma once
#include "main.h"
#include "globals.hpp"

class Odom {
  public:

    // Getters & Setters
    int * getL();
    int * getR();
    int * getDL();
    int * getDR();

    double * getThetaDeg();
    double * getThetaRad();
    double * getYaw();
    double * getX();
    double * getY();

    double returnX();
    double returnY();

    Odom& calibrateGyro();
    Odom& zero();
    Odom& reset();

    static void start(void* ignore);
    void run();
    void stop();

  private:
    static bool isRunning;

    static double yaw;
    
    static int currentL, currentR;
    static int deltaL, deltaR, lastDeltaL, lastDeltaR;

    static double inertL, inertR, inertT;
    static double thetaRad, thetaDeg, offset, posX, posY;

    static double output, DesiredX, DesiredY, Desiredtheta;
};
