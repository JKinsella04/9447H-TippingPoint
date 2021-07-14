#pragma once
#include "main.h"
#include "globals.hpp"
#include "odometry.hpp"

// Odom calculations and conversion functions from Jay Katyan of 2602H. 
class Odometry{
  public:

    Odometry();

    void calibrateGyro();

    double getX();

    double getY();

    double getThetaRad();

    double getThetaDeg();

    double radToDeg(double rad);

    double degToRad(double deg);

    double filter(const double& currentVal, const double& lastVal);

    static void start(void* ignore);

    void reset();

    void track();

    void stopTracking();
  private:
    static bool isRunning;
    static double x, y, angle, diff;
    static double sideDistance, backDistance, sideDiameter, backDiameter;
};