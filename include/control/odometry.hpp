#pragma once
#include "main.h"
#include "globals.hpp"

// Odom calculations and conversion functions from Jay Katyan of 2602H. 
class Odometry{
  public:

    Odometry();

    void calibrateGyro();

    // Get X coordinate from Odom calc.
    double * getX();

    // Get Y coordinate from Odom calc.
    double * getY();

    // Get theta in Radians.
    double * getThetaRad();

    // Get theta in Degrees.
    double * getThetaDeg();

    // Get Rotation Sensor value.
    double * getL();

    // Get Drive base motor encoder values.
    double * getEncoderCount();

    
    // Get X coord not as a pointer.
    double returnX();
     
    // Get Y coord not as a pointer.
    double returnY();

    /*
    Convert Radians to Degrees
    @param rad value in radians.
    */
    double radToDeg(double rad);

    /*
    Convert Degrees to Radians
    @param deg value in degrees.
    */
    double degToRad(double deg);

    double filter(const double& currentVal, const double& lastVal);

    static void start(void* ignore);

    void reset();

    void track();

    void stopTracking();
  private:
    static bool isRunning;
    static double x, y, angle, diff, thetaDeg, odomL, encoderCount;
    static double sideDistance, backDistance, sideDiameter, backDiameter;
};
