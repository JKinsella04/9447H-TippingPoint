#pragma once
#include "main.h"
#include "globals.hpp"



enum class ChassisState { 
    DRIVE, TURN, OPCONTROL, OFF
}; 

class Chassis {
    public:
    
    /*
    sets the state of the chassis.
    */
    void setState(ChassisState s);

    /*
    Sets PID constants for both Chassis::drive() and Chassis::turn().
    */
    Chassis& withGains(double kP_, double kD_ = 0, double kI_ = 0);

    /*
    Sets the tolerance range for both Chassis::drive() and Chassis::turn().
    */
    Chassis& withTol(int tol_);

    /*
    Updates ChassisState and sets target position.
    */
    Chassis& drive(double target);

    /*
    Updates ChassisState and sets target theta.
    */
    Chassis& turn(double theta_);

    static void start(void* ignore);

    Chassis& run();

    private:
    static bool isSettled;
    static bool isRunning;

    int tol;
    double target, theta, current;
};
