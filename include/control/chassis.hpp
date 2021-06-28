#pragma once
#include "main.h"
#include "globals.hpp"
#include "pros/motors.h"

#define CONVERSION 4169.079328314997
#define DRIVE_CONVERSION 94.48818897637795
#define HOLD pros::E_MOTOR_BRAKE_HOLD
#define BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define COAST pros::E_MOTOR_BRAKE_COAST

enum class ChassisState { 
    DRIVE, TURN, OPCONTROL, IDLE
}; 

class Chassis {
    public:
    
    /*
    Constructor.
    */
    Chassis();

    ~Chassis();
    /*
    Sets state for Chassis state machine.
    */
    void setState(ChassisState s);
    /*
    sets brake mode
    */
    void setBrakeType(pros::motor_brake_mode_e_t state);

    /*
    Delays thread until action is completed.
    */
    void waitUntilSettled();

    /*
    stops motors and resets relative variables.
    */
    void reset();

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

    void run();

    private:
    static bool isSettled;
    static bool isRunning;

    static int tol;
    static double target, theta, current, output;
};
