#pragma once
#include "main.h"
#include "globals.hpp"

#define CONVERSION 4169.079328314997
#define DRIVE_CONVERSION 94.48818897637795
#define HOLD pros::E_MOTOR_BRAKE_HOLD
#define BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define COAST pros::E_MOTOR_BRAKE_COAST

struct ChassisTarget {
    double x;
    double y;
    double theta;
    double speedDrive;
    double speedTurn;
    double rateDrive;
    double rateTurn;
};

// TODO: Fix *theta
// TODO: Fix turn, Point, 
// TODO: 

enum class ChassisState { 
    DRIVE, POINT, TURN, OPCONTROL, BALANCE, IDLE
}; 

class Chassis {
    public:
    
    /*
    Constructors.
    */
    Chassis();
    Chassis(double *odomSide_, double *theta_, double *posX_, double *posY_);
    
    /*
    Destructor.
    */
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
    return current chassis state.
    */
    ChassisState getState();

    /*
    Delays thread until action is completed.
    */
    void waitUntilSettled();

    /*
    stops motors and resets variables.
    */
    void reset();

    /*
    Sets drive PID constants.
    */
    Chassis& withGains(double kP_ = 0.5, double kI_ = 0.01, double kD_ = 0.25);

    /*
    Sets turn PID constants.
    */
    Chassis& withTurnGains(double kP_ = 133, double kI_ = 0, double kD_ = 66);

    /*
    Sets the tolerance range for both Chassis::drive() and Chassis::turn().
    */
    Chassis& withTol(int tol_ = 1000);

    /*
    Sets Target angle to reach while driving.
    */
    Chassis& withAngle(double theta, double rate, double speed);

    /*
    Updates ChassisState and sets target position in inches.
    */
    Chassis& drive(double target_, double rate, double speed);

    /*
    Updates ChassisState and sets target position from given (x,y) coords.
    */
    Chassis& drive(double x, double y, double driveRate, double driveSpeed, double turnRate, double turnSpeed);

    /*
    Updates ChassisState and sets target theta.
    */
    Chassis& turn(double theta, double rate, double speed);

    /*
    Updates ChassisState to start balancing the bridge.
    */
    Chassis& balance(double rate, double speed);

    static void start(void* ignore);

    void run();

    void left(double input);

    void right(double input);

    void stop();

    private:
    static bool isSettled;
    static bool isRunning;

    static std::vector<ChassisTarget> target;
    
    static double *odomSide, *theta, *posX, *posY;
    static int tol;
    static double current, drive_output, turn_output, LslewOutput, RslewOutput, TslewOutput;
    static bool adjustAngle;

    static double distToTarget, absAngleToTarget, relAngleToTarget;
    static double relXToPoint, relYToPoint;
    static double mvmtXPower, mvmtYPower;
};
