#pragma once
#include "globals.hpp"
#include "misc.hpp"

#define CONVERSION 4169.079328314997 // Convert 2.75in Wheels to Inches with V5 rotation Sensor.
#define BASE_CONVERSION 35.2589412388  // Convert 3.25 to Inches with Motor Encoders.
#define DRIVE_CONVERSION 94.48818897637795 // Convert joystick input to scale for voltage.
#define driveErr 0
#define turnErr 1

struct ChassisTarget {
    double x;
    double y;
    double controlX;
    double controlY;
    double theta;
    double thetaTwo;
    double speedDrive;
    double speedTurn;
    double accel_rate;
    double decel_rate;
    double rateTurn;
    bool reverse;
};

enum class ChassisState { 
    DRIVE, POINT, TURN, OPCONTROL, BALANCE, DEBUG, IDLE 
}; 

class Chassis {
    public:
    
    /*
    Constructors.
    */
    Chassis();
    Chassis(double *rotation_, double *theta_, double *posX_, double *posY_);
    
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
    Return current drive error.
    */
    double getDriveError();

    /*
    Return current turn error.
    */
    double getTurnError();
    
    /*
    Return Drive Tolerance.
    */
    double getTol();

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
    Sets the tolerance range for both lateral and turn movements.
    Also set whether to check dist sensor.
    */
    Chassis& withTol(double tol_ = 20, double turn_tol_ = 2, bool checkDist_ = false);

    /*
    Sets Target angle to reach while driving.
    */
    Chassis& withAngle(double theta, double rate = 900, double speed = 9000);

    /*
    Sets Target angle to reach while driving then updates it to the second target angle after reaching the first angle.
    */
    Chassis& withAngles(double theta, double thetaTwo, double rate = 900, double speed = 9000);

    /*
    Updates ChassisState and sets target position in inches.
    */
    Chassis& drive(double target_, double accel_rate = 900, double speed = 9000);

    /*
    Updates ChassisState and uses motor encoders.
    */
    Chassis& eDrive(double e_target_, double accel_rate = 900, double decel_rate = 900, double speed = 9000);

    /*
    Updates ChassisState and sets target position from given (x,y) coords.
    */
    Chassis& drive(coords endPoint, coords controlPoint, bool reverse = false, double driveRate = 900, double driveSpeed = 9000, double turnRate = 900, double turnSpeed = 9000);

    /*
    Updates ChassisState and sets target theta.
    */
    Chassis& turn(double theta, double rate = 900, double speed = 9000);

    /*
    Updates ChassisState to start balancing the bridge.
    */
    Chassis& balance(double rate = 900, double speed = 9000);

    static void start(void* ignore);

    void run();

    void calcDir();

    void left(double input);

    void right(double input);

    void moveToPoint(ChassisTarget target);

    void stop();

    private:
    static bool isSettled;
    static bool isRunning;
    static bool checkAccel, checkDist, checkErr;
    
    static double *theta, *posX, *posY, *rotation;
    static double drive_tol, turn_tol, current, drive_output, turn_output, LslewOutput, RslewOutput, TslewOutput;
    static bool adjustAngle, turnComplete;
    
    static double distToTarget;
    static double driveError, turnError;

    static double tempTarget, tempTheta;
};
