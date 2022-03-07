#pragma once
#include "globals.hpp"
#include "misc.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QSpeed.hpp"

#define CONVERSION 4169.079328314997 // Convert 2.75in Wheels to Inches with V5 rotation Sensor.
#define CIRCUMFERENCE 0.1016  // Circumference in meters.
#define DRIVE_CONVERSION 94.48818897637795 // Convert joystick input to scale for voltage.
#define LEFT 1
#define RIGHT 2

struct ChassisTarget {
    QLength x;
    QLength y;
    double controlX;
    double controlY;
    QAngle theta;
    QAngle thetaTwo;
    QSpeed speedDrive;
    QAngularSpeed speedTurn;
    QAcceleration accel_rate;
    QAcceleration decel_rate;
    QAngularAcceleration rateTurn;
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
    Chassis(QLength *rotation_, QAngle *theta_, double *posX_, double *posY_);
    
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
    Set voltage limit for the drive base motors.
    @param limit voltage limit in mV.
    */
    void setVoltLimit(double limit = 12000);

    /*
    return current chassis state.
    */
    ChassisState getState();

    /*
    Return current drive error.
    */
    QLength getDriveError();

    /*
    Return current turn error.
    */
    QAngle getTurnError();
    
    /*
    Return Drive Tolerance.
    */
    QLength getTol();

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
    Chassis& withGains(double kP_ = 15, double kI_ = 0, double kD_ = 6.25);

    /*
    Sets turn PID constants.
    */
    Chassis& withTurnGains(double kP_ = 133, double kI_ = 0, double kD_ = 66);

    /*
    Sets the tolerance range for lateral movements.
    Also set whether to check dist sensor.
    */
    Chassis& withTol(QLength tol_ = 1_in, bool checkBack_ = false);


    /*
    Sets the tolerance for turning.
    */
    Chassis& withTurnTol(QAngle turn_tol_ = 2_deg);

    /*
    Sets Target angle to reach while driving.
    */
    Chassis& withAngle(QAngle theta, QAngularAcceleration rate = 2_radps2, QAngularSpeed speed = 2_radps);

    /*
    Sets Target angle to reach while driving then updates it to the second target angle after reaching the first angle.
    */
    Chassis& withAngles(QAngle theta, QAngle thetaTwo, QAngularAcceleration rate = 2_radps2, QAngularSpeed speed = 2_radps);
    
    /*
    Turn with only one side
    */
    Chassis& swingTurn(int oneSide);

    /*
    Updates ChassisState and sets target position in inches.
    */
    Chassis& drive(QLength target_,  QAcceleration accel_rate = 2_ftps2, QAcceleration decel_rate = 2_ftps2,  QSpeed speed = 4.87_ftps);

    /*
    Updates ChassisState and sets target position from given (x,y) coords.
    */
    // Chassis& drive(coords endPoint, coords controlPoint, bool reverse = false, double driveRate = 900, double driveSpeed = 9000, double turnRate = 900, double turnSpeed = 9000);

    /*
    Updates ChassisState and sets target theta.
    */
    Chassis& turn(QAngle theta, QAngularAcceleration rate = 2_radps2, QAngularSpeed speed = 2_radps);

    /*
    Updates ChassisState to start balancing the bridge.
    */
    Chassis& balance(QAcceleration rate = 1_ftps2, QSpeed speed = 4.87_ftps);

    static void start(void* ignore);

    void run();

    void left(double input);

    void right(double input);


    // void moveToPoint(ChassisTarget target);

    void stop();

    private:
    static bool isSettled;
    static bool isRunning;
    static bool checkBack, justTurn, checkErr;
    
    static double turnError, driveError, *posX, *posY;
    static double current, drive_output, turn_output, LslewOutput, RslewOutput, TslewOutput;
    static bool adjustAngle, turnComplete, twoAngles;

    static int oneSide;
    
    static QLength *rotation, lastRot, drive_tol;
    static QAngle *theta, turn_tol;

    static double brakeTime;
    static bool isBraking, gotTime, isParking;
    
};
