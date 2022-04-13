#pragma once
#include "main.h"
#include "globals.hpp"
#include "positionTracking.hpp"

enum class FrontLiftState { 
    DOWN, MIDDLE, UP, OPCONTROL
}; 

class FrontLift{
  public:
  
  FrontLiftState getState();

  /*
  Return current clampState;
  */
  bool getClampState();  
  
  /*
  Updates FrontLiftState to either ZERO, UP, OPCONTROL.
  @param FrontLiftState s wanted state.
  */
  FrontLift& setState(FrontLiftState s);

  /*
  Updates FrontLiftState to either ZERO, UP, OPCONTROL.
  @param FrontLiftState s wanted state.
  @param double delay wanted delay in MS.
  */
  FrontLift& setState(FrontLiftState s, QTime delay_);

  /*
  Toggle Front clamp to opposite state.
  */
  FrontLift& toggleClamp();

  /*
  Sets the FrontLift's clamp once the robot has moved far enough.
  @param bool clamp_ True for clamped False for released.
  */
  FrontLift& delayClamp(QTime delay_);

  /*
  Set tolerance for frontLift movement.
  @param tol_ tolerance.
  */
  FrontLift& withTol(double tol_);

  /*
  Sets FrontLift motor brake type.
  @param pros::motor_brake_mode_e_t brake type.
  */
  void setBrakeType(pros::motor_brake_mode_e_t state);

  /*
  Delay thread until settled.
  */
  void waitUntilSettled();

  /*
  Delay calling thread until the front clamp clamps.
  */
  void waitUntilClamped();

  /*
  tares position of FrontLift motors.
  */
  void reset();
  

  static void start(void * ignore);

  void run();

  private:
  static PositionTracker * robot;

  static bool isRunning;
  static bool isSettled;
  static bool isDelayingClamp, clampState, lastClampState, checkFrontLift;

  static double output, target, current, tol, slewOutput;
  static double downPos, midPos, upPos;
  static QTime delay, lastTimeCheck;

  void move(double target);

  void updateClamp();

};

class Clamp{
  public:
  // Constructor
  /*
  Set solenoid port and its initial state.
  */
  Clamp(std::uint8_t port, bool state);

  /*
  Return current clamp state.
  */
  bool getState();

  /*
  Toggle piston to opposite of current state.
  */
  void toggle();

  private:
  bool clampState;
  pros::ADIDigitalOut * piston = nullptr;
};