#pragma once
#include "main.h"
#include "globals.hpp"

enum class LiftState { 
    DOWN, MIDDLE, UP, OPCONTROL
}; 

class Lift{
  public:
  
  LiftState getState();  
  
  /*
  Updates LiftState to either ZERO, UP, OPCONTROL, IDLE.
  @param LiftState s wanted state.
  */
  Lift& setState(LiftState s);

  /*
  Sets the lift's clamp.
  @param bool clamp_ True for clamped False for released.
  */
  Lift& setClamp(bool state_);

  /*
  Sets the lift's clamp once the robot has moved far enough.
  @param bool clamp_ True for clamped False for released.
  */
  Lift& delayClamp(bool state_);

  /*
  Sets lift motor brake type.
  @param pros::motor_brake_mode_e_t brake type.
  */
  void setBrakeType(pros::motor_brake_mode_e_t state);

  /*
  Delay thread until settled.
  */
  void waitUntilSettled();

  /*
  tares position of lift motors.
  */
  void reset();
  

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning;
  static bool isSettled;
  static bool isDelayingClamp, clampState, lastClampState, checkLift;

  static double output, target, current, tol, slewOutput;

  void move(double target);

};