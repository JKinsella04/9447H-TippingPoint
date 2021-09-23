#pragma once
#include "main.h"
#include "globals.hpp"

enum class MobileGoalState { 
    ZERO, UP, OPCONTROL,IDLE
}; 

class MobileGoal{
  public:

  MobileGoalState getState();  
  
  /*
  Updates MobileGoalState to either ZERO, UP, OPCONTROL, IDLE.
  @param MobileGoalState s wanted state.
  */
  MobileGoal& setState(MobileGoalState s);

  /*
  Sets mobileGoal motor brake type.
  @param pros::motor_brake_mode_e_t brake type.
  */
  void setBrakeType(pros::motor_brake_mode_e_t state);

  /*
  Tares Position of mobile Goal motors.
  */
  void reset();
  
  /*
  Needs to run before Mobile Goal moves to either other state.
  */
  void setup();

  /*
  Delay thread until settled.
  */
  void waitUntilSettled();

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning;
  static bool isSettled;

  static double output, target, current, tol, slewOutput, lastTarget;

  void move(double target);

};