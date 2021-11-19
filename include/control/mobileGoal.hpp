#pragma once
#include "main.h"
#include "globals.hpp"

enum class MobileGoalState { 
    DOWN, UP, MIDDLE, OPCONTROL, SETUP, IDLE
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
  Updates MobileGoalState after the delay has passed.
  */
  MobileGoal& setState(MobileGoalState s, double delay);
  
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
  Delay thread until settled.
  */
  void waitUntilSettled();

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning;
  static bool isSettled;
  static bool isSetup;

  static double output, target, current, tol, slewOutput, lastTarget, delay;

  void move(double target);

};