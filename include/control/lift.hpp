#pragma once
#include "main.h"
#include "globals.hpp"
#include "pros/motors.h"

enum class LiftState { 
    ZERO, UP, OPCONTROL,IDLE
}; 

class Lift{
  public:
  LiftState getState();  
  
  Lift& setState(LiftState s);

  void setBrakeType(pros::motor_brake_mode_e_t state);

  void waitUntilSettled();

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning;
  static bool isSettled;

  static double output, target, current, tol;

  void move(double target);

};