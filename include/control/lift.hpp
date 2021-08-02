#pragma once
#include "main.h"
#include "globals.hpp"

enum class LiftState { 
    ZERO, SMALL, MIDDLE, TALL, IDLE
}; 

class Lift{
  public:
  LiftState getState();  
  
  Lift& setState(LiftState s);

  void waitUntilSettled();

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning;
  static bool isSettled;

  static double output, target, current, tol;

  void move(double target);

};