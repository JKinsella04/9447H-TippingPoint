#pragma once
#include "globals.hpp"
#include "misc.hpp"

enum class BackLiftState { 
    DOWN, UP, OPCONTROL
}; 

class BackLift{
  public:

  BackLiftState getState();  
  
  /*
  Updates BackLiftState to either ZERO, UP, OPCONTROL, IDLE.
  @param BackLiftState s wanted state.
  */
  BackLift& setState(BackLiftState s);

  /*
  Updates BackLiftState after the delay has passed.
  */
  BackLift& setState(BackLiftState s, double delay);

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning;

  void move(double target);

};

