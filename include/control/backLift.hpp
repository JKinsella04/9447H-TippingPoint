#pragma once
#include "globals.hpp"
#include "misc.hpp"

enum class BackLiftState { 
    AUTON, OPCONTROL
}; 

class BackLift{
  public:

  BackLiftState getState();

  /*
  Return current clamp state.
  @return clampState
  */
  bool getClampState();  
  
  /*
  Updates BackLiftState to either ZERO, UP, OPCONTROL.
  @param BackLiftState s wanted state.
  */
  BackLift& setState(BackLiftState s);

  /*
  Swap BackClamp to other state.
  */
  BackLift& toggleClamp();

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning, clampState, lastClampState;
  static double delay;

  void updateClamp();
};

