#pragma once
#include "globals.hpp"
#include "misc.hpp"

enum class BackLiftState { 
    AUTON, OPCONTROL, IDLE
}; 

class BackLift{
  public:
  /*
  Return current Back lift state.
  */
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
  Toggle Back clamp to opposite state.
  */
  BackLift& toggleClamp(double delay_ = 0);

  /*
  Delay clamp for specified amount of time.
  @param delay_ time in seconds.
  */
  BackLift& delayClamp(double delay_);

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning, clampState, lastClampState, checkDist, isDelayingClamp;
  static double delay;

  /*
  Check current clampState to lastClampState and update if needed.
  */
  void updateClamp();
};

