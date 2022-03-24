#pragma once
#include "globals.hpp"
#include "misc.hpp"
#include "positionTracking.hpp"

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
  BackLift& toggleClamp(QTime delay_ = 0_ms);

  /*
  Delay clamp for specified amount of time.
  @param delay_ time in seconds.
  */
  BackLift& delayClamp(QTime delay_);

  static void start(void * ignore);

  void run();

  private:
  static PositionTracker * robot;
  
  static bool isRunning, clampState, lastClampState, checkDist, isDelayingClamp;
  static QTime delay, lastTimeCheck;

  /*
  Check current clampState to lastClampState and update if needed.
  */
  void updateClamp();
};

