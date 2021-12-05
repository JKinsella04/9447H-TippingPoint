#pragma once
#include "main.h"
#include "globals.hpp"

enum class MobileGoalState { 
    DOWN, UP, OPCONTROL
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

  static void start(void * ignore);

  void run();

  private:
  static bool isRunning;

  void move(double target);

};