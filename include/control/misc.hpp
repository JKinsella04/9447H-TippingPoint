#pragma once
#include "main.h"
#include "globals.hpp"
#include "pros/adi.hpp"
#include "pros/imu.h"

#define GOAL_OUT 1000
#define GOAL_IN  0

#define BOTH 0
#define LEFT 1
#define RIGHT 2

// namespace intake{
//     /*
//     Set speed for intakes.
//     */
//     void spin(double speed);
    
//     /*
//     Set target and speed for intakes.
//     */
//     void spin(double target, double speed);

// }

namespace draggers {
  /*
  Sets the dragger's state.
  @param int which_ Either both or just one of the draggers.
  @param bool dragger_ True for extended False for retracted.
  */
  void setState(int which_ = BOTH, bool state_ = false);

} // namespace draggers

namespace macro {
class Slew {
public:
  Slew(double accel_);
  Slew(double accel_, double decel_);
  Slew(double accel_, double decel_, bool reversible_);
  
  /*
  Sets accel and decel rates plus reversible boolean.
  */
  Slew &withGains(double accel_, double decel_ = 0, bool reversible_ = false);

  /*
  Sets limit.
  */
  Slew &withLimit(double limit_);
  
  /*
  Calculates and returns slew output.
  */
  double calculate(int input);
  
  /*
  Returns output without doing new calculation.
  */
  double getOutput();

  /*
  Clears all stored values.
  */
  void reset();

private:
  double accel, decel;
  double input, output, limit;
  bool isReversible, noDecel, isLimited;
};

class PID {
public:
  PID(double kP_ = 0, double kI_ = 0, double kD_ = 0);

  /*
  set the PID values
  */
  PID &set(double kP_, double kI_, double kD_);

  /*
  calculates output based off set values.
  */
  double calculate(double target = 0, double current = 0);

  /*
  Sets error to given value.
  */
  void setError(double error_);

  /*
  Returns error.
  */
  double getError();

  /*
  Clears all stored values.
  */
  void reset();

private:
  double target = 0, current = 0, error = 0, integral = 0, derivative = 0,
         prevError = 0, kP = 0, kI = 0, kD = 0, output = 0;
};
} // namespace macro
