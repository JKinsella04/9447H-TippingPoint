#pragma once
#include "globals.hpp"
#include "okapi/api/units/QLength.hpp"

struct coords {
    double x;
    double y;
};

struct PID_constants{
  double kP;
  double kI;
  double kD;
};

namespace conveyer {
/*
Set speed (rpm) for the conveyer.
*/
void spin(double speed);

/*
Set target and speed for conveyer.
*/
void spin(double target, double speed);

} // namespace conveyer
namespace macro {
/*
Print to terminal.
*/
void print(std::string s, double value);

/**
takes a value and clips it between a lower and upper limit.
*/
double clip(double n, double lower, double upper);
/**
convert given degree to readians.
*/
double toRad(double degree);
/**
convert given radian to degree.
*/
double toDeg(double radian);

/*
Bezier Curve Calculation.
p0 - Start Point
p1 - Control Point
p2 - End Point
t - Increment
pFinal - Curve Output
*/
coords quadracticBezier( coords p0, coords p1, coords p2, double t);

/*
Bezier Curve Calculation.
p0 - Start Point
p1 - Control Point
p2 - 2nd Control Point
p2 - End Point
t - Increment
pFinal - Curve Output
*/
coords cubicBezier ( coords p0, coords p1, coords p2, coords p3, double t);

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
  Sets error to given value.
  */
  PID &setError(double error_);

  /*
  calculates output based off set values.
  */
  double calculate(double target = 0, double current = 0);

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
  bool calcErr = true;
};

class GoalCover{
public:
/*
Toggle goal cover to opposite of current state.
*/
void toggle();
/*
Get current goal cover state.
*/
bool getState();
private:
static bool state;
};
} // namespace macro
