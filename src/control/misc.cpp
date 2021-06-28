#include "control/misc.hpp"

namespace macro {

Slew::Slew(double accel_) : accel(accel_), decel(0) { noDecel = true; }

Slew::Slew(double accel_, double decel_) : accel(accel_), decel(decel_) {
  noDecel = false;
}

Slew::Slew(double accel_, double decel_, bool reversible_)
    : accel(accel_), decel(decel_), isReversible(reversible_) {
  noDecel = false;
}

Slew &Slew::withLimit(double limit_) {
  isLimited = true;
  limit = limit_;
  return *this;
}
double Slew::calculate(int input) {
  if (!noDecel) {

    if (!isReversible) {
      if (input > output + accel) {
        output += accel;
      } else if (input < output - decel) {
        output -= decel;
      } else {
        output = input;
      }
    } else {
      if (input > 0) {
        if (input > output + accel) {
          output += accel;
        } else if (input < output - decel) {
          output -= decel;
        } else {
          output = input;
        }
      }
      if (input < 0) {
        if (input < output - accel) {
          output -= accel;
        } else if (input > output + decel) {
          output += decel;
        } else {
          output = input;
        }
      }

      if (input == 0) {
        if (input < output - decel) {
          output -= decel;
        } else if (input > output + decel) {
          output += decel;
        } else {
          output = input;
        }
      }
    }

    if (isLimited) {
      if (limit > 0 && output > limit) {
        output = limit;
      }

      if (limit < 0 && output < limit) {
        output = limit;
      }
    }
  } else {
    if (input > 0) {
      if (input > output + accel)
        output += accel;
      else
        output = input;
    }

    if (input < 0) {
      if (input < output - accel)
        output -= accel;
      else
        output = input;
    }
  }
  return output;
}

double Slew::getOutput(){ 
    return output;
}

void Slew::reset(){
    input = output = 0;
}

double PID::target = 0, PID::current = 0, PID::error = 0, PID::integral = 0,
       PID::derivative = 0, PID::prevError = 0, PID::kP = 0, PID::kI = 0,
       PID::kD = 0, PID::output = 0;

PID &PID::set(double kP_, double kI_, double kD_) {
  kP = kP_;
  kI = kI_;
  kD = kD_;

  return *this;
}

double PID::calculate(double target, double current) {
  // Proportional Calculation
  if (error == 0) {
    error = target - current;
  }
  // Integral Calculation
  integral = integral + error;
  if (error == 0)
    integral = 0;
  if (integral > 12000)
    integral = 12000;

  // Derivative Calculation
  derivative = error - prevError;
  prevError = error;

  // Output Calculation
  output = error * kP + integral * kI + derivative * kD;
  error = 0;
  return output;
}

void PID::setError(double error_) { error = error_; }

double PID::getError() { return prevError; }
} // namespace macro
