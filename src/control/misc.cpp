#include "control/misc.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

// namespace intake {
// void spin(double speed) { Intake.move(speed); }

// void spin(double target, double speed) { Intake.move_relative(target, speed); }
// } // namespace intake

//TODO: Stop mobile goal before hitting wheels! (Potentiometer??)

namespace Dragger {
void setState(bool state_) {
    dragger.set_value(state_);
  }
} // namespace draggers

namespace macro {

void print(std::string s, double value){
  std::cout << s << value << std::endl;
}

double angleWrap(double rad){
  while(rad < -PI){
    rad += 2 * PI;
  }
  while(rad > PI){
    rad -= 2 * PI;
  }
  return rad;
}

double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

double toRad(double degree){
  return degree * PI/180;
}

double toDeg(double radian){
  return radian * 180/PI;
}



Slew::Slew(double accel_) : accel(accel_), decel(0) { noDecel = true; }

Slew::Slew(double accel_, double decel_) : accel(accel_), decel(decel_) {
  noDecel = false;
}

Slew::Slew(double accel_, double decel_, bool reversible_) : accel(accel_), decel(decel_), isReversible(reversible_) {
  noDecel = false;
}

Slew &Slew::withGains(double accel_, double decel_, bool reversible_){
  accel = accel_;
  decel = decel_;
  isReversible = reversible_;
  return *this;
}

Slew &Slew::withLimit(double limit_) {
  isLimited = true;
  limit = limit_;
  return *this;
}

double Slew::calculate(int input) { //From 7k Tower Takeover.
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

PID::PID(double kP_, double kI_, double kD_) : kP(kP_), kI(kI_), kD(kD_) {}

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

void PID::reset(){ output = target = current = error = integral = derivative = prevError = 0; }

} // namespace macro
