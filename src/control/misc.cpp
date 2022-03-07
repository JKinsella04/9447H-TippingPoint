#include "control/misc.hpp"

namespace conveyer {
void spin(double speed) { intake.move_velocity(speed); }

void spin(double target, double speed) { intake.move_relative(target, speed); }
} // namespace intake

namespace macro {

void print(std::string s, double value){
  std::cout << s << value << std::endl;
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

coords quadracticBezier ( coords p0, coords p1, coords p2, double t){
  coords cp;
  cp.x = p1.x * 2 - ( p0.x + p2.x ) /2;
  cp.y = p1.y * 2 - ( p0.y + p2.y ) /2;
  
  coords pFinal;
  pFinal.x = pow(1 - t, 2) * p0.x + (1 - t) * 2 * t * cp.x + t * t * p2.x;
  pFinal.y = pow(1 - t, 2) * p0.y + (1 - t) * 2 * t * cp.y + t * t * p2.y;
  return pFinal;
}

coords cubicBezier ( coords p0, coords p1, coords p2, coords p3, double t){
  coords cp;
  cp.x = p1.x * 2 - ( p0.x + p2.x ) /2;
  cp.y = p1.y * 2 - ( p0.y + p2.y ) /2;

  coords pFinal;
  pFinal.x = pow( 1 - t, 3) * p0.x + pow( 1 - t, 2) * 3 * t * cp.x + (1 - t) * 3 * t * t * p2.x + t * t * t * p3.x;
  pFinal.y = pow( 1 - t, 3) * p0.y + pow( 1 - t, 2) * 3 * t * cp.y + (1 - t) * 3 * t * t * p2.y + t * t * t * p3.y;
  return pFinal;
}

Slew::Slew(double accel_) : accel(accel_), decel(0) { noDecel = true; }

Slew::Slew(double accel_, double decel_) : accel(accel_), decel(decel_) {
  noDecel = false;
}

Slew::Slew(double accel_, double decel_, bool reversible_) : accel(accel_), decel(decel_), isReversible(reversible_) {
  noDecel = false;
}

Slew &Slew::withGains(double accel_, double decel_, bool reversible_){
  accel = accel_ * 12000/4.78;
  decel = decel_ * 12000/4.78;
  isReversible = reversible_;
  return *this;
}

Slew &Slew::withLimit(double limit_) {
  isLimited = true;
  limit = limit_ * 12000/4.78;
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

PID& PID::setError(double error_) { 
  calcErr = false;
  error = error_; 
  return *this;
}

double PID::calculate(double target, double current) {
  // Proportional Calculation
  error = target - current;

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
  return output;
}

double PID::getError() { return error; }

void PID::reset(){ output = target = current = error = integral = derivative = prevError = 0; }
} // namespace macro
