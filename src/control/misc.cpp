#include "control/misc.hpp"
#include "curvePoint.hpp"

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

// std::vector<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
//   if(abs(linePoint1.y - linePoint2.y) < 0.003){
//     linePoint1.y = linePoint2.y + 0.003;
//   }
//   if(abs(linePoint1.x - linePoint2.x) < 0.003){
//     linePoint1.x = linePoint2.x + 0.003;
//   }
  
//   double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);

  
//   double x1 =  linePoint1.x - circleCenter.x;
//   double y1 = linePoint1.y - circleCenter.y;

//   double quadracticA = 1.0 + pow(m1,2);
//   double quadracticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) *x1);
//   double quadracticC = ((pow(m1,2)) * pow(x1,2)) - (2.0 * y1 * m1 * x1) + pow(y1,2) - pow(radius, 2);

//   std::vector<Point> allPoints;

//   try{
//     double xRoot1 = (-quadracticB + sqrt(pow(quadracticB,2) -( 4 *quadracticA*quadracticC)))/(2*quadracticA);

//     double yRoot1 = m1 * (xRoot1 - x1) + y1;
    
//     // Reapply offset
//     xRoot1 += circleCenter.x;
//     yRoot1 += circleCenter.y;

//     double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
//     double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
    
//     if(xRoot1 > minX && xRoot1 < maxX){
//       allPoints.push_back(new Point(xRoot1,yRoot1));
//       allPoints.yRoot1 = yRoot1;
//     }

//     double xRoot2 = (-quadracticB - sqrt(pow(quadracticB,2) -( 4 *quadracticA*quadracticC)))/(2*quadracticA);
//     double yRoot2 = m1 * (xRoot2 - x1) + y1;
    
//     // Reapply offset
//     xRoot2 += circleCenter.x;
//     yRoot2 += circleCenter.y;

//     if(xRoot2 > minX && xRoot2 < maxX){
//       allPoints.xRoot2 = xRoot2;
//       allPoints.yRoot2 = yRoot2;
//     }

//   }catch(std::exception e){
    
//   }
//   return allPoints;
// }

// curvePoint getFollowPointPath(std::vector<curvePoint> pathPoints, double xPos, double yPos, double followRadius){
//   curvePoint followMe = new curvePoint(pathPoints.at(0));
// }



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

PID& PID::setError(double error_) { error = error_; return *this;}

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

double PID::getError() { return prevError; }

void PID::reset(){ output = target = current = error = integral = derivative = prevError = 0; }

} // namespace macro
