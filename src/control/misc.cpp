#include "control/misc.hpp"

namespace macro{
   double PID::target = 0, PID::current = 0, PID::error = 0, PID::integral = 0, 
          PID::derivative = 0, PID::prevError = 0, PID::kP = 0, PID::kI =0, PID::kD = 0, PID::output = 0;
   PID& PID::set(double kP_, double kI_, double kD_){
        kP = kP_;
        kI = kI_;
        kD = kD_;

        return *this;
    }
    
    double PID::calculate(double target, double current){
        // Proportional Calculation
        if(error == 0){
            error = target - current;
        }
        // Integral Calculation
        integral = integral + error;
        if( error == 0) integral = 0;
        if( integral > 12000) integral = 12000;
        
        // Derivative Calculation
        derivative = error - prevError;
        prevError = error;

        // Output Calculation
        output = error*kP + integral*kI + derivative*kD;
        error = 0;
        return output;
    }
    
    void PID::setError(double error_){
        error = error_;
    }
    double PID::getError(){
        return prevError;
    }
}
