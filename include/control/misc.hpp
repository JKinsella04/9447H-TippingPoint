#pragma once
#include "main.h"

template<class State>
class StateMachine{
    public:
    void setState(State newState){
        state = newState;
    }
    
    State state;

};

namespace macro {
    class PID{
        public:
            /*
            set the PID values
            */
            PID& set(double kP_, double kI_, double kD_);

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
        private:
          static double target, current, error, integral, derivative, prevError, kP, kI, kD, output;
    };
}

