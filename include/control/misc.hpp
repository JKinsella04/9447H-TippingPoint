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
            double calculate(double target, double current);

        private:
          double target, current, error, integral, derivative, prevError, kP, kD, kI, output;
    };
}

