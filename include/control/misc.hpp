#pragma once
#include "main.h"
#include "globals.hpp"

#define ACCEL 1
#define DECCEL 2

namespace intake{
    /*
    Set speed for intakes.
    */
    void spin(double speed);
    
    /*
    Set target and speed for intakes.
    */
    void spin(double target, double speed);

}

namespace mobileGoal{
    /*
    Moves mobile Goal to stated position.
    */
    void pos1(pros::Motor m);
    void pos2(pros::Motor m);

    /*
    Tares Mobile Goal.
    */
    void tare(pros::Motor m);   
};

namespace macro {
    class Slew{
        public:

        Slew(double accel_);
        Slew(double accel_, double decel_);
        Slew(double accel_, double decel_, bool reversible_);

        Slew& withLimit(double limit_);

        double calculate(int input);

        double getOutput();

        void reset();

        private:
            double accel, decel;
            double input, output, limit;
            bool isReversible, noDecel, isLimited;
    };

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
            double target = 0, current = 0, error = 0, integral = 0, derivative = 0, prevError = 0, kP = 0, kI = 0, kD = 0, output = 0;
    };
}

