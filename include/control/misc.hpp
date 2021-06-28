#pragma once
#include "main.h"

#define ACCEL 1
#define DECCEL 2

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
          static double target, current, error, integral, derivative, prevError, kP, kI, kD, output;
    };
}

