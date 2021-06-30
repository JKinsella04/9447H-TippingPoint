#pragma once
#include "main.h"
#include "globals.hpp"
#include "control/odometry.hpp"
#include <math.h>

class PurePursuit {
    public:
        /*
        Drive to target ordered pair with given speed.
        */
        void goToPoint(double x, double y, double speed);

    private:
        static double distToTarget, absAngleToTarget, relAngleToTarget; 
        static double relXToPoint, relYToPoint;
        static double mvmtXPower, mvmtYPower; 
};