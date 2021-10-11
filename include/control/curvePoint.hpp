#pragma once
#include "globals.hpp"

class curvePoint{
  public:
    curvePoint(double x_, double y_, double moveSpeed_, double turnSpeed_,
               double followDistance_, double pointLength_,
               double slowDownTurnRadians_, double slowDownTurnAmount_);

    curvePoint(curvePoint const &thisPoint);

  private:
    double x, y, moveSpeed, turnSpeed, followDistance, pointLength,
        slowDownTurnRadians, slowDownTurnAmount;

};