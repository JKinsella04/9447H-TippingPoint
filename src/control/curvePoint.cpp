#include "curvePoint.hpp"

curvePoint::curvePoint(double x_, double y_, double moveSpeed_, double turnSpeed_, double followDistance_, 
double pointLength_, double slowDownTurnRadians_, double slowDownTurnAmount_){
    x = x_;
    y = y_;
    moveSpeed = moveSpeed_;
    turnSpeed = turnSpeed_;
    followDistance = followDistance_;
    pointLength = pointLength_;
    slowDownTurnRadians = slowDownTurnRadians_;
    slowDownTurnAmount = slowDownTurnAmount_;
}

curvePoint::curvePoint(curvePoint const &thisPoint){
    x = thisPoint.x;
    y = thisPoint.y;
    moveSpeed = thisPoint.moveSpeed;
    turnSpeed = thisPoint.turnSpeed;
    followDistance = thisPoint.followDistance;
    pointLength = thisPoint.pointLength;
    slowDownTurnRadians = thisPoint.slowDownTurnRadians;
    slowDownTurnAmount = thisPoint.slowDownTurnAmount;
}