#include "control/purePursuit.hpp"
Odom Odom;

double PurePursuit::distToTarget, PurePursuit::absAngleToTarget, PurePursuit::relAngleToTarget; 
double PurePursuit::relXToPoint, PurePursuit::relYToPoint;
double PurePursuit::mvmtXPower, PurePursuit::mvmtYPower; 

void PurePursuit::goToPoint(double x, double y, double speed){
    distToTarget = hypot(x - Odom.getX(), y- Odom.getX());

    absAngleToTarget = atan2(y-Odom.getY(), x-Odom.getX());

    relAngleToTarget = absAngleToTarget - Odom.getThetaRad();

    relXToPoint = cos(relAngleToTarget) * distToTarget;
    relYToPoint = sin(relAngleToTarget) * distToTarget;

    mvmtXPower = relXToPoint / ( fabs(relXToPoint) + fabs(relYToPoint));
    mvmtYPower = relYToPoint / ( fabs(relXToPoint) + fabs(relYToPoint));

    std::cout << "XPow:" << mvmtXPower << "YPow:" << mvmtYPower << std::endl;

}