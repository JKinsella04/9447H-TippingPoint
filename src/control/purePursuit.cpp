#include "control/purePursuit.hpp"
#include "odometry.hpp"

static Odometry Odom;

double PurePursuit::distToTarget, PurePursuit::absAngleToTarget, PurePursuit::relAngleToTarget; 
double PurePursuit::relXToPoint, PurePursuit::relYToPoint;
double PurePursuit::mvmtXPower, PurePursuit::mvmtYPower; 

void PurePursuit::goToPoint(double target_x, double target_y, double speed){
    distToTarget = hypot(target_x - Odom.getX(), target_y- Odom.getX());

    absAngleToTarget = atan2(target_y-Odom.getY(), target_x-Odom.getX());

    relAngleToTarget = absAngleToTarget - Odom.getThetaRad();

    relXToPoint = cos(relAngleToTarget) * distToTarget;
    relYToPoint = sin(relAngleToTarget) * distToTarget;

    mvmtXPower = relXToPoint / ( fabs(relXToPoint) + fabs(relYToPoint));
    mvmtYPower = relYToPoint / ( fabs(relXToPoint) + fabs(relYToPoint));


	double leftPower = mvmtYPower + mvmtXPower;
	double rightPower = mvmtYPower - mvmtXPower;

    std::cout << "LPow:" << leftPower << "RPow:" << rightPower << std::endl;

	// LF.move(leftPower * 1000);
	// LB.move(leftPower * 1000);
	// RF.move(rightPower * 1000);
	// RB.move(rightPower * 1000);

}