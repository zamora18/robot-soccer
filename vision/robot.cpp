#include "robot.h"


double Robot::getOrientation()
{
	return orientation;
}

void Robot::setOrientation(double value)
{
	orientation = value;
}
Robot::Robot()
{
	orientation = 0;
	VisionObject();
}

Robot::Robot(Point2d location, double speed, double direction, double orientation)
{
	this->orientation = orientation;
	VisionObject(location, speed, direction);
}
