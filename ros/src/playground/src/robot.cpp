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
	threadstart = false;
	VisionObject();
}

Robot::Robot(Point2d location, double orientation)
{
	this->orientation = orientation;
	threadstart = false;
	this->location = location;
}

Robot::Robot(int MASK)
{
	threadstart = false;
	mask = MASK;
}


