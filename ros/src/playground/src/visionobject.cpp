#include "visionobject.h"
#include <iostream>

VisionObject::VisionObject()
{
	location.x = 0;
	location.y = 0;
	speed = 0;
	direction = 0;
}

VisionObject::VisionObject(Point2d location, double speed, double direction)
{
	this->location = location;
	this->speed = speed;
	this->direction = direction;
}


Point2d VisionObject::getLocation() const
{
	return location;
}

void VisionObject::setLocation(const Point2d &value)
{
	location = value;
}

double VisionObject::getSpeed() const
{
	return speed;
}

void VisionObject::setSpeed(double value)
{
	speed = value;
}

double VisionObject::getDirection() const
{
	return direction;
}

void VisionObject::setDirection(double value)
{
	direction = value;
}

void VisionObject::toString()
{
	cout << "(" << location.x << "," << location.y << ")";
}
