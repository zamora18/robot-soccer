#include "visionobject.h"
#include <iostream>

VisionObject::VisionObject()
{
	location.x = 0;
	location.y = 0;
}

VisionObject::VisionObject(Point2d loc)
{
	location = loc;
}


Point2d VisionObject::getLocation() const
{
	return location;
}

void VisionObject::setLocation(const Point2d &value)
{
	location = value;
}

void VisionObject::toString()
{
	cout << "(" << location.x << "," << location.y << ")";
}

Scalar VisionObject::getLowHSV()
{
	return lowHSV;
}

void VisionObject::setLowHSV(Scalar hsv)
{
	lowHSV = hsv;
}
Scalar VisionObject::getHighHSV()
{
	return highHSV;
}
void VisionObject::setHighHSV(Scalar hsv)
{
	highHSV = hsv;
}

void VisionObject::setMask(unsigned MASK)
{
	mask = MASK;
}
unsigned VisionObject::getMask()
{
	return mask;
}

void VisionObject::setThreadStart(bool start)
{
	threadstart = start;
}

bool VisionObject::getThreadStart()
{
	return threadstart;
}

void VisionObject::setNodeIdent(string identity)
{
	nodeIdent = identity;
}
string VisionObject::getNodeIdent()
{
	return nodeIdent;
}



