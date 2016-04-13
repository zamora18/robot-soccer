#include "visionobject.h"
#include <iostream>

typedef struct 
{
	int colors[6];
} colorarray;

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


bool VisionObject::initColor(string color, int* LowH, int* LowS, int* LowV, int* HighH, int* HighS, int* HighV)
{
	colorarray colors;
	if(color == "b")
	{
		colors = (colorarray){81,110,75,255,189,255};
	}
	else if(color == "bc")
	{
		colors = (colorarray){77,111,0,45,217,255};
	}
	else if(color == "o")
	{
		colors = (colorarray){6,24,33,255,201,255};
	}
	else if(color == "g")
	{
		colors = (colorarray){49,80,16,1424,186,255};
	}
	else if(color == "p")
	{
		colors = (colorarray){119, 164,16,123,181,255};
	}
	else if(color == "r")
	{
		colors = (colorarray){0,179,81,105,226,255};
	}
	else 
		return false;

	*LowH = colors.colors[0];
	*HighH = colors.colors[1];
	*LowS = colors.colors[2];
	*HighS = colors.colors[3];
	*LowV = colors.colors[4];
	*HighV = colors.colors[5];

	return true;


}
