#ifndef ROBOT_H
#define ROBOT_H

#include "visionobject.h"

class Robot : public VisionObject
{

	protected:
		double orientation;
	public:
		Robot();
		Robot(Point2d location, double speed, double direction, double orientation);
		double getOrientation();
		void setOrientation(double value);
};

#endif // ROBOT_H
