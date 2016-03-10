#ifndef ROBOT_H
#define ROBOT_H

#include "visionobject.h"

class Robot : public VisionObject
{

	protected:
		double orientation;

	public:
		Robot();
		Robot(int MASK);
		Robot(Point2d loc, double orientation);
		double getOrientation();
		void setOrientation(double value);


};

#endif // ROBOT_H
