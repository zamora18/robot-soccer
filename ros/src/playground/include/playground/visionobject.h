#ifndef VISIONOBJECT_H
#define VISIONOBJECT_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace cv;
using namespace std;


class VisionObject
{
	protected:
		Point2d location;
		double speed;
		double direction;


	public:
		VisionObject();
		VisionObject(Point2d location, double speed, double direction);



		Point2d getLocation() const;
		void setLocation(const Point2d &value);
		double getSpeed() const;
		void setSpeed(double value);
		double getDirection() const;
		void setDirection(double value);
		void toString();
};

#endif // VISIONOBJECT_H
