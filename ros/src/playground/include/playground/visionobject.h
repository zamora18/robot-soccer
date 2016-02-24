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
		Scalar lowHSV;
		Scalar highHSV;


	public:
		VisionObject();
		VisionObject(Point2d location, double speed, double direction);



		Point2d getLocation() const;
		void setLocation(const Point2d &value);
		double getSpeed() const;
		void setSpeed(double value);
		double getDirection() const;
		void setDirection(double value);
		Scalar getLowHSV();
		void setLowHSV(Scalar hsv);
		Scalar getHighHSV();
		void setHighHSV(Scalar hsv);
		void toString();

		
};

#endif // VISIONOBJECT_H
