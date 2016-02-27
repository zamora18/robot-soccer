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
		Scalar lowHSV;
		Scalar highHSV;
		unsigned mask;
		bool threadstart;


	public:
		VisionObject();
		VisionObject(Point2d loc);



		Point2d getLocation() const;
		void setLocation(const Point2d &value);
		Scalar getLowHSV();
		void setLowHSV(Scalar hsv);
		Scalar getHighHSV();
		void setHighHSV(Scalar hsv);
		void toString();

		void setMask(unsigned MASK);
		unsigned getMask();

		void setThreadStart(bool start);
		bool getThreadStart();
		
};

#endif // VISIONOBJECT_H
