#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include "robot.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CIRCLE_DIAMETER_IN_CM 49.35

using namespace cv;
using namespace std;

class ImageProcessor
{

	protected:
		double scalingfactor;
		Point2d center;
		VideoCapture cap;
		
	public:
		ImageProcessor();
		ImageProcessor(string inputsource);

		vector<vector<Point> >  getContours(Mat contourOutput);

		bool initializeRobot(Robot* robot1, Mat img);

		double findAngleTwoPoints(Point2d p1, Point2d p2);

		void erodeDilate(Mat img);

		Vec3f findCenterCircle(Mat img);

		Point2d imageToFieldTransform(Point2d p);

		Point2d fieldToImageTransform(Point2d p);

		bool read(Mat* img);

		void setCenter(Point2d centeroffield);
		Point2d getCenter();

		void setScalingFactor(double scaling);
		double getScalingFactor();
};

#endif // IMAGEPROCESSOR_H
