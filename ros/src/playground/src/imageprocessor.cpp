#include "imageprocessor.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <math.h>
#include <iostream>


#define CENTER_OF_ROBOT_OFFSET_IN_M .03175
#define ERODE_DILATE_SIZE 5


using namespace cv;
using namespace std;


//use for webcam, 0 for first webcam, 1 for second
ImageProcessor::ImageProcessor()
{
	
}

ImageProcessor::ImageProcessor(double scalingfactor, Point2d center)
{
	setCenter(center);
	setScalingFactor(scalingfactor);

}

ImageProcessor::ImageProcessor(int capnumber)
{
	cap.open(capnumber);
}

//open the input string source
ImageProcessor::ImageProcessor(string inputsource)
{
	cap.open(inputsource);

}

void ImageProcessor::setCenter(Point2d centeroffield)
{
	center = centeroffield;
}

Point2d ImageProcessor::getCenter()
{
	return center;
}

void ImageProcessor::setScalingFactor(double scaling)
{
	scalingfactor = scaling;
}
double ImageProcessor::getScalingFactor()
{
	return scalingfactor;
}

//reads in the image from the input source
bool ImageProcessor::read(Mat* img)
{
	return cap.read(*img);
}

//gets the conntours of a thresholded img
vector<vector<Point> >  ImageProcessor::getContours(Mat contourOutput)
{

	vector<vector<Point> > contours;

	//detect edges
	//Canny(contourOutput, contourOutput, 0,255,3);


	//find countours
	findContours(contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	return contours;

}


bool ImageProcessor::initializeBall(VisionObject* ball, Mat img)
{
	Mat contourOutput;

	contourOutput = img.clone();

	//get the contours of the image
	vector<vector<Point> > contours = getContours(contourOutput);

	vector<Moments> contourmoments = vector<Moments>();

	for(int i = 0; i < contours.size(); i++)
	{
		//eliminates any noise
		//if(contourArea(contours[i]) > 100)
		//{
			contourmoments.push_back(moments(contours[i]));
		//}

	}

	vector<Point2d> objects = vector<Point2d>();


	for(int i = 0; i < contourmoments.size(); i++)
	{
		double x = (contourmoments[i].m10)/(contourmoments[i].m00);
		double y = (contourmoments[i].m01)/(contourmoments[i].m00);
		objects.push_back(Point2d(x,y));
	}

	if(objects.size() != 1)
	{
		cout << "CANT FIND BALL" << endl;
		return false;
	}
	else
	{
		ball->setLocation(imageToFieldTransform(objects[0]));
		return true;
	}
}
//initializes the robot positions, can be used to update position also
bool ImageProcessor::initializeRobot(Robot *robot, Mat img, Point2d boxloc)
{

	Mat contourOutput;

	contourOutput = img.clone();

	//get the contours of the image
	vector<vector<Point> > contours = getContours(contourOutput);

	vector<Moments> contourmoments = vector<Moments>();




	for(int i = 0; i < contours.size(); i++)
	{
		//eliminates any noise
		if(contourArea(contours[i]) > 50)
		{
			contourmoments.push_back(moments(contours[i]));
		}

	}

	vector<Point2d> objects = vector<Point2d>();

	for(int i = 0; i < contourmoments.size(); i++)
	{
		double x = (contourmoments[i].m10)/(contourmoments[i].m00);
		double y = (contourmoments[i].m01)/(contourmoments[i].m00);
		objects.push_back(Point2d(x,y));
	}

	Point2d p1, p2;

	if(objects.size() != 2)
	{
		return false;
	}

	if(contourmoments[0].m00 > contourmoments[1].m00)
	{
		p1 = objects[0];
		p2 = objects[1];
	}
	else
	{
		p2 = objects[0];
		p1 = objects[1];
	}

	double angle = findAngleTwoPoints(p1, p2);



	robot->setOrientation(angle);

	p1.x = (p1.x + p2.x)/2;
	p1.y = (p1.y + p2.y)/2;

	robot->setLocation(imageToFieldTransform(p1));

	//correctRobotCenter(robot);

	return true;

}

void ImageProcessor::correctRobotCenter(Robot* robot)
{
	Point2d robotcenter = robot->getLocation();
	double theta = robot->getOrientation();
	Point2d actualcenter;
	actualcenter.x = robotcenter.x + CENTER_OF_ROBOT_OFFSET_IN_M * cos(theta*180/M_PI);
	actualcenter.y = robotcenter.y + CENTER_OF_ROBOT_OFFSET_IN_M * sin(theta*180/M_PI);
	if(abs(actualcenter.x - robotcenter.x) > CENTER_OF_ROBOT_OFFSET_IN_M || abs(actualcenter.y - robotcenter.y) > CENTER_OF_ROBOT_OFFSET_IN_M)
	{
		cout << "bad calculation" << endl;
		cout << actualcenter.x << "," << actualcenter.y << endl;
		return;
	}
	robot->setLocation(actualcenter);
}

//finds the angle between two points, p1->p2
double ImageProcessor::findAngleTwoPoints(Point2d p1, Point2d p2)
{

	double angle;
	angle = atan2(p2.y - p1.y, p2.x - p1.x)*180/M_PI;

	if (angle < 0)
		angle +=360;

	angle = 360 - angle;

	return angle;
}

//erodes and dilates an image to reduce noise
void ImageProcessor::erodeDilate(Mat img)
{
	//averages pixels in ovals to get rid of background noise
//	morphological opening (remove small objects from the foreground)
	erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(ERODE_DILATE_SIZE, ERODE_DILATE_SIZE)) );
	dilate( img, img, getStructuringElement(MORPH_ELLIPSE, Size(ERODE_DILATE_SIZE, ERODE_DILATE_SIZE)) );

	//morphological closing (fill small holes in the foreground)
	dilate( img, img, getStructuringElement(MORPH_ELLIPSE, Size(ERODE_DILATE_SIZE, ERODE_DILATE_SIZE)) );
	erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(ERODE_DILATE_SIZE, ERODE_DILATE_SIZE)) );
}

//finds the circle in the center of the field and returns it
//Vec3f contains point center ([0],[1]), and radius at [2]
Vec3f ImageProcessor::findCenterCircle(Mat img)
{
	Mat imgclone = img.clone();

	cvtColor(img, img, CV_BGR2GRAY);

	//blur to avoid false detection
	//blur(img, img, Size(3,3));

	vector<Vec3f> circles;


	HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1, img.rows/8);
	
	while(circles.size() != 1)
	{
		cap.read(img);

		cvtColor(img, img, CV_BGR2GRAY);

		HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1, img.rows/8);
	}

	// for( size_t i = 0; i < circles.size(); i++ )
	// {
	// 	 Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	// 	 int radius = cvRound(circles[i][2]);
	// 	 // circle center
	// 	 circle( imgclone, center, 3, Scalar(0,255,0), -1, 8, 0 );
	// 	 // circle outline
	// 	 circle( imgclone, center, radius, Scalar(0,0,255), 3, 8, 0 );
	// }

	// imshow("circles!", imgclone);

	return circles[0];

}

//returns the transform of a point p in the image to a cartesian plot on the field
Point2d ImageProcessor::imageToFieldTransform(Point2d p)
{
	//cout << scalingfactor << endl;
	p.x =  (p.x - center.x)*scalingfactor;
	p.y =  (center.y - p.y)*scalingfactor;
	return p;
}

//returns the transform of a point on the field to a point in the image
Point2d ImageProcessor::fieldToImageTransform(Point2d p)
{
	p.x = (p.x/scalingfactor) + center.x;
	p.y = center.y - (p.y/scalingfactor) ;
	return p;
}


void ImageProcessor::initializeCenter(Mat centercircle)
{
	//clones image so that we can do operations on it w/o messing up original image


	Vec3f centercirc;

	// //locate the center circle of the image
	centercirc = findCenterCircle(centercircle);

	// //set the image center
	center = Point2d(centercirc[0], centercirc[1]);

	// //with the center find the scaling factor
	scalingfactor = CIRCLE_DIAMETER_IN_M/(centercirc[2]*2);
}

void ImageProcessor::invertObjForAway(VisionObject* obj)
{
	Point2d p = obj->getLocation();
	p.x = -p.x;
	p.y = -p.y;
	obj->setLocation(p);

}

void ImageProcessor::invertRobotForAway(Robot* robot)
{
	double theta = robot->getOrientation();

	if (theta < 180)
		theta += 180;
	else
		theta -= 180;

	robot->setOrientation(theta);

	invertObjForAway(robot);
}
