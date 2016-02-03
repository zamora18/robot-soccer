#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include "visionobject.h"
#include "robot.h"

#include <ros/ros.h>
#include "playground/coords.h"

#define CIRCLE_DIAMETER_IN_CM 49.35

using namespace cv;
using namespace std;

double scalingfactor;
Point2d center;
VideoCapture cap;

vector<vector<Point> >  getContours(Mat contourOutput);

bool initializeRobot(Robot* robot1, Mat img);

double findAngleTwoPoints(Point2d p1, Point2d p2);

void erodeDilate(Mat img);

Vec3f findCenterCircle(Mat img);

Point2d imageToFieldTransform(Point2d center, Point2d p);

Point2d fieldToImageTransform(Point2d center, Point2d p);

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vision_talker");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<playground::coords>("vision", 5);


	// cap;
	cap.open("http://192.168.1.36:8080/stream?topic=/image&dummy=param.mjpg");

	/*if(!cap.isOpened())
	{
		cout << "cap is closed" << endl;
		return -1;
	}//*/

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 80;
	int iHighH = 100;

	int iLowS = 47;
	int iHighS = 241;

	int iLowV = 0;
	int iHighV = 255;

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	Mat imgTemp;
	cap.read(imgTemp);

	Mat centercircle = imgTemp.clone();

	Vec3f centercirc;

	centercirc = findCenterCircle(centercircle);

	center = Point2d(centercirc[0], centercirc[1]);

	scalingfactor = CIRCLE_DIAMETER_IN_CM/(centercirc[2]*2);


	cvtColor(imgTemp, imgTemp, COLOR_BGR2HSV);

	Robot robot = Robot();

	//thresh hold the image
	inRange(imgTemp, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgTemp);


	erodeDilate(imgTemp);

	initializeRobot(&robot, imgTemp);

	//Mat imgLines = Mat::zeros(imgTemp.size(), CV_8UC3);

	cout << "initialized" << endl;

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		Mat imgOriginal;


		//load in the camera image
		bool readSuccess = cap.read(imgOriginal);

		//if the read failed exit
		if(!readSuccess)
		{
			cout << "could not read frame from camera" << endl;
			break;
		}


		//blur(imgOriginal, imgOriginal, Size(3,3));

		Mat imgHSV;

		//convert from the input image to a HSV image
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);


		Mat imgThresholded, imgBW;

		centercircle = imgOriginal.clone();



		//thresh hold the image
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);





		//cvtColor(imgHSV, imgBW, COLOR_BGR2GRAY);

		//otsu thresholding
		//threshold(imgBW, imgBW, 0,255, THRESHopencv draw line between points_BINARY | THRESH_OTSU);


		erodeDilate(imgThresholded);

		initializeRobot(&robot, imgThresholded);		

		Point2d robotlocation = fieldToImageTransform(center, robot.getLocation());
		Point2d end;

		double angle = robot.getOrientation();

		double sinx = (cos(angle * M_PI/180));
		double siny = (sin(angle * M_PI/180));

		siny *= -1;

		end.x = robotlocation.x + 50 * sinx;
		end.y = robotlocation.y + 50 * siny;




		line(imgOriginal, robotlocation, end, Scalar(0,0,255), 2);


		// show the original image with tracking line
		imshow("Raw Image", imgOriginal);
		//show the new image
		imshow("thresh", imgThresholded);



		//imshow("BWOTSU", imgBW);



		if(waitKey(1) >= 0)
		{
			break;
		}

		// -------------------------------------
		playground::coords msg;
		msg.robot_x = robot.getLocation().x;
		msg.robot_y = robot.getLocation().y;
		msg.robot_theta = robot.getOrientation();
		msg.center_x = center.x;
		msg.center_y = center.y;

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


vector<vector<Point> >  getContours(Mat contourOutput)
{

	vector<vector<Point> > contours;

	//detect edges
	//Canny(contourOutput, contourOutput, 0,255,3);


	//find countours
	findContours(contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	return contours;

}

bool initializeRobot(Robot *robot, Mat img)
{

	Mat contourOutput;

	contourOutput = img.clone();

	vector<vector<Point> > contours = getContours(contourOutput);


	for(int i = 0; i < contours.size(); i++)
	{
		drawContours(img, contours, i, Scalar(0,0,255), 2);
	}

	vector<Moments> contourmoments = vector<Moments>();




	for(int i = 0; i < contours.size(); i++)
	{
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

		circle(img, objects[i], 5, Scalar(0,0,0), 2,8, 0);
	}

	Point2d p1, p2;

	if(objects.size() < 2)
	{
		return false;
	}

	if(contourmoments[0].m00 < contourmoments[1].m00)
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

	stringstream ss;

	ss << "angle = " << angle;

	line(img, p1, p2, Scalar(0,255,255), 2);
	line(img, p1, Point(img.size().width, p1.y), Scalar(0,255,255), 2);
	putText(img, ss.str(), Point(p1.x+50,p1.y-50), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255),2);


	robot->setOrientation(angle);
	robot->setLocation(imageToFieldTransform(center, p2));


}




double findAngleTwoPoints(Point2d p1, Point2d p2)
{

	double angle;
	angle = atan2(p2.y - p1.y, p2.x - p1.x)*180/M_PI;

	if (angle < 0)
		angle +=360;

	angle = 360 - angle;

	return angle;
}


void erodeDilate(Mat img)
{
	//averages pixels in ovals to get rid of background noise
//	morphological opening (remove small objects from the foreground)
	erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	//morphological closing (fill small holes in the foreground)
	dilate( img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
}


Vec3f findCenterCircle(Mat img)
{
	Mat imgclone = img.clone();

	cvtColor(img, img, CV_BGR2GRAY);

	//blur to avoid false detection
	//blur(img, img, Size(3,3));

	vector<Vec3f> circles;

	do
	{	//cout << "robot location = (" << p2.x << "," << p2.y << ")" << "orientation = " << angle << endl;
		HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1, img.rows/8);

	} while (circles.size() != 1);

	for( size_t i = 0; i < circles.size(); i++ )
	{
		 Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		 int radius = cvRound(circles[i][2]);
		 // circle center
		 circle( imgclone, center, 3, Scalar(0,255,0), -1, 8, 0 );
		 // circle outline
		 circle( imgclone, center, radius, Scalar(0,0,255), 3, 8, 0 );
	}

	imshow("circles!", imgclone);

	return circles[0];

}

Point2d imageToFieldTransform(Point2d center, Point2d p)
{
	p.x =  (p.x - center.x)*scalingfactor;
	p.y =  (center.y - p.y)*scalingfactor;
	return p;
}

Point2d fieldToImageTransform(Point2d center, Point2d p)
{
	p.x = (p.x/scalingfactor) + center.x;
	p.y = center.y - (p.y/scalingfactor) ;
	return p;
}




