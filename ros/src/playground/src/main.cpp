#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include "visionobject.h"
#include "robot.h"
#include "imageprocessor.h"

#include <ros/ros.h>
#include "playground/coords.h"



using namespace cv;
using namespace std;




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vision_talker");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<playground::coords>("vision", 5);


	// cap;
	//cap.open("http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg");

	ImageProcessor video = ImageProcessor("http://192.168.1.48:8080/stream?topic=/image&dummy=param.mjpg");

	/*if(!cap.isOpened())
	{
		cout << "cap is closed" << endl;
		return -1;
	}//*/

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 80;
	int iHighH = 100;

	int iLowS = 47;
	int iHighS = 255;

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

	video.read(&imgTemp);

	//clones image so that we can do operations on it w/o messing up original image
	Mat centercircle = imgTemp.clone();

	Vec3f centercirc;

	//locate the center circle of the image
	centercirc = video.findCenterCircle(centercircle);

	//set the image center
	video.setCenter(Point2d(centercirc[0], centercirc[1]));

	//with the center find the scaling factor
	video.setScalingFactor(CIRCLE_DIAMETER_IN_CM/(centercirc[2]*2));

	//covert original immage to HSV to find robots
	cvtColor(imgTemp, imgTemp, COLOR_BGR2HSV);

	Robot robot = Robot();

	//thresh hold the image
	inRange(imgTemp, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgTemp);

	//get rid of noise
	video.erodeDilate(imgTemp);

	//find position and angle of robot
	video.initializeRobot(&robot, imgTemp);

	//yay we did it!
	cout << "initialized" << endl;

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		Mat imgOriginal;


		//load in the camera image
		bool readSuccess = video.read(&imgOriginal);

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


		video.erodeDilate(imgThresholded);

		video.initializeRobot(&robot, imgThresholded);		

		Point2d robotlocation = video.fieldToImageTransform(robot.getLocation());
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
		msg.center_x = video.getCenter().x;
		msg.center_y = video.getCenter().y;

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


