#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_msgs/Pose2D.h"
#include <math.h>
#include "visionobject.h"
#include "robot.h"
#include "imageprocessor.h"

#include <ros/ros.h>
#include "playground/coords.h"

#define ESC 1048603


using namespace cv;
using namespace std;




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vision_talker");

	ros::NodeHandle n;
	ros::Publisher pubrobot = n.advertise<geometry_msgs::Pose2D>("vision_robot_position", 5);
	ros::Publisher pubball = n.advertise<geometry_msgs::Pose2D>("vision_ball_position", 5);


	// cap;
	//cap.open("http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg");

	ImageProcessor video = ImageProcessor("http://192.168.1.48:8080/stream?topic=/image&dummy=param.mjpg");
	//ImageProcessor video = ImageProcessor(); //use for webcam

	/*if(!cap.isOpened())
	{
		cout << "cap is closed" << endl;
		return -1;
	}//*/

	namedWindow("BallControl", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	namedWindow("RobotControl", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	//namedWindow("Robot2Control", CV_WINDOW_AUTOSIZE);

	int ballLowH = 111;
	int ballHighH = 179;

	int ballLowS = 41;
	int ballHighS = 162;

	int ballLowV = 183;
	int ballHighV = 255;


	int robot1LowH = 76;
	int robot1HighH = 107;

	int robot1LowS = 50;
	int robot1HighS = 137;

	int robot1LowV = 189;
	int robot1HighV = 255;

	int robot2LowH = 76;
	int robot2HighH = 107;

	int robot2LowS = 168;
	int robot2HighS = 255;

	int robot2LowV = 189;
	int robot2HighV = 255;	

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "BallControl", &ballLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "BallControl", &ballHighH, 179);

	cvCreateTrackbar("LowS", "BallControl", &ballLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "BallControl", &ballHighS, 255);

	cvCreateTrackbar("LowV", "BallControl", &ballLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "BallControl", &ballHighV, 255);

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "RobotControl", &robot1LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "RobotControl", &robot1HighH, 179);

	cvCreateTrackbar("LowS", "RobotControl", &robot1LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "RobotControl", &robot1HighS, 255);

	cvCreateTrackbar("LowV", "RobotControl", &robot1LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "RobotControl", &robot1HighV, 255);

	cvCreateTrackbar("LowH", "Robot2Control", &robot2LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Robot2Control", &robot2HighH, 179);

	cvCreateTrackbar("LowS", "Robot2Control", &robot2LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Robot2Control", &robot2HighS, 255);

	cvCreateTrackbar("LowV", "Robot2Control", &robot2LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Robot2Control", &robot2HighV, 255);

	Mat imgTemp;

	video.read(&imgTemp);

	//clones image so that we can do operations on it w/o messing up original image
	Mat centercircle = imgTemp.clone();

	Vec3f centercirc;

	// //locate the center circle of the image
	centercirc = video.findCenterCircle(centercircle);

	// //set the image center
	video.setCenter(Point2d(centercirc[0], centercirc[1]));

	// //with the center find the scaling factor
	video.setScalingFactor(CIRCLE_DIAMETER_IN_CM/(centercirc[2]*2));

	//covert original immage to HSV to find robots
	cvtColor(imgTemp, imgTemp, COLOR_BGR2HSV);

	Robot robot = Robot();
	Robot robot2 = Robot();

	VisionObject ball = VisionObject();

	//thresh hold the image
	inRange(imgTemp, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgTemp);

	//get rid of noise
	video.erodeDilate(imgTemp);

	// //find position and angle of robot
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


		Mat imgRobotThresh, imgBW, imgBallThresh, imgRobot2Thresh;

		//centercircle = imgOriginal.clone();



		//thresh hold the image
		inRange(imgHSV, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgRobotThresh);
		inRange(imgHSV, Scalar(ballLowH, ballLowS, ballLowV), Scalar(ballHighH, ballHighS, ballHighV), imgBallThresh);
		// inRange(imgHSV, Scalar(robot2LowH, robot2LowS, robot2LowV), Scalar(robot2HighH, robot2HighS, robot2HighV), imgRobot2Thresh);






		//cvtColor(imgHSV, imgBW, COLOR_BGR2GRAY);

		//otsu thresholding
		//threshold(imgBW, imgBW, 0,255, THRESHopencv draw line between points_BINARY | THRESH_OTSU);


		video.erodeDilate(imgRobotThresh);
		video.erodeDilate(imgBallThresh);
		// video.erodeDilate(imgRobot2Thresh);

		video.initializeRobot(&robot, imgRobotThresh);		
		video.initializeBall(&ball, imgBallThresh);
		// video.initializeRobot(&robot2, imgRobot2Thresh);

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
		imshow("robotthresh", imgRobotThresh);
		imshow("ballthresh", imgBallThresh);
		// imshow("robot2thresh", imgRobot2Thresh);


		//imshow("BWOTSU", imgBW);



		if(waitKey(1) == ESC)
		{
			break;
		}

		// -------------------------------------
		geometry_msgs::Pose2D robot1pos;
		geometry_msgs::Pose2D ballpos;
		robot1pos.x = robot.getLocation().x;
		robot1pos.y = robot.getLocation().y;
		robot1pos.theta = robot.getOrientation();


		ballpos.x = ball.getLocation().x;
		ballpos.y = ball.getLocation().y;

		pubrobot.publish(robot1pos);
		pubball.publish(ballpos);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


