#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_msgs/Pose2D.h"
#include <math.h>
#include "visionobject.h"
#include "robot.h"
#include "imageprocessor.h"
#include <pthread.h>

#include <ros/ros.h>
#include "playground/coords.h"

#define ESC 1048603


using namespace cv;
using namespace std;

Mat currentImage;
pthread_cond_t robot1cond;
pthread_cond_t robot1done;
pthread_mutex_t mrobot1cond;
pthread_mutex_t mrobot1done;
bool done1, start1;

double scalingfactor;
Point2d center;

void* trackRobot(void* robotobject);

void* trackBall(void* ballobject);


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vision_talker");

	ros::NodeHandle n;
	ros::Publisher pubrobot = n.advertise<geometry_msgs::Pose2D>("vision_robot_position", 5);
	ros::Publisher pubball = n.advertise<geometry_msgs::Pose2D>("vision_ball_position", 5);


	// cap;
	//cap.open("http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg");

	ImageProcessor video = ImageProcessor("http://192.168.1.78:8080/stream?topic=/image&dummy=param.mjpg");
	//ImageProcessor video = ImageProcessor(0); //use for webcam

	/*if(!cap.isOpened())
	{
		//cout << "cap is closed" << endl;
		return -1;
	}//*/

	namedWindow("BallControl", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	namedWindow("RobotControl", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	//namedWindow("Robot2Control", CV_WINDOW_AUTOSIZE);

	int ballLowH = 158;
	int ballHighH = 175;

	int ballLowS = 41;
	int ballHighS = 150;

	int ballLowV = 183;
	int ballHighV = 255;


	int robot1LowH = 160;
	int robot1HighH = 179;

	int robot1LowS = 117;
	int robot1HighS = 255;

	int robot1LowV = 164;
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

	/*cvCreateTrackbar("LowH", "Robot2Control", &robot2LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Robot2Control", &robot2HighH, 179);

	cvCreateTrackbar("LowS", "Robot2Control", &robot2LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Robot2Control", &robot2HighS, 255);

	cvCreateTrackbar("LowV", "Robot2Control", &robot2LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Robot2Control", &robot2HighV, 255);*/

//	event = 0;
	Mat imgTemp;

	video.read(&imgTemp);

	Mat centercircle = imgTemp.clone();

	//find the center circle
	video.initializeCenter(centercircle);

	scalingfactor = video.getScalingFactor();

	center = video.getCenter();
	//covert original immage to HSV to find robots
	//cvtColor(imgTemp, imgTemp, COLOR_BGR2HSV);

	Robot robot = Robot();
	Robot robot2 = Robot();

	VisionObject ball = VisionObject();

	//thresh hold the image
	//inRange(imgTemp, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgTemp);

	//get rid of noise
	//video.erodeDilate(imgTemp);

	// //find position and angle of robot
	//video.initializeRobot(&robot, imgTemp);

	//initialize threads
	pthread_t robot1thread;

	pthread_cond_init(&robot1cond, NULL);
	pthread_cond_init(&robot1done, NULL);

	pthread_mutex_init(&mrobot1cond, NULL);
	pthread_mutex_init(&mrobot1done, NULL);

	pthread_create(&robot1thread, NULL, trackRobot, &robot);

	robot.setLowHSV(Scalar(robot1LowH, robot1LowS, robot1LowV));
	robot.setHighHSV(Scalar(robot1HighH, robot1HighS, robot1HighV));

	//yay we did it!
	//cout << "initialized" << endl;

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		Mat imgOriginal;

		//cout << "reading" << endl;
		//load in the camera image
		bool readSuccess = video.read(&imgOriginal); //check this

	//	event = 1;
		//if the read failed exit
		if(!readSuccess)
		{
			//cout << "could not read frame from camera" << endl;
			break;
		}


		
		//cout << "cloning" << endl;

		currentImage = imgOriginal.clone();

		//cout << "go threads go" << endl;

		pthread_mutex_lock(&mrobot1cond);
		done1 = false;
		start1 = true;
		pthread_cond_signal(&robot1cond);
		pthread_mutex_unlock(&mrobot1cond);

		//cout << "done telling threads to go" << endl;

		//blur(imgOriginal, imgOriginal, Size(3,3));

		Mat imgHSV;

		//convert from the input image to a HSV image
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);


		Mat imgRobotThresh, imgBW, imgBallThresh, imgRobot2Thresh;

		//centercircle = imgOriginal.clone();



		//thresh hold the image
		//inRange(imgHSV, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgRobotThresh);
		inRange(imgHSV, Scalar(ballLowH, ballLowS, ballLowV), Scalar(ballHighH, ballHighS, ballHighV), imgBallThresh);
		// inRange(imgHSV, Scalar(robot2LowH, robot2LowS, robot2LowV), Scalar(robot2HighH, robot2HighS, robot2HighV), imgRobot2Thresh);






		//cvtColor(imgHSV, imgBW, COLOR_BGR2GRAY);

		//otsu thresholding
		//threshold(imgBW, imgBW, 0,255, THRESHopencv draw line between points_BINARY | THRESH_OTSU);


		//video.erodeDilate(imgRobotThresh);
		video.erodeDilate(imgBallThresh);
		// video.erodeDilate(imgRobot2Thresh);

		//video.initializeRobot(&robot, imgRobotThresh);		
		video.initializeBall(&ball, imgBallThresh);
		// video.initializeRobot(&robot2, imgRobot2Thresh);


		//wait for various threads to finish

		////cout << "waiting for thread to finish" << endl;

		pthread_mutex_lock(&mrobot1done);
		while(!done1)
			pthread_cond_wait(&robot1done, &mrobot1done);
		pthread_mutex_unlock(&mrobot1done);

		////cout << "thread finished! finishing and displaying" << endl;

		Point2d robotlocation = video.fieldToImageTransform(robot.getLocation());
		Point2d end;

		double angle = robot.getOrientation();

		double sinx = (cos(angle * M_PI/180));
		double siny = (sin(angle * M_PI/180));

		siny *= -1;

		end.x = robotlocation.x + 50 * sinx;
		end.y = robotlocation.y + 50 * siny;

		////cout << "robot stuffs done" << endl;


		line(imgOriginal, robotlocation, end, Scalar(0,0,255), 2);

		circle(imgOriginal, video.fieldToImageTransform(ball.getLocation()), 10, Scalar(0,0,255));
		
		stringstream ss, ss1;
		ss << "(" << (int)ball.getLocation().x << "," << (int)ball.getLocation().y << ")";
		putText(imgOriginal, ss.str(), video.fieldToImageTransform(ball.getLocation()), 1, FONT_HERSHEY_PLAIN, Scalar(0,0,255));

		ss1 << "(" << (int)robot.getLocation().x << "," << (int)robot.getLocation().y << "," << (int)robot.getOrientation() << ")";
		putText(imgOriginal, ss1.str(), video.fieldToImageTransform(robot.getLocation()), 1, FONT_HERSHEY_PLAIN, Scalar(0,0,255));

		//cout << "line drawn" << endl;

		// show the original image with tracking line
		imshow("Raw Image", imgOriginal);
		//show the new image
		//imshow("robotthresh", imgRobotThresh);
		imshow("ballthresh", imgBallThresh);
		// imshow("robot2thresh", imgRobot2Thresh);//*/


		//imshow("BWOTSU", imgBW);

		//cout << "leaging" << endl;

		if(waitKey(1) == ESC)
		{
			break;
		}

		//cout << "sending ros message" << endl;

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
	//pthread_exit(NULL);
	pthread_mutex_destroy(&mrobot1cond);
	pthread_mutex_destroy(&mrobot1done);
	pthread_cond_destroy(&robot1cond);
	pthread_cond_destroy(&robot1done);
	return 0;
}



void* trackRobot(void* robotobject)
{
	
	Robot* robot = (Robot*)(robotobject);
	ImageProcessor video = ImageProcessor(scalingfactor, center);
	int c = 0;
	while(1)
	{
		Mat robotimage;
		//insert condwait here

		//cout << "wait for thread go command" << endl;

		pthread_mutex_lock(&mrobot1cond);
		while(!start1)
			pthread_cond_wait(&robot1cond, &mrobot1cond);
		pthread_mutex_unlock(&mrobot1cond);

		//cout << "go command received" << endl;

		//event = 0;
		cvtColor(currentImage, robotimage, COLOR_BGR2HSV);
		inRange(robotimage, robot->getLowHSV(), robot->getHighHSV(), robotimage);


		video.erodeDilate(robotimage);

		video.initializeRobot(robot, robotimage);

		//cout << "sending thread done command" << endl;

		pthread_mutex_lock(&mrobot1done);
		done1 = true;
		start1 = false;
		pthread_cond_signal(&robot1done);
		pthread_mutex_unlock(&mrobot1done);

		//cout << "thread done command sent" << endl;

		//cout << ++c << endl;
		//imshow("robot thread threshold", robotimage);

	}
}

void* trackBall(void* ballobject)
{
	VisionObject* ball = (VisionObject*)(ballobject);
}