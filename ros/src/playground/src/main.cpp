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
#define ROBOT1_TRACKING_MASK 0x1
#define ROBOT2_TRACKING_MASK 0x2
#define BALL_TRACKING_MASK 0x4
#define ALL_THREADS_DONE 0x7


using namespace cv;
using namespace std;

Mat currentImage;
pthread_cond_t threadsstartcond;
pthread_cond_t threadsdonecond;
pthread_mutex_t mthreadsstartcond;
pthread_mutex_t mthreadsdonecond;
bool done1, start;
unsigned threadsdone;


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
	namedWindow("Robot2Control", CV_WINDOW_AUTOSIZE);

	int ballLowH = 158;
	int ballHighH = 175;

	int ballLowS = 41;
	int ballHighS = 150;

	int ballLowV = 183;
	int ballHighV = 255;


	int robot1LowH = 79;
	int robot1HighH = 126;

	int robot1LowS = 48;
	int robot1HighS = 140;

	int robot1LowV = 171;
	int robot1HighV = 255;

	int robot2LowH = 125;
	int robot2HighH = 154;

	int robot2LowS = 0;
	int robot2HighS = 97;

	int robot2LowV = 189;
	int robot2HighV = 255;	

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "BallControl", &ballLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "BallControl", &ballHighH, 179);

	cvCreateTrackbar("LowS", "BallControl", &ballLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "BallControl", &ballHighS, 255);

	cvCreateTrackbar("LowV", "BallControl", &ballLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "BallControl", &ballHighV, 255);

	//Create trackbars in "robotControl" window
	cvCreateTrackbar("LowH", "RobotControl", &robot1LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "RobotControl", &robot1HighH, 179);

	cvCreateTrackbar("LowS", "RobotControl", &robot1LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "RobotControl", &robot1HighS, 255);

	cvCreateTrackbar("LowV", "RobotControl", &robot1LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "RobotControl", &robot1HighV, 255);

	//create robot2 trackbars
	cvCreateTrackbar("LowH", "Robot2Control", &robot2LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Robot2Control", &robot2HighH, 179);

	cvCreateTrackbar("LowS", "Robot2Control", &robot2LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Robot2Control", &robot2HighS, 255);

	cvCreateTrackbar("LowV", "Robot2Control", &robot2LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Robot2Control", &robot2HighV, 255);

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

	Robot robot1 = Robot(ROBOT1_TRACKING_MASK);
	Robot robot2 = Robot(ROBOT2_TRACKING_MASK);

	VisionObject ball = VisionObject();
	ball.setMask(BALL_TRACKING_MASK);

	//thresh hold the image
	//inRange(imgTemp, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgTemp);

	//get rid of noise
	//video.erodeDilate(imgTemp);

	// //find position and angle of robot
	//video.initializeRobot(&robot, imgTemp);

	//initialize threads


	threadsdone = -1;

	pthread_t robot1thread;
	pthread_t robot2thread;
	pthread_t ballthread;

	pthread_cond_init(&threadsstartcond, NULL);
	pthread_cond_init(&threadsdonecond, NULL);

	pthread_mutex_init(&mthreadsstartcond, NULL);
	pthread_mutex_init(&mthreadsdonecond, NULL);

	pthread_create(&robot1thread, NULL, trackRobot, &robot1);
	pthread_create(&robot2thread, NULL, trackRobot, &robot2);
	pthread_create(&ballthread, NULL, trackBall, &ball);



	//yay we did it!
	cout << "initialized" << endl;

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
			cout << "could not read frame from camera" << endl;
			break;
		}


		robot1.setLowHSV(Scalar(robot1LowH, robot1LowS, robot1LowV));
		robot1.setHighHSV(Scalar(robot1HighH, robot1HighS, robot1HighV));

		robot2.setLowHSV(Scalar(robot2LowH, robot2LowS, robot2LowV));
		robot2.setHighHSV(Scalar(robot2HighH, robot2HighS, robot2HighV));

		ball.setHighHSV(Scalar(ballLowH, ballLowS, ballLowV));
		ball.setHighHSV(Scalar(ballHighH, ballHighS, ballHighV));

		

	

		Mat imgHSV, imgBallThresh;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		// cout << "cloning" << endl;

		currentImage = imgHSV.clone();

		// cout << "go threads go" << endl;

		pthread_mutex_lock(&mthreadsstartcond);
		threadsdone = 0;
		robot1.setThreadStart(true);
		robot2.setThreadStart(true);
		ball.setThreadStart(true);
		pthread_cond_broadcast(&threadsstartcond);
		pthread_mutex_unlock(&mthreadsstartcond);

		// cout << "done telling threads to go" << endl;

		//blur(imgOriginal, imgOriginal, Size(3,3));

		

		//convert from the input image to a HSV image
		
 

		//Mat imgRobotThresh, imgBW, imgBallThresh, imgRobot2Thresh;

		//centercircle = imgOriginal.clone();



		//thresh hold the image
		//inRange(imgHSV, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgRobotThresh);
		inRange(imgHSV, Scalar(ballLowH, ballLowS, ballLowV), Scalar(ballHighH, ballHighS, ballHighV), imgBallThresh);
		//inRange(imgHSV, Scalar(robot2LowH, robot2LowS, robot2LowV), Scalar(robot2HighH, robot2HighS, robot2HighV), imgRobot2Thresh);






		//cvtColor(imgHSV, imgBW, COLOR_BGR2GRAY);

		//otsu thresholding
		//threshold(imgBW, imgBW, 0,255, THRESHopencv draw line between points_BINARY | THRESH_OTSU);


		//video.erodeDilate(imgRobotThresh);
		video.erodeDilate(imgBallThresh);
		// video.erodeDilate(imgRobot2Thresh);

		//video.initializeRobot(&robot, imgRobotThresh);		
		//video.initializeBall(&ball, imgBallThresh);
		// video.initializeRobot(&robot2, imgRobot2Thresh);


		//wait for various threads to finish

		// cout << "waiting for thread to finish" << endl;

		pthread_mutex_lock(&mthreadsdonecond);
		while(threadsdone != ALL_THREADS_DONE && !(robot1.getThreadStart() && robot2.getThreadStart() && ball.getThreadStart()))
		{
			pthread_cond_wait(&threadsdonecond, &mthreadsdonecond);
		}

		threadsdone = 0;
		pthread_mutex_unlock(&mthreadsdonecond);


		start = false;
		// cout << "threads finished! finishing and displaying" << endl;

		Point2d robotlocation = video.fieldToImageTransform(robot1.getLocation());
		Point2d end;

		double angle = robot1.getOrientation();

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

		ss1 << "(" << (int)robot1.getLocation().x << "," << (int)robot1.getLocation().y << "," << (int)robot1.getOrientation() << ")";
		putText(imgOriginal, ss1.str(), video.fieldToImageTransform(robot1.getLocation()), 1, FONT_HERSHEY_PLAIN, Scalar(0,0,255));

		//cout << "line drawn" << endl;

		// show the original image with tracking line
		imshow("Raw Image", imgOriginal);
		//show the new image
		//imshow("robotthresh", imgRobotThresh);
		imshow("ballthresh", imgBallThresh);
		// imshow("robot2thresh", imgRobot2Thresh);//*/


		//imshow("BWOTSU", imgBW);

		//cout << "leaging" << endl;

		int keypress = waitKey(1);

		//weird bug in keypress, sometimes returens value that needs to be %256
		if(keypress == 27 || keypress % 256 == 27) //did we press esc?
		{
			break;
		}
		else if(keypress >= 0)
		{
			cout << keypress << endl;
		}

		//cout << "sending ros message" << endl;

		// -------------------------------------
		geometry_msgs::Pose2D robot1pos;
		geometry_msgs::Pose2D ballpos;
		robot1pos.x = robot1.getLocation().x;
		robot1pos.y = robot1.getLocation().y;
		robot1pos.theta = robot1.getOrientation();


		ballpos.x = ball.getLocation().x;
		ballpos.y = ball.getLocation().y;

		pubrobot.publish(robot1pos);
		pubball.publish(ballpos);
		ros::spinOnce();
		loop_rate.sleep();
	}
	//pthread_exit(NULL);
	int exitstatus = 1;
	exitstatus = pthread_cancel(robot1thread);
	exitstatus |= pthread_cancel(robot2thread);
	exitstatus |= pthread_cancel(ballthread);
	pthread_mutex_destroy(&mthreadsstartcond);
	pthread_mutex_destroy(&mthreadsdonecond);
	pthread_cond_destroy(&threadsstartcond);
	pthread_cond_destroy(&threadsdonecond);
	cout << exitstatus << endl;
	return 0;
}



void* trackRobot(void* robotobject)
{
	
	Robot* robot = (Robot*)(robotobject);
	ImageProcessor video = ImageProcessor(scalingfactor, center);
	Mat robotimage;
	int c = 0;
	while(1)
	{
		
		//insert condwait here

		// cout << "wait for thread go command " << robot->getMask() << endl;

		pthread_mutex_lock(&mthreadsstartcond);
		threadsdone |= robot->getMask();
		robot->setThreadStart(false);
		// cout << "threadsdone = " << threadsdone << endl;
		pthread_cond_signal(&threadsdonecond);
		while(threadsdone & robot->getMask() != 0 || !robot->getThreadStart())
		{
			// cout <<"waiting " << robot->getMask() << endl;
			pthread_cond_wait(&threadsstartcond, &mthreadsstartcond);
		}
		pthread_mutex_unlock(&mthreadsstartcond);

		// cout << "go command received " << robot->getMask() << endl;

		//event = 0;
		inRange(currentImage, robot->getLowHSV(), robot->getHighHSV(), robotimage);


		video.erodeDilate(robotimage);

		video.initializeRobot(robot, robotimage);

		// cout << "sending thread done command " << robot->getMask() << endl;

		/*pthread_mutex_lock(&mthreadsdonecond);
		
		pthread_mutex_unlock(&mthreadsdonecond );//*/

		// cout << "thread done command sent " << robot->getMask() << endl;

		// cout << ++c << " " << robot->getMask()<< endl;
		//imshow("robot thread threshold", robotimage);

	}
	cout << "thread shutting down " << robot->getMask() << endl;

	return NULL;
}

void* trackBall(void* ballobject)
{
	VisionObject* ball = (VisionObject*)(ballobject);
	ImageProcessor video = ImageProcessor(scalingfactor, center);
	Mat ballimage;
	int c = 0;
	while(1)
	{
		
		//insert condwait here

		// cout << "wait for thread go command " << ball->getMask() << endl;

		pthread_mutex_lock(&mthreadsstartcond);
		threadsdone |= ball->getMask();
		ball->setThreadStart(false);
		//cout << "threadsdone = " << threadsdone << endl;
		pthread_cond_signal(&threadsdonecond);
		while(threadsdone & ball->getMask() != 0 || !ball->getThreadStart())
		{
			// cout <<"waiting " << ball->getMask() << endl;
			pthread_cond_wait(&threadsstartcond, &mthreadsstartcond);
		}
		pthread_mutex_unlock(&mthreadsstartcond);

		// cout << "go command received " << ball->getMask() << endl;

		//event = 0;
		inRange(currentImage, ball->getLowHSV(), ball->getHighHSV(), ballimage);


		video.erodeDilate(ballimage);

		video.initializeBall(ball, ballimage);

		// cout << "sending thread done command " << ball->getMask() << endl;

		/*pthread_mutex_lock(&mthreadsdonecond);
		
		pthread_mutex_unlock(&mthreadsdonecond );//*/

		// cout << "thread done command sent " << ball->getMask() << endl;

		// cout << ++c << " " << ball->getMask()<< endl;
		//imshow("robot thread threshold", robotimage);

	}
	cout << "thread shutting down " << ball->getMask() << endl;

	return NULL;
}