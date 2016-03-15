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
#define a_KEY 1048673
#define h_KEY 1048680

#define ALL_THREADS_DONE_MASK 0x3
#define BALL_THREAD_MASK 0x4
#define ALLY1_THREAD_MASK 0x1
#define OPPONENT_THREAD_MASK 0x2

#define THRESHHOLD_BOX_WIDTH 250




using namespace cv;
using namespace std;

Mat currentImage;
pthread_cond_t robot1cond;
pthread_cond_t robot1done;
pthread_mutex_t mrobot1cond;
pthread_mutex_t mrobot1done;
bool done1, start1;
int threadstatus;
bool away;

bool showAlly1Thresh = false;
bool showAlly2Thresh = false;
bool showOpp1Thresh = false;
bool showOpp2Thresh = false;
bool showBallThresh = false;

double scalingfactor;
Point2d center;

int ballLowH;
int ballHighH;
int ballLowS;
int ballHighS;
int ballLowV;
int ballHighV;


int robot1LowH;
int robot1HighH;
int robot1LowS;
int robot1HighS;
int robot1LowV;
int robot1HighV;

int robot2LowH;
int robot2HighH;
int robot2LowS;
int robot2HighS;
int robot2LowV;
int robot2HighV;

void* trackRobot(void* robotobject);

void* trackBall(void* ballobject);

void drawRobotLine(Mat img, Robot robot);

Point2d threshholdBox(VisionObject obj, Mat* img);

bool initColors();

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vision_talker");

	ros::NodeHandle n;
		
	
	ros::Publisher pubball = n.advertise<geometry_msgs::Pose2D>("vision_ball_position", 5);

	away = false;

	// cap;
	//cap.open("http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg");

	ImageProcessor video = ImageProcessor("http://192.168.1.79:8080/stream?topic=/image&dummy=param.mjpg");
	// ImageProcessor video = ImageProcessor(0); //use for webcam

	/*if(!cap.isOpened())
	{
		//cout << "cap is closed" << endl;
		return -1;
	}//*/

	namedWindow("BallControl", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	namedWindow("RobotControl", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	namedWindow("Robot2Control", CV_WINDOW_AUTOSIZE);





//	event = 0;
	Mat imgTemp;

	video.read(&imgTemp);

	Mat centercircle = imgTemp.clone();

	//find the center circle
	video.initializeCenter(centercircle);

	scalingfactor = video.getScalingFactor();
	// scalingfactor = 1;

	center = video.getCenter();
	//covert original immage to HSV to find robots
	//cvtColor(imgTemp, imgTemp, COLOR_BGR2HSV);

	Robot robot = Robot();
	robot.setNodeIdent("ally");
	robot.setMask(ALLY1_THREAD_MASK);

	Robot robot2 = Robot();
	robot2.setNodeIdent("opponent");
	robot2.setMask(OPPONENT_THREAD_MASK);


	VisionObject ball = VisionObject();
	ball.setMask(BALL_THREAD_MASK);

	//thresh hold the image
	//inRange(imgTemp, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgTemp);

	//get rid of noise
	//video.erodeDilate(imgTemp);

	// //find position and angle of robot
	//video.initializeRobot(&robot, imgTemp);

	if(!initColors())
		exit(-1);

	ballLowH = 160;
	ballHighH = 178;
	ballLowS = 46;
	ballHighS = 200;
	ballLowV = 132;
	ballHighV = 255;

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

	//initialize threads

	threadstatus = ALL_THREADS_DONE_MASK;


	pthread_t robot1thread;
	pthread_t robotoppthread;

	pthread_cond_init(&robot1cond, NULL);
	pthread_cond_init(&robot1done, NULL);

	pthread_mutex_init(&mrobot1cond, NULL);
	pthread_mutex_init(&mrobot1done, NULL);

	pthread_create(&robot1thread, NULL, trackRobot, &robot);
	pthread_create(&robotoppthread, NULL, trackRobot, &robot2);




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


		robot.setLowHSV(Scalar(robot1LowH, robot1LowS, robot1LowV));
		robot.setHighHSV(Scalar(robot1HighH, robot1HighS, robot1HighV));
		robot2.setLowHSV(Scalar(robot2LowH, robot2LowS, robot2LowV));
		robot2.setHighHSV(Scalar(robot2HighH, robot2HighS, robot2HighV));

		//cout << "cloning" << endl;

		//cout << "go threads go" << endl;

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		currentImage = imgHSV.clone();

		pthread_mutex_lock(&mrobot1cond);
		threadstatus = 0;
		pthread_cond_broadcast(&robot1cond);
		pthread_mutex_unlock(&mrobot1cond);

		//cout << "done telling threads to go" << endl;

		//blur(imgOriginal, imgOriginal, Size(3,3));

		

		//convert from the input image to a HSV image
		


		Mat imgRobotThresh, imgBW, imgBallThresh, imgRobot2Thresh;

		//centercircle = imgOriginal.clone();



		//thresh hold the image


		
		inRange(imgHSV, Scalar(ballLowH, ballLowS, ballLowV), Scalar(ballHighH, ballHighS, ballHighV), imgBallThresh);
		






		//cvtColor(imgHSV, imgBW, COLOR_BGR2GRAY);

		//otsu thresholding
		//threshold(imgBW, imgBW, 0,255, THRESHopencv draw line between points_BINARY | THRESH_OTSU);

		 video.erodeDilate(imgBallThresh);

		// video.initializeRobot(&robot, imgRobotThresh);		
		
		// video.initializeRobot(&robot2, imgRobot2Thresh);


		//wait for various threads to finish

		////cout << "waiting for thread to finish" << endl;

		////cout << "thread finished! finishing and displaying" << endl;


		drawRobotLine(imgOriginal, robot);
		drawRobotLine(imgOriginal, robot2);
		
		if(!video.initializeBall(&ball, imgBallThresh) && away)
			video.invertObjForAway(&ball);

		if(away)
			video.invertObjForAway(&ball);

		circle(imgOriginal, video.fieldToImageTransform(ball.getLocation()), 10, Scalar(0,0,255));
		
		stringstream ss;

		ss << "(" << ball.getLocation().x << "," << ball.getLocation().y << ")";
		putText(imgOriginal, ss.str(), video.fieldToImageTransform(ball.getLocation()), 1, FONT_HERSHEY_PLAIN, Scalar(0,0,255));




		//cout << "line drawn" << endl;

		//threshholdBox(ball, imgOriginal);

		// show the original image with tracking line
		imshow("Raw Image", imgOriginal);
		//show the new image
		if(showAlly1Thresh)
		{
			inRange(imgHSV, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgRobotThresh);
			video.erodeDilate(imgRobotThresh);
			imshow("robotthresh", imgRobotThresh);
		}
		if (showBallThresh)
		{
			imshow("ballthresh", imgBallThresh);
		}
		if(showOpp1Thresh)
		{
			inRange(imgHSV, Scalar(robot2LowH, robot2LowS, robot2LowV), Scalar(robot2HighH, robot2HighS, robot2HighV), imgRobot2Thresh);
			video.erodeDilate(imgRobot2Thresh);
			imshow("robot2thresh", imgRobot2Thresh);//*/
		}


		//imshow("BWOTSU", imgBW);

		//cout << "leaging" << endl;

		int keypress = waitKey(1);

		//weird bug in keypress, sometimes returens value that needs to be %256
		if(keypress == 27 || keypress % 256 == 27) //did we press esc?
		{
			break;
		}
		else if(keypress == 'a' ||keypress % 256 == 'a')
		{
			away = true;
		}
		else if(keypress % 256 == 'h' || keypress == 'h')
		{
			away = false;
		}
		else if(keypress % 256 == '1' || keypress == '1')
		{
			showAlly1Thresh = !showAlly1Thresh;
		}
		else if(keypress % 256 == '2' || keypress == '2')
		{
			showAlly2Thresh = !showAlly2Thresh;
		}
		else if(keypress % 256 == '3' || keypress == '3')
		{
			showOpp1Thresh = !showOpp1Thresh;
		}
		else if(keypress % 256 == '4' || keypress == '4')
		{
			showOpp2Thresh = !showOpp2Thresh;
		}
		else if(keypress % 256 == '5' || keypress == '5')
		{
			showBallThresh = !showBallThresh;
		}
		else if(keypress >= 0)
		{
			cout << keypress << endl;
		}

		//cout << "sending ros message" << endl;

		// -------------------------------------
		
		geometry_msgs::Pose2D ballpos;



		ballpos.x = ball.getLocation().x;
		ballpos.y = ball.getLocation().y;

		
		pubball.publish(ballpos);
		ros::spinOnce();
		loop_rate.sleep();
	}
	//pthread_exit(NULL);
	int exitstatus = 1;
	exitstatus = pthread_cancel(robot1thread);
	exitstatus |= pthread_cancel(robotoppthread);
	pthread_mutex_destroy(&mrobot1cond);
	pthread_mutex_destroy(&mrobot1done);
	pthread_cond_destroy(&robot1cond);
	pthread_cond_destroy(&robot1done);
	cout << exitstatus << endl;
	return 0;
}



void* trackRobot(void* robotobject)
{
	
	Robot* robot = (Robot*)(robotobject);
	ImageProcessor video = ImageProcessor(scalingfactor, center);
	ros::NodeHandle n;
	stringstream ss;
	ss << "vision_" << robot->getNodeIdent() << "_position";
	ros::Publisher pubrobot = n.advertise<geometry_msgs::Pose2D>(ss.str(), 5);
	int c = 0;
	while(1)
	{
		Mat robotimage;
		//insert condwait here

		//cout << "wait for thread go command" << endl;

		pthread_mutex_lock(&mrobot1cond);
		while(threadstatus & robot->getMask() == robot->getMask())
		{
			pthread_cond_wait(&robot1cond, &mrobot1cond);
		}
		threadstatus |= robot->getMask();
		robotimage = currentImage.clone();
		pthread_mutex_unlock(&mrobot1cond);

		
		//cout << "go command received" << endl;
		//Point2d boxloc = threshholdBox(*robot, &robotimage);
		//event = 0;
		inRange(robotimage, robot->getLowHSV(), robot->getHighHSV(), robotimage);


		video.erodeDilate(robotimage);

		if(!(video.initializeRobot(robot, robotimage, Point2d(0,0)/*boxloc*/)) && away)
			video.invertRobotForAway(robot);

		//cout << "sending thread done command" << endl;


		if(away)
			video.invertRobotForAway(robot);


		geometry_msgs::Pose2D robot1pos;
		robot1pos.x = robot->getLocation().x;
		robot1pos.y = robot->getLocation().y;
		robot1pos.theta = robot->getOrientation();
		pubrobot.publish(robot1pos);



		//cout << "thread done command sent" << endl;

		//cout << ++c << endl;
		//imshow("robot thread threshold", robotimage);

	}
	cout << "thread shutting down" << endl;

	return NULL;
}

void* trackBall(void* ballobject)
{
	VisionObject* ball = (VisionObject*)(ballobject);

}


void drawRobotLine(Mat img, Robot robot)
{
	ImageProcessor video = ImageProcessor(scalingfactor, center);
	Point2d robotlocation = video.fieldToImageTransform(robot.getLocation());
	Point2d end;

	double angle = robot.getOrientation();

	double sinx = (cos(angle * M_PI/180));
	double siny = (sin(angle * M_PI/180));

	siny *= -1;

	end.x = robotlocation.x + 50 * sinx;
	end.y = robotlocation.y + 50 * siny;

	////cout << "robot stuffs done" << endl;


	line(img, robotlocation, end, Scalar(0,0,255), 2);

	stringstream ss;

	ss << "(" << robot.getLocation().x << "," << robot.getLocation().y << "," << (int)robot.getOrientation() << ")";
	putText(img, ss.str(), video.fieldToImageTransform(robot.getLocation()), 1, FONT_HERSHEY_PLAIN, Scalar(0,0,255));
}


Point2d threshholdBox(VisionObject obj, Mat* img)
{
	ImageProcessor video = ImageProcessor(scalingfactor, center);
	Point2d loc = video.fieldToImageTransform(obj.getLocation());



	loc.x -= (THRESHHOLD_BOX_WIDTH/2);
	loc.y -= (THRESHHOLD_BOX_WIDTH/2);

	if(loc.x < 0)
	{
		loc.x = 0;
	}
	else if(loc.x +THRESHHOLD_BOX_WIDTH > (*img).size().width)
	{
		loc.x = (*img).size().width - THRESHHOLD_BOX_WIDTH;
	}

	if(loc.y < 0)
	{
		loc.y = 0;
	}
	else if(loc.y + THRESHHOLD_BOX_WIDTH > (*img).size().height)
	{
		loc.y = (*img).size().height - THRESHHOLD_BOX_WIDTH;
	}

	*img = (*img)(Rect(loc.x, loc.y, THRESHHOLD_BOX_WIDTH, THRESHHOLD_BOX_WIDTH));

	return loc;

}



bool initColors()
{
		// put in the values of the thing
	string allycolor, opponentcolor;
	cout << "Ally1 Color :";
	cin >> allycolor;

	cout << "OPP1 Color :";
	cin >> opponentcolor;

	if (allycolor == "r")
	{
		robot1LowH = 0;
		robot1HighH = 179;

		robot1LowS = 81;
		robot1HighS = 105;

		robot1LowV = 226;
		robot1HighV = 255;
	}
	else if (allycolor == "o")
	{
		robot1LowH = 10;
		robot1HighH = 24;

		robot1LowS = 58;
		robot1HighS = 255;

		robot1LowV = 207;
		robot1HighV = 255;
	}
	else if (allycolor == "p")
	{
		robot1LowH = 125;
		robot1HighH = 164;

		robot1LowS = 0;
		robot1HighS = 78;

		robot1LowV = 110;
		robot1HighV = 255;
	}
	else if (allycolor == "bc")
	{
		robot1LowH = 77;
		robot1HighH = 111;

		robot1LowS = 0;
		robot1HighS = 87;

		robot1LowV = 230;
		robot1HighV = 255;
	}
	else if (allycolor == "b")
	{
		robot1LowH = 81;
		robot1HighH = 110;

		robot1LowS = 174;
		robot1HighS = 255;

		robot1LowV = 189;
		robot1HighV = 255;
	}
	else if (allycolor == "g")
	{
		robot1LowH = 46;
		robot1HighH = 94;

		robot1LowS = 0;
		robot1HighS = 179;

		robot1LowV = 186;
		robot1HighV = 255;
	}
	else
		return false;

	// opponent colors
	if (opponentcolor == "r")
	{
		robot2LowH = 0;
		robot2HighH = 179;

		robot2LowS = 81;
		robot2HighS = 105;

		robot2LowV = 226;
		robot2HighV = 255;
	}
	else if (opponentcolor == "o")
	{
		robot2LowH = 10;
		robot2HighH = 24;

		robot2LowS = 58;
		robot2HighS = 255;

		robot2LowV = 207;
		robot2HighV = 255;
	}
	else if (opponentcolor == "p")
	{
		robot2LowH = 125;
		robot2HighH = 164;

		robot2LowS = 0;
		robot2HighS = 78;

		robot2LowV = 110;
		robot2HighV = 255;
	}
	else if (opponentcolor == "bc")
	{
		robot2LowH = 77;
		robot2HighH = 111;

		robot2LowS = 0;
		robot2HighS = 87;

		robot2LowV = 230;
		robot2HighV = 255;
	}
	else if (allycolor == "b")
	{
		robot2LowH = 81;
		robot2HighH = 110;

		robot2LowS = 174;
		robot2HighS = 255;

		robot2LowV = 189;
		robot2HighV = 255;
	}
	else if (opponentcolor == "g")
	{
		robot2LowH = 46;
		robot2HighH = 94;

		robot2LowS = 0;
		robot2HighS = 179;

		robot2LowV = 186;
		robot2HighV = 255;
	}
	else
		return false;

	if(allycolor == opponentcolor)
		return false;
	return true;
}
