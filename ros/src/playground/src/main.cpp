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

#define ALL_THREADS_DONE_MASK 0xF
#define BALL_THREAD_MASK 0x10
#define ALLY1_THREAD_MASK 0x1
#define ALLY2_THREAD_MASK 0x2
#define OPPONENT1_THREAD_MASK 0x4
#define OPPONENT2_THREAD_MASK 0x8


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


int ally1LowH;
int ally1HighH;
int ally1LowS;
int ally1HighS;
int ally1LowV;
int ally1HighV;

int opp1LowH;
int opp1HighH;
int opp1LowS;
int opp1HighS;
int opp1LowV;
int opp1HighV;

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
	namedWindow("Ally1Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	namedWindow("Opp1Control", CV_WINDOW_AUTOSIZE);





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

	Robot ally1 = Robot();
	ally1.setNodeIdent("ally");
	ally1.setMask(ALLY1_THREAD_MASK);

	Robot opp1 = Robot();
	opp1.setNodeIdent("opponent");
	opp1.setMask(OPPONENT1_THREAD_MASK);


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
	cvCreateTrackbar("LowH", "Ally1Control", &ally1LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Ally1Control", &ally1HighH, 179);

	cvCreateTrackbar("LowS", "Ally1Control", &ally1LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Ally1Control", &ally1HighS, 255);

	cvCreateTrackbar("LowV", "Ally1Control", &ally1LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Ally1Control", &ally1HighV, 255);

	cvCreateTrackbar("LowH", "Opp1Control", &opp1LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Opp1Control", &opp1HighH, 179);

	cvCreateTrackbar("LowS", "Opp1Control", &opp1LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Opp1Control", &opp1HighS, 255);

	cvCreateTrackbar("LowV", "Opp1Control", &opp1LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Opp1Control", &opp1HighV, 255);

	//initialize threads

	threadstatus = ALL_THREADS_DONE_MASK;


	pthread_t ally1thread;
	pthread_t robotoppthread;

	pthread_cond_init(&robot1cond, NULL);
	pthread_cond_init(&robot1done, NULL);

	pthread_mutex_init(&mrobot1cond, NULL);
	pthread_mutex_init(&mrobot1done, NULL);

	pthread_create(&ally1thread, NULL, trackRobot, &ally1);
	pthread_create(&robotoppthread, NULL, trackRobot, &opp1);




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


		ally1.setLowHSV(Scalar(ally1LowH, ally1LowS, ally1LowV));
		ally1.setHighHSV(Scalar(ally1HighH, ally1HighS, ally1HighV));
		opp1.setLowHSV(Scalar(opp1LowH, opp1LowS, opp1LowV));
		opp1.setHighHSV(Scalar(opp1HighH, opp1HighS, opp1HighV));

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
		


		Mat imgAlly1Thresh, imgBW, imgBallThresh, imgOpp1Thresh;

		//centercircle = imgOriginal.clone();



		//thresh hold the image


		
		inRange(imgHSV, Scalar(ballLowH, ballLowS, ballLowV), Scalar(ballHighH, ballHighS, ballHighV), imgBallThresh);
		






		//cvtColor(imgHSV, imgBW, COLOR_BGR2GRAY);

		//otsu thresholding
		//threshold(imgBW, imgBW, 0,255, THRESHopencv draw line between points_BINARY | THRESH_OTSU);

		 video.erodeDilate(imgBallThresh);

		// video.initializeRobot(&robot, imgAlly1Thresh);		
		
		// video.initializeRobot(&robot2, imgRobot2Thresh);


		//wait for various threads to finish

		////cout << "waiting for thread to finish" << endl;

		////cout << "thread finished! finishing and displaying" << endl;


		drawRobotLine(imgOriginal, ally1);
		drawRobotLine(imgOriginal, opp1);
		
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
			inRange(imgHSV, Scalar(ally1LowH, ally1LowS, ally1LowV), Scalar(ally1HighH, ally1HighS, ally1HighV), imgAlly1Thresh);
			video.erodeDilate(imgAlly1Thresh);
			imshow("robotthresh", imgAlly1Thresh);
		}
		if (showBallThresh)
		{
			imshow("ballthresh", imgBallThresh);
		}
		if(showOpp1Thresh)
		{
			inRange(imgHSV, Scalar(opp1LowH, opp1LowS, opp1LowV), Scalar(opp1HighH, opp1HighS, opp1HighV), imgOpp1Thresh);
			video.erodeDilate(imgOpp1Thresh);
			imshow("opp1thresh", imgOpp1Thresh);//*/
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
	exitstatus = pthread_cancel(ally1thread);
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
		ally1LowH = 0;
		ally1HighH = 179;

		ally1LowS = 81;
		ally1HighS = 105;

		ally1LowV = 226;
		ally1HighV = 255;
	}
	else if (allycolor == "o")
	{
		ally1LowH = 10;
		ally1HighH = 24;

		ally1LowS = 58;
		ally1HighS = 255;

		ally1LowV = 207;
		ally1HighV = 255;
	}
	else if (allycolor == "p")
	{
		ally1LowH = 125;
		ally1HighH = 164;

		ally1LowS = 0;
		ally1HighS = 78;

		ally1LowV = 200;
		ally1HighV = 255;
	}
	else if (allycolor == "bc")
	{
		ally1LowH = 77;
		ally1HighH = 111;

		ally1LowS = 0;
		ally1HighS = 87;

		ally1LowV = 230;
		ally1HighV = 255;
	}
	else if (allycolor == "b")
	{
		ally1LowH = 81;
		ally1HighH = 110;

		ally1LowS = 174;
		ally1HighS = 255;

		ally1LowV = 189;
		ally1HighV = 255;
	}
	else if (allycolor == "g")
	{
		ally1LowH = 46;
		ally1HighH = 94;

		ally1LowS = 0;
		ally1HighS = 179;

		ally1LowV = 186;
		ally1HighV = 255;
	}
	else
		return false;

	// opponent colors
	if (opponentcolor == "r")
	{
		opp1LowH = 0;
		opp1HighH = 179;

		opp1LowS = 81;
		opp1HighS = 105;

		opp1LowV = 226;
		opp1HighV = 255;
	}
	else if (opponentcolor == "o")
	{
		opp1LowH = 10;
		opp1HighH = 24;

		opp1LowS = 58;
		opp1HighS = 255;

		opp1LowV = 207;
		opp1HighV = 255;
	}
	else if (opponentcolor == "p")
	{
		opp1LowH = 125;
		opp1HighH = 164;

		opp1LowS = 0;
		opp1HighS = 78;

		opp1LowV = 200;
		opp1HighV = 255;
	}
	else if (opponentcolor == "bc")
	{
		opp1LowH = 77;
		opp1HighH = 111;

		opp1LowS = 0;
		opp1HighS = 87;

		opp1LowV = 230;
		opp1HighV = 255;
	}
	else if (allycolor == "b")
	{
		opp1LowH = 81;
		opp1HighH = 110;

		opp1LowS = 174;
		opp1HighS = 255;

		opp1LowV = 189;
		opp1HighV = 255;
	}
	else if (opponentcolor == "g")
	{
		opp1LowH = 46;
		opp1HighH = 94;

		opp1LowS = 0;
		opp1HighS = 179;

		opp1LowV = 186;
		opp1HighV = 255;
	}
	else
		return false;

	if(allycolor == opponentcolor)
		return false;
	return true;
}
