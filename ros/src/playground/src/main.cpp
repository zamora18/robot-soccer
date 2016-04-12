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
#include "playground/GameState.h"

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

int allynumplayers;
int oppnumplayers;

bool showAlly1Thresh = false;
bool showAlly2Thresh = false;
bool showOpp1Thresh = false;
bool showOpp2Thresh = false;
bool showBallThresh = false;
bool playgame = false;
bool twovtwo;

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

int ally2LowH;
int ally2HighH;
int ally2LowS;
int ally2HighS;
int ally2LowV;
int ally2HighV;

int opp1LowH;
int opp1HighH;
int opp1LowS;
int opp1HighS;
int opp1LowV;
int opp1HighV;

int opp2LowH;
int opp2HighH;
int opp2LowS;
int opp2HighS;
int opp2LowV;
int opp2HighV;

void* trackRobot(void* robotobject);

void* trackBall(void* ballobject);

void drawRobotLine(Mat img, Robot robot);

Point2d threshholdBox(VisionObject obj, Mat* img);

bool initColors();

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vision_talker");

	ros::NodeHandle n;
		
	cout << "How many Allies? : ";
	cin >> allynumplayers;
	
	cout << "How many Opponents? : ";
	cin >> oppnumplayers;

	playgame = false;
	if (allynumplayers == 1)
		twovtwo = false;
	else
		twovtwo = true;

	if(!initColors())
		exit(-1);
	
	ros::Publisher pubstart = n.advertise<playground::GameState>("game_state", 1, true);

	playground::GameState gamestatemsginit;

	gamestatemsginit.play = playgame;
	gamestatemsginit.two_v_two = twovtwo;

	pubstart.publish(gamestatemsginit);

	ros::Publisher pubball = n.advertise<geometry_msgs::Pose2D>("vision_ball_position", 5);

	away = false;

	// cap;
	//cap.open("http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg");

	ImageProcessor video = ImageProcessor("http://192.168.1.78:8080/stream?topic=/image&dummy=param.mjpg");
	// ImageProcessor video = ImageProcessor(0); //use for webcam

	/*if(!cap.isOpened())
	{
		//cout << "cap is closed" << endl;
		return -1;
	}//*/

	namedWindow("BallControl", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	namedWindow("Ally1Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	if (allynumplayers > 1)
		namedWindow("Ally2Control", CV_WINDOW_AUTOSIZE);
	namedWindow("Opp1Control", CV_WINDOW_AUTOSIZE);
	if (oppnumplayers > 1)
		namedWindow("Opp2Control", CV_WINDOW_AUTOSIZE);





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
	ally1.setNodeIdent("ally1");
	ally1.setMask(ALLY1_THREAD_MASK);

	Robot ally2 = Robot();
	ally2.setNodeIdent("ally2");
	ally2.setMask(ALLY2_THREAD_MASK);

	Robot opp1 = Robot();
	opp1.setNodeIdent("opponent1");
	opp1.setMask(OPPONENT1_THREAD_MASK);

	Robot opp2 = Robot();
	opp2.setNodeIdent("opponent2");
	opp2.setMask(OPPONENT2_THREAD_MASK);

	ros::Publisher temppub;
	if (twovtwo && oppnumplayers < 2)
	{
		// cout << "sending msg for 2v1" << endl;
		temppub = n.advertise<geometry_msgs::Pose2D>("vision_opponent2_position", 1, true);

		geometry_msgs::Pose2D robot1pos;
		robot1pos.x = 10;
		robot1pos.y = 10;
		robot1pos.theta = 0;
		temppub.publish(robot1pos);

	}	


	VisionObject ball = VisionObject();
	ball.setMask(BALL_THREAD_MASK);

	//thresh hold the image
	//inRange(imgTemp, Scalar(robot1LowH, robot1LowS, robot1LowV), Scalar(robot1HighH, robot1HighS, robot1HighV), imgTemp);

	//get rid of noise
	//video.erodeDilate(imgTemp);

	// //find position and angle of robot
	//video.initializeRobot(&robot, imgTemp);

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

	cvCreateTrackbar("LowH", "Ally2Control", &ally2LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Ally2Control", &ally2HighH, 179);

	cvCreateTrackbar("LowS", "Ally2Control", &ally2LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Ally2Control", &ally2HighS, 255);

	cvCreateTrackbar("LowV", "Ally2Control", &ally2LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Ally2Control", &ally2HighV, 255);

	cvCreateTrackbar("LowH", "Opp2Control", &opp2LowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Opp2Control", &opp2HighH, 179);

	cvCreateTrackbar("LowS", "Opp2Control", &opp2LowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Opp2Control", &opp2HighS, 255);

	cvCreateTrackbar("LowV", "Opp2Control", &opp2LowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Opp2Control", &opp2HighV, 255);

	//initialize threads

	threadstatus = ALL_THREADS_DONE_MASK;


	pthread_t ally1thread;
	pthread_t ally2thread;
	pthread_t opp1thread;
	pthread_t opp2thread;


	pthread_cond_init(&robot1cond, NULL);
	pthread_cond_init(&robot1done, NULL);

	pthread_mutex_init(&mrobot1cond, NULL);
	pthread_mutex_init(&mrobot1done, NULL);

	pthread_create(&ally1thread, NULL, trackRobot, &ally1);
	if(allynumplayers > 1)
		pthread_create(&ally2thread, NULL, trackRobot, &ally2);

	pthread_create(&opp1thread, NULL, trackRobot, &opp1);

	if(oppnumplayers > 1)
		pthread_create(&opp2thread, NULL, trackRobot, &opp2);




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

		ally2.setLowHSV(Scalar(ally2LowH, ally2LowS, ally2LowV));
		ally2.setHighHSV(Scalar(ally2HighH, ally2HighS, ally2HighV));

		opp1.setLowHSV(Scalar(opp1LowH, opp1LowS, opp1LowV));
		opp1.setHighHSV(Scalar(opp1HighH, opp1HighS, opp1HighV));

		opp2.setLowHSV(Scalar(opp2LowH, opp2LowS, opp2LowV));
		opp2.setHighHSV(Scalar(opp2HighH, opp2HighS, opp2HighV));

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
		drawRobotLine(imgOriginal, ally2);
		drawRobotLine(imgOriginal, opp2);
		
		if(!video.initializeBall(&ball, imgBallThresh) && away)
			video.invertObjForAway(&ball);

		if(away)
			video.invertObjForAway(&ball);

		circle(imgOriginal, video.fieldToImageTransform(ball.getLocation()), 10, Scalar(0,0,255));
		
		stringstream ss;

		ss << "(" << ball.getLocation().x << "," << ball.getLocation().y << ")";
		putText(imgOriginal, ss.str(), video.fieldToImageTransform(ball.getLocation()), 1, FONT_HERSHEY_PLAIN, Scalar(0,0,255));

		if (!playgame)
			circle(imgOriginal, Point2d(5,5), 10, Scalar(0,0,255));
		else
			circle(imgOriginal, Point2d(5,5), 10, Scalar(0,255,0));


		//cout << "line drawn" << endl;

		//threshholdBox(ball, imgOriginal);

		// show the original image with tracking line
		imshow("Raw Image", imgOriginal);
		//show the new image

		Mat imgAlly2Thresh, imgOpp2Thresh;
		if(showAlly1Thresh)
		{
			inRange(imgHSV, Scalar(ally1LowH, ally1LowS, ally1LowV), Scalar(ally1HighH, ally1HighS, ally1HighV), imgAlly1Thresh);
			video.erodeDilate(imgAlly1Thresh);
			imshow("robotthresh", imgAlly1Thresh);
		}
		if(showAlly2Thresh)
		{
			inRange(imgHSV, Scalar(ally2LowH, ally2LowS, ally2LowV), Scalar(ally2HighH, ally2HighS, ally2HighV), imgAlly2Thresh);
			video.erodeDilate(imgAlly2Thresh);
			imshow("robot2thresh", imgAlly2Thresh);
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
		if(showOpp2Thresh)
		{
			inRange(imgHSV, Scalar(opp2LowH, opp2LowS, opp2LowV), Scalar(opp2HighH, opp2HighS, opp2HighV), imgOpp2Thresh);
			video.erodeDilate(imgOpp2Thresh);
			imshow("opp2thresh", imgOpp2Thresh);//*/
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
		else if(keypress % 256 == 's' || keypress == 's')
		{
			showAlly1Thresh = false;
			showAlly2Thresh = false;
			showOpp1Thresh = false;
			showOpp2Thresh = false;
			showBallThresh = false;

		}
		else if(keypress % 256 == ' ' || keypress == ' ')
		{
			playgame = !playgame;

			// send data
			playground::GameState gamestatemsg;

			gamestatemsg.play = playgame;
			gamestatemsg.two_v_two = twovtwo;
			gamestatemsg.usgoal = false;
			gamestatemsg.themgoal = false;

			pubstart.publish(gamestatemsg);

		}
		else if (keypress % 256 == 'u' || keypress == 'u') // we scored
		{
			playground::GameState gamestatemsg;

			gamestatemsg.play = playgame;
			gamestatemsg.two_v_two = twovtwo;
			gamestatemsg.usgoal = true;
			gamestatemsg.themgoal = false;

			pubstart.publish(gamestatemsg);
		}
		else if (keypress % 256 == 't' || keypress == 't')
		{
			playground::GameState gamestatemsg;
			
			gamestatemsg.play = playgame;
			gamestatemsg.two_v_two = twovtwo;
			gamestatemsg.usgoal = false;
			gamestatemsg.themgoal = true;

			pubstart.publish(gamestatemsg);

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
	if (allynumplayers > 1)
		exitstatus |= pthread_cancel(ally2thread);
	exitstatus |= pthread_cancel(opp1thread);
	if (oppnumplayers > 1)
		exitstatus |= pthread_cancel(opp2thread);
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
	// cout << "there should be something = " << robot->getNodeIdent() << endl;
	//ss << "vision_" << robot->getNodeIdent() << "_position";
	ros::Publisher pubrobot = n.advertise<geometry_msgs::Pose2D>("vision_" + robot->getNodeIdent() + "_position", 5);
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

    string robotident = robot.getNodeIdent();

    Scalar color;

    if (robotident == "ally1")
    {
        //blue for nugget
        color = Scalar(255,0,0);
    }
    else if (robotident == "ally2")
    {
        //yellow for fry
        color = Scalar(255,255,0);
    }
    else
    {
        //red for enemy
        color = Scalar(0,0,255);
    }

	line(img, robotlocation, end, color, 2);

	stringstream ss;

	ss << "(" << robot.getLocation().x << "," << robot.getLocation().y << "," << (int)robot.getOrientation() << ")";
	putText(img, ss.str(), video.fieldToImageTransform(robot.getLocation()), 1, FONT_HERSHEY_PLAIN, color);
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
	string ally1color, ally2color, opp1color, opp2color;
	VisionObject vo = VisionObject();
	cout << "Ally1 Color :";
	cin >> ally1color;

	if (allynumplayers > 1)
	{
		cout << "Ally2 Color :";
		cin >> ally2color;
	}

	cout << "OPP1 Color :";
	cin >> opp1color;

	if (oppnumplayers > 1)
	{
		cout << "OPP2 Color :";
		cin >> opp2color;
	}
	
	if (!vo.initColor(ally1color, &ally1LowH, &ally1LowS,&ally1LowV,&ally1HighH,&ally1HighS,&ally1HighV))
		return false;
	if (allynumplayers > 1)
		if (!vo.initColor(ally2color, &ally2LowH, &ally2LowS,&ally2LowV,&ally2HighH,&ally2HighS,&ally2HighV))
			return false;
	if (!vo.initColor(opp1color, &opp1LowH, &opp1LowS,&opp1LowV,&opp1HighH,&opp1HighS,&opp1HighV))
		return false;
	if (oppnumplayers > 1)
		if (!vo.initColor(opp2color, &opp2LowH, &opp2LowS,&opp2LowV,&opp2HighH,&opp2HighS,&opp2HighV))
			return false;

	return true;
}
