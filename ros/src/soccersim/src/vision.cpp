#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

#define FIELD_WIDTH     3.40  // in meters
#define FIELD_HEIGHT    2.38 
#define ROBOT_RADIUS    0.10
#define GUI_NAME        "Soccer Overhead Camera"

// Mouse click parameters, empirically found
// The smaller the number, the more positive the error
// (i.e., it will be above the mouse in +y region)
#define FIELD_WIDTH_PIXELS      540.0 // measured from threshold of goal to goal
#define FIELD_HEIGHT_PIXELS     378.0 // measured from inside of wall to wall
#define CAMERA_WIDTH            640.0
#define CAMERA_HEIGHT           480.0

// These colours need to match the Gazebo materials
Scalar red[]    = {Scalar(0,   128, 128), Scalar(10,  255, 255)};
Scalar yellow[] = {Scalar(20,  128, 128), Scalar(30,  255, 255)};
Scalar green[]  = {Scalar(55,  128, 128), Scalar(65,  255, 255)};
Scalar blue[]   = {Scalar(115, 128, 128), Scalar(125, 255, 255)};
Scalar purple[] = {Scalar(145, 128, 128), Scalar(155, 255, 255)};

// Keep track of who the user wants to move
bool moveHome1;
bool moveHome2;
bool moveAway1;
bool moveAway2;
bool moveBall;

// Handlers for vision position publishers
ros::Publisher home1_pub;
ros::Publisher home2_pub;
ros::Publisher away1_pub;
ros::Publisher away2_pub;
ros::Publisher ball_pub;
// for publishing internally from the vision window
ros::Publisher home1_position_pub;
ros::Publisher home2_position_pub;
ros::Publisher away1_position_pub;
ros::Publisher away2_position_pub;
ros::Publisher ball_position_pub;

void thresholdImage(Mat& imgHSV, Mat& imgGray, Scalar color[])
{
    inRange(imgHSV, color[0], color[1], imgGray);

    erode(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
    dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
}

Point2d getCenterOfMass(Moments moment)
{
    double m10 = moment.m10;
    double m01 = moment.m01;
    double mass = moment.m00;
    double x = m10 / mass;
    double y = m01 / mass;
    return Point2d(x, y);
}

bool compareMomentAreas(Moments moment1, Moments moment2)
{
    double area1 = moment1.m00;
    double area2 = moment2.m00;
    return area1 < area2;
}

Point2d imageToWorldCoordinates(Point2d point_i)
{
    // Define center of the field
    // It may be better to draw a circle in the middle of the mesh
    // that is identical to the actual field, and then use that
    // to create a scale factor to do these calculations off of.
    Point2d centerOfField(CAMERA_WIDTH / 2.0, CAMERA_HEIGHT / 2.0);

    // Move origin to center
    Point2d center_w = (point_i - centerOfField);

    // Scale to real world units (meters)
    // You have to split up the pixel to meter conversion
    // because it is a rect, not a square!
    center_w.x *= (FIELD_WIDTH  / FIELD_WIDTH_PIXELS);
    center_w.y *= (FIELD_HEIGHT / FIELD_HEIGHT_PIXELS);

    // mirror over y-axis
    center_w.y = -center_w.y;

    return center_w;
}

void getRobotPose(Mat& imgHsv, Scalar color[], geometry_msgs::Pose2D& robotPose)
{
    Mat imgGray;
    thresholdImage(imgHsv, imgGray, color);

    vector< vector<Point> > contours;
    vector<Moments> mm;
    vector<Vec4i> hierarchy;
    findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if (hierarchy.size() != 2)
        return;

    for(int i = 0; i < hierarchy.size(); i++)
        mm.push_back(moments((Mat)contours[i]));

    std::sort(mm.begin(), mm.end(), compareMomentAreas);
    Moments mmLarge = mm[mm.size() - 1];
    Moments mmSmall = mm[mm.size() - 2];

    Point2d centerLarge = imageToWorldCoordinates(getCenterOfMass(mmLarge));
    Point2d centerSmall = imageToWorldCoordinates(getCenterOfMass(mmSmall));

    Point2d robotCenter = (centerLarge + centerSmall) * (1.0 / 2);
    Point2d diff = centerSmall - centerLarge;
    double angle = atan2(diff.y, diff.x);

    //convert angle to degrees
    angle = angle *180/M_PI;
    robotPose.x = robotCenter.x;
    robotPose.y = robotCenter.y;
    robotPose.theta = angle;
}

void getBallPose(Mat& imgHsv, Scalar color[], geometry_msgs::Pose2D& ballPose)
{
    Mat imgGray;
    thresholdImage(imgHsv, imgGray, color);

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if (hierarchy.size() != 1)
        return;

    Moments mm = moments((Mat)contours[0]);
    Point2d ballCenter = imageToWorldCoordinates(getCenterOfMass(mm));

    ballPose.x = ballCenter.x;
    ballPose.y = ballCenter.y;
    ballPose.theta = 0;
}

void processImage(Mat frame)
{
    Mat imgHsv;
    cvtColor(frame, imgHsv, COLOR_BGR2HSV);

    geometry_msgs::Pose2D poseHome1;
    geometry_msgs::Pose2D poseHome2;
    geometry_msgs::Pose2D poseAway1;
    geometry_msgs::Pose2D poseAway2;
    geometry_msgs::Pose2D poseBall;

    getRobotPose(imgHsv, blue,   poseHome1);
    getRobotPose(imgHsv, green,  poseHome2);
    getRobotPose(imgHsv, red,    poseAway1);
    getRobotPose(imgHsv, purple, poseAway2);
    getBallPose(imgHsv,  yellow, poseBall);

    home1_pub.publish(poseHome1);
    home2_pub.publish(poseHome2);
    away1_pub.publish(poseAway1);
    away2_pub.publish(poseAway2);
    ball_pub.publish(poseBall);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        processImage(frame);
        imshow(GUI_NAME, frame);        
        waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// void sendBallMessage(int x, int y) {
//     // Expects x, y in pixels

//     // Convert pixels to meters for sending  simulated ball position from mouse clicks
//     float x_meters, y_meters;

//     // shift data by half the pixels so (0, 0) is in center
//     x_meters = x - (CAMERA_WIDTH/2.0);   
//     y_meters = y - (CAMERA_HEIGHT/2.0);

//     // Multiply by aspect-ratio scaling factor
//     x_meters = x_meters * (FIELD_WIDTH/FIELD_WIDTH_PIXELS);
//     y_meters = y_meters * (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);

//     // mirror y over y-axis
//     y_meters = -1*y_meters;

//     // cout << "x: " << x_meters << ", y: " << y_meters << endl;

//     geometry_msgs::Vector3 msg;
//     msg.x = x_meters;
//     msg.y = y_meters;
//     msg.z = 0;
//     ball_position_pub.publish(msg);
// }

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    static bool mouse_left_down = false;
    bool send_msg = false;

    // Ask the user which object to move, via keypress 1-5
    updateWhoToMove();

    // Allow mouse click and mosut drag to
    // move the object (ball/robot) selected
    // by the key press
    if (event == EVENT_LBUTTONDOWN) {
        mouse_left_down = true;

    } else if (event == EVENT_MOUSEMOVE) {
        if (mouse_left_down) send_msg = true;

    } else if (event == EVENT_LBUTTONUP) {
        send_msg = true;
        mouse_left_down = false;
    }

    // If I should send a message...
    if (send_msg) {

        // Figure out the world coordinates to send
        Point2d img(x, y);
        Point2d world = imageToWorldCoordinates(img);

        // Create the message to send
        geometry_msgs::Vector3 msg;
        msg.x = world.x;
        msg.y = world.y;
        msg.z = 0.01; // just above the floor
     
        if      (moveHome1)     home1_position_pub.publish(msg);
        else if (moveHome2)     home2_position_pub.publish(msg);
        else if (moveAway1)     away2_position_pub.publish(msg);
        else if (moveAway2)     away2_position_pub.publish(msg);
        else if (moveBall)      ball_position_pub.publish(msg);
    }
    
}

void updateWhoToMove() {
    // Wait 1ms for a keypress from user
    int key = waitKey(1);
    switch(key % 256) {
        case '1':
            moveHome1 = !moveHome1
            moveHome2 = false;
            moveAway1 = false;
            moveAway2 = false;
            moveBall = false;
            break;
        case '2':
            moveHome1 = false;
            moveHome2 = !moveHome2;
            moveAway1 = false;
            moveAway2 = false;
            moveBall = false;
            break;
        case '3':
            moveHome1 = false;
            moveHome2 = false;
            moveAway1 = !moveAway1;
            moveAway2 = false;
            moveBall = false;
            break;
        case '4':
            moveHome1 = false;
            moveHome2 = false;
            moveAway1 = false;
            moveAway2 = !moveAway2;
            moveBall = false;
            break;
        case '5':
            moveHome1 = false;
            moveHome2 = false;
            moveAway1 = false;
            moveAway2 = false;
            moveBall = !moveBall;
            break;
        default:
            moveHome1 = false;
            moveHome2 = false;
            moveAway1 = false;
            moveAway2 = false;
            moveBall = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_sim");
    ros::NodeHandle nh;

    // Create OpenCV Window and add a mouse callback for clicking
    namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
    setMouseCallback(GUI_NAME, mouseCallback, NULL);

    // Create internal ball/bot position publishers
    home1_position_pub = nh.advertise<geometry_msgs::Vector3>("/home1/forced_position", 1);
    home2_position_pub = nh.advertise<geometry_msgs::Vector3>("/home2/forced_position", 1);
    away1_position_pub = nh.advertise<geometry_msgs::Vector3>("/away1/forced_position", 1);
    away2_position_pub = nh.advertise<geometry_msgs::Vector3>("/away2/forced_position", 1);
    ball_position_pub = nh.advertise<geometry_msgs::Vector3>("/ball/forced_position", 1);

    // Subscribe to camera
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/camera1/image_raw", 1, imageCallback);

    // Create Vision Publishers
    home1_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/home1", 5);
    home2_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/home2", 5);
    away1_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/away1", 5);
    away2_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/away2", 5);
    ball_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);
    ros::spin();
    return 0;
}