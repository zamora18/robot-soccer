#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

using namespace cv;
using namespace std;

vector<vector<Point> > getContours(Mat img);

int main(int argc, char *argv[])
{
	VideoCapture cap;

	vector<vector<Point> > contours;

	cap.open("http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg");

	/*if(!cap.isOpened())
	{
		cout << "cap is opened" << endl;
		return -1;
	}*/

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 171;//43;
	int iHighH = 179;//63;

	int iLowS = 136;//77;
	int iHighS = 255;//255;

	int iLowV = 208;//175;
	int iHighV = 255;//255;

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	Mat imgTemp;
	cap.read(imgTemp);

	Mat imgLines = Mat::zeros(imgTemp.size(), CV_8UC3);


	//used to store the position of the object from the previous frame
	int iPrevX = -1;
	int iPrevY = -1;

	Point center;

	center.x = imgTemp.size().width/2;
	center.y = imgTemp.size().height/2;

	cout << "initialized" << endl;

	while(true)
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

		Mat imgHSV;

		//convert from the input image to a HSV image
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);


		Mat imgThresholded, imgBW;

		//cvtColor(imgOriginal, imgThresholded, COLOR_RGB2GRAY);

		blur(imgThresholded, imgThresholded, Size(3,3));

		//thresh hold the image
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

		//otsu thresholding
		//threshold(imgThresholded, imgThresholded, 0, 255, THRESH_BINARY | THRESH_OTSU);



		//averages pixels in ovals to get rid of background noise
		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)) );

		//morphological closing (fill small holes in the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );



//		//Find the moments in the thresholded image
//		Moments oMoments = moments(imgThresholded);

//		//
//		double dM01 = oMoments.m01;
//		double dM10 = oMoments.m10;
//		double dArea = oMoments.m00;

//		//if the area is <=10000, its just noise
//		if(dArea > 10000)
//		{
//			/* calculate pos of image
//			 * the x coordinate is m10/m00
//			 * y is m01/m00
//			 */
//			int posX = (dM10/dArea) - center.x;
//			int posY = center.y - (dM01/dArea);

//			//check to make sure they are valid points
//			/*if(iPrevX >= 0 && iPrevY >= 0 && posX >= 0 && posY >= 0)
//			{
//				//draw a line from the previous point to new point
//				line(imgLines,Point(posX, posY), Point(iPrevX, iPrevY), Scalar(0,0,255), 3);
//			}*/

//			//update the previous point to the new point
//			iPrevX = posX;
//			iPrevY = posY;
//		}
//		//int distanceFromCenter = (int)sqrt(pow(center.x - iPrevX, 2) + pow(center.y - iPrevY, 2));
//		//cout << "distance from center = " << distanceFromCenter << endl;

//		cout << "(" << iPrevX << "," << iPrevY << ")" << endl;


		contours = getContours(imgThresholded);


		for(int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(255,255,0);
			drawContours(imgOriginal, contours, i, color, 3);
		}







		// show the original image with tracking line
		imshow("Raw Image", imgOriginal);
		//show the new image
		imshow("BW", imgThresholded);



		if(waitKey(1) >= 0)
		{
			break;
		}

	}
	return 0;
}


vector<vector<Point> > getContours(Mat img)
{
	Mat contourOutput;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;


	//detect edges
	Canny(img, contourOutput, 0,255, 3);

	//find the contors based on contours
	findContours(contourOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	//double area = contourArea(contourOutput);

	//cout << "contourarea = " << area << endl;

	return contours;
}









