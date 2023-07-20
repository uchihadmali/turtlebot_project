#include <iostream>
#include <stdio.h>
#include <fstream>
#include <tf/tf.h>
 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>



#include <math.h>


using namespace std;
using namespace cv;


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "detect_color");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    while(ros::ok()){

        ros::Time begin = ros::Time::now();
    	
        VideoCapture cap(0); //capture the video from web cam
     
   
        if ( !cap.isOpened() )  // if not success, exit program
        {
            cout << "Cannot open the web cam" << endl;
            return -1;
        }

        namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"

        int iLowH = 0;  int iHighH = 179;
        int iLowS = 0; int iHighS = 255;
        int iLowV = 0; int iHighV = 255;

        //Create trackbars in "Control" window
        createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        createTrackbar("HighH", "Control", &iHighH, 179);

        createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        createTrackbar("HighS", "Control", &iHighS, 255);

        createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        createTrackbar("HighV", "Control", &iHighV, 255);

        while (true)
        {
            Mat imgOriginal;

            bool bSuccess = cap.read(imgOriginal); // read a new frame from video

            if (!bSuccess) //if not success, break loop
            {
                cout << "Cannot read a frame from video stream" << endl;
                break;
            }

            Mat imgHSV, imgThresholded;
            cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
            inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            imshow("Thresholded Image", imgThresholded); //show the thresholded image
            imshow("Original", imgOriginal); //show the original image

            if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            {
                cout << "esc key is pressed by user" << endl;
                break;
            }
        }

        loop_rate.sleep();
    
        ros::spinOnce();
    
       }

  return 0;
}
