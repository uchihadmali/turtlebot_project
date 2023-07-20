#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/core/core.hpp>
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Point.h"
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <vector>
#include <sys/time.h>

#define PI 3.14159265
#define MIN_COMPONENT_AREA_THRESHOLD 600  // can be tweaked for better performance
const double kinectFovX = 28.75;
int low_H, high_H, low_S, high_S, low_V, high_V;

class TargetFinder
{
    ros::NodeHandle nh_;
    ros::Publisher target_position_pub;
    image_transport::ImageTransport it_;
    image_transport::Subscriber rgb_image_sub_;
    image_transport::Subscriber depth_image_sub_;
    image_transport::Publisher detection_pub;
    int posX, posY;
    bool target_found = false;

  public:

  TargetFinder()
    : it_(nh_)
  {
      // Subscribe to input video feed and publish detection output

      rgb_image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
         &TargetFinder::imageCb, this);
      depth_image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
         &TargetFinder::depthCb, this);
      detection_pub = it_.advertise("/detection_result", 1);
  }

  ~TargetFinder()
  { }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

      cv_bridge::CvImagePtr cv_ptr;

      try{
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         }
      catch (cv_bridge::Exception& e){
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
         }

      cv::Mat color_image = cv_ptr->image.clone();
      cv::cvtColor(color_image, color_image, cv::COLOR_BGR2HSV);

      cv::Mat thresholded_image;
      cv::inRange(color_image, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), thresholded_image);

      //morphological opening (removes small objects from the foreground)
      cv::erode(thresholded_image, thresholded_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::dilate(thresholded_image, thresholded_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

      //morphological closing (removes small holes from the foreground)
      cv::dilate(thresholded_image, thresholded_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::erode(thresholded_image, thresholded_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

      cv::Mat labels, stats, centroids;
      int n_labels = cv::connectedComponentsWithStats(thresholded_image, labels, stats, centroids, 8);

      // finding max sized component
      int max_size = 0; int max_num = 0;
      for(int i=1; i<n_labels; i++){
          int im_area = stats.at<int>(i,4);
          if(im_area > max_size){
              max_size = im_area;
              max_num = i;
          }
      }

      if (max_size > MIN_COMPONENT_AREA_THRESHOLD  && !target_found){
          //std::cout << "posX: " << centroids.at<double>(max_num,0) << std::endl;
          //std::cout << "posY:" << centroids.at<double>(max_num,1) << std::endl;

          posX = centroids.at<double>(max_num,0);
          posY = centroids.at<double>(max_num,1);

          target_found = true;
      }


      // publishing thresholded image for visualization purposes
      sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", thresholded_image).toImageMsg();
      detection_pub.publish(image_msg);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_ptr;
      try{
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
         }
      catch (cv_bridge::Exception& e){
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
         }

      if(target_found)
      {
          target_found = false;
          int target_distance = cv_ptr->image.at<short int>(cv::Point(posX, posY));
          if(target_distance != 0)
          {
              double targetPositionY = target_distance * sin((kinectFovX* ((posX - 320)/ 320.0))* (PI/180));
              double targetPositionX = sqrt(pow(target_distance,2) - pow(targetPositionY,2));
              ROS_INFO("Depth: %d , targetX: %f, targetY: %f",target_distance, targetPositionX, targetPositionY);

              // use targetPositionX and targetPositionY to give following targets to the robot.
          }
      }
  }

};

int main(int argc, char** argv)
{
        ros::init(argc, argv, "target_finder");
        ros::NodeHandle parameterServer("~");

        //get params from server
        parameterServer.getParam("low_H",low_H);
        parameterServer.getParam("high_H",high_H);
        parameterServer.getParam("low_S",low_S);
        parameterServer.getParam("high_S",high_S);
        parameterServer.getParam("low_V", low_V);
        parameterServer.getParam("high_V", high_V);

        //      color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
        //                    'white': [[180, 18, 255], [0, 0, 231]],
        //                    'red1': [[180, 255, 255], [159, 50, 70]],
        //                    'red2': [[9, 255, 255], [0, 50, 70]],
        //                    'green': [[89, 255, 255], [36, 50, 70]],
        //                    'blue': [[128, 255, 255], [90, 50, 70]],
        //                    'yellow': [[35, 255, 255], [25, 50, 70]],
        //                    'purple': [[158, 255, 255], [129, 50, 70]],
        //                    'orange': [[24, 255, 255], [10, 50, 70]],
        //                    'gray': [[180, 18, 230], [0, 0, 40]]}
        // format : [high H,S,V], [LOW H,S,V]. Note that these are rough estimations, you should fine tune these values.

        TargetFinder target_finder;
        std::cout << "Target finder module started!" << std::endl;

	ros::spin();

	return 0;
}





