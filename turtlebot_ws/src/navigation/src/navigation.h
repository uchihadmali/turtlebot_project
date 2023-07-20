/*
 File name: RosThread.h
 Author: Berkay Gümüş
 E-mail: berkay.gumus@boun.edu.tr
 Date created: 17.12.2019
 Date last modified: 24.08.2020
 */

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <cfloat>
#include <iostream>
#include <string>
#include <vector>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>
//file storage
//#include <opencv2/core.hpp>
//#include "opencv2/opencv.hpp"
//#include <opencv2/core/persistence.hpp>


#define PI 3.14159265359
#define LOOP_RATE 10

struct Point2D{
    double x;
    double y;
};

typedef struct {
    double x;
    double y;
    double rho;
    double h;

}  obstacle;

class Navigation{
  public:

   Navigation(int argc,char** argv,int);
    ~Navigation();
    void work();
    void work2();

  private:
    int argc;
    char** argv;


    int N; // Number of obstacles;

    Point2D robpos;
    Point2D goal;
    Point2D start_goal;
    double robrad;
    std::vector<Point2D> obspos;
    std::vector<double> obsrad;

    int plan_count=0;


    std::vector<double> distance;
    float pose_x;//position and heading
    float pose_y=0;
    float pose_theta = 0;
    float prev_pose_x = 0;//previous position
    float prev_pose_y = 0;
    bool initialize_prev = 0;
    int id;//robot ID
    double gama_sum=0;//gamma
    double beta_product = 1;//beta
    double gama_b_dot[2];
    double beta_b_dot[2];
    double normalizer=1;
    double fi_b_dot[2];
    double alpha;
    int goal_reached=0;


    ////////////////////////////////parameters
    double threshold;
    double k;
    double f = LOOP_RATE;
    double step_size = 1/f;
    bool quite_mode;//not to print values,
    bool print_fi_b_dot;//to print derivative of fi with respect to b
    bool print_distances; //to print distances among robots
    bool print_eta;
    bool print_fi_eta_dot;
    double obs_pos_x=-1;
    double obs_pos_y=-1;
    ////////////////////////////////


    ros::NodeHandle getParamHandle;
    ros::NodeHandle poseOdomSub;
    ros::NodeHandle poseSub;
    ros::NodeHandle modelStateSub;
    ros::NodeHandle ObsPosSub;
    ros::NodeHandle poseOverheadCamSub;
    ros::NodeHandle velPub;
    ros::NodeHandle posPub;
    ros::NodeHandle trSub;

    ros::Publisher vel_pub;
    ros::Publisher pos_pub;
    double orientation2theta(double, double, double, double);//calculates angles using quaternion

    void gamaFinder();
    void gamaBDotFinder();
    void betaFinder();
    void betaBDotFinder();
    void gamaEtaDotFinder();
    void fiBDotFinder();
    void fiEtaDotFinder();
    void etaFinder();
    void Comparator();
    void  apfNavigation();
    void odomCallback(const nav_msgs::Odometry::ConstPtr&);
    void obsCallback(const geometry_msgs::Pose& msg);

//////////////////////turtlesim completed/////////////////////
    /*void poseOdomCallback0(const turtlesim::Pose::ConstPtr&);
    void poseOdomCallback1(const turtlesim::Pose::ConstPtr&);
    void poseOdomCallback2(const turtlesim::Pose::ConstPtr&);
    void poseOdomCallback3(const turtlesim::Pose::ConstPtr&);
    void poseOdomCallback4(const turtlesim::Pose::ConstPtr&);
    void poseOdomCallback5(const turtlesim::Pose::ConstPtr&);
    void poseOdomCallback6(const turtlesim::Pose::ConstPtr&);*/

/////////////////////completed part /////////////////////////////

    void gazeboOdomCallback(const nav_msgs::Odometry::ConstPtr&);
    

///////////////////////////////////



    void distanceCalculator();

    int flag=1;
    double epsilon=0.2;
    int completed = 0;
    int completed_stop = 0;
    int time_counter = 0;
    double path_distance = 0;


    void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg);
    //void turtleService();

    void setVelocity();
    void setPosition();
    void gradVelocity();
    double pose_msg_x;
    double pose_msg_y;
    double linear_vel;
    double angular_vel;
    geometry_msgs::Twist vel_msg;
    int counto=0;



};
