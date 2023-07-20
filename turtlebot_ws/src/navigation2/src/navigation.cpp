#include "navigation.h"
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>

using namespace std;
//using namespace cv;
bool target_found = false;

Navigation ::Navigation (int argc,char** argv,int ID){

    this->argc = argc;
    this->argv = argv;

    this->id = ID;//robot ID

    //PARAMETERS FROM YAML FILE
    //goal topology
    getParamHandle.getParam("/tolerance", epsilon);
    vector<double> posvec;
    getParamHandle.getParam("/robpos", posvec);
    start_goal.x=posvec[0]; start_goal.y=posvec[1];
    getParamHandle.getParam("/robpos_follower", posvec);
    robpos.x = posvec[0];  robpos.y= posvec[1];
    getParamHandle.getParam("/N", this->N);
    getParamHandle.getParam("/obspos", posvec);
    for (int i=0; i<N; i++){
        Point2D a;
        a.x=posvec[i*2]; a.y=posvec[i*2+1];
        obspos.push_back(a);
    }

    //getting radius info
    getParamHandle.getParam("/obsro", obsrad);
    getParamHandle.getParam("/ro_r", robrad);
    getParamHandle.getParam("/ro_follower", robrad_follower);
    getParamHandle.getParam("/alpha", this->alpha);


    //k
    getParamHandle.getParam("/k", this->k);


    //threshold distance
    getParamHandle.getParam("/threshold", this->threshold);

    getParamHandle.getParam("/quite_mode", this->quite_mode);

    getParamHandle.getParam("/print_fi_b_dot", this->print_fi_b_dot);

    getParamHandle.getParam("/print_distances", this->print_distances);

    getParamHandle.getParam("/print_eta", this->print_eta);

    getParamHandle.getParam("/print_fi_eta_dot", this->print_fi_eta_dot);


}

Navigation ::~Navigation (){

}

double Navigation::orientation2theta(double  x, double  y, double  z, double  w){
  //calculates angles using quaternion
  double  siny_cosp = 2*(w*z + x*y);
  double  cosy_cosp = 1 - 2*(y*y + z*z);
  double  theta = atan2(siny_cosp,cosy_cosp);
  return theta;
}



//subscribers for odometry from gazebo
void  Navigation::gazeboOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  int temp_id = 0;
  if(temp_id == id){
    //converts quaternion to angles
    pose_theta = orientation2theta(0, 0, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
    if(!initialize_prev){
      initialize_prev = true;
    }
    else{
      path_distance = path_distance + sqrt(pow(prev_pose_x-pose_x,2)+pow(prev_pose_y-pose_y,2));//path distance update
    }
    prev_pose_x = pose_x;//previous position
    prev_pose_y = pose_y;

  }

  Point2D new_rob_pos;
  new_rob_pos.x = msg->pose.pose.position.x; new_rob_pos.y =msg->pose.pose.position.y;
  robpos = new_rob_pos;


}
void Navigation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
   //get the position
   this->pose_x= msg->pose.pose.position.x;
   this->pose_y= msg->pose.pose.position.y;
   this->pose_theta = orientation2theta(0, 0, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   //print pose
   //ROS_INFO("Position: (%f,%f,%f)",pose_x,pose_y,pose_theta);

}
void Navigation::modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
  
    // get the position of the model
    this->pose_x= msg->pose[1].position.x;
    this->pose_y= msg->pose[1].position.y;
    
    // print the position to the console
    //ROS_INFO("Model position: (%f, %f)", pose_x, pose_y);
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  Navigation::distanceCalculator(){
  //calculates distances among robots



}


void  Navigation::gamaFinder(){
  //calculates gamma values
  this->gama_sum=(pose_x-goal.x)*(pose_x-goal.x)+(pose_y-goal.y)*(pose_y-goal.y);
  //ROS_INFO("Gama sum is: (%f)", gama_sum);
}

void  Navigation::gamaBDotFinder(){
  this->gama_b_dot[0]=2*(pose_x-goal.x);
  this->gama_b_dot[1]=2*(pose_y-goal.y);
  //ROS_INFO("Gama dot: (%f, %f)", gama_b_dot[0], gama_b_dot[1]);
}

void  Navigation::betaFinder(){
  this->beta_product=1;
  //if(target_found){
    /*beta_product=((pose_x-obs_pos_x)*(pose_x-obs_pos_x)+(pose_y-obs_pos_y)*(pose_y-obs_pos_y)
    -(obsrad[0]+robrad)*(obsrad[0]+robrad))*
    ((pose_x-obs_pos_x)*(pose_x-obs_pos_x)+(pose_y-obs_pos_y+10)*(pose_y-obs_pos_y+10)
    -(obsrad[0]+robrad)*(obsrad[0]+robrad));*/
  //beta_product=(pose_x-goal.x)*(pose_x-goal.x)+(pose_y-goal.y)*(pose_y-goal.y)
  //  -(robrad_follower+robrad)*(robrad_follower+robrad);
  //}
  //ROS_INFO("Beta: (%f)", beta_product);
  /*ROS_INFO("Beta: (%f)", beta_product);
  ROS_INFO("Obstacle Poses-1: (%f,%f)", obspos.at(0).x,obspos.at(0).y);
  ROS_INFO("Obstacle Poses-2: (%f,%f)", obspos.at(1).x,obspos.at(1).y);
  ROS_INFO("current position: (%f,%f)", pose_x,pose_y);
  ROS_INFO("Radiuses obs1,obs2,robot: (%f,%f,%f)", obsrad.at(0),obsrad.at(1),robrad);*/
}

void  Navigation::betaBDotFinder(){
  //calculates derivative of beta with respect to b(position) >> beta_b_dot
  this->beta_b_dot[0]=0;
  this->beta_b_dot[1]=0;
    /*beta_b_dot[0]=2*(pose_x-obs_pos_x)*((pose_x-obs_pos_x)*(pose_x-obs_pos_x)+(pose_y-obs_pos_y+10)*(pose_y-obs_pos_y+10)
    -(obsrad[0]+robrad)*(obsrad[0]+robrad))+
    2*(pose_x-obs_pos_x)*((pose_x-obs_pos_x)*(pose_x-obs_pos_x)+(pose_y-obs_pos_y)*(pose_y-obs_pos_y)
    -(obsrad[0]+robrad)*(obsrad[0]+robrad));

    beta_b_dot[1]=2*(pose_y-obs_pos_y)*((pose_x-obs_pos_x)*(pose_x-obs_pos_x)+(pose_y-obs_pos_y+10)*(pose_y-obs_pos_y+10)
    -(obsrad[0]+robrad)*(obsrad[0]+robrad))+
    2*(pose_y-obs_pos_y+10)*((pose_x-obs_pos_x)*(pose_x-obs_pos_x)+(pose_y-obs_pos_y)*(pose_y-obs_pos_y)
    -(obsrad[0]+robrad)*(obsrad[0]+robrad));*/
    //beta_b_dot[0]=2*(pose_x-goal.x);
    //beta_b_dot[1]=2*(pose_y-goal.y);
  
  //ROS_INFO("Beta dot (%f, %f,%d)", beta_b_dot[0], beta_b_dot[1],i);
  //ROS_INFO("Divider: (%f,%d)",(pow((pose_x-obspos.at(i).x),2)+pow((pose_y-obspos.at(i).y),2)-
  //pow((obsrad.at(i)+robrad),2)),i);
    /*ROS_INFO("Obstacle Poses-1: (%f,%f)", obspos.at(0).x,obspos.at(0).y);
  ROS_INFO("Obstacle Poses-2: (%f,%f)", obspos.at(1).x,obspos.at(1).y);
  ROS_INFO("current position: (%f,%f)", pose_x,pose_y);
  ROS_INFO("Radiuses obs1,obs2,robot: (%f,%f,%f)", obsrad.at(0),obsrad.at(1),robrad);*/
  //ROS_INFO("Beta dot (%f, %f)", beta_b_dot[0], beta_b_dot[1]);
}



void  Navigation::fiBDotFinder(){
	//finding gradient of U
	this->fi_b_dot[0]=(pow(gama_sum,k-1)*(k*gama_b_dot[0]*beta_product-gama_sum*beta_b_dot[0]))/(beta_product*beta_product);
	this->fi_b_dot[1]=(pow(gama_sum,k-1)*(k*gama_b_dot[1]*beta_product-gama_sum*beta_b_dot[1]))/(beta_product*beta_product);
	//this->fi_b_dot[0]=(pow(gama_sum,k-1)*(k*gama_b_dot[0])-beta_b_dot[0]);
	//this->fi_b_dot[1]=(pow(gama_sum,k-1)*(k*gama_b_dot[1])-beta_b_dot[1]);
	//normalizing gradient
	this->normalizer =sqrt(fi_b_dot[0]*fi_b_dot[0]+fi_b_dot[1]*fi_b_dot[1]);
	fi_b_dot[0]=fi_b_dot[0]/normalizer;
	fi_b_dot[1]=fi_b_dot[1]/normalizer;
	//ROS_INFO("Normalized derivative: (%f, %f)", fi_b_dot[0], fi_b_dot[1]);
}


void  Navigation::Comparator(){
  

}

void Navigation::setPosition(){
  //calculates the next position
  this->pose_msg_x=pose_x-alpha*fi_b_dot[0];
  this->pose_msg_y=pose_y-alpha*fi_b_dot[1];
  //ROS_INFO("New Position: (%f, %f)", pose_msg_x, pose_msg_y);
}
void  Navigation::apfNavigation(){
  //calculates velocities and eta values
 Navigation::gamaFinder(); //calculates gamma
 Navigation::gamaBDotFinder(); //calculates derivative of gamma with respect to b(position)
 Navigation::betaFinder(); //calculates beta
 Navigation::betaBDotFinder(); //calculates derivative of beta with respect to b(position)
 Navigation::fiBDotFinder(); //calculates derivative of fi with respect to b(position)

 Navigation::setVelocity();//calculates linear and angular velocities
 //Navigation::gradVelocity();
 //Navigation::setPosition(); //calculates the next position
 Navigation::Comparator(); //checks goal topology is realized

}

void  Navigation::setVelocity(){
  double  vx, vy;
  
  //cout << "completed: " << completed << endl;
  if(completed == 1){//if goal topology is realized
    time_counter ++;
    if(time_counter>10000){ //if it lasts 10 seconds
      vx = 0;
      vy = 0;
      cout <<"set velocity zero" << endl;
      completed_stop = 1;
    }
    else{//waits 10 seconds to stop
      cout << "completed but continue" << endl;
      vx = 0.01;
      vy = 0;
      completed_stop = 0;
    }

  }
  else{//if goal topology is not realized
    time_counter = 0;
    vx=-fi_b_dot[0];
    vy=-fi_b_dot[1];
    completed_stop = 0;
  }


  double velocity_angle;
  bool velZero = false;
  if(vx==0){
    if(vy==0){
      velZero = true;
      linear_vel = 0;
      angular_vel = 0;
    }
    else if(vy>0){
      velocity_angle = PI/2;
    }
    else if(vy<0){
      velocity_angle = -PI/2;
    }

  }
  else{
    velocity_angle = atan(vy/vx);//velocity angle
    if(vx<0){
      velocity_angle = velocity_angle + PI;
    }
  }

  if(velZero==false){
    double delta_angle = velocity_angle-pose_theta;//angle between heading and velocity calculated by realization
    linear_vel = cos(delta_angle)*0.2; //linear velocity; max linear velocity is 0.2
    angular_vel = sin(delta_angle)*3; //angular velocity; max angular velocity is PI/3
  }
  if(abs(obs_pos_y)>0.1){angular_vel=-obs_pos_y*0.5;}
  else{
   angular_vel=0;
  }
  if(abs(obs_pos_x-1)>0.1){
   linear_vel=(obs_pos_x-1)*0.4;
  }
  else{
  linear_vel=0;
  }
}


void Navigation::obsCallback(const geometry_msgs::Pose& msg){
  obs_pos_x=msg.position.x*0.001;
  obs_pos_y=msg.position.y*0.001;
    

  goal.y=msg.position.x*sin(pose_theta)+msg.position.y*cos(pose_theta)+pose_y;
  goal.x=-msg.position.y*sin(pose_theta)+msg.position.x*cos(pose_theta)+pose_x;
  target_found=true;
  counto++;
  ROS_INFO("obs pos: (%f,%f)",obs_pos_x,obs_pos_y);
  ROS_INFO("position: (%f,%f)",pose_x,pose_y);
  ROS_INFO("message: (%f,%f)",msg.position.x,msg.position.y);
    


}


void  Navigation::work(){

  ostringstream ss;
  ss << id + 1;
  string name =  "robot" + ss.str(); //message name for velocity at gazebo

  vel_pub = velPub.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);//publisher for velocity at gazebo
  //pos_pub = posPub.advertise<geometry_msgs::Pose>("/cmd_pos", 100);//publisher for position at gazebo
  //ros::Subscriber calc_sub11 = poseSub.subscribe("/odom",1000,&Navigation ::gazeboOdomCallback,this);
  //ros::Subscriber model_states_sub = modelStateSub.subscribe("/gazebo/model_states",1000,&Navigation ::modelStatesCallback,this);
  ros::Subscriber odom_sub= modelStateSub.subscribe("/odom",1000,&Navigation::odomCallback,this);
  ros::Subscriber obs_pos_sub = ObsPosSub.subscribe("/leader_pos",1000,&Navigation::obsCallback,this);

  ros::Rate loop_rate(f);

  

  
  this->goal.x=start_goal.x;
  this->goal.y=start_goal.y;

  while (ros::ok()){

    flag = 1;
    
    Navigation::apfNavigation();
    vel_msg.linear.x = this->linear_vel;
    vel_msg.angular.z = this->angular_vel;
    vel_pub.publish(vel_msg); //publishes velocity message

    ros::spinOnce();
    loop_rate.sleep();
  }
}
