#include <ros/ros.h>
#include <navigation.h>

int main(int argc,char** argv){

    ros::init(argc,argv,"navigation2");
    int mode;
    int minik = 1;
    Navigation* nav  = new Navigation(argc,argv,minik);
    nav->work();

}
