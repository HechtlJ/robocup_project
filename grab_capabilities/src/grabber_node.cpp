#include <ros/ros.h>
#include "grabber.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;

    Grabber grabber(nh);
    
    

    //ros::spin();
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
}