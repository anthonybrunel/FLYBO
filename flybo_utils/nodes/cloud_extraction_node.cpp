#include <ros/ros.h>
#include "../include/cloud_extraction/cloudmanager.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_extraction_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    CloudManager cloud_manager(nh,nh_private);

    ros::spin();
}
