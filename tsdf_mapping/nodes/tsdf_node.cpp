#include <ros/ros.h>


#include "../include/tsdfrosinterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tsdf_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    TSDFRosInterface tsdf_mapper(nh,nh_private);
    ros::spin();


    return 0;
}
