#include <ros/ros.h>

#include "../include/VoxelApp.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_cam_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    DepthSimApp app(nh,nh_private);

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}
