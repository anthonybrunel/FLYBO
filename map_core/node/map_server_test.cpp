#include <ros/ros.h>
#include "map_core/mapcore.h"
#include "map_core/mapcoreserver.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server_test");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");


    ros::Rate loop(100);
    MapCoreServer::Ptr mapServer;
            mapServer.reset(new MapCoreServer(nh,nh_private));
    ros::spin();


    mapServer->logExploration();
    return 0;

}
