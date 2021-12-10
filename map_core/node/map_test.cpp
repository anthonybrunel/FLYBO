#include <ros/ros.h>
#include "map_core/mapcore.h"
#include "map_core/mapcoreserver.h"
#include <glog/logging.h>

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "map_test");
    ros::NodeHandle nh("~");

//    ros::Duration(5.0).sleep();

    MapCore map;

    map.init(nh);


    ros::Rate loop(100);
    bool debug;
    //debug mode: see below ->  publish visual information about the map
    nh.param("map_core/debug", debug, false);

    if(!debug)
        ros::spin();
    else{
        while(nh.ok()){
            map.publish_frontier();


//            map.publish_free();
            map.publish_occ();
//            map.publish_unknown();
            map.publish_prob();
//            map.publish_scan();
            loop.sleep();
            ros::spinOnce();

        }
    }

}
