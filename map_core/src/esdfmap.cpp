#include "map_core/esdfmap.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "../utils/timer.hpp"
EDFMap::EDFMap()
{

}

void EDFMap::init(ros::NodeHandle &nh,VoxelBuffer *voxBuffer, Vector3i size, float resolution, float trunc_dist)
{

    nh.param("map_core/esdf_mode", mode_occupancy, 0);

    _voxBuffer = voxBuffer;
    _size = size;
    _resolution = resolution;
    _truncation_dist = trunc_dist;

    _dist_buffer.resize(size(0)*size(1)*size(2),100000);
    _tmp_buffer1.resize(size(0)*size(1)*size(2),0);
    _tmp_buffer2.resize(size(0)*size(1)*size(2),0);

    _dist_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/esdf", 10);
//    _esdfUpdateTimer = nh.createTimer(ros::Duration(0.5),&EDFMap::compute_edt, this);
has_started = false;
    esdf_slice_ = nh.advertise<sensor_msgs::PointCloud2>("/map_core/esdf_slice", 10);
    switch(mode_occupancy){
    case 0:
        occupancy_func = std::bind(&EDFMap::occypancy_frontiere_occ,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
        break;
    case 1:
        occupancy_func = std::bind(&EDFMap::occypancy_occ,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
        break;
    case 2:
        occupancy_func = std::bind(&EDFMap::occypancy_unk_occ,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
        break;
    }
    update_timer.restart();
}

void EDFMap::compute_edt()
{

    //    std::cout << "Compute distance map" << std::endl;
    const Vector3i &max = _size-Vector3i::Ones();
    const Vector3i &min = Vector3i(0,0,0);
    const Vector3i &size = _voxBuffer->_size;

    Timer t;
    for(int x=min[0]; x<=max[0]; x++) {
        for(int y=min[1]; y<=max[1]; y++) {

            distance_transform([&](int z) {return occupancy_func(x,y,z);},
            [&](int z, float val) {at(_tmp_buffer1,Vector3i(x,y,z)) = val;},
            min[2], max[2],size.z());

        }
    }


    for(int x=min[0]; x<=max[0]; x++) {
        for(int z=min[2]; z<=max[2]; z++) {
            distance_transform([&](int y) {return at(_tmp_buffer1, Vector3i(x,y,z));},
            [&](int y, float val) {at(_tmp_buffer2,Vector3i(x,y,z)) = val;},
            min[1], max[1],size.y());
        }
    }


    for(int y=min[1]; y<=max[1]; y++) {
        for(int z=min[2]; z<=max[2]; z++) {
            distance_transform([&](int x) {return at(_tmp_buffer2, Vector3i(x,y,z));},
            [&](int x, float val) { at(_dist_buffer, Vector3i(x,y,z)) = std::min(_resolution * std::sqrt(val), _truncation_dist);},
            min[0], max[0],size.x());
        }
    }
    publish_distance_field();
    if(debug_vizu_){
        publish_slice();
//        std::cout << "esdf update: " << t.elapsed_ms() << std::endl;;
    }
//    std::cout << "esdf update: " << t.elapsed_ms() << std::endl;;

    has_started=false;
}

void EDFMap::publish_slice()
{
    pcl::PointXYZI pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZI> cloud;



    int count = 0;
    Eigen::Vector3i slice_pos_z;
    _voxBuffer->getVoxelPos(Eigen::Vector3f(0,0,1.5),slice_pos_z);
    int z = slice_pos_z.z();
    cloud.reserve(_size.x()*_size.y()*_size.z());
    for(size_t x = 0; x<_size.x(); ++x){
        for(size_t y = 0; y<_size.y(); ++y){

            const Eigen::Vector3i pos(x,y,z);

            _voxBuffer->convert(pos,ref_pt);
            pt.getVector3fMap() = ref_pt;
            pt._PointXYZI::intensity = at(_dist_buffer,pos);
            cloud.points.push_back(pt);
            count++;
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";


    if(count > 0){

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        esdf_slice_.publish(cloud_msg);

    }
}

void EDFMap::publish_distance_field()
{

    pcl::PointXYZI pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZI> cloud;



    int count = 0;
    cloud.reserve(_size.x()*_size.y()*_size.z());
    for(size_t x = 0; x<_size.x(); ++x){
        for(size_t y = 0; y<_size.y(); ++y){
            for(size_t z = 0;z< _size.z(); ++z){

                const Eigen::Vector3i pos(x,y,z);


                if ((at(_dist_buffer,pos) < _truncation_dist)) {
                    pt.getVector3fMap() = pos.cast<float>();
                    pt._PointXYZI::intensity = at(_dist_buffer,pos);
                    cloud.points.push_back(pt);
                    count++;
                }
            }
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";


    if(count > 0){
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        _dist_pub.publish(cloud_msg);

    }

}
