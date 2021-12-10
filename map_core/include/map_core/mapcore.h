#pragma once
#include <ros/ros.h>
#include <Eigen/Core>
#include "esdfmap.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "voxelbuffer.h"

#define DEBUG_VERBOSE 0

#include "camerafrustum.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "frontiermanager.h"
#include "../utils/timer.hpp"
#include "gridhash.h"
#include <functional>
class MapCore
{
public:
    MapCore();
    ~MapCore();
    void init(ros::NodeHandle& nh);

    void odomCallback(const nav_msgs::OdometryConstPtr &odom);

//    void depthOdomCalbackFrustum(const sensor_msgs::ImageConstPtr &depth_msg_i,const nav_msgs::OdometryConstPtr &odom_msg_i);

    void depthOdomCalbackRayCast(const sensor_msgs::ImageConstPtr &depth_msg_i,const nav_msgs::OdometryConstPtr &odom_msg_i);

    void raycast_free(const Eigen::Vector3i &point_idx,
                      const Eigen::Vector3i &origin_idx, std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i> > &updated);

    void probabilistic_raycast(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal, float d);

    void fast_voxel_trasversal_integrate(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal, const Eigen::Vector3f &goal_c, std::vector<bool> &integrate_list);
    void tag_free_around(float clearance);

    void isFrontierAsUnknown(const Eigen::Vector3i &cell);
    void frontierAsUnknown(const Eigen::Vector3i& bbox_min, const Eigen::Vector3i& bbox_max);

    void frontierAsFree(const Eigen::Vector3i& bbox_min, const Eigen::Vector3i& bbox_max);

    void publish_map();
    void publish_complete_map();

    //vizualize
    void publish_free();
    void publish_occ();
    void publish_frontier();
    void publish_prob();
    void publish_unknown();

    void publish_scan();

    void frontier_mode0(const Eigen::Vector3i &min_grid, const Eigen::Vector3i &max_grid){
        //Frontier updater
        auto isFrontier =[this](const Eigen::Vector3i& neight_pt) {
            if(_buffer->isInsideBuf(neight_pt)){
                if((_buffer->at(neight_pt) & VoxelBuffer::occupied_flag))
                    return 2;
                if(_buffer->at(neight_pt) & VoxelBuffer::visited_flag){
                    return 1;
                }
            }
            return 0;
        };

        for(int x = min_grid.x(); x <= max_grid.x(); ++x ){
            for(int y = min_grid.y(); y <= max_grid.y(); ++y ){
                for(int z = min_grid.z(); z <= max_grid.z(); ++z ){
                    Eigen::Vector3i pt_in_grid(x,y,z);

                    if(_buffer->isInsideBuf(pt_in_grid)){
                        if(!(_buffer->at(pt_in_grid) & VoxelBuffer::visited_flag) && !(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag)){
                            uint8_t res = 0;
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(1,0,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(-1,0,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,1,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,-1,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,0,1));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,0,-1));

                            if((res & 2) == 2 || res == 0){
                                if(_buffer->at(pt_in_grid) & VoxelBuffer::frontier_flag){
                                    frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),0));
                                    _buffer->at(pt_in_grid) &= ~(1u << 2);//clear frontier in box
                                }
                                continue;
                            }else{
                                _buffer->at(pt_in_grid) |= VoxelBuffer::frontier_flag;
                                frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),1));
                            }
                        }
                        else if(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag || _buffer->at(pt_in_grid) & VoxelBuffer::visited_flag){
                            frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),0));
                            _buffer->at(pt_in_grid) &= ~(1u << 2);//clear frontier in box
                        }

                    }

                }
            }
        }

    }
    void frontier_mode1(const Eigen::Vector3i &min_grid, const Eigen::Vector3i &max_grid){

        //Frontier updater
        auto isFrontier =[this](const Eigen::Vector3i& neight_pt) {
            if(_buffer->isInsideBuf(neight_pt)){
                if((_buffer->at(neight_pt) & VoxelBuffer::occupied_flag))
                    return 2;
                if(_buffer->at(neight_pt) == VoxelBuffer::unknown){
                    return 1;
                }
            }
            return 0;
        };

        for(int x = min_grid.x(); x <= max_grid.x(); ++x ){
            for(int y = min_grid.y(); y <= max_grid.y(); ++y ){
                for(int z = min_grid.z(); z <= max_grid.z(); ++z ){
                    Eigen::Vector3i pt_in_grid(x,y,z);

                    if(_buffer->isInsideBuf(pt_in_grid)){
                        if((_buffer->at(pt_in_grid) & VoxelBuffer::visited_flag) && !(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag)){
                            uint8_t res = 0;
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(1,0,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(-1,0,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,1,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,-1,0));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,0,1));
                            res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,0,-1));

                            if((res & 2) == 2 || res == 0){
                                if(_buffer->at(pt_in_grid) & VoxelBuffer::frontier_flag){
                                    frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),0));
                                    _buffer->at(pt_in_grid) &= ~(1u << 2);//clear frontier in box
                                }
                                continue;
                            }else{
                                _buffer->at(pt_in_grid) |= VoxelBuffer::frontier_flag;
                                frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),1));
                            }
                        }
                        else if(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag || !(_buffer->at(pt_in_grid) & VoxelBuffer::visited_flag)){
                            frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),0));
                            _buffer->at(pt_in_grid) &= ~(1u << 2);//clear frontier in box
                        }

                    }

                }
            }
        }
    }
private:

    EDFMap::Ptr _esdfmap;


    ros::Subscriber _odom_sub, _scan_sub;

    message_filters::Subscriber<sensor_msgs::Image>* depth_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_filter_sub_;

    message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Odometry>* sync_depth_odom_;

    std::string _frame_id;
    bool _has_cloud = false;

    VoxelBuffer::Ptr _buffer;
    bool inflate_ = true;


    //agent
    nav_msgs::Odometry odom_;
    Eigen::Isometry3f _pos;
    Eigen::Vector3f _agent_size;
    float uav_radius_;
    ros::Time _tpos;

    CameraFrustum cam_;


    Eigen::Vector3f _map_size;
    Eigen::Vector3i grid_origin_;
    Eigen::Vector3f map_origin_;
    float _resolution;


    ros::Publisher _map_pub;
    ros::Publisher _frontier_pub;

    std::vector<Eigen::Matrix<float,5,1>, Eigen::aligned_allocator<Eigen::Matrix<float,5,1>> > cell_to_update_;
    std::vector<Eigen::Vector4i> frontier_to_update_;


    //publisher

    //Vizualizer
    ros::Publisher _uk_pub;

    ros::Publisher _occ_pub;
    ros::Publisher _free_pub;
    ros::Publisher _frontier_visu_pub;
    ros::Publisher _prob_pub;



    int frontier_mode_ = 0;

    std::vector<uint8_t> _occ_current;


    bool _vizu;


    Timer t_since_start_;


    GridHash::Ptr grid_hash_;

    std::function<void(const Eigen::Vector3i&, const Eigen::Vector3i&)> frontier_func;

};

