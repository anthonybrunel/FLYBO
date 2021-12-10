#pragma once
#include <ros/ros.h>
#include "tsdfvoxelgridevaluator.h"
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include "../utils/timer.hpp"

class TSDFRosInterface
{
public:
    TSDFRosInterface(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
    ~TSDFRosInterface(){
        if(save_mesh_thread.joinable())
            save_mesh_thread.join();
        delete sync_depth_odom_;

        delete depth_sub_;
        delete odom_sub_;
    }
    void depthOdomCalback(const sensor_msgs::ImageConstPtr &depth_msg_i,const nav_msgs::OdometryConstPtr &odom_msg_i);


    void publicSurfaceCloud(const t::geometry::PointCloud &surf_cloud);


    void save_mesh_fn(::geometry::TriangleMesh mesh){
        tsdf_map_->save_mesh(mesh);
        has_finish = true;
    }

    void save_pc_fn(::geometry::PointCloud pc){
        tsdf_map_->save_pc(pc);
        has_finish = true;
    }

    void save_mesh_callback(){
        if(has_finish){
            has_finish = false;
            if(save_mesh_thread.joinable())
                save_mesh_thread.join();
            ::geometry::TriangleMesh mesh;
            try {

                 mesh = tsdf_map_->extractMesh().ToLegacyTriangleMesh();

            } catch (std::runtime_error e) {
                std::cout << e.what()<<std::endl;
                return;
            }
            save_mesh_thread = std::thread(&TSDFRosInterface::save_mesh_fn,this,mesh);
            save_timer.restart();
        }

    }

    void save_pc_callback(){
//        ::geometry::PointCloud pc;

//        pc =
//        std::cout <<tsdf_map_->ExtractSurfacePoints().To(::core::Device("CPU:0")).ToString() << std::endl;
//        save_timer.restart();

        if(has_finish){
            has_finish = true;
            if(save_mesh_thread.joinable())
                save_mesh_thread.join();
            ::geometry::PointCloud pc;
            try {

                 pc = tsdf_map_->extractPointCloud().ToLegacyPointCloud();

            } catch (std::runtime_error e) {
                std::cout << e.what()<<std::endl;
                save_timer.restart();
                return;
            }
            save_mesh_thread = std::thread(&TSDFRosInterface::save_pc_fn,this,pc);
            save_timer.restart();
        }
    }

    void publish_mesh();
private:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, nav_msgs::Odometry> sync_image_odo_t ;
    std::atomic_bool has_finish;
    Timer save_timer;
    double save_every_ms = 20*1000;

    std::thread save_mesh_thread;
    std::mutex save_lock;
    bool has_odom = false;
    ros::NodeHandle nh_,nh_private_;


    TSDFVoxelGridEvaluator::Ptr tsdf_map_;


    message_filters::Subscriber<sensor_msgs::Image>* depth_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_;

    message_filters::Synchronizer<sync_image_odo_t>* sync_depth_odom_;

    ros::Publisher surface_cloud_pub_;
    ros::Publisher mesh_pub_;
};

