#pragma once
#include <ros/ros.h>
#include <Eigen/Core>
#include "esdfmap.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "voxelbuffer.h"

#include <tbb/concurrent_vector.h>
#include "frontiermanager.h"
#include "../utils/timer.hpp"
class MapCoreServer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MapCoreServer> Ptr;

    MapCoreServer();
    MapCoreServer(const ros::NodeHandle & nh_i,const ros::NodeHandle & nhprivate_i);
    ~MapCoreServer();
    void init(ros::NodeHandle & nh_i);



    void getDistance(const Eigen::Vector3f &pos, float & dist);
    void getDistance(const Eigen::Vector3i &pos, float & dist);

    uint8_t getState(const Eigen::Vector3i &pos);
    inline uint8_t getState(const size_t i){
        return map_state_[i];
    }
    inline size_t get_idx(const Eigen::Vector3i &coord){
        return buffer_->get_idx(coord);
    }

    inline float getTsdfValue(size_t idx){
        return buffer_->tsdf_[idx];
    }
    inline float getWeight(size_t idx){
        return buffer_->weight_[idx];
    }
    inline uint8_t getState(const Eigen::Vector3f &coord){
        Eigen::Vector3i pos;
        convert(coord,pos);
        return map_state_[buffer_->get_idx(pos)];
    }

    void trilinearInterpolation(const Eigen::Vector3f &pos,float & dist, Eigen::Vector3f &grad);

    void getDistAndGrad(const Eigen::Vector3f &pos, float & dist, Eigen::Vector3f &grad);


    void esdfCallback(const sensor_msgs::PointCloud2ConstPtr & esdf_i);


    void mapCallback(const sensor_msgs::PointCloud2ConstPtr & map_i);


    void frontierCallback(const sensor_msgs::PointCloud2ConstPtr & frontier_i);


    void convert(const Eigen::Vector3f &in, Eigen::Vector3i& out){
        buffer_->getVoxelPos(in,out);
    }

    void convert(const Eigen::Vector3i &in, Eigen::Vector3f& out){
        buffer_->convert(in,out);
    }

    bool is_inside(const Eigen::Vector3f &p){
        return buffer_->is_inside(p);
    }
    bool is_inside(const Eigen::Vector3i &p){
        return buffer_->is_inside(p);
    }

    bool isInsideBuf(const Eigen::Vector3i &p){
        return buffer_->isInsideBuf(p);
    }

    BoundingMap3D<float> getBoudingMapReal(){
        return buffer_->getBoudingMapReal();
    }
    void publish_frontier();
    void publish_occ();
    void publish_esdf_slice();

    void getFrontiers(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& frontiers){
        frontierManager_->getFrontiers(frontiers);
    }
    void getRemovedFrontiers(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& removed_frontiers){
        frontierManager_->getRemovedFrontiers(removed_frontiers);
    }


    void getMapState(std::vector<uint8_t>& map_){
//        map_.resize(map_state_.size());
        map_ = std::vector<uint8_t>(map_state_.begin(),map_state_.end());
    }

    void getEdtMap(std::vector<float>& esdf_i){
//        map_.resize(map_state_.size());
        esdf_i = std::vector<float>(esdf_.begin(),esdf_.end());
    }

    void clamp(Eigen::Vector3i &coord){
        buffer_->clamp(coord);
    }

    void clamp(Eigen::Vector3f &coord){
        buffer_->clamp(coord);
    }

    bool clampRay(const Eigen::Vector3i &origin, const Eigen::Vector3f &dir, Eigen::Vector3i &result){
        return buffer_->clampGridRay(origin, dir, result);
    }
    void get3DGridSize(Eigen::Vector3i &size_o){
        size_o = buffer_->_size;
    }



    void setFrontierFeatures(const Eigen::Vector3i &coord,const Features &f){
        frontierManager_->setFeatures(coord,f);
    }
    void getFrontierFeatures(const Eigen::Vector3i &coord,Features &f){
        frontierManager_->getFeatures(coord,f);
    }

    float getResolution(){
        return resolution_;
    }


    void logExploration();
private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;


    float resolution_;
    float resolution3_;
    Eigen::Vector3f map_size_;
    Eigen::Vector3i grid_origin_;
    Eigen::Vector3f map_origin_;

    VoxelBuffer::Ptr buffer_;
    FrontierManager::Ptr frontierManager_;


//    std::vector<float> esdf_;
    tbb::concurrent_vector<float> esdf_;
    tbb::concurrent_vector<uint8_t> map_state_;

    ros::Subscriber map_sub_;
    ros::Subscriber esdf_sub_;
    ros::Subscriber frontier_sub_;


    //Visu publisher
    ros::Publisher frontier_pub_;
    ros::Publisher occ_pub_;
    ros::Publisher esdf_pub_;



    float delta_insertion_ = 1000;//[ms]
    float score_ = 0;//[m3]
    Timer t_;
    std::vector<std::pair<float, float>> scores_;//m3/s discover


    std::string save_path_ = "";

};

