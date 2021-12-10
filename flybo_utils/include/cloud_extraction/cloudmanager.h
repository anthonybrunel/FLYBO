#pragma once
#include <ros/ros.h>
#include <Eigen/Core>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "parallel_hashmap/phmap.h"
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include "flybo_utils/timer.hpp"

static size_t __p1 = 73856093;
static size_t __p2 = 19349669;
static size_t __p3 = 83492791;


class CamUtils{
public:
    CamUtils(){

    }


    inline void computeParametersFronFoV(int width, double fovx_deg, double fovy_deg, double max_distance = 5.){
        fovx_deg_ = fovx_deg;
        fovy_deg_ = fovy_deg;
        fovx_rad_ = fovx_deg*M_PI/180.;
        fovy_rad_ = fovy_deg*M_PI/180.;
        width_ = width;
        double tanx = tan((fovx_rad_/2.));
        max_distance_ = max_distance;

        height_ = (width_/((tanx)/tan(fovy_rad_/2.)))+0.5;
        double ratio = (double) width_ / (double) height_;
        ppx_ = width_/2.;
        ppy_ = height_/2.;

        fx_ =static_cast<double>(height_)/(2.*tan(fovy_rad_/2.));
        fy_ = fx_;
        std::cout << "[CloudExtractor] fovx;fovy: " << fovx_deg << ";" << fovy_deg  <<std::endl
         << "[CloudExtractor] Res [width,height]: " << width <<"x" << height_ <<std::endl
         << "[CloudExtractor] Max range: " << max_distance_ <<std::endl
         << "[CloudExtractor] Focal (fx,fy): " << fx_ << "," << fy_ <<std::endl
        << "[CloudExtractor] Center (ppx,ppy): " << ppx_ << "," << ppy_ <<std::endl;


    }

    Eigen::Vector3d deproj(const Eigen::Vector2i & pxl,const float d,const Eigen::Matrix4d &T){
        Eigen::Vector3d pts;
        pts.x() = 1.0f;
        pts.y() = -(pxl.x() - ppx_)/fx_;
        pts.z() = -(pxl.y() - ppy_)/fy_;
        pts *= d;

        pts = T.block(0,0,3,3)*pts + T.col(3).head(3);
        return pts;
    }

    int width_,height_;
    double fx_,fy_,ppx_,ppy_,fovx_deg_,fovy_deg_,fovx_rad_,fovy_rad_;


    double max_distance_ = 5.;

    Eigen::Matrix4d intrinsic_;

};

class Cell{
public:

    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f point = Eigen::Vector3f::Zero();
    uint32_t count = 0;
    void insertPoint(const Eigen::Vector3f &p){
        point +=p;
        count++;
    }

    Eigen::Vector3f getPoint()const{
        return center;

//        return point/static_cast<float>(count);

    }


};
struct CellCoordKey{
    Eigen::Vector3i coord_;

    int64_t hash_value ()const
    {
          return ((__p1 * static_cast<int64_t>(coord_.x()))^
                  (__p2 * static_cast<int64_t>(coord_.y()))^
                  (__p3 * static_cast<int64_t>(coord_.z()))) % static_cast<int64_t>(1e9);
    }

    bool operator==(const CellCoordKey &rhs) const {
        for (auto i = 0; i < 3; i++) {
            if (coord_[i] != rhs.coord_[i]) return false;
        }
        return true;
    }
};

struct Hasher{
    int64_t operator()(const CellCoordKey &coord_cell) const {
        return coord_cell.hash_value();
    }
};


class GridHash
{
public:
    typedef std::shared_ptr<GridHash> Ptr;

    GridHash(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):_nh(nh){
        buffer_mapping_.reserve(99999999);
        scan_pub_ = _nh.advertise<sensor_msgs::PointCloud2>("/cloud_extraction/scan",10);
        nh_private.param("cloud_extraction/save_path",_save_path,std::string(""));
        pcd.reset(new open3d::geometry::PointCloud());
    }
    ~GridHash(){
        try {
            if(_save_path.size() < 3){
                std::cout << "[GridHash] could not save GT pointcloud: " << _save_path <<std::endl;
                return;
            }
            toPly(_save_path);
        } catch (...) {
            std::cout << "[GridHash] could not save GT pointcloud..."<<std::endl;
        }
    }
    void insertPoint(const Eigen::Vector3f &p){

        CellCoordKey cellCoord;
        cellCoord.coord_ = getGridCoord(p);
        auto map_it = buffer_mapping_.find(cellCoord);
        if(map_it == buffer_mapping_.end()) {
            Cell cell;
            cell.insertPoint(p);
            cell.center = getWorldCoord(cellCoord.coord_);
            buffer_mapping_.emplace(cellCoord,cell);
            count+=1;
        }else{
            map_it->second.insertPoint(p);
        }
    }

    Eigen::Vector3i getGridCoord(const Eigen::Vector3f &p){
        return Eigen::Vector3i(static_cast<int>(roundf(p.x()/resolution_)),
                static_cast<int>(roundf(p.y()/resolution_)),
                static_cast<int>(roundf(p.z()/resolution_)));
    }

    Eigen::Vector3f getWorldCoord(const Eigen::Vector3i &p){
        return p.cast<float>()*resolution_;
    }


    void publishScan(){
        if (buffer_mapping_.empty())
            return;
        Timer t;
        sensor_msgs::PointCloud2 scan_msg;
        scan_msg.header.frame_id = "world";
        scan_msg.header.stamp = ros::Time::now();
        sensor_msgs::PointCloud2Modifier modifier(scan_msg);


        modifier.setPointCloud2FieldsByString(1,"xyz");

                modifier.resize(pcd->points_.size());
//        modifier.resize(buffer_mapping_.size());
        sensor_msgs::PointCloud2Iterator<float> it_x(scan_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(scan_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(scan_msg, "z");

//        for(auto it = buffer_mapping_.begin(); it != buffer_mapping_.end(); ++it){
//            Eigen::Vector3d p =it->second.center.cast<double>();
//            (*it_x) =  p(0);
//            (*it_y) =  p(1);
//            (*it_z) =  p(2);
//            ++it_x;
//            ++it_y;
//            ++it_z;
//        }

        for(size_t i = 0; i < pcd->points_.size() ;++i){
            Eigen::Vector3d p = pcd->points_[i];
            (*it_x) =  p(0);
            (*it_y) =  p(1);
            (*it_z) =  p(2);
            ++it_x;
            ++it_y;
            ++it_z;
        }
        scan_pub_.publish(scan_msg);

    }

    void toPly(const std::string &path){

        std::cout << "[GridHash] Save GT cloud to: " << path<<std::endl;

        open3d::geometry::PointCloud cloud;
        cloud.points_.reserve(buffer_mapping_.size());

        for(auto it = buffer_mapping_.begin(); it != buffer_mapping_.end(); ++it){
            cloud.points_.push_back(it->second.getPoint().cast<double>());
        }
        open3d::io::WritePointCloudToPLY(path,cloud,open3d::io::WritePointCloudOption());
    }

    std::shared_ptr<open3d::geometry::PointCloud> pcd;
    phmap::flat_hash_map<CellCoordKey, Cell, Hasher> buffer_mapping_;

    float resolution_ = 0.01f;
    ros::NodeHandle _nh;
    ros::Publisher scan_pub_;
    uint count = 0;
    std::string _save_path = "";
};

class CloudManager
{
public:
    CloudManager(const ros::NodeHandle &nh_i,const ros::NodeHandle &nh_private_i):_nh(nh_i),_nh_private(nh_private_i),_grid(nh_i,nh_private_i){

        double max_distance; bool useFov,useRealSensor;
        _nh_private.param("cloud_manager/useFov",useFov,false);
        _nh_private.param("cloud_manager/useRealSensor",useRealSensor,false);
        _nh_private.param("cloud_manager/max_range",max_distance,5.);
        int width,height;
        double fx,fy,ppx,ppy,fovx,fovy;
        _nh_private.param("cloud_manager/width",width,640);
        _nh_private.param("cloud_manager/heigh",height,480);
        _nh_private.param("cloud_manager/fx",fx,461.453);
        _nh_private.param("cloud_manager/fy",fy,460.859);
        _nh_private.param("cloud_manager/ppx",ppx,344.191);
        _nh_private.param("cloud_manager/ppy",ppy,245.387);
        _nh_private.param("cloud_manager/fovx",fovx,90.);
        _nh_private.param("cloud_manager/fovy",fovy,60.);
        _nh_private.param("cloud_manager/resolution",_grid.resolution_,0.005f);
        _cam.computeParametersFronFoV(width,fovx,fovy,max_distance);

        depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(_nh,"/cloud_manager/depth",1);
        odom_filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(_nh,"/cloud_manager/odometry",1);

        sync_depth_odom_ = new message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Odometry>(*depth_sub_,*odom_filter_sub_,1);
        //        sync_depth_odom_->registerCallback(boost::bind(&MapCore::depthOdomCalbackFrustum,this, _1, _2));
        sync_depth_odom_->registerCallback(boost::bind(&CloudManager::depthOdomCalback,this, _1, _2));

        vizu_scan_timer = _nh.createTimer(ros::Duration(1.),
                                         &CloudManager::publishScan,
                                         this);

        Eigen::Vector3d _map_size,map_origin_;
        _nh_private.param("cloud_extraction/x_size", _map_size.x(), 25.);
        _nh_private.param("cloud_extraction/y_size", _map_size.y(), 25.);
        _nh_private.param("cloud_extraction/z_size", _map_size.z(), 5.);


        _nh_private.param("cloud_extraction/origin_x", map_origin_.x(), 0.);
        _nh_private.param("cloud_extraction/origin_y", map_origin_.y(), 0.);
        _nh_private.param("cloud_extraction/origin_z", map_origin_.z(), 0.);


        grid_min = -_map_size/2.+map_origin_;
        grid_max = map_origin_+_map_size/2.;
        std::cout << "[CloudManager] Min Bound: " << grid_min.transpose() << "(m)" << std::endl;
        std::cout << "[CloudManager] Max Bound: " << grid_max.transpose() << "(m)" << std::endl;

    }

    void depthOdomCalback(const sensor_msgs::ImageConstPtr &depth_msg_i,const nav_msgs::OdometryConstPtr &odom_msg_i);


    bool isInside(Eigen::Vector3d pt){
        return pt.x() > grid_min.x() &&
        pt.x() < grid_max.x() &&
        pt.y() > grid_min.y() &&
        pt.y() < grid_max.y() &&
        pt.z() > grid_min.z() &&
        pt.z() < grid_max.z();
    }

    void publishScan(const ros::TimerEvent &){
        _grid.publishScan();
    }
private:
    CamUtils _cam;
    GridHash _grid;
    ros::NodeHandle _nh_private;
    ros::NodeHandle _nh;
    Eigen::Vector3d grid_min;
    Eigen::Vector3d grid_max;

    ros::Subscriber _odom_sub, _scan_sub;

    message_filters::Subscriber<sensor_msgs::Image>* depth_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_filter_sub_;

    message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Odometry>* sync_depth_odom_;

    ros::Timer vizu_scan_timer;
};

