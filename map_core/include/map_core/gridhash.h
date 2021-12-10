#ifndef GRIDHASH_H
#define GRIDHASH_H

#include "../../utils/parallel_hashmap/phmap.h"
#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

static size_t __p1 = 73856093;
static size_t __p2 = 19349669;
static size_t __p3 = 83492791;

class Cell{
public:


    Eigen::Vector3f point = Eigen::Vector3f::Zero();
    uint32_t count = 0;
    void insertPoint(const Eigen::Vector3f &p){
        point +=p;
        count++;
    }

    Eigen::Vector3f getPoint()const{return point/static_cast<float>(count);}


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

    GridHash(ros::NodeHandle &nh){
        buffer_mapping_.reserve(99999999);
        scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_core/scan",10);
    }

    void insertPoint(const Eigen::Vector3f &p){

        CellCoordKey cellCoord;
        cellCoord.coord_ = getGridCoord(p);
        auto map_it = buffer_mapping_.find(cellCoord);
        if(map_it == buffer_mapping_.end()) {
            Cell cell;
            cell.insertPoint(p);
            buffer_mapping_.emplace(cellCoord,cell);
        }else{
            map_it->second.insertPoint(p);
        }
    }

    Eigen::Vector3i getGridCoord(const Eigen::Vector3f &p){
        return Eigen::Vector3i(static_cast<int>(roundf(p.x()/resolution_)),
                static_cast<int>(roundf(p.y()/resolution_)),
                static_cast<int>(roundf(p.z()/resolution_)));
    }

    void publishScan(){
        if (buffer_mapping_.empty())
            return;
        sensor_msgs::PointCloud2 scan_msg;
        scan_msg.header.frame_id = "world";
        scan_msg.header.stamp = ros::Time::now();
        sensor_msgs::PointCloud2Modifier modifier(scan_msg);

        modifier.setPointCloud2FieldsByString(1,"xyz");

        modifier.resize(buffer_mapping_.size());
        sensor_msgs::PointCloud2Iterator<float> it_x(scan_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(scan_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(scan_msg, "z");


        for(auto it = buffer_mapping_.begin(); it != buffer_mapping_.end(); ++it){
            Eigen::Vector3f p = it->second.getPoint();
            (*it_x) =  p(0);
            (*it_y) =  p(1);
            (*it_z) =  p(2);
            ++it_x;
            ++it_y;
            ++it_z;
        }
        scan_pub_.publish(scan_msg);

    }

    phmap::flat_hash_map<CellCoordKey, Cell, Hasher> buffer_mapping_;

    float resolution_ = 0.01;

    ros::Publisher scan_pub_;
};

#endif // GRIDHASH_H
