#pragma once
#include <vector>
#include <Eigen/Core>
#include <../utils/plf_list.h>
#include <memory>
#include <mutex>
#include <iostream>


struct Features{
    float planarity = 0;


    float tmp = 0;
};


class FrontierManager
{
public:


    typedef std::shared_ptr<FrontierManager> Ptr;

    FrontierManager(const Eigen::Vector3i& map_size_i);

    void insertFrontier(const Eigen::Vector3i& p);
    void removeFrontier(const Eigen::Vector3i& p);


    inline size_t get_idx(const Eigen::Vector3i &coord){
        return map_size_(2)*map_size_(1)*coord(0)+map_size_(2)*coord(1)+coord(2);
    }


    void lock(){
        frontier_mutex_.lock();
    }

    void release(){
        frontier_mutex_.unlock();
    }

    void setFeatures(const Eigen::Vector3i &coord,const Features &f){
        frontier_features_[get_idx(coord)] = f;
    }

    void getFeatures(const Eigen::Vector3i &coord,Features &f){
        f = frontier_features_[get_idx(coord)];
    }

    typedef  plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>::iterator frontierIterator;
    plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontier_list_;
    std::vector<Features> frontier_features_;


    bool store_removed_ = true;
    plf::list<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> removed_frontier_;


    //copy frontiers inside a vector for the other thread
    void getFrontiers(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& frontiers){
        lock();
        frontiers.resize(frontier_list_.size());
        int i = 0;
        for(frontierIterator it = frontier_list_.begin(); it != frontier_list_.end(); ++it){
            frontiers[i++] = *it;
        }
        release();
    }


    void getRemovedFrontiers(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& removed_frontier){
        lock();
        removed_frontier.resize(removed_frontier_.size());
        int i = 0;
        for(frontierIterator it = removed_frontier_.begin(); it != removed_frontier_.end(); ++it){
            removed_frontier[i++] = *it;
        }
        removed_frontier_.clear();
        release();
    }


private:

    //used for get frontier data
    std::mutex frontier_mutex_;

    std::vector<
        std::pair<int,
                    frontierIterator>> frontier_map_;

    Eigen::Vector3i map_size_;

};

