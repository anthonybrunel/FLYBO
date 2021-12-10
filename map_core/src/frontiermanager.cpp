#include "../include/map_core/frontiermanager.h"


FrontierManager::FrontierManager(const Eigen::Vector3i &map_size_i):map_size_(map_size_i)
{
    frontier_map_.resize(map_size_.x()*map_size_.y()*map_size_.z(),std::pair<uint8_t,frontierIterator>(0,frontier_list_.end()));
    frontier_list_.reserve(map_size_.x()*map_size_.y()*map_size_.z()*0.2);// no more than 20% of the map will be a frontier i hope

   frontier_features_.resize(map_size_.x()*map_size_.y()*map_size_.z());

}

void FrontierManager::insertFrontier(const Eigen::Vector3i& p)
{

    std::pair<int,frontierIterator> &v = frontier_map_[get_idx(p)];
    if(v.first == 1)//already inserted
        return;
    v.first = 1;
    frontier_list_.push_front(p);
    v.second = frontier_list_.begin();

}

void FrontierManager::removeFrontier(const Eigen::Vector3i& p)
{
    std::pair<int,frontierIterator> &v = frontier_map_[get_idx(p)];
    if(v.first == 0)//already deleted
        return;
    v.first = 0;
//    if(store_removed_)
//        removed_frontier_.push_back(*v.second);
    frontier_list_.erase(v.second);
}
