#include "../include/map_core/mapcoreserver.h"
#include <pcl/point_types.h>
#include "../utils/timer.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <fstream>

MapCoreServer::MapCoreServer()
{

}

MapCoreServer::MapCoreServer(const ros::NodeHandle &nh_i, const ros::NodeHandle &nhprivate_i):nh_(nh_i),nh_private_(nhprivate_i)
{
    nh_private_.param("map_core/resolution", resolution_, 0.1f);
    nh_private_.param("map_core/x_size", map_size_.x(), 20.f);
    nh_private_.param("map_core/y_size", map_size_.y(), 20.f);
    nh_private_.param("map_core/z_size", map_size_.z(), 2.6f);

    buffer_.reset(new VoxelBuffer);

    nh_private_.param("map_core/origin_x", map_origin_.x(), 0.f);
    nh_private_.param("map_core/origin_y", map_origin_.y(), 0.f);
    nh_private_.param("map_core/origin_z", map_origin_.z(), 1.1f);

    nh_private_.param("map_core/save_folder", save_path_, std::string(""));
    std::cout << "[MapServer] Vol log Save path: "<< save_path_ << std::endl;
    //    grid_origin_ = Eigen::Vector3i::Zero();
    //    grid_origin_.x() = ((map_size_.x()/2.+map_origin_.x())*1./resolution_+0.5);
    //    grid_origin_.y() = ((map_size_.y()/2.+map_origin_.y())*1./resolution_+0.5);
    //    grid_origin_.z() = ((map_size_.z()/2.+map_origin_.z())*1./resolution_+0.5);
    //    _scan_sub = nh.subscribe<sensor_msgs::PointCloud2>("/map_core/cloud", 10, &MapCore::cloudCallback, this);

    //origin = half of the map;
    buffer_->init(map_size_,map_origin_, resolution_);
    esdf_.resize(buffer_->_size(0)*buffer_->_size(1)*buffer_->_size(2),1.);
    map_state_.resize(buffer_->_size(0)*buffer_->_size(1)*buffer_->_size(2),0);
    std::fill(map_state_.begin(), map_state_.end(), uint8_t(0));
    buffer_->print_infos();

    frontierManager_.reset(new FrontierManager(buffer_->_size));

    map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map_core/map", 100, &MapCoreServer::mapCallback, this);
    esdf_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map_core/esdf", 100, &MapCoreServer::esdfCallback, this);
    frontier_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map_core/frontier", 100, &MapCoreServer::frontierCallback, this);



    frontier_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_server/frontier_vizu", 100);
    occ_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_server/occ_vizu", 100);
    esdf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_server/esdf_slice_vizu", 100);

    std::cout << "[MapServer] Initialized" <<std::endl;
    resolution3_ = resolution_*resolution_*resolution_;

    scores_.reserve(900*4);

    scores_.emplace_back(std::pair<float,float>(0,0));
    t_.restart();
}

MapCoreServer::~MapCoreServer()
{

}

void MapCoreServer::init(ros::NodeHandle &nh_i)
{
    nh_i.param("map_core/resolution", resolution_, 0.2f);
    nh_i.param("map_core/x_size", map_size_.x(), 25.f);
    nh_i.param("map_core/y_size", map_size_.y(), 25.f);
    nh_i.param("map_core/z_size", map_size_.z(), 5.f);
    nh_i.param("map_core/origin_x", map_origin_.x(), 0.f);
    nh_i.param("map_core/origin_y", map_origin_.y(), 0.f);
    nh_i.param("map_core/origin_z", map_origin_.z(), 0.f);
    resolution3_ = resolution_*resolution_*resolution_;
    buffer_.reset(new VoxelBuffer);
    Eigen::Vector3i origin;

    origin.x() = 0;//static_cast<int>(_map_size.x() * 1/2.);
    origin.y() = 0;// static_cast<int>(_map_size.y() * 1/2.);
    origin.z() = 0;// static_cast<int>(_map_size.z() * 1/2.);

    buffer_->init(map_size_,map_origin_, resolution_);
    buffer_->_flag_buffer.clear();//clear we use a tbb vector
    esdf_.resize(buffer_->_size(0)*buffer_->_size(1)*buffer_->_size(2),1.f);

    map_sub_ = nh_i.subscribe<sensor_msgs::PointCloud2>("/map_core/map", 10, &MapCoreServer::mapCallback, this);
    esdf_sub_ = nh_i.subscribe<sensor_msgs::PointCloud2>("/map_core/esdf", 10, &MapCoreServer::esdfCallback, this);


    frontier_sub_ = nh_i.subscribe<sensor_msgs::PointCloud2>("/map_core/frontier", 10, &MapCoreServer::esdfCallback, this);
}

void MapCoreServer::getDistance(const Eigen::Vector3f &pos, float &dist)
{
    dist = esdf_[buffer_->get_idx_f(pos)];
}

void MapCoreServer::getDistance(const Eigen::Vector3i &pos, float &dist)
{
    dist = esdf_[buffer_->get_idx(pos)];

}

uint8_t MapCoreServer::getState(const Eigen::Vector3i &pos)
{
    return map_state_[buffer_->get_idx(pos)];
}

void MapCoreServer::trilinearInterpolation(const Eigen::Vector3f &pos,float &dist, Eigen::Vector3f &grad)
{
    Eigen::Vector3i p0;
    Eigen::Vector3f pts = pos.array()-0.5*resolution_;
    buffer_->getVoxelPos(pts,p0);
    Eigen::Vector3f gcenter;
    buffer_->convert(p0,gcenter);
    Eigen::Vector3d pdiff = (pos.cast<double>()-gcenter.cast<double>())/resolution_;

    float c_arr[2][2][2];
    for(size_t x = 0; x < 2; x++) {
        for(size_t y = 0; y < 2; y++) {
            for(size_t z = 0; z < 2; z++) {
                getDistance(Eigen::Vector3i(p0+Eigen::Vector3i(x,y,z)),c_arr[x][y][z]);
            }
        }
    }


    // Trilinear interpolation
    double c00 = (1-pdiff[0])*c_arr[0][0][0] + pdiff[0]*c_arr[1][0][0];
    double c01 = (1-pdiff[0])*c_arr[0][0][1] + pdiff[0]*c_arr[1][0][1];
    double c10 = (1-pdiff[0])*c_arr[0][1][0] + pdiff[0]*c_arr[1][1][0];
    double c11 = (1-pdiff[0])*c_arr[0][1][1] + pdiff[0]*c_arr[1][1][1];

    double c0 = (1-pdiff[1])*c00 + pdiff[1]*c10;
    double c1 = (1-pdiff[1])*c01 + pdiff[1]*c11;


    dist = (1-pdiff[2])*c0 + pdiff[2]*c1;

    grad[2] = (c1-c0)/resolution_;
    grad[1] = ((1-pdiff[2])*(c10 - c00) + pdiff[2]*(c11 - c01))/resolution_;

    grad[0] = (1-pdiff[2])*(1-pdiff[1])*(c_arr[1][0][0] - c_arr[0][0][0]);
    grad[0] += (1-pdiff[2])*pdiff[1]*(c_arr[1][1][0] - c_arr[0][1][0]);
    grad[0] += pdiff[2]*(1-pdiff[1])*(c_arr[1][0][1] - c_arr[0][0][1]);
    grad[0] += pdiff[2]*pdiff[1]*(c_arr[1][1][1] - c_arr[0][1][1]);

    grad[0] /= resolution_;
}

void MapCoreServer::getDistAndGrad(const Eigen::Vector3f &pos, float &dist, Eigen::Vector3f &grad)
{
    trilinearInterpolation(pos,dist,grad);
}

void MapCoreServer::esdfCallback(const sensor_msgs::PointCloud2ConstPtr &esdf_i)
{
    Timer t;

    //                std::cout << "esdf; " << t.elapsed_ms()  <<std::endl;

    for (sensor_msgs::PointCloud2ConstIterator<float> itx(*esdf_i, "x"), iti(*esdf_i, "intensity");
         itx != itx.end(); ++itx, ++iti) {
        esdf_[buffer_->get_idx(Eigen::Vector3i(itx[0],itx[1],itx[2]))] = static_cast<float>(iti[0]);
    }
    //        publish_esdf_slice();
    //                std::cout << "esdf; " << t.elapsed_ms()  <<std::endl;


}

void MapCoreServer::mapCallback(const sensor_msgs::PointCloud2ConstPtr &map_i)
{

    Timer t;
    //    pcl::PointCloud<pcl::PointXYZI> cloud;
    //    pcl::fromROSMsg(*map_i,cloud);
    //    for(size_t i = 0; i < cloud.size(); ++i){
    //        map_state_[5] = static_cast<uint8_t>(cloud.points[i]._PointXYZI::intensity);
    ////        map_state_[_buffer->get_idx(cloud.points[i].getArray3fMap().cast<int>())] = static_cast<uint8_t>(cloud.points[i]._PointXYZI::intensity);
    //    }
    //                std::cout << "map; " << t.elapsed_ms() <<std::endl;

    int idx ;
    for (sensor_msgs::PointCloud2ConstIterator<float> itx(*map_i, "x");
         itx != itx.end(); ++itx) {

        idx = buffer_->get_idx(Eigen::Vector3i(itx[0],itx[1],itx[2]));
        if(((map_state_[idx]&(~(1u << 2))) == VoxelBuffer::unknown) && ((static_cast<uint8_t>(itx[3])&(~(1u << 2))) != VoxelBuffer::unknown)){
            score_+=resolution3_;
        }
//        if(map_state_[idx] & VoxelBuffer::frontier_flag){
//            map_state_[idx] = static_cast<uint8_t>(itx[3]) | VoxelBuffer::frontier_flag;
//        }else{
//            map_state_[idx] = static_cast<uint8_t>(itx[3]);
//        }
        map_state_[idx] = static_cast<uint8_t>(itx[3]);

        buffer_->at(idx) = map_state_[idx];
        buffer_->tsdf_[idx] = itx[4];
        buffer_->weight_[idx] = itx[5];

    }


    if(score_ > 5){
        if(t_.elapsed_ms() > delta_insertion_){
            scores_.push_back(std::pair<float,float>(scores_[scores_.size()-1].first+t_.elapsed_ms(),score_));
            std::cout << "Time - Volume coverage: " << scores_[scores_.size()-1].first << "(ms) - " << score_ << std::endl;
            t_.restart();
        }
    }else{
        t_.restart();
    }

//        std::cout << "map0; " << t.elapsed_ms() <<std::endl;

//        publish_occ();

    //    //debuf visu
    //        std::cout << "map1; " << t.elapsed_ms() <<std::endl;
}
void MapCoreServer::frontierCallback(const sensor_msgs::PointCloud2ConstPtr &frontier_i)
{

    Timer t;
    //                std::cout << "frontier; " << t.elapsed_ms() <<std::endl;

    frontierManager_->lock();
    for (sensor_msgs::PointCloud2ConstIterator<float> itx(*frontier_i, "x"), iti(*frontier_i, "intensity");
         itx != itx.end(); ++itx, ++iti) {
        int idx = buffer_->get_idx(Eigen::Vector3i(itx[0],itx[1],itx[2]));

        if(iti[0] > 0.5f){
            frontierManager_->insertFrontier(Eigen::Vector3i(itx[0],itx[1],itx[2]));
            map_state_[idx] |= VoxelBuffer::frontier_flag;
        }else{

            frontierManager_->removeFrontier(Eigen::Vector3i(itx[0],itx[1],itx[2]));
            map_state_[idx] &= ~(1u << 2);

        }
        buffer_->at(idx) = map_state_[idx];
    }
    //                std::cout << "frontier; " << t.elapsed_ms() <<std::endl;
    //    std::cout << frontierManager_->frontier_list_.size() << std::endl;
    //    std::cout << "frontier; " << t.elapsed_ms() <<std::endl;

//    publish_frontier();

    frontierManager_->release();
    //        std::cout << "frontier; " << t.elapsed_ms() <<std::endl;

}

void MapCoreServer::publish_frontier()
{


    if(static_cast<int>(frontierManager_->frontier_list_.size()) == 0)
        return;


    Eigen::Vector3f ref_pt;
    sensor_msgs::PointCloud2 cloud_o;


    int count = 0;
    sensor_msgs::PointCloud2Modifier modifier(cloud_o);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(frontierManager_->frontier_list_.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_o, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_o, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_o, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(cloud_o, "intensity");



    for (FrontierManager::frontierIterator it = frontierManager_->frontier_list_.begin();
         it != frontierManager_->frontier_list_.end() ; ++it) {
        size_t idx = get_idx(*it);
        //        if(frontierManager_->frontier_features_[idx].planarity < 0.0001){
        //            continue;
        //        }
        buffer_->convert(*it,ref_pt);
        (*it_x) = ref_pt.x();
        (*it_y) = ref_pt.y();
        (*it_z) = ref_pt.z();
        (*it_i) = frontierManager_->frontier_features_[idx].planarity;
        ++it_x;
        ++it_y;
        ++it_z;
        ++it_i;
        count++;
    }
    if(count > 0){

        modifier.resize(count);

        cloud_o.width = count;
        cloud_o.height = 1;
        cloud_o.is_dense = true;
        cloud_o.header.frame_id = "world";
        cloud_o.header.stamp = ros::Time::now();


        frontier_pub_.publish(cloud_o);

    }

}

void MapCoreServer::publish_occ()
{


    Eigen::Vector3f ref_pt;
    sensor_msgs::PointCloud2 cloud_o;

    int count = 0;
    sensor_msgs::PointCloud2Modifier modifier(cloud_o);
    modifier.resize(buffer_->_size.x()*buffer_->_size.y()*buffer_->_size.z());
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(buffer_->_size.x()*buffer_->_size.y());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_o, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_o, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_o, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(cloud_o, "intensity");


    for(size_t x = 0; x<buffer_->_size.x(); ++x){
        for(size_t y = 0; y<buffer_->_size.y(); ++y){
//            for(size_t z = 0;z< buffer_->_size.z(); ++z){
                size_t z= 12;
                const Eigen::Vector3i pos(x,y,z);


                if(buffer_->tsdf_[buffer_->get_idx(pos)]>-1. && buffer_->tsdf_[buffer_->get_idx(pos)]<1.){

//                if ((map_state_[buffer_->get_idx(pos)] & VoxelBuffer::occupied_flag)) {
                    buffer_->convert(pos,ref_pt);
                    (*it_x) = ref_pt.x();
                    (*it_y) = ref_pt.y();
                    (*it_z) = ref_pt.z();
                    ++it_x;
                    ++it_y;
                    ++it_z;
                    (*it_i) = buffer_->weight_[buffer_->get_idx(pos)];
                    ++it_i;

                    count++;
                }
//            }
        }
    }
    modifier.resize(count);

    cloud_o.width = count;
    cloud_o.height = 1;
    cloud_o.is_dense = true;
    cloud_o.header.frame_id = "world";
    cloud_o.header.stamp = ros::Time::now();

    if(count > 0){

        occ_pub_.publish(cloud_o);

    }


}

void MapCoreServer::publish_esdf_slice()
{

    Eigen::Vector3f ref_pt;



    sensor_msgs::PointCloud2 cloud_o;


    int count = 0;
    sensor_msgs::PointCloud2Modifier modifier(cloud_o);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(buffer_->_size.x()*buffer_->_size.y());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_o, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_o, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_o, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(cloud_o, "intensity");

    Eigen::Vector3i slice_pos_z;
    buffer_->getVoxelPos(Eigen::Vector3f(0,0,1.2),slice_pos_z);
    int z = slice_pos_z.z();
    for(size_t x = 0; x<buffer_->_size.x(); ++x){
        for(size_t y = 0; y<buffer_->_size.y(); ++y){

            const Eigen::Vector3i pos(x,y,z);

            buffer_->convert(pos,ref_pt);
            (*it_x) = ref_pt.x();
            (*it_y) = ref_pt.y();
            (*it_z) = ref_pt.z();
            ++it_x;
            ++it_y;
            ++it_z;
            if(esdf_[buffer_->get_idx(pos)] > 1.){
                (*it_i) = 1.;

            }else{
                (*it_i) = esdf_[buffer_->get_idx(pos)];
            }
            ++it_i;

            count++;
        }
    }

    if(count > 0){

        modifier.resize(count);

        cloud_o.width = count;
        cloud_o.height = 1;
        cloud_o.is_dense = true;
        cloud_o.header.frame_id = "world";
        cloud_o.header.stamp = ros::Time::now();


        esdf_pub_.publish(cloud_o);

    }

}

void MapCoreServer::logExploration()
{

    if(save_path_.size()<2){
        std::cout << "[MapCoreServer] Volume explored: bad directory path "<<std::endl;
    }
    std::ofstream outdata; // outdata is like cin
    DIR *dp;
    int i = 0;
    struct dirent *ep;

    dp = opendir (save_path_.c_str());

    if (dp != NULL)
    {
        while (ep = readdir (dp))
            i++;

        (void) closedir (dp);
    }
    else
        perror ("[MapCoreServer] Volume explored: Couldn't open the directory");

    i-=1;

    std::string file = save_path_;
    file+= "/result_"+std::to_string(i)+".txt";

    std::cout <<"[MapCoreServer] Volume Explored log to file " << file<<std::endl;
    outdata.open(file); // opens the file
    if( !outdata ) { // file couldn't be opened
        std::cerr << "[MapCoreServer] Volume Explored: Error: file could not be opened" << std::endl;
        return;
    }

    for (i=0; i<scores_.size(); ++i){
        outdata << scores_[i].first << " " << scores_[i].second << std::endl;
    }
    outdata.close();

}
