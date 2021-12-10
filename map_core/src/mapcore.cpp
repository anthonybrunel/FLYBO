#include "map_core/mapcore.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

MapCore::MapCore()
{

}

MapCore::~MapCore()
{
    delete depth_sub_;
    delete sync_depth_odom_;
    delete odom_filter_sub_;
}

void MapCore::init(ros::NodeHandle &nh)
{
    nh.param("map_core/resolution", _resolution, 0.2f);
    nh.param("map_core/x_size", _map_size.x(), 25.f);
    nh.param("map_core/y_size", _map_size.y(), 25.f);
    nh.param("map_core/z_size", _map_size.z(), 5.f);
    nh.param("map_core/uav_radius", uav_radius_, 1.f);

    nh.param("map_core/x_agent_size", _agent_size.x(), 2.f);
    nh.param("map_core/y_agent_size", _agent_size.y(), 2.f);
    nh.param("map_core/z_agent_size", _agent_size.z(), 2.f);

    nh.param("map_core/origin_x", map_origin_.x(), 0.f);
    nh.param("map_core/origin_y", map_origin_.y(), 0.f);
    nh.param("map_core/origin_z", map_origin_.z(), 0.f);
    nh.param("map_core/frontier_mode", frontier_mode_, 0);//0 unknown is a frontier, 1 free is a frontier


    nh.param("map_core/vizu", _vizu, true);


    nh.param("map_core/frame_id", _frame_id, std::string("world"));
    _buffer.reset(new VoxelBuffer);


    _buffer->init(_map_size,map_origin_, _resolution);

    _buffer->print_infos();
    cam_.init(nh);



    _map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/map",10);
    _frontier_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/frontier", 10);



    _occ_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/occ_cloud", 10);
    _free_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/free_cloud", 10);
    _prob_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/prob_cloud", 10);
    _frontier_visu_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/frontier_visu_cloud", 10);
    _uk_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_core/unknown_cloud", 10);



    _pos = Eigen::Isometry3f::Identity();
    bool enableSdf = true;
    nh.param("map_core/active_esdf", enableSdf, enableSdf);
    if(enableSdf){
        std::cout << "[MapCore] EnableSdf";
        _esdfmap.reset(new EDFMap);
        _esdfmap->init(nh,_buffer.get(), _buffer->_size,_resolution,1.0);
    }

    depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/map_core/depth",1);
    odom_filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/map_core/odometry",1);

    sync_depth_odom_ = new message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Odometry>(*depth_sub_,*odom_filter_sub_,1);
    //        sync_depth_odom_->registerCallback(boost::bind(&MapCore::depthOdomCalbackFrustum,this, _1, _2));
    sync_depth_odom_->registerCallback(boost::bind(&MapCore::depthOdomCalbackRayCast,this, _1, _2));


    switch(frontier_mode_){
    case 0:
        std::cout << "[MapCore] Frontier is an UNKNOWN  volume neighboring an FREE volume \n";

        frontier_func = std::bind(&MapCore::frontier_mode0,this,std::placeholders::_1,std::placeholders::_2);
        break;
    case 1:
        std::cout << "[MapCore] Frontier is an FREE volume neighboring an UNKNOWN volume \n";
        frontier_func = std::bind(&MapCore::frontier_mode1,this,std::placeholders::_1,std::placeholders::_2);
        break;
    }


    publish_complete_map();
    std::cout << "[MapCore] Initialized" <<std::endl;
    t_since_start_.restart();

    grid_hash_.reset(new GridHash(nh));
}



void MapCore::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{

    odom_ = *odom;
    _pos.linear() = Eigen::Quaternionf(odom->pose.pose.orientation.w,
                                       odom->pose.pose.orientation.x,
                                       odom->pose.pose.orientation.y,
                                       odom->pose.pose.orientation.z).toRotationMatrix();


    _pos.translation().x() = odom->pose.pose.position.x;
    _pos.translation().y() = odom->pose.pose.position.y;
    _pos.translation().z() = odom->pose.pose.position.z;

    _tpos = odom->header.stamp;

}

//void MapCore::depthOdomCalbackFrustum(const sensor_msgs::ImageConstPtr &depth_msg_i, const nav_msgs::OdometryConstPtr &odom_msg_i)
//{
//    _has_cloud = true;

//    Timer t;
//    cv_bridge::CvImagePtr cv_ptr;
//    try{
//        cv_ptr = cv_bridge::toCvCopy(depth_msg_i, sensor_msgs::image_encodings::MONO16);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    odom_ = *odom_msg_i;
//    odomCallback(odom_msg_i);
//    cam_.update(odom_);
//    CameraFrustum::box b = cam_.computeBBox();

//    Eigen::Vector3f max(b.maxx_,b.maxy_,b.maxz_);
//    Eigen::Vector3f min(b.minx_,b.miny_,b.minz_);
//    Eigen::Vector3i max_grid,min_grid, odom_pos;
//    _buffer->getVoxelPos(min,min_grid);
//    _buffer->getVoxelPos(max,max_grid);
//    Eigen::Vector3f odom_posf;
//    odom_posf.x() = odom_.pose.pose.position.x;
//    odom_posf.y() = odom_.pose.pose.position.y;
//    odom_posf.z() = odom_.pose.pose.position.z;
//    _buffer->getVoxelPos(odom_posf,odom_pos);

//    Eigen::Vector3f center;
//    int size = std::abs(max_grid.x()-min_grid.x()) * std::abs(max_grid.y()-min_grid.y()) * std::abs(max_grid.z()-min_grid.z());
//    uint16_t * dist_image = (uint16_t*)cv_ptr->image.data;
//    uint_fast8_t flag;
//    //update map

//    Eigen::Vector3f ros_pt;
//    int min_dist = cam_.near_*1000;//[mm]
//    cell_to_update_.clear();
//    cell_to_update_.reserve(size);



//    //projection mapping
//    //compute tsdf and probabilities occupation with random sampling projection
//    float tmp_depth;
//    double tx = 0;
//    int cpt = 0;
//    Timer time;
//    Eigen::Vector2f pxls[8];
//    double d = 0;
//    for(int x = min_grid.x(); x <= max_grid.x(); ++x ){
//        for(int y = min_grid.y(); y <= max_grid.y(); ++y ){
//            for(int z = min_grid.z(); z <= max_grid.z(); ++z ){
//                Eigen::Vector3i pt_in_grid(x,y,z);

//                if(_buffer->isInsideBuf(pt_in_grid)){
//                    //                    for(int k =0; k < 32; ++k){
//                    _buffer->convert(pt_in_grid,center);
//                    //                    if(!(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag)){
//                    //                        if(!cam_.isOccluded(center,dist_image,_resolution,flag)){
//                    //                            _buffer->at(pt_in_grid) |= flag;
//                    //                            cell_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),flag));
//                    //                        }
//                    //                    }

//                    cam_.back_transform(center);
//                    size_t idx_buffer = _buffer->get_idx(pt_in_grid);
//                    uint8_t state;
//                    //                    cam_.reproj(,pxls[0]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,0.1,-0.1),pxls[1]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,-0.1,0.1),pxls[2]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,-0.1,-0.1),pxls[3]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,0.1,0.1),pxls[4]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,0.1,-0.1),pxls[5]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,-0.1,0.1),pxls[6]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,-0.1,-0.1),pxls[7]);

//                    bool occluded = false;
//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,-0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,-0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,-0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,-0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    if(!occluded){
//                        //                    if(cam_.retrieveClosestDepth(center, dist_image, _resolution, tmp_depth)){
//                        //                        if(_buffer->updateVoxelProbability(idx_buffer,tmp_depth,center,state)){
//                        //                            _buffer->at(idx_buffer) = state;
//                        //                            Eigen::Matrix<float,5,1> data;
//                        //                            data(0) = pt_in_grid.x();
//                        //                            data(1) = pt_in_grid.y();
//                        //                            data(2) = pt_in_grid.z();
//                        //                            data(3) = static_cast<float>(state);
//                        //                            data(4) = _buffer->prob_[idx_buffer];
//                        //                            cell_to_update_.push_back(data);
//                        //                        }
//                        //                    }

//                        if(!cam_.isOccluded(center,dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth)){

//                            if(_buffer->updateVoxelProbability(idx_buffer,tmp_depth,center,state)){
//                                _buffer->at(idx_buffer) = state;
//                                Eigen::Matrix<float,5,1> data;
//                                data(0) = pt_in_grid.x();
//                                data(1) = pt_in_grid.y();
//                                data(2) = pt_in_grid.z();
//                                data(3) = static_cast<float>(state);
//                                data(4) = _buffer->prob_[idx_buffer];
//                                cell_to_update_.push_back(data);
//                            }
//                        }
//                    }
//                }
//                //
//            }
//        }
//    }
//    std::cout << "t: " << time.elapsed_ms() << " " << d << std::endl;

//    //    return;
//    for(size_t i = 0; i < cam_.height_; ++i){
//        for(size_t j = 0; j < cam_.width_; ++j){
//            int idx = i*cam_.width_+j;
//            if(dist_image[idx] > min_dist){
//                float meas_depth = static_cast<float>(dist_image[idx])/1000.f;
//                cam_.deproj(Eigen::Vector2f(j,i),ros_pt,meas_depth);
//                Eigen::Vector3i pt_in_grid;
//                _buffer->getVoxelPos(ros_pt,pt_in_grid);
//                if(!_buffer->isInsideBuf(Eigen::Vector3i(pt_in_grid)))
//                    continue;
//                Eigen::Vector3f vox_center;
//                _buffer->convert(pt_in_grid,vox_center);
//                cam_.back_transform(vox_center);
//                size_t idx_buffer = _buffer->get_idx(pt_in_grid);
//                uint8_t state;
//                meas_depth = vox_center.norm();
//                //                if(!_buffer->updateVoxOccProbability(idx_buffer,state))
//                //                    continue;
//                //all obstacle will be considered as occupied if there is outlier it will be removed in next observation

//                if(_buffer->_flag_buffer[idx_buffer] & VoxelBuffer::occupied_flag)
//                    continue;
//                _buffer->_flag_buffer[idx_buffer] |= VoxelBuffer::occupied_flag;
//                //                _buffer->_flag_buffer[idx_buffer] |= VoxelBuffer::visited_flag;
//                //

//                _buffer->at(idx_buffer) = state;
//                Eigen::Matrix<float,5,1> data;
//                data(0) = pt_in_grid.x();
//                data(1) = pt_in_grid.y();
//                data(2) = pt_in_grid.z();
//                data(3) = static_cast<float>(VoxelBuffer::occupied_flag);
//                data(4) = _buffer->prob_[idx_buffer];
//                cell_to_update_.push_back(data);

//                //                cell_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),VoxelBuffer::occupied_flag/*+VoxelBuffer::visited_flag*/));
//                _buffer->updateVoxOccProbability(idx_buffer,state);
//                //                grid_hash_->insertPoint(ros_pt);

//                //                if(!inflate_){
//                //                    for(int x = -1; x<=1; ++x){
//                //                        for(int y = -1; y<=1; ++y){
//                //                            for(int z = -1; z<=1; ++z){
//                //                                if(!_buffer->isInsideBuf(Eigen::Vector3i(pt_in_grid+Eigen::Vector3i(x,y,z)))
//                //                                        )
//                //                                    continue;
//                //                                size_t idx = _buffer->get_idx(pt_in_grid+Eigen::Vector3i(x,y,z));
//                //                                if(_buffer->_flag_buffer[idx] & VoxelBuffer::occupied_flag)
//                //                                    continue;
//                //                                cell_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x()+x,pt_in_grid.y()+y,pt_in_grid.z()+z,VoxelBuffer::occupied_flag+VoxelBuffer::visited_flag));
//                //                                _buffer->_flag_buffer[idx] |= VoxelBuffer::occupied_flag+VoxelBuffer::visited_flag;

//                //                            }
//                //                        }
//                //                    }
//                //                }


//            }
//        }
//    }







//    //Map free space:

//    for(int x = min_grid.x(); x <= max_grid.x(); ++x ){
//        for(int y = min_grid.y(); y <= max_grid.y(); ++y ){
//            for(int z = min_grid.z(); z <= max_grid.z(); ++z ){
//                Eigen::Vector3i pt_in_grid(x,y,z);

//                if(_buffer->isInsideBuf(pt_in_grid)){
//                    //                    for(int k =0; k < 32; ++k){
//                    _buffer->convert(pt_in_grid,center);
//                    //                    if(!(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag)){
//                    //                        if(!cam_.isOccluded(center,dist_image,_resolution,flag)){
//                    //                            _buffer->at(pt_in_grid) |= flag;
//                    //                            cell_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),flag));
//                    //                        }
//                    //                    }

//                    cam_.back_transform(center);
//                    size_t idx_buffer = _buffer->get_idx(pt_in_grid);
//                    uint8_t state;
//                    //                    cam_.reproj(,pxls[0]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,0.1,-0.1),pxls[1]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,-0.1,0.1),pxls[2]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,-0.1,-0.1),pxls[3]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,0.1,0.1),pxls[4]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,0.1,-0.1),pxls[5]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,-0.1,0.1),pxls[6]);
//                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,-0.1,-0.1),pxls[7]);

//                    bool occluded = false;
//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,-0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,-0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,-0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,-0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

//                    if(!occluded){
//                        //                    if(cam_.retrieveClosestDepth(center, dist_image, _resolution, tmp_depth)){
//                        //                        if(_buffer->updateVoxelProbability(idx_buffer,tmp_depth,center,state)){
//                        //                            _buffer->at(idx_buffer) = state;
//                        //                            Eigen::Matrix<float,5,1> data;
//                        //                            data(0) = pt_in_grid.x();
//                        //                            data(1) = pt_in_grid.y();
//                        //                            data(2) = pt_in_grid.z();
//                        //                            data(3) = static_cast<float>(state);
//                        //                            data(4) = _buffer->prob_[idx_buffer];
//                        //                            cell_to_update_.push_back(data);
//                        //                        }
//                        //                    }

//                        if(!cam_.isOccluded(center,dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth)){

//                            if(_buffer->updateVoxelProbability(idx_buffer,tmp_depth,center,state)){
//                                _buffer->at(idx_buffer) = state;
//                                Eigen::Matrix<float,5,1> data;
//                                data(0) = pt_in_grid.x();
//                                data(1) = pt_in_grid.y();
//                                data(2) = pt_in_grid.z();
//                                data(3) = static_cast<float>(state);
//                                data(4) = _buffer->prob_[idx_buffer];
//                                cell_to_update_.push_back(data);
//                            }
//                        }
//                    }
//                }
//                //
//            }
//        }
//    }



//    //    std::cout << cell_to_update_.size() << " " << size <<std::endl;

//    frontier_to_update_.clear();
//    frontier_to_update_.reserve(size);

//    frontier_func(min_grid,max_grid);

//    //    //Frontier updater
//    //    auto isFrontier =[this](const Eigen::Vector3i& neight_pt) {
//    //        if(_buffer->isInsideBuf(neight_pt)){
//    //            if((_buffer->at(neight_pt) & VoxelBuffer::occupied_flag))
//    //                return 2;
//    //            if(_buffer->at(neight_pt) & VoxelBuffer::visited_flag){
//    //                return 1;
//    //            }
//    //        }
//    //        return 0;
//    //    };

//    //    for(int x = min_grid.x(); x <= max_grid.x(); ++x ){
//    //        for(int y = min_grid.y(); y <= max_grid.y(); ++y ){
//    //            for(int z = min_grid.z(); z <= max_grid.z(); ++z ){
//    //                Eigen::Vector3i pt_in_grid(x,y,z);

//    //                if(_buffer->isInsideBuf(pt_in_grid)){
//    //                    if(!(_buffer->at(pt_in_grid) & VoxelBuffer::visited_flag) && !(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag)){
//    //                        uint8_t res = 0;
//    //                        res |= isFrontier(pt_in_grid +Eigen::Vector3i(1,0,0));
//    //                        res |= isFrontier(pt_in_grid +Eigen::Vector3i(-1,0,0));
//    //                        res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,1,0));
//    //                        res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,-1,0));
//    //                        res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,0,1));
//    //                        res |= isFrontier(pt_in_grid +Eigen::Vector3i(0,0,-1));

//    //                        if((res & 2) == 2 || res == 0){
//    //                            if(_buffer->at(pt_in_grid) & VoxelBuffer::frontier_flag){
//    //                                frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),0));
//    //                                _buffer->at(pt_in_grid) &= ~(1u << 2);//clear frontier in box
//    //                            }
//    //                            continue;
//    //                        }else{
//    //                            _buffer->at(pt_in_grid) |= VoxelBuffer::frontier_flag;
//    //                            frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),1));
//    //                        }
//    //                    }
//    //                    else if(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag || _buffer->at(pt_in_grid) & VoxelBuffer::visited_flag){
//    //                        frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),0));
//    //                        _buffer->at(pt_in_grid) &= ~(1u << 2);//clear frontier in box
//    //                    }

//    //                }

//    //            }
//    //        }
//    //    }
//    if(t_since_start_.elapsed_ms() < 10000)
//        tag_free_around(uav_radius_);
//    else{
//        tag_free_around(0.3);
//    }
//    publish_map();

//    //    std::cout <<"Insertion time: " << t.elapsed_ms() << " " << cell_to_update_.size() << std::endl;


//}

void MapCore::depthOdomCalbackRayCast(const sensor_msgs::ImageConstPtr &depth_msg_i, const nav_msgs::OdometryConstPtr &odom_msg_i)
{
    _has_cloud = true;

    Timer t;
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(depth_msg_i, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    odom_ = *odom_msg_i;
    odomCallback(odom_msg_i);
    cam_.update(odom_);
    CameraFrustum::box b = cam_.computeBBox();

    Eigen::Vector3f max(b.maxx_,b.maxy_,b.maxz_);
    Eigen::Vector3f min(b.minx_,b.miny_,b.minz_);
    Eigen::Vector3i max_grid,min_grid, odom_pos;
    _buffer->getVoxelPos(min,min_grid);
    _buffer->getVoxelPos(max,max_grid);
    Eigen::Vector3f odom_posf;
    odom_posf.x() = odom_.pose.pose.position.x;
    odom_posf.y() = odom_.pose.pose.position.y;
    odom_posf.z() = odom_.pose.pose.position.z;
    _buffer->getVoxelPos(odom_posf,odom_pos);

    Eigen::Vector3f center;
    int size = std::abs(max_grid.x()-min_grid.x()) * std::abs(max_grid.y()-min_grid.y()) * std::abs(max_grid.z()-min_grid.z());
    uint16_t * dist_image = (uint16_t*)cv_ptr->image.data;
    uint_fast8_t flag;
    //update map

    std::vector<bool> integrate_list(_buffer->tsdf_.size(),false);
    Eigen::Vector3f ros_pt,cam_pt;
    int min_dist = cam_.near_*1000;//[mm]
    cell_to_update_.clear();
    cell_to_update_.reserve(size);

    uint16_t d_mm = 0;
    //raycast test
    for(size_t i = 0; i < cam_.height_; ++i){
        for(size_t j = 0; j < cam_.width_;++j){
            int idx = i*cam_.width_+j;
            d_mm = dist_image[idx];
            //black pixel is added as free space ray
            if(dist_image[idx] < 10){
                continue;
            }
            cam_.deproj(Eigen::Vector2f(j,i),ros_pt,static_cast<float>(d_mm)/1000.f);
            cam_.deproj_relative(Eigen::Vector2f(j,i),cam_pt,static_cast<float>(d_mm)/1000.f);

            Eigen::Vector3i pt_in_grid;
            _buffer->getVoxelPos(ros_pt,pt_in_grid);

            fast_voxel_trasversal_integrate(odom_posf,ros_pt,cam_pt,integrate_list);

        }
    }


    float tmp_depth;

    //update occ visible and front
    for(int x = min_grid.x(); x <= max_grid.x(); ++x ){
        for(int y = min_grid.y(); y <= max_grid.y(); ++y ){
            for(int z = min_grid.z(); z <= max_grid.z(); ++z ){
                Eigen::Vector3i pt_in_grid(x,y,z);

                if(_buffer->isInsideBuf(pt_in_grid)){
                    //                    for(int k =0; k < 32; ++k){
                    _buffer->convert(pt_in_grid,center);
                    //                    if(!(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag)){
                    //                        if(!cam_.isOccluded(center,dist_image,_resolution,flag)){
                    //                            _buffer->at(pt_in_grid) |= flag;
                    //                            cell_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),flag));
                    //                        }
                    //                    }

                    cam_.back_transform(center);
                    size_t idx_buffer = _buffer->get_idx(pt_in_grid);
                    uint8_t state;
                    state = _buffer->tsdfState(idx_buffer);
                    //                    cam_.reproj(,pxls[0]);
                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,0.1,-0.1),pxls[1]);
                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,-0.1,0.1),pxls[2]);
                    //                    cam_.reproj(center-Eigen::Vector3f(0.1,-0.1,-0.1),pxls[3]);
                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,0.1,0.1),pxls[4]);
                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,0.1,-0.1),pxls[5]);
                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,-0.1,0.1),pxls[6]);
                    //                    cam_.reproj(center-Eigen::Vector3f(-0.1,-0.1,-0.1),pxls[7]);

                    if(integrate_list[idx_buffer]){
                        Eigen::Matrix<float,5,1> data;
                        data(0) = pt_in_grid.x();
                        data(1) = pt_in_grid.y();
                        data(2) = pt_in_grid.z();
                        state &= ~(1u << 2);
                        data(3) = static_cast<float>(state);


                        data(4) = _buffer->tsdf_[idx_buffer];
                        cell_to_update_.push_back(data);

                    }else{
                        bool occluded = false;
                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,-0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(0.05,-0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,-0.05,0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);

                        occluded = occluded || cam_.isOccluded(center-Eigen::Vector3f(-0.05,-0.05,-0.05),dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth);


                        if(!occluded){
                            if(!cam_.isOccluded(center,dist_image,_resolution,flag,_buffer->_resolution*6,tmp_depth)){

                                if(_buffer->updateTsdfProjection(idx_buffer,state,tmp_depth,center.x())){

                                    Eigen::Matrix<float,5,1> data;
                                    data(0) = pt_in_grid.x();
                                    data(1) = pt_in_grid.y();
                                    data(2) = pt_in_grid.z();
                                    state &= ~(1u << 2);
                                    data(3) = state;
                                    _buffer->at(idx_buffer) = state;

                                    data(4) = _buffer->tsdf_[idx_buffer];
                                    cell_to_update_.push_back(data);
                                }
                            }

                        }
                    }
                }
                //
            }
        }
    }


    frontier_to_update_.clear();
    frontier_to_update_.reserve(size);
    if(t_since_start_.elapsed_ms() < 10000)
        tag_free_around(uav_radius_);
    else{
        tag_free_around(0.2);
    }

    //Frontier updater with frustum bbox
    frontier_func(min_grid,max_grid);
    //    frontierAsFree(min_grid,max_grid);



    publish_map();
    if(_esdfmap)
        _esdfmap->noneBlockingUpdate();//TRY TO UPDATE THE EDT MAP AND PUBLISH IT

    //        std::cout << "[MAPCORE]: t=" << t.elapsed_ms() <<std::endl;

}



void MapCore::fast_voxel_trasversal_integrate(const Eigen::Vector3f &pos, const Eigen::Vector3f &goal, const Eigen::Vector3f &goal_c, std::vector<bool> &integrate_list)
{

    if(!_buffer->is_inside(goal)){
        return;
    }
    Eigen::Matrix<int,3,1> start;
    Eigen::Matrix<int,3,1> end;

    Eigen::Matrix<float,3,1> startf;
    Eigen::Matrix<float,3,1> endf;

    Eigen::Vector3f ray = (goal-pos).normalized();
    startf = _buffer->toGridCoordinateSystem(goal-ray*(_buffer->_resolution*2+0.1));
    endf = _buffer->toGridCoordinateSystem(goal+ray*_buffer->_resolution*2);
    start = startf.array().floor().cast<int>();
    end = endf.array().floor().cast<int>();




    if(!_buffer->isInsideBuf(end)){
        Eigen::Vector3f res;
        if(!_buffer->clampGridRay(startf,ray,res)){
            return;
        }
        endf = res;
        end = (endf).cast<int>();

    }



    //    _buffer->getVoxelPos(startf,end);
    //    _buffer->getVoxelPos(endf,start);
    Eigen::Vector3i keep_start = start;

    float resolution = _buffer->_resolution;
    float half_res = _buffer->_resolution*0.5f;
    float step[3],tMax[3],tDelta[3];


    for(int i = 0; i < 3; ++i){
        if (ray[i] > 0.0f) {
            step[i] = 1;
            tDelta[i] =  1.f / ray[i];
            tMax[i] = (start[i]+1-startf[i])/ray[i];
        } else if (ray[i] < 0.0f) {
            step[i] = -1;
            tDelta[i] = 1.f / -ray[i];
            tMax[i] = (startf[i]-start[i])/(-ray[i]);
        } else {
            step[i] = 0;
            tDelta[i] = 0;
            tMax[i] = FLT_MAX;
        }


        //                std::cout << tMax[i] << " " << tDelta[i]  << std::endl;

    }
    Eigen::Vector3i vox;
    Eigen::Vector3f center(0,0,0);
    size_t idx_buff;
    uint8_t state;

    while(end != start) {

        if (tMax[0] < tMax[1] && tMax[0] < tMax[2]) {
            start[0] += step[0];
            tMax[0] += tDelta[0];

        } else if (tMax[1] < tMax[2]) {
            start[1] += step[1];
            tMax[1] += tDelta[1];
        } else {
            start[2] += step[2];
            tMax[2] += tDelta[2];
        }

        //        std::cout << start.transpose() << std::endl;
        vox << start[0], start[1], start[2];
        if(!_buffer->isInsideBuf(vox)){
            //                        std::cout << "no inside wtf" << std::endl;
            //                        std::cout << pos.transpose() << std::endl;
            //                        std::cout << goal.transpose() << std::endl;

            //                        std::cout << ray.transpose() << std::endl;
            //                        std::cout << (endf-startf).normalized().transpose() << std::endl;

            //                        std::cout << startf.transpose() << std::endl;
            //                        std::cout << endf.transpose() << std::endl;
            //                        std::cout << start.transpose() << std::endl;
            //                        std::cout << end.transpose() << std::endl;
            //                        std::cout << tMax[0] << " " << tDelta[0]  << std::endl;
            //                        std::cout << tMax[1] << " " << tDelta[1]  << std::endl;
            //                        std::cout << tMax[2] << " " << tDelta[2]  << std::endl;
            return;
        }

        _buffer->convert(vox,center);
        idx_buff = _buffer->get_idx(vox);
        cam_.back_transform(center);
        if(_buffer->updateVoxelProbability(idx_buff,goal_c.z(),center,state)){
            _buffer->at(idx_buff) = state;


            integrate_list[idx_buff] = true;
        }
    }



}

void MapCore::tag_free_around(float clearance = 0.1)
{
    //check occlusion pt <2m
    //min height max height (ground)
    Eigen::Vector3i pose_in_grid;
    _buffer->getVoxelPos(_pos.translation(),pose_in_grid);
    float inv_radius = clearance*_buffer->_inv_resolution;
    Eigen::Vector3i step = (_buffer->_inv_resolution * Eigen::Vector3f(clearance,clearance,clearance)+Eigen::Vector3f(0.5,0.5,0.5)).cast<int>();
    for(int x = -step.x(); x<=step.x(); ++x)   {
        for(int y = -step.y(); y<=step.y(); ++y)   {
            for(int z = -step.z(); z<=step.z(); ++z)   {
                if(sqrt(x*x+y*y+z*z) > inv_radius )
                    continue;
                Eigen::Vector3i pt_in_grid = pose_in_grid+Eigen::Vector3i(x,y,z);

                if(_buffer->isInsideBuf(pt_in_grid)){
                    size_t idx_buffer = _buffer->get_idx(pt_in_grid);
                    _buffer->at(idx_buffer) &= ~(1u << 0);//remove occupied cell
                    _buffer->at(idx_buffer) |= VoxelBuffer::visited_flag ;// add visited

                    uint8_t state_tmp;
                    _buffer->updateTsdfFree(idx_buffer);
                    //                    _buffer->updateVoxFreeProbability(idx_buffer,state_tmp);
                    Eigen::Matrix<float,5,1> data;
                    data(0) = pt_in_grid.x();
                    data(1) = pt_in_grid.y();
                    data(2) = pt_in_grid.z();
                    data(3) = _buffer->at(idx_buffer)&~(1u << 2);//remove frontier (will be update later
                    data(4) = _buffer->tsdf_[idx_buffer];
                    cell_to_update_.push_back(data);
                }
            }
        }
    }
}

void MapCore::frontierAsUnknown(const Eigen::Vector3i &bbox_min, const Eigen::Vector3i &bbox_max)
{
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

    for(int x = bbox_min.x(); x <= bbox_min.x(); ++x ){
        for(int y = bbox_min.y(); y <= bbox_min.y(); ++y ){
            for(int z = bbox_min.z(); z <= bbox_min.z(); ++z ){
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

void MapCore::frontierAsFree(const Eigen::Vector3i &bbox_min, const Eigen::Vector3i &bbox_max)
{
    auto isFrontier =[this](const Eigen::Vector3i& neight_pt) {
        if(_buffer->isInsideBuf(neight_pt)){
            if((_buffer->at(neight_pt) & VoxelBuffer::occupied_flag))
                return 2;
            if(!_buffer->at(neight_pt)){
                return 1;
            }
        }
        return 0;
    };

    for(int x = bbox_min.x()-2; x <= bbox_max.x()+2; ++x ){
        for(int y = bbox_min.y()-2; y <= bbox_max.y()+2; ++y ){
            for(int z = bbox_min.z()-2; z <= bbox_max.z()+2; ++z ){
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

                        if((res & 2) == 2 || !res){
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
                    else if(_buffer->at(pt_in_grid) & VoxelBuffer::occupied_flag){
                        frontier_to_update_.push_back(Eigen::Vector4i(pt_in_grid.x(),pt_in_grid.y(),pt_in_grid.z(),0));
                        _buffer->at(pt_in_grid) &= ~(1u << 2);//clear frontier in box
                    }

                }

            }
        }
    }

}

void MapCore::publish_map()
{
    if(!_has_cloud)
        return;
    pcl::PointXYZI pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZI> cloud;


    pcl::PointCloud<pcl::PointXYZI> cloud_frontier;


    cloud.resize(cell_to_update_.size());
    cloud_frontier.resize(frontier_to_update_.size());

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = _frame_id;


    cloud_frontier.width = cloud_frontier.points.size();
    cloud_frontier.height = 1;
    cloud_frontier.is_dense = true;
    cloud_frontier.header.frame_id = _frame_id;


    for(size_t i = 0; i < frontier_to_update_.size(); ++i){
        pt.getVector3fMap() = frontier_to_update_[i].head(3).cast<float>();
        pt._PointXYZI::intensity = frontier_to_update_[i](3);
        cloud_frontier.points[i] = pt;
    }


    if(cloud_frontier.points.size() > 0){
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_frontier, cloud_msg);
        _frontier_pub.publish(cloud_msg);
    }


    if(cell_to_update_.size() > 0){
        sensor_msgs::PointCloud2 map_update_msg;

        sensor_msgs::PointCloud2Modifier modifier(map_update_msg);
        modifier.setPointCloud2Fields(6,"x",1,sensor_msgs::PointField::FLOAT32,
                                      "y",1,sensor_msgs::PointField::FLOAT32,
                                      "z",1,sensor_msgs::PointField::FLOAT32,
                                      "s",1,sensor_msgs::PointField::FLOAT32,
                                      "p",1,sensor_msgs::PointField::FLOAT32,
                                      "w",1,sensor_msgs::PointField::FLOAT32);

        modifier.resize(cell_to_update_.size());
        sensor_msgs::PointCloud2Iterator<float> it_x(map_update_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(map_update_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(map_update_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_s(map_update_msg, "s");
        sensor_msgs::PointCloud2Iterator<float> it_p(map_update_msg, "p");
        sensor_msgs::PointCloud2Iterator<float> it_w(map_update_msg, "w");


        int cpt = 0;
        for(size_t i = 0; i < cell_to_update_.size(); ++i){
            cpt++;
            (*it_x) =  cell_to_update_[i](0);
            (*it_y) =  cell_to_update_[i](1);
            (*it_z) =  cell_to_update_[i](2);
            (*it_s) =  cell_to_update_[i](3);
            (*it_p) =  cell_to_update_[i](4);
            size_t idx_buffer = _buffer->get_idx(cell_to_update_[i].head(3).cast<int>());
            (*it_w) =  _buffer->weight_[idx_buffer];


            ++it_x;
            ++it_y;
            ++it_z;
            ++it_p;
            ++it_s;
            ++it_w;
        }
        _map_pub.publish(map_update_msg);
    }







}

void MapCore::publish_complete_map()
{
    pcl::PointXYZI pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZI> cloud;



    int count = 0;
    cloud.reserve(_buffer->_size.x()*_buffer->_size.y()*_buffer->_size.z());
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers_check;

    for(size_t x = 0; x<_buffer->_size.x(); ++x){
        for(size_t y = 0; y<_buffer->_size.y(); ++y){
            for(size_t z = 0;z< _buffer->_size.z(); ++z){
                const Eigen::Vector3i pos(x,y,z);


                //                _buffer->convert(pos,ref_pt);
                pt.getVector3fMap() = pos.cast<float>();
                pt._PointXYZI::intensity = static_cast<float>(_buffer->at(pos));
                cloud.points.push_back(pt);

                count++;
            }
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = _frame_id;



    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    _map_pub.publish(cloud_msg);

}

void MapCore::publish_occ()
{
    if(!_has_cloud)
        return;
    pcl::PointXYZI pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZI> cloud;



    int count = 0;
    cloud.reserve(_buffer->_size.x()*_buffer->_size.y()*_buffer->_size.z());
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> frontiers_check;

    for(size_t x = 0; x<_buffer->_size.x(); ++x){
        for(size_t y = 0; y<_buffer->_size.y(); ++y){
            for(size_t z = 0;z< _buffer->_size.z(); ++z){
                const Eigen::Vector3i pos(x,y,z);

                if ((_buffer->at(pos) & VoxelBuffer::occupied_flag)) {
                    _buffer->convert(pos,ref_pt);
//                    if(ref_pt.z()> 5.)
//                        continue;

                    pt.getVector3fMap() = ref_pt;
                    pt._PointXYZI::intensity = ref_pt.z();
                    cloud.points.push_back(pt);

                    count++;
                }
            }
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = _frame_id;


    if(count > 0){

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        _occ_pub.publish(cloud_msg);

    }

}



void MapCore::publish_free()
{
    if(!_has_cloud)
        return;
    pcl::PointXYZ pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZ> cloud;



    int count = 0;
    cloud.reserve(_buffer->_size.x()*_buffer->_size.y()*_buffer->_size.z());
    for(size_t x = 0; x<_buffer->_size.x(); ++x){
        for(size_t y = 0; y<_buffer->_size.y(); ++y){
            for(size_t z = 0;z< _buffer->_size.z(); ++z){
                const Eigen::Vector3i pos(x,y,z);


                if (_buffer->at(pos) & VoxelBuffer::visited_flag) {
                    _buffer->convert(pos,ref_pt);
                    pt.getVector3fMap() = ref_pt;
                    cloud.points.push_back(pt);
                    count++;
                }
            }
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = _frame_id;


    if(count > 0){

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        _free_pub.publish(cloud_msg);

    }

}


void MapCore::publish_frontier()
{
    if(!_has_cloud)
        return;
    pcl::PointXYZ pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZ> cloud;



    int count = 0;
    cloud.reserve(_buffer->_size.x()*_buffer->_size.y()*_buffer->_size.z());
    for(size_t x = 0; x<_buffer->_size.x(); ++x){
        for(size_t y = 0; y<_buffer->_size.y(); ++y){
            for(size_t z = 0;z< _buffer->_size.z(); ++z){
                const Eigen::Vector3i pos(x,y,z);


                if ((_buffer->at(pos) & VoxelBuffer::frontier_flag)) {
                    _buffer->convert(pos,ref_pt);
                    pt.getVector3fMap() = ref_pt;
                    cloud.points.push_back(pt);
                    count++;
                }
            }
        }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = _frame_id;


    if(count > 0){

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        _frontier_visu_pub.publish(cloud_msg);

    }

}

void MapCore::publish_prob()
{
    if(!_has_cloud)
        return;
    pcl::PointXYZI pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZI> cloud;



    int count = 0;
    cloud.reserve(_buffer->_size.x()*_buffer->_size.y()*_buffer->_size.z());
    for(size_t x = 0; x<_buffer->_size.x(); ++x){
        for(size_t y = 0; y<_buffer->_size.y(); ++y){
//            for(size_t z = 0;z< _buffer->_size.z(); ++z){
            size_t z = 12;
                const Eigen::Vector3i pos(x,y,z);

                if(_buffer->tsdf_[_buffer->get_idx(pos)]>-1. && _buffer->tsdf_[_buffer->get_idx(pos)]<1.){

                    _buffer->convert(pos,ref_pt);

                    pt.getVector3fMap() = ref_pt;
                    pt._PointXYZI::intensity = _buffer->weight_[_buffer->get_idx(pos)];
                    cloud.points.push_back(pt);
                    count++;
                }
//            }
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = _frame_id;

    if(count > 0){

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        _prob_pub.publish(cloud_msg);

    }

}

void MapCore::publish_unknown()
{
    if(!_has_cloud)
        return;
    pcl::PointXYZ pt;
    Eigen::Vector3f ref_pt;

    pcl::PointCloud<pcl::PointXYZ> cloud;



    int count = 0;
    cloud.reserve(_buffer->_size.x()*_buffer->_size.y()*_buffer->_size.z());
    for(size_t x = 0; x<_buffer->_size.x(); ++x){
        for(size_t y = 0; y<_buffer->_size.y(); ++y){
            for(size_t z = 0;z< _buffer->_size.z(); ++z){
                const Eigen::Vector3i pos(x,y,z);


                if ((_buffer->at(pos) == VoxelBuffer::unknown)) {
                    _buffer->convert(pos,ref_pt);
                    pt.getVector3fMap() = ref_pt;
                    cloud.points.push_back(pt);
                    count++;
                }
            }
        }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = _frame_id;

    if(count > 0){

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        _uk_pub.publish(cloud_msg);

    }
}

void MapCore::publish_scan()
{
    grid_hash_->publishScan();
}
