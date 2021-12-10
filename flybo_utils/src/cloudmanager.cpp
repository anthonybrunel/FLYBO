#include "../include/cloud_extraction/cloudmanager.h"

#include <cv_bridge/cv_bridge.h>

void CloudManager::depthOdomCalback(const sensor_msgs::ImageConstPtr &depth_msg_i, const nav_msgs::OdometryConstPtr &odom_msg_i)
{
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
    Eigen::Matrix4d T;
    T.block(0,0,3,3) = Eigen::Quaternion<double>(
                odom_msg_i->pose.pose.orientation.w,
                odom_msg_i->pose.pose.orientation.x,
                odom_msg_i->pose.pose.orientation.y,
                odom_msg_i->pose.pose.orientation.z

                ).toRotationMatrix();
    T.col(3).head(3) = Eigen::Vector3d(odom_msg_i->pose.pose.position.x,
                         odom_msg_i->pose.pose.position.y,
                         odom_msg_i->pose.pose.position.z);


    uint16_t * dist_image = (uint16_t*)cv_ptr->image.data;
    //update map


    uint16_t d_mm = 0;
    //raycast test
    open3d::geometry::PointCloud cloud;
    cloud.points_.reserve(_cam.height_*_cam.width_);
    for(size_t i = 0; i < _cam.height_; ++i){
        for(size_t j = 0; j < _cam.width_;++j){
            int idx = i*_cam.width_+j;
            d_mm = dist_image[idx];
            //black pixel is added as free space ray
            if(dist_image[idx] < 1){
                continue;
            }
            Eigen::Vector3d pts = _cam.deproj(Eigen::Vector2i(j,i),static_cast<float>(d_mm)/1000.f,T);
            if(!isInside(pts)){
                continue;
            }
            _grid.insertPoint(pts.cast<float>());
            cloud.points_.push_back( pts);
        }
    }
    *_grid.pcd += cloud;
    _grid.pcd = _grid.pcd->VoxelDownSample(0.1);
    std::cout << "Current number of pts: " << _grid.count << std::endl;
    std::cout << "[CloudManager] Cloud insertion: " << t.elapsed_ms() << "(ms)" << std::endl;
}
