#include "simulatecamera.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>




SimulateCamera::SimulateCamera()
{

}

SimulateCamera::SimulateCamera(const ros::NodeHandle &nh_i, const ros::NodeHandle &nh_private_i):nh_(nh_i),nh_private_(nh_private_i)
{
    nh_i.param("/camera/frame_id", frame_id_, std::string("world"));
//    cloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/camera/cloud",10);

    freespace_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/camera/freespace_pointcloud",10);
    //focal from fovy and resolution
    //focal from fovy and fovx and horizontal resolution
    //focal from fx,fy,ppx,ppy, resolution (realistic
    nh_private_.param("/depth_cam_node/camera/max_range",far_,5.f);

    bool useFov;
    nh_private_.param("/depth_cam_node/camera/useFov",useFov,false);
    if(useFov){
        std::cout << "[VulkanCamera] Use fov simulate camera: " <<std::endl;

        nh_private_.param("/depth_cam_node/camera/width",width_,640);
        nh_private_.param("/depth_cam_node/camera/fovx",fovx_,75.f);
        nh_private_.param("/depth_cam_node/camera/fovy",fovy_,60.f);
        float tanx = tanf((fovx_*M_PI/180.f/2.));
        //        if(fovx_ > 90.f){
        //            tanx = M_PI-tanx;
        //        }

        height_ = ((float)width_/((tanx)/tan(fovy_*M_PI/180/2.)))+0.5;
        focal_ = (float)height_/(2.*tanf((fovy_*M_PI/180.)/2.));
        ppx_ = (float)width_/2.f;
        ppy_ = (float)height_/2.f;

    }
    else{
        std::cout << "[VulkanCamera] Use default simulate camera: " <<std::endl;

        focal_ = height_/(2.*tanf((fovy_*M_PI/180.)/2.));

        ppx_ = (float)width_/2.f;
        ppy_ = (float)height_/2.f;
    }
    noisy_far_ = 0.0024*far_*far_;


    std::cout << "[Vulkan-SimulateCamera] fovx;fovy: " << fovx_ << ";" << fovy_  <<std::endl
     << "[Vulkan-SimulateCamera] Res [width,height]: " << width_ <<"x" << height_ <<std::endl
     << "[Vulkan-SimulateCamera] Max range: " << far_ <<std::endl
     << "[Vulkan-SimulateCamera] Focal (fx,fy): " << focal_ << "," << focal_ <<std::endl
    << "[Vulkan-SimulateCamera] Center (ppx,ppy): " << ppx_ << "," << ppy_ <<std::endl;


}

void SimulateCamera::send_depth(const float *depth_i,const nav_msgs::Odometry& odom_i)
{
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2 free;
    current_odom_ = odom_i;
    cloud.header.stamp = odom_i.header.stamp;
    free.header.stamp = odom_i.header.stamp;
//    depth_to_pc2(depth_i,cloud,free);
//    cloud_pub_.publish(cloud);
//    freespace_pub_.publish(free);

}



void SimulateCamera::depth_to_pc2(const float *depth_i, sensor_msgs::PointCloud2 &cloud_o, sensor_msgs::PointCloud2 &free_o)
{

    int size = height_*width_;
    sensor_msgs::PointCloud2Modifier modifier(cloud_o);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(size);
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_o, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_o, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_o, "z");


    sensor_msgs::PointCloud2Modifier modifier2(free_o);
    modifier2.setPointCloud2FieldsByString(1,"xyz");
    modifier2.resize(size);
    sensor_msgs::PointCloud2Iterator<float> it2_x(free_o, "x");
    sensor_msgs::PointCloud2Iterator<float> it2_y(free_o, "y");
    sensor_msgs::PointCloud2Iterator<float> it2_z(free_o, "z");


    int cpt = 0;
    int cpt2 = 0;
    int idx = 0;
    Eigen::Matrix3f R(Eigen::Quaternionf(current_odom_.pose.pose.orientation.w,current_odom_.pose.pose.orientation.x,
                                         current_odom_.pose.pose.orientation.y,current_odom_.pose.pose.orientation.z));
    Eigen::Vector3f t(current_odom_.pose.pose.position.x,current_odom_.pose.pose.position.y,current_odom_.pose.pose.position.z);


    for(int h=0; h<height_; ++h){
        for(int w = 0; w<width_; ++w){

            int idx = h*width_+w;
            const float &d = depth_i[idx];
            float z;
            float x;
            float y;
            z = linearize_depth(d,near_,far_);
            y = (static_cast<float>(h-ppy_)*z)/focal_;
            x = (static_cast<float>(w-ppx_)*z)/focal_;
            Eigen::Vector3f pts(z,-x,-y);


            float norm = pts.norm();
            //free cloud

            if(depth_i[idx] >=1.0f || norm > far_  || norm < 0.001f){
                cpt2++;
                pts.x() = 1.;
                pts.z() = -(static_cast<float>(h-ppy_))/focal_;
                pts.y() = -(static_cast<float>(w-ppx_))/focal_;
                pts = pts.normalized()*(far_+0.5);
                pts = R*pts+t;
                x=pts.x();
                y=pts.y();
                z=pts.z();
                (*it2_x) =  x;
                (*it2_y) =  y;
                (*it2_z) =  z;
                ++it2_x;
                ++it2_y;
                ++it2_z;

                cpt++;
                //from vulkan coordinate system to ros coord syst

                if(!applyTransform_){
                    x = 9;
                    z = -(static_cast<float>(h-ppy_)*x)/focal_;
                    y = -(static_cast<float>(w-ppx_)*x)/focal_;
                }
                (*it_x) =  x;
                (*it_y) =  y;
                (*it_z) =  z;
                ++it_x;
                ++it_y;
                ++it_z;
            }else{

                cpt++;
                //from vulkan coordinate system to ros coord syst

                if(applyTransform_){
                    pts = R*pts+t;
                    x=pts.x();
                    y=pts.y();
                    z=pts.z();

                }else{
                    x = linearize_depth(d,near_,far_);
                    z = -(static_cast<float>(h-ppy_)*x)/focal_;
                    y = -(static_cast<float>(w-ppx_)*x)/focal_;
                }
                (*it_x) =  x;
                (*it_y) =  y;
                (*it_z) =  z;
                ++it_x;
                ++it_y;
                ++it_z;
            }
        }
    }

    modifier.resize(cpt);
    cloud_o.height = 1;
    cloud_o.width = cpt;
    cloud_o.is_dense = true;
    cloud_o.header.frame_id = "world";
    cloud_o.header.stamp = current_odom_.header.stamp;

    modifier2.resize(cpt2);
    free_o.height = 1;
    free_o.width = cpt2;
    free_o.header.frame_id = "world";

}

