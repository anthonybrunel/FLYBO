#ifndef SIMULATECAMERA_H
#define SIMULATECAMERA_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <random>
class SimulateCamera
{
public:
    SimulateCamera();
    SimulateCamera(const ros::NodeHandle &nh_i,const ros::NodeHandle &nh_private_i);



    void send_depth(const float *depth_i, const nav_msgs::Odometry &odom_i);

    void depth_to_pc2(const float *depth_i, sensor_msgs::PointCloud2 &cloud_o, sensor_msgs::PointCloud2 &free_o);

    inline float linearize_depth(float d,float zNear,float zFar)
    {
        return zNear * zFar / (zFar + d * (zNear - zFar));
    }

    inline float nonlinear_depth(float linearDepth,float zNear,float zFar)
    {
        float nonLinearDepth = (zFar + zNear - 2.0 * zNear * zFar / linearDepth) / (zFar - zNear);
        nonLinearDepth = (nonLinearDepth + 1.0) / 2.0;
        return nonLinearDepth;
    }


    inline void linearize_depth_buffer(float *depth){
        for(int i = 0; i < height_*width_; ++i){
            if(depth[i] >= 1.)
                depth[i] = 0.f;
            else
                depth[i] = linearize_depth(depth[i],near_,far_);
        }

    }
    //Lukas Schmid, Michael Pantic, Raghav Khanna, Lionel Ott, Roland Siegwart, and Juan Nieto,
    //"An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments",
    //in IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 1500-1507, April 2020
    //https://github.com/ethz-asl/unreal_cv_ros
    inline void add_z_noise(float* depth){

        const int N = width_*height_;
        float new_depth[N];
        for (int y = 0; y <height_;++y){
            for (int x = 0; x <width_;++x){

                distribution.param(std::normal_distribution<float>::param_type(0.f, 0.5f));
                int noisy_x = static_cast<int>(roundf(x + distribution(generator_)));
                int noisy_y = static_cast<int>(roundf(y + distribution(generator_)));
//                noisy_x = x;
//                noisy_y = y;


                noisy_x = std::min(std::max(noisy_x,0),width_-1);
                noisy_y = std::min(std::max(noisy_y,0),height_-1);

                int i = y*width_ + x;
                int j = noisy_y*width_ + noisy_x;
                if(depth[j] >= near_){
                    float sigma_mu = 0.0024f*depth[j]*depth[j];
                    distribution.param(std::normal_distribution<float>::param_type(sigma_mu,sigma_mu));
                    new_depth[i] = depth[j] + distribution(generator_);
                }else if(depth[i] >= near_){
                    float sigma_mu = 0.0024f*depth[i]*depth[i];

                    distribution.param(std::normal_distribution<float>::param_type(sigma_mu,sigma_mu));
                    new_depth[i] = depth[i] + distribution(generator_);
                }else{
                    new_depth[i] = depth[i];
                    continue;
                }


//                distribution.param(std::normal_distribution<float>::param_type(depth[i], 0.05f));
//                if(depth[i] >= far_-0.0001f){
//                    depth[i] = 0;
//                }

            }


        }
        memcpy(depth,new_depth,sizeof(new_depth));
        // z scaling gaussian noise see https://arxiv.org/pdf/1909.09548.pdf
        //        for(int i = 0; i < height_*width_; ++i){
        //            if(depth[i] < near_ || depth[i] > (far_-noisy_far_)){
        //                continue;
        //            }


        //            distribution.param(std::normal_distribution<float>::param_type(depth[i], 0.0024f*depth[i]*depth[i]));
        //            depth[i] = distribution(generator_);
        //        }

    }

    inline float compute_z_noise(float depth){
        distribution.param(std::normal_distribution<float>::param_type(depth, 0.0024f*depth*depth));
        return distribution(generator_);

    }

    std::default_random_engine generator_;
    std::normal_distribution<float> distribution;


    int width_ = 640;
    int height_ = 480;

    float ppx_ = (float)width_/2.f;
    float ppy_ = (float)height_/2.f;

    float focal_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher cloud_pub_;
    ros::Publisher freespace_pub_;

    nav_msgs::Odometry current_odom_;

    std::string cloud_topic_ = "/camera/depth_cloud";
    std::string frame_id_ = "world";

    float fovy_ = 60.f;//deg
    float fovx_ = 75.f;//deg
    float near_=0.001f;
    float far_ = 5.f;
    float noisy_far_;
    float min_dist_ = 0.01;

    bool applyTransform_ = true;

};

#endif // SIMULATECAMERA_H
