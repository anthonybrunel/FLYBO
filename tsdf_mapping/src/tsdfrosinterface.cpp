#include "tsdfrosinterface.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>

TSDFRosInterface::TSDFRosInterface(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):
    nh_(nh),nh_private_(nh_private)
{
    has_finish = true;
    std::cout << "[TSDFRosInterface] loading" <<std::endl;

    bool useFov,useRealSensor;
    double max_distance;
    nh_private.param("tsdf_mapping/useFov",useFov,false);
    nh_private.param("tsdf_mapping/useRealSensor",useRealSensor,false);
    nh_private.param("tsdf_mapping/max_range",max_distance,5.);
    max_distance +=0.3;
    int width,height;
    double fx,fy,ppx,ppy,fovx,fovy;
    nh_private.param("tsdf_mapping/width",width,640);
    nh_private.param("tsdf_mapping/heigh",height,480);
    nh_private.param("tsdf_mapping/fx",fx,461.453);
    nh_private.param("tsdf_mapping/fy",fy,460.859);
    nh_private.param("tsdf_mapping/ppx",ppx,344.191);
    nh_private.param("tsdf_mapping/ppy",ppy,245.387);
    nh_private.param("tsdf_mapping/fovx",fovx,90.);
    nh_private.param("tsdf_mapping/fovy",fovy,60.);
    std::string save_folder;
    nh_private.param("tsdf_mapping/save_folder",save_folder,std::string(""));

    int block_count = 50000;

    //3cm and 15cm
    float voxel_size = 0.03f;//low res make denser and noisy mesh with hole, high res reduce noise
    float sdf_trunc = 0.09f;//high number remove hole
    //    float voxel_size = 0.1f;//low res make denser and noisy mesh with hole, high res reduce noise
    //    float sdf_trunc = 0.3f;//high number remove hole

    nh_private.param("tsdf_mapping/tsdf_voxel_size",voxel_size,voxel_size);

    nh_private.param("tsdf_mapping/sdf_trunc",sdf_trunc,sdf_trunc);

    //block_resolution * voxel_size * 0.5
    // Device
    std::string device_code = "cuda:0";

    core::Device device(device_code);

    tsdf_map_.reset(new TSDFVoxelGridEvaluator({{"tsdf", core::Dtype::Float32},
                                                {"weight", core::Dtype::UInt16},
                                                {"color", core::Dtype::UInt16}},voxel_size, sdf_trunc, 16,
                                               block_count, device,save_folder));

    if(useFov){
        tsdf_map_->initCameraFromFoV(width,fovx,fovy,max_distance);
    }else{
        tsdf_map_->initCameraFromFoV(width,fovx,fovy,max_distance);
    }


    depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/tsdf_mapping/depth",10);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/tsdf_mapping/odometry",10);
    sync_depth_odom_ = new message_filters::Synchronizer<sync_image_odo_t>(sync_image_odo_t(2),*depth_sub_,*odom_sub_);
    sync_depth_odom_->registerCallback(boost::bind(&TSDFRosInterface::depthOdomCalback,this, _1, _2));



    surface_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/tsdf_mapping/occ_cloud", 10);
    mesh_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tsdf_mapping/mesh", 10);

}



void TSDFRosInterface::depthOdomCalback(const sensor_msgs::ImageConstPtr &depth_msg_i, const nav_msgs::OdometryConstPtr &odom_msg_i)
{
    if(!has_odom){
        save_timer.restart();
    }

    has_odom = true;
    Eigen::Matrix4f transform;
    transform.setIdentity();
    //convert ros coordinate system to vision/open3d coordinate system

    Eigen::Matrix3f R(Eigen::Quaternionf(odom_msg_i->pose.pose.orientation.w,odom_msg_i->pose.pose.orientation.x, odom_msg_i->pose.pose.orientation.y, odom_msg_i->pose.pose.orientation.z));

    transform.matrix().col(0).head(3) = -1.f*R.col(1);
    transform.matrix().col(1).head(3) = -1.f*R.col(2);
    transform.matrix().col(2).head(3) = R.col(0);

    transform.block(0,0,3,3).transposeInPlace();
    transform.matrix().col(3).head(3) = -transform.block(0,0,3,3)*Eigen::Vector3f(odom_msg_i->pose.pose.position.x,odom_msg_i->pose.pose.position.y,odom_msg_i->pose.pose.position.z);

    //    transform = transform.inverse();
    //    transform.matrix().col(3).head(3) = -1.f*transform.block(0,0,3,3)*transform.matrix().col(3).head(3);

    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(depth_msg_i, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    tsdf_map_->integrate((uint16_t*)cv_ptr->image.data,transform);

    if(save_timer.elapsed_ms()>save_every_ms){
        Timer t;
                save_pc_callback();
//                publish_mesh();
//        save_mesh_callback();
        std::cout << "Extract t:" << t.elapsed_ms() << std::endl;
    }
    //    Timer t;
    //        t::geometry::PointCloud surf_cloud = tsdf_map_->ExtractSurfacePoints();
    //    //    std::cout << "Integrate depth frame to tsdf" << std::endl;
    //    std::shared_ptr<::geometry::PointCloud> downsampled_cloud = surf_cloud.ToLegacyPointCloud().VoxelDownSample(0.04);

    ////    publicSurfaceCloud(tsdf_map_->ExtractSurfacePoints());
    //    std::cout << "tsdf extraction:" << t.elapsed_ms() << " " << downsampled_cloud->points_.size()  << std::endl;

}

void TSDFRosInterface::publicSurfaceCloud(const t::geometry::PointCloud &surf_cloud)
{

    if(surf_cloud.IsEmpty())
        return;

    std::shared_ptr<::geometry::PointCloud> downsampled_cloud = surf_cloud.ToLegacyPointCloud().VoxelDownSample(0.04);
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = "world";
    cloud.header.stamp = ros::Time::now();
    cloud.width  = surf_cloud.GetPoints().GetShape(0);
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false; // there may be invalid points
    //for fields setup
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    //    modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
    modifier.setPointCloud2FieldsByString(1,"xyz");
    //    modifier.resize(surf_cloud.GetPoints().GetShape(0));
    modifier.resize(downsampled_cloud->points_.size());

    //iterators
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
    //    sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud, "r");
    //    sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud, "g");
    //    sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud, "b");
    //GetDataPtr


    core::Tensor tensor_points ;
    //    if(!surf_cloud.GetPoints().IsContiguous()){
    //        tensor_points = surf_cloud.GetPoints().Contiguous().To(core::Device("CPU:0"),core::Dtype::Float32);
    //    }else{
    //        tensor_points = surf_cloud.GetPoints().To(core::Device("CPU:0"),core::Dtype::Float32);
    //    }
    //    cloud.data.resize(cloud.width*3);
    //    memcpy (&cloud.data[0], tensor_points.GetDataPtr(), surf_cloud.GetPoints().GetShape(0));


    for(size_t i = 0; i < cloud.width; ++i,++out_x,++out_y,++out_z){
        //        *out_x = *((float*)tensor_points.GetDataPtr()+i*3);
        //        *out_y = *((float*)tensor_points.GetDataPtr()+i*3+1);
        //        *out_z = *((float*)tensor_points.GetDataPtr()+i*3+2);
        *out_x = downsampled_cloud->points_[i].x();
        *out_y = downsampled_cloud->points_[i].y();
        *out_z = downsampled_cloud->points_[i].z();

        // store colors
        //        *out_r = r;
        //        *out_g = g;
        //        *out_b = b;


        //        ++out_r;
        //        ++out_g;
        //        ++out_b;
    }



    surface_cloud_pub_.publish(cloud);

}

void TSDFRosInterface::publish_mesh()
{
    ::geometry::TriangleMesh mesh = tsdf_map_->extractMesh().ToLegacyTriangleMesh();
    std::cout << "TRIANGLE MESH\n";
//    mesh.gettr
    const auto& triangles = mesh.triangles_;
    const auto& vertices = mesh.vertices_;
    visualization_msgs::MarkerArray markers;

    int id = 0;

//    mesh_pub_.publish(markers);
    visualization_msgs::Marker mk;
    mk.points.resize(triangles.size()*3);
    mk.colors.resize(triangles.size()*3);
    for(size_t j = 0; j < triangles.size() ;++j){

        mk.header.frame_id = "world";
        mk.header.stamp    = ros::Time::now();
        mk.type            = visualization_msgs::Marker::TRIANGLE_LIST;
        mk.id              = j ;


        mk.action             = visualization_msgs::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.lifetime = ros::Duration(0);
        geometry_msgs::Vector3 v;
        v.x = 1.;
        v.y = 1.;
        v.z = 1.;
        mk.scale = v;
        mk.color.a = 1;
        for(int i = 0;i<3;++i){
            geometry_msgs::Point p;

            p.x = vertices[triangles[j](i)].x();
            p.y = vertices[triangles[j](i)].y();
            p.z = vertices[triangles[j](i)].z();
            mk.points[j*3+i] = p;
        }

    }

    markers.markers.push_back(mk);
    mesh_pub_.publish(markers);

}


