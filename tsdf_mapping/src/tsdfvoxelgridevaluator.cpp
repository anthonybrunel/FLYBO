#include "../include/tsdfvoxelgridevaluator.h"
#include <Eigen/Core>
#include <iostream>

#include "../utils/timer.hpp"
#include <fstream>


#include <string>
#include <iostream>
#include <filesystem>
#include "dirent.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

TSDFVoxelGridEvaluator::TSDFVoxelGridEvaluator(std::unordered_map<std::string, core::Dtype> attr_dtype_map, float voxel_size, float sdf_trunc, int64_t block_resolution, int64_t block_count, const core::Device &device, const std::string &save_folder):
    TSDFVoxelGrid(attr_dtype_map,voxel_size,sdf_trunc,block_resolution,block_count,device), save_folder_(save_folder){
    std::cout << "[TSDFVoxelGridEvaluator]: Custom initialization" << std::endl;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Error);

    directory_number = generate_file_number(save_folder_);
    if (directory_number >=0){
        save_folder_ = save_folder_+"/meshes_"+std::to_string(directory_number);
    }
}

TSDFVoxelGridEvaluator::~TSDFVoxelGridEvaluator()
{
    //swap to cpu and extract the final mesh
//    this->To(Device("CPU:0"));

    auto mesh = ExtractSurfaceMesh(-1,weight_tresh).ToLegacyTriangleMesh();
    save_mesh(mesh);
//    save_tsdf(save_folder_+"/tsdf_pcd_"+std::to_string(num));
}

int TSDFVoxelGridEvaluator::generate_file_number(const std::string &path) const
{
    if(path.length() < 3){
        return -1;
    }
    std::ofstream outdata; // outdata is like cin
    DIR *dp;
    int i = 0;
    struct dirent *ep;

    dp = opendir (path.c_str());

    if (dp != NULL)
    {
        while (ep = readdir (dp))
            i++;

        (void) closedir (dp);
    }
    else{
        std::cout << "[TSDF]: could not open directory: " << path << std::endl;;

        return -1;
    }

    i-=1;

    return i;
}

void TSDFVoxelGridEvaluator::save_mesh(const std::string &path_filename,geometry::TriangleMesh &mesh)
{

    ::t::geometry::TriangleMesh test;
    mesh.RemoveDuplicatedVertices();
    mesh.RemoveDuplicatedTriangles();
    mesh.RemoveDegenerateTriangles();
//    mesh_legacy->SimplifyVertexClustering()
    open3d::io::WriteTriangleMesh(path_filename+".ply",
                                  mesh);
}

void TSDFVoxelGridEvaluator::save_mesh(geometry::TriangleMesh &mesh)
{
    struct stat st = {0};

    if(directory_number >= 0){
        Timer t;
        if (stat(save_folder_.c_str(), &st) == -1) {
            std::cout << "[TSDF]: creating directory at " << save_folder_ << std::endl;
            mkdir(save_folder_.c_str(), 0700);
        }

        int num = generate_file_number(save_folder_);
        std::string path = (save_folder_+"/mesh_"+std::to_string(num));


        save_mesh(path,mesh);
        std::cout << "[TSDF]: saving mesh to: " << path  << " time spent for saving: " << t.elapsed_ms() << std::endl;



    }else{
    }
}

void TSDFVoxelGridEvaluator::save_pc(const std::string &path_filename, geometry::PointCloud &pc)
{

    open3d::io::WritePointCloudToPLY(path_filename+".ply",
                                  pc,open3d::io::WritePointCloudOption());
}

void TSDFVoxelGridEvaluator::save_pc(geometry::PointCloud &pc)
{
    struct stat st = {0};

    if(directory_number >= 0){
        Timer t;
        if (stat(save_folder_.c_str(), &st) == -1) {
            std::cout << "[TSDF]: creating directory at " << save_folder_ << std::endl;
            mkdir(save_folder_.c_str(), 0700);
        }

        int num = generate_file_number(save_folder_);
        std::string path = (save_folder_+"/mesh_"+std::to_string(num));


        save_pc(path,pc);
        std::cout << "[TSDF]: saving mesh to: " << path  << " time spent for saving: " << t.elapsed_ms() << std::endl;



    }else{
    }
}

void TSDFVoxelGridEvaluator::save_tsdf(const std::string &path_filename)
{


    ::t::io::WritePointCloud(path_filename+".ply",getTsdf());
}

TSDFVoxelGridEvaluator::TSDFVoxelGridEvaluator():
    TSDFVoxelGrid(){


    std::cout << "[TSDFVoxelGridEvaluator]: Default initialization" << std::endl;
}


void TSDFVoxelGridEvaluator::integrate(const uint16_t *img,const Eigen::Matrix4f &extrinsic)
{

    std::vector<uint16_t> c(cam_.height_*cam_.width_*3,255);

    Tensor depth_tensor = Tensor(img,{cam_.height_, cam_.width_},
                                 Dtype::UInt16,
                                 device_);

    //    ::t::geometry::Image depth(depth_tensor);
    //    ::t::geometry::Image color(color_tensor);
    extrinsic_ = ::eigen_converter::EigenMatrixToTensor(extrinsic).To(device_);
    try {
        //        this->Integrate(depth_tensor,color_tensor,intrinsic_,extrinsic_,1000.0f,cam_.max_distance_);
        this->Integrate(depth_tensor,intrinsic_,extrinsic_,1000.0f,cam_.max_distance_);

    } catch (std::runtime_error e) {
//        std::cout << e.what() << std::endl;
    }

//    Timer t;
//    ExtractSurfacePoints();
//    t.print_timer_ms();
}



::t::geometry::PointCloud TSDFVoxelGridEvaluator::getTsdf()
{
    //need to implement a gpu version which compare the gt with estimate
    //transfert data from gpu to cpu is very slow maybe we shall extract every tsdf voxel on gpu and bring it back on cpu
    Tensor indices;

    struct ColoredVoxel16i {
        float tsdf;
        uint16_t weight;

        uint16_t r;
        uint16_t g;
        uint16_t b;
    };
    Timer t;
    //,
    core::Tensor active_addrs,block_values,block_keys;
    block_hashmap_->GetActiveIndices(active_addrs);
    block_values = block_hashmap_->GetValueTensor().To(Device("CPU:0"));
    block_keys = block_hashmap_->GetKeyTensor().To(Device("CPU:0"));;
    active_addrs = active_addrs.To(Device("CPU:0"), core::Dtype::Int64);
    int64_t resolution = block_resolution_;
    int64_t resolution3 = resolution*resolution*resolution;

    const int64_t* indices_ptr =
            static_cast<const int64_t*>(active_addrs.GetDataPtr());
    //    const void* keys_ptr = const_cast<void*>(block_keys.GetDataPtr());
    int64_t total_count=0,iter_count=0;



    for(int i = 0; i<active_addrs.GetLength();++i){
        int64_t block_idx = indices_ptr[i];

        //block_keys.GetDtype().ByteSize()*block_keys.GetDtype().ByteSize()*block_keys.GetShape()[1] -> int32,int32,int32 x,y,z

        for(int64_t xv = 0; xv<resolution; ++xv){
            for(int64_t yv = 0; yv<resolution; ++yv){
                for(int64_t zv = 0; zv<resolution; ++zv){

                    ColoredVoxel16i *vox_ptr = static_cast<ColoredVoxel16i*>(static_cast<void*>(
                                                                                 static_cast<uint8_t*>(block_values.GetDataPtr()) +
                                                                                 (((zv * resolution + yv) * resolution + xv) * resolution +block_idx) * sizeof (ColoredVoxel16i)));
                    if(vox_ptr == nullptr)
                        continue;
                    if(vox_ptr->weight >= weight_tresh){
                        total_count++;
                        //                            std::cout << vox_ptr->tsdf*sdf_trunc_  << " " << (int)vox_ptr->weight << std::endl;
                        //                            std::cout << block_values.GetShape().ToString() << std::endl;
                    }

                }
            }
        }
    }
    Tensor points, sdf_values;
    points = core::Tensor({total_count, 3}, core::Dtype::Float32,
                          block_values.GetDevice());
    sdf_values = core::Tensor({total_count, 1}, core::Dtype::Float32,
                              block_values.GetDevice());

    for(int i = 0; i<active_addrs.GetLength();++i){
        int64_t block_idx = indices_ptr[i];

        //block_keys.GetDtype().ByteSize()*block_keys.GetDtype().ByteSize()*block_keys.GetShape()[1] -> int32,int32,int32 x,y,z
        int * block_key_ptr = static_cast<int*>(static_cast<void*>(
                                                    static_cast<uint8_t*>(block_keys.GetDataPtr()) + block_idx*12));


        int64_t xb = block_key_ptr[0];
        int64_t yb = block_key_ptr[1];
        int64_t zb = block_key_ptr[2];

        for(int64_t xv = 0; xv<resolution; ++xv){
            for(int64_t yv = 0; yv<resolution; ++yv){
                for(int64_t zv = 0; zv<resolution; ++zv){

                    ColoredVoxel16i *vox_ptr = static_cast<ColoredVoxel16i*>(static_cast<void*>(
                                                                                 static_cast<uint8_t*>(block_values.GetDataPtr()) +
                                                                                 (((zv * resolution + yv) * resolution + xv) * resolution +block_idx) * sizeof (ColoredVoxel16i)));
                    if(vox_ptr == nullptr)
                        continue;
                    if(vox_ptr->weight >= weight_tresh){
                        int64_t x = xb * resolution + xv;
                        int64_t y = yb * resolution + yv;
                        int64_t z = zb * resolution + zv;
                        float* point_ptr = static_cast<float*>(static_cast<void*>(
                                                                   static_cast<uint8_t*>(points.GetDataPtr()) + iter_count*12));

                        float* sdf_ptr = static_cast<float*>(static_cast<void*>(
                                                                 static_cast<uint8_t*>(points.GetDataPtr()) + iter_count*12));

                        point_ptr[0] = x;
                        point_ptr[1] = y;
                        point_ptr[2] = z;
                        sdf_ptr[0] = vox_ptr->tsdf*sdf_trunc_;
                        iter_count++;

                    }

                }
            }
        }

    }

    auto pcd = ::t::geometry::PointCloud(points);
    pcd.SetPointAttr("sdf",sdf_values);
    t.print_timer_ms();

    return pcd;
}
