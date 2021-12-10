#pragma once

#include <cmath>
#include <open3d/Open3D.h>
#include <memory>

class CamUtils{
public:
    CamUtils(){

    }


    inline void computeParametersFronFoV(int width, double fovx_deg, double fovy_deg, double max_distance = 5.){
        fovx_deg_ = fovx_deg;
        fovy_deg_ = fovy_deg;
        fovx_rad_ = fovx_deg*M_PI/180.;
        fovy_rad_ = fovy_deg*M_PI/180.;
        width_ = width;
        double tanx = tan((fovx_rad_/2.));
        max_distance_ = max_distance;

        height_ = (width_/((tanx)/tan(fovy_rad_/2.)))+0.5;
        double ratio = (double) width_ / (double) height_;
        ppx_ = width_/2.;
        ppy_ = height_/2.;

        fx_ =static_cast<double>(height_)/(2.*tan(fovy_rad_/2.));
        fy_ = fx_;
        std::cout << "[TSDFMapping] fovx;fovy: " << fovx_deg << ";" << fovy_deg  <<std::endl
         << "[TSDFMapping] Res [width,height]: " << width <<"x" << height_ <<std::endl
         << "[TSDFMapping] Max range: " << max_distance_ <<std::endl
         << "[TSDFMapping] Focal (fx,fy): " << fx_ << "," << fy_ <<std::endl
        << "[TSDFMapping] Center (ppx,ppy): " << ppx_ << "," << ppy_ <<std::endl;


    }

    int width_,height_;
    double fx_,fy_,ppx_,ppy_,fovx_deg_,fovy_deg_,fovx_rad_,fovy_rad_;


    double max_distance_ = 5.;

    Eigen::Matrix4d intrinsic_;

};


using namespace open3d;
using namespace open3d::core;

class TSDFVoxelGridEvaluator : public t::geometry::TSDFVoxelGrid{
public:
    typedef std::shared_ptr<TSDFVoxelGridEvaluator> Ptr;
    TSDFVoxelGridEvaluator();

    TSDFVoxelGridEvaluator(std::unordered_map<std::string, core::Dtype> attr_dtype_map,
                  float voxel_size = 3.0 / 512.0, /* in meter */
                  float sdf_trunc = 0.04,         /*  in meter  */
                  int64_t block_resolution = 16, /*  block Tensor resolution  */
                  int64_t block_count = 1000,
                           const core::Device &device = core::Device("CPU:0"),const std::string &save_folder = "");
    ~TSDFVoxelGridEvaluator();

    inline void initCameraFromFoV(const int width, const double fovx_deg, const double fovy_deg, const double max_distance){
        cam_.computeParametersFronFoV(width,fovx_deg,fovy_deg,max_distance);

        intrinsic_ = Tensor(
                std::vector<float>({static_cast<float>(cam_.fx_), 0,
                                    static_cast<float>(cam_.ppx_), 0,
                                    static_cast<float>(cam_.fy_),
                                    static_cast<float>(cam_.ppy_), 0,
                                    0, 1}),
                {3, 3}, Dtype::Float32).To(device_);
    }

    //save data for evaluation
    int generate_file_number(const std::string &path) const;

    void save_mesh(const std::string &path_filename, geometry::TriangleMesh &mesh);
    void save_mesh(geometry::TriangleMesh &mesh);

    void save_pc(const std::string &path_filename, geometry::PointCloud &pc);
    void save_pc(geometry::PointCloud &pc);


    void save_tsdf(const std::string &path_filename);
    const ::t::geometry::TriangleMesh extractMesh(){

        return this->ExtractSurfaceMesh(-1,weight_tresh);
    }

    const ::t::geometry::PointCloud extractPointCloud(){

        return this->ExtractSurfacePoints(-1,weight_tresh,SurfaceMaskCode::VertexMap | SurfaceMaskCode::NormalMap);
    }

    void integrate(const uint16_t *img, const  Eigen::Matrix4f &extrinsic);

    t::geometry::PointCloud getTsdf();

private:
    Tensor intrinsic_;
    Tensor extrinsic_;

    CamUtils cam_;

    std::string save_folder_ = "";

    int directory_number = -1;
    float weight_tresh = 1.f; // for tsdf/mesh extraction
    //pointcloud
};
