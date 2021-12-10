#include "map_core/voxelbuffer.h"

VoxelBuffer::VoxelBuffer()
{

}

void VoxelBuffer::init(const Eigen::Vector3f& size, const Eigen::Vector3f &origin_i, float resolution)
{
    _resolution = resolution;

    _inv_resolution = std::ceil(1./resolution);
    _size.x() =  size.x()*_inv_resolution+1;
    _size.y() =  size.y()*_inv_resolution+1;
    _size.z() =  size.z()*_inv_resolution+1;
    _origin = ((_size.cast<float>()/2.).cast<float>()).cast<int>();
    _min_grid_bound = -_origin+(origin_i*_inv_resolution).cast<int>();
    _max_grid_bound = _origin+(origin_i*_inv_resolution).cast<int>();
    _min_real_bound = _min_grid_bound.cast<float>()*_resolution;
    _max_real_bound = _max_grid_bound.cast<float>()*_resolution;

    _origin -= (origin_i*_inv_resolution).cast<int>();
    _originf = _origin.cast<float>();




    _flag_buffer.resize(_size(0)*_size(1)*_size(2));
    prob_.resize(_size(0)*_size(1)*_size(2),0.f);
    std::fill(_flag_buffer.begin(), _flag_buffer.end(), uint8_t(0));


    tsdf_.resize(_size(0)*_size(1)*_size(2),-2);

    weight_.resize(_size(0)*_size(1)*_size(2),0.f);

    if(resolution < 0.15){
        sdf_trunc = resolution*3;
        sdf_occ = 0.333;
    }
    else {
        sdf_trunc = resolution+0.3;
        sdf_occ = 0.4;
    }
}

void VoxelBuffer::insert_flag(const Eigen::Vector3i &pos, const u_int8_t flag)
{
    at(pos) |= flag;
}






