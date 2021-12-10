#ifndef VOXELBUFFER_H
#define VOXELBUFFER_H
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <memory>

#include "../utils/boundingmap.hpp"

class VoxelBuffer
{
public:
    typedef std::shared_ptr<VoxelBuffer> Ptr;
    //0 unknown
    //1 occupied
    //2 visited
    static const uint8_t unknown = 0;
    static const uint8_t occupied_flag = (1 << 0);
    static const uint8_t visited_flag = (1 << 1);
    static const uint8_t frontier_flag = (1 << 2);

    VoxelBuffer();

    void print_infos(){
        std::cout << "[Buffer INFO]: size: " << _size.transpose()  <<
                  "\n[Buffer INFO] origin: " << _origin.transpose()
               << "\n[Buffer INFO]  min max: " <<  _min_grid_bound.transpose() << " & " << _max_grid_bound.transpose()
               << "\n[Buffer INFO] min max: " <<  _min_real_bound.transpose() << " & " << _max_real_bound.transpose()
               <<"\n[Buffer INFO] res: " << _resolution
              <<"\n[Buffer INFO] SDF Occupancy: " << sdf_occ
                <<"\n[Buffer INFO] SDF Truncation: " << sdf_trunc << std::endl;

    }
    void init(const Eigen::Vector3f &size, const Eigen::Vector3f &origin_i, float resolution);

    void insert_flag(const Eigen::Vector3i &pos,const u_int8_t flag);

    void getVoxelPos(const Eigen::Vector3f &p, Eigen::Vector3i &idx){
        idx = (p / _resolution+Eigen::Vector3f(0.5,0.5,0.5)).array().floor().cast<int>();
        idx.noalias() += _origin;
    }

    void toGridCoordinateSystem(const Eigen::Vector3f &p, Eigen::Vector3f &res){
        res = (p / _resolution)+_originf+Eigen::Vector3f(0.5,0.5,0.5);
    }

    Eigen::Vector3f toGridCoordinateSystem(const Eigen::Vector3f &p) const{
        return (p / _resolution)+_originf+Eigen::Vector3f(0.5,0.5,0.5);
    }

    inline size_t get_idx(const Eigen::Vector3i &coord){
        return _size(2)*_size(1)*coord(0)+_size(2)*coord(1)+coord(2);
    }

    inline size_t get_idx_f(const Eigen::Vector3f &p){
        Eigen::Vector3i idx = (p / _resolution+Eigen::Vector3f(0.5,0.5,0.5)).array().cast<int>();
        idx.noalias() += _origin;

        return _size(2)*_size(1)*idx(0)+_size(2)*idx(1)+idx(2);
    }

    inline uint8_t & at(const int idx){

        return _flag_buffer[idx];

    }

    inline uint8_t & at(const Eigen::Vector3i &coord){

        return _flag_buffer[_size(2)*_size(1)*coord(0)+_size(2)*coord(1)+coord(2)];

    }


    inline void convert(const Eigen::Vector3i &cell, Eigen::Vector3f &pt){
        pt = (cell-_origin).cast<float>()*_resolution;
    }

    inline bool is_inside(const Eigen::Vector3f &pt){
        return
                pt.x() > _min_real_bound.x() &&
                pt.x() < _max_real_bound.x() &&
                pt.y() > _min_real_bound.y() &&
                pt.y() < _max_real_bound.y() &&
                pt.z() > _min_real_bound.z() &&
                pt.z() < _max_real_bound.z();

    }
    inline bool is_inside(const Eigen::Vector3i &pt){
        return
                pt.x() > _min_grid_bound.x() &&
                pt.x() < _max_grid_bound.x() &&
                pt.y() > _min_grid_bound.y() &&
                pt.y() < _max_grid_bound.y() &&
                pt.z() > _min_grid_bound.z() &&
                pt.z() < _max_grid_bound.z();

    }

    inline bool isInsideBuf(const Eigen::Vector3i &pt){
        return
                pt.x() >= 0 &&
                pt.x() < _size.x() &&
                pt.y() >= 0 &&
                pt.y() < _size.y() &&
                pt.z() >= 0 &&
                pt.z() < _size.z();

    }


    inline bool isOcc(const Eigen::Vector3i &coord){

        return (at(coord) & occupied_flag);
    }
    inline bool isUnknown(const Eigen::Vector3i &coord){

        return (at(coord) & unknown);
    }
    inline bool isFrontier(const Eigen::Vector3i &coord){

        return (at(coord) & frontier_flag);
    }

    inline bool isVisited(const Eigen::Vector3i &coord){

        return (at(coord) & visited_flag);
    }

    BoundingMap3D<float> getBoudingMapReal(){
        return BoundingMap3D<float>(_min_real_bound,_max_real_bound);
    }


    float clamp(float n, float lower, float upper) {
        return std::max(lower, std::min(n, upper));
    }
    inline void clamp(Eigen::Vector3i &coord){
        coord.x() = clamp(coord.x(),0,_size.x());
        coord.y() = clamp(coord.y(),0,_size.y());
        coord.z() = clamp(coord.z(),0,_size.z());
    }

    inline void clamp(Eigen::Vector3f &coord){
        coord.x() = clamp(coord.x(),_min_real_bound.x(),_max_real_bound.x());
        coord.y() = clamp(coord.y(),_min_real_bound.y(),_max_real_bound.y());
        coord.z() = clamp(coord.z(),_min_real_bound.z(),_max_real_bound.z());
    }

    const float espi = 0.00001;

    bool clampGridRay(const Eigen::Vector3i &origin, const Eigen::Vector3f &dir, Eigen::Vector3i &result){

        float dist_max = 1000/0.2;
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        Eigen::Vector3f invDir;
        if(abs(dir.x()) < espi){
           return clamp2D(1,2,origin,dir,result) ;
        }
        if( abs(dir.y()) < espi){
            return clamp2D(0,2,origin,dir,result) ;

        }
        if(abs(dir.z()) < espi){
            return clamp2D(0,1,origin,dir,result) ;

        }
        invDir.x() = 1.f / dir.x();
        invDir.y() =  1.f / dir.y();
        invDir.z() =  1.f / dir.z();
        bool sign[3];
        sign[0] = (invDir.x() < 0);
        sign[1] = (invDir.y() < 0);
        sign[2] = (invDir.z() < 0);

        float tmp = sign[0] ? _size.x()-1 : 0;
        tmin = (tmp - origin.x()) * invDir.x();
        tmp = sign[0] ? 0 : _size.x()-1;
        tmax = (tmp - origin.x()) * invDir.x();
        tmp = sign[1] ? _size.y()-1 : 0;
        tymin = (tmp - origin.y()) * invDir.y();
        tmp = sign[1] ? 0 : _size.y()-1;
        tymax = (tmp - origin.y()) * invDir.y();

        if ((tmin > tymax) || (tymin > tmax)){
            return false;
        }
        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;

        tmp = sign[2] ? _size.z()-1 : 0;
        tzmin = (tmp - origin.z()) * invDir.z();
        tmp = sign[2] ? 0 : _size.z()-1;
        tzmax = (tmp - origin.z()) * invDir.z();

        if ((tmin > tzmax) || (tzmin > tmax)){
            return false;
        }
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;
        if (tmin < dist_max) {
            //            std::cout << tmin << " " << tmax << " "  << tymin << " "  << tzmin << " " << tymax <<  " " << tzmax <<std::endl;
            result = origin+(dir*tmax).cast<int>();
            return true;
        }
        return false;
    }


    bool clamp2D(int idx1, int idx2,const Eigen::Vector3i &origin,const Eigen::Vector3f &dir, Eigen::Vector3i &result){

        if( abs(dir(idx1)) <= espi){
            return clamp1D(idx2,origin,dir,result) ;

        }
        if(abs(dir(idx2)) <= espi){
            return clamp1D(idx1,origin,dir,result) ;

        }
        float invd1 = 1.f/dir(idx1);

        float invd2 = 1.f/dir(idx2);

        float tmin, tmax, tmin2, tmax2;
        bool sign[2];
        sign[0] = (invd1 < 0);
        sign[1] = (invd2 < 0);
        float tmp = sign[0] ? _size(idx1) -1 : 0;
        tmin = (tmp - origin(idx1)) * invd1;
        tmp = sign[0] ? 0 : _size(idx1)-1;
        tmax = (tmp - origin(idx1)) * invd1;

        tmp = sign[1] ? _size(idx2)-1 : 0;
        tmin2 = (tmp - origin(idx2)) * invd2;
        tmp = sign[1] ? 0 : _size(idx2)-1;
        tmax2 = (tmp - origin(idx2)) * invd2;

        if ((tmin > tmax2) || (tmin2 > tmax)){
            return false;
        }
        if (tmin2 > tmin)
            tmin = tmin2;
        if (tmax2 < tmax)
            tmax = tmax2;
        result = origin+(dir*tmax).cast<int>();
        return true;
    }

    bool clamp1D(const int &idx1, const Eigen::Vector3i &origin,const Eigen::Vector3f &dir, Eigen::Vector3i &result ){
        int tmp = dir(idx1) < 0 ? 0 : _size(idx1)-1;
        result = origin;
        result(idx1) = tmp;
        return true;


    }
    //Give the origin as float inside the grid coordinate
    bool clampGridRay(const Eigen::Vector3f &origin, const Eigen::Vector3f &dir, Eigen::Vector3f &result){
        float dist_max = 1000/0.2;
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        Eigen::Vector3f invDir;
        invDir.x() =  1.f / dir.x() ;
        invDir.y() =  1.f / dir.y() ;
        invDir.z() =  1.f / dir.z();
        bool sign[3];
        sign[0] = (invDir.x() < 0);
        sign[1] = (invDir.y() < 0);
        sign[2] = (invDir.z() < 0);

        float tmp = sign[0] ? _size.x()-1 : 0;
        tmin = (tmp - origin.x()) * invDir.x();
        tmp = sign[0] ? 0 : _size.x()-1;
        tmax = (tmp - origin.x()) * invDir.x();
        tmp = sign[1] ? _size.y()-1 : 0;
        tymin = (tmp - origin.y()) * invDir.y();
        tmp = sign[1] ? 0 : _size.y()-1;
        tymax = (tmp - origin.y()) * invDir.y();

        if ((tmin > tymax) || (tymin > tmax)){
            return false;
        }
        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        tmp = sign[2] ? _size.z()-1 : 0;
        tzmin = (tmp - origin.z()) * invDir.z();
        tmp = sign[2] ? 0 : _size.z()-1;
        tzmax = (tmp - origin.z()) * invDir.z();

        if ((tmin > tzmax) || (tzmin > tmax)){
            return false;
        }
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;

        if (tmin < dist_max) {
            result = origin+(dir*tmax);
            return true;
        }

        return false;
    }




    //0 mean nothing
    float noise_ = 0.05f;
    float invnoise_ = 1.f/noise_;






    float sdf_trunc = 0.3f;//30cm
    float sdf_occ = 0.3f;//1/3 sdf range

    //if range < max_rage -0.5
    bool updateFarVisited(size_t idx, uint8_t &state_o){
        if(tsdf_[idx]>sdf_occ){
            state_o = (_flag_buffer[idx] & (1u << 2)) | visited_flag;
            at(idx) = state_o;
            return true;
        }else{
            if(fabs(tsdf_[idx])<sdf_occ){
                return false;
            }else{
                if(tsdf_[idx] <= -1.f){
                    tsdf_[idx] = 1.f;
                    state_o = (_flag_buffer[idx] & (1u << 2)) | visited_flag;
                    at(idx)=state_o;
                    return true;
                }
            }
        }
        return false;
    }
    bool updateTsdfProjection(size_t idx, uint8_t &state_o,float depth, float z_vox){

        float diff = (depth-z_vox); //-6ms

        //update tsdf to free voxel (no depth information)
        if(depth<=0.001f){
            uint8_t tmp = tsdfState(idx);

            tsdf_[idx] = (weight_[idx] * tsdf_[idx] + 1.f) / (weight_[idx] + 1);
            if (weight_[idx] < 1000) weight_[idx] += 1.f;

            uint8_t new_state = tsdfState(idx);


            if(tmp != new_state){
                state_o=new_state;
                at(idx) = state_o;
                return true;
            }else {
                return true;
            }
        }

        //occluded
        if((diff) < -sdf_trunc*sdf_occ){
            return false;
        }

        uint8_t tmp = tsdfState(idx);

        diff = diff < sdf_trunc ? diff/sdf_trunc : 1.;
        tsdf_[idx] = (weight_[idx] * tsdf_[idx] + diff) / (weight_[idx] + 1);
        if (weight_[idx] < 1000) weight_[idx] += 1.f;

        uint8_t new_state = tsdfState(idx);

        state_o=new_state;

        if(tmp != new_state){
            state_o=new_state;
            at(idx) = state_o;
            return true;
        }else {
            return true;
        }
    }
    inline bool updateTsdfFree(size_t idx){

        uint8_t tmp = tsdfState(idx);
        //        state = getState(idx);
        tsdf_[idx] = 1.;
        weight_[idx] =0.f;

        uint8_t new_state = tsdfState(idx);


        if(tmp != new_state){
            return true;
        }else {
            return false;
        }
    }
    inline bool updateVoxelProbability(size_t idx, float meas_depth, Eigen::Vector3f &rel_pos, uint8_t &state){

        float diff = (meas_depth-rel_pos.x()); //-6ms

        if((diff) < -sdf_trunc*sdf_occ){
            return false;
        }

        uint8_t tmp = tsdfState(idx);

        diff = diff < sdf_trunc ? diff/sdf_trunc : 1.;
        tsdf_[idx] = (weight_[idx] * tsdf_[idx] + diff) / (weight_[idx] + 1);
        if (weight_[idx] < 1000) weight_[idx] += 1.f;
//        weight_[idx] += 1.f;
        //        state = getState(idx);
        uint8_t new_state = tsdfState(idx);

        state=new_state;

        if(tmp != new_state){
            state=new_state;
            at(idx) = state;
            return true;
        }else {
            return true;
        }

    }

    inline uint8_t tsdfState(size_t idx){
        if(fabs(tsdf_[idx])<=sdf_occ){
            return (_flag_buffer[idx] & (1u << 2)) | occupied_flag | visited_flag;
        }else if(tsdf_[idx]>sdf_occ){
            return  visited_flag;
        }else{
            return _flag_buffer[idx] & ~(1u << 2);
        }
    }
    inline uint8_t getState(size_t idx){
        if(prob_[idx]<0.){
            return visited_flag;
        }else if(prob_[idx]>0.){
            return occupied_flag;
        }else{
            return _flag_buffer[idx] & ~(1u << 2);
        }
    }
    std::vector<uint8_t> _flag_buffer;

    std::vector<float> prob_;

    std::vector<float> tsdf_;
    std::vector<float> weight_;

    float occ_prob_= 0.65;
    float free_prob_= 0.35;

    Eigen::Vector3i _size;
    Eigen::Vector3i _origin;
    Eigen::Vector3f _originf;

    Eigen::Vector3i _min_grid_bound;
    Eigen::Vector3i _max_grid_bound;

    Eigen::Vector3f _min_real_bound;
    Eigen::Vector3f _max_real_bound;


    float _resolution;
    float _inv_resolution;


};

#endif // VOXELBUFFER_H
