#pragma once
#include <vector>
#include <limits>
#include "voxelbuffer.h"
#include <ros/ros.h>
#include <functional>
#include "../utils/timer.hpp"
#include <thread>
#include <atomic>

class EDFMap
{
public:
    typedef std::shared_ptr<EDFMap> Ptr;

      typedef Eigen::Matrix<int, 3, 1> Vector3i;
    EDFMap();
    
    void init(ros::NodeHandle &nh,VoxelBuffer *voxBuffer, Vector3i size, float resolution, float trunc_dist);
    
    void noneBlockingUpdate(){

        if(!has_started && update_timer.elapsed_ms() >=350){
            if(edt_thread.joinable())
                edt_thread.join();
            has_started=true;
            update_timer.restart();
            edt_thread = std::thread(&EDFMap::compute_edt,this);
        }

    }
    
    template< typename Scalar>
    inline Scalar &at(std::vector<Scalar>& buf, Vector3i coord){
        return buf[_size(2)*_size(1)*coord(0)+_size(2)*coord(1)+coord(2)];

    }


    void compute_edt();

    template <typename fget, typename fset>
    inline void distance_transform(fget get, fset set, int start, int end, int lenght){
        int v[lenght];
        float z[lenght+1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<float>::max();
        z[start+1] = std::numeric_limits<float>::max();
        for(int q = start+1; q <= end; q++) {
          k++;
          float s;
          do {

            k--;
            s = ((get(q) + q * q) - (get(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
          } while(s <= z[k]);
          k++;
          v[k] = q;
          z[k] = s;
          z[k+1] = std::numeric_limits<float>::max();
        }

        k = start;

        for(int q = start; q <= end; q++) {
          while(z[k+1] < q) {k++;}

          float val = (q - v[k]) * (q - v[k]) + get(v[k]);
          set(q, val);

        }
    }

    float occypancy_frontiere_occ(int x,int y, int z){
        return _voxBuffer->isOcc(Vector3i(x,y,z)) || _voxBuffer->isFrontier(Vector3i(x,y,z)) ? 0 : std::numeric_limits<float>::max();
    }
    float occypancy_occ(int x,int y, int z){
        return _voxBuffer->isOcc(Vector3i(x,y,z)) ? 0 : std::numeric_limits<float>::max();
    }
    float occypancy_unk_occ(int x,int y, int z){
        return _voxBuffer->isOcc(Vector3i(x,y,z)) || _voxBuffer->isUnknown(Vector3i(x,y,z)) ? 0 : std::numeric_limits<float>::max();
    }
    void publish_slice();

    void publish_distance_field();

private:
    VoxelBuffer *_voxBuffer;

    Vector3i _size;
    float _resolution;
    float _truncation_dist = 2.;

    std::vector<float> _dist_buffer;

    std::vector<float> _tmp_buffer1, _tmp_buffer2;
    

    ros::Timer _esdfUpdateTimer;

    ros::Publisher _dist_pub;

    bool debug_vizu_ = false;
    ros::Publisher esdf_slice_;

    int mode_occupancy = 0;//0 frontier + obstacle, 1 obstacle
    std::function<float(int,int,int)> occupancy_func;


    Timer update_timer;
    std::thread edt_thread;
    std::atomic<bool> has_started;
};

