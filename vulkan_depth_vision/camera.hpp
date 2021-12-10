//
// Created by anthony on 18/05/19.
//

#ifndef VULKANVOXEL_CAMERA_HPP
#define VULKANVOXEL_CAMERA_HPP
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/geometric.hpp>
#include <cmath>
#include <iostream>
#include "simulatecamera.h"
static struct{
    bool left = false, right = false, up = false, down = false, forward = false, backward = false;
} _keys;

static struct{
    int W=640, H=480, xpos=W/2, ypos=H/2;
} _mouse_pos;


static float clip( float n, float lower, float upper )
{
    n = ( n > lower ) * n + !( n > lower ) * lower;
    return ( n < upper ) * n + !( n < upper ) * upper;
}

class Camera{
public:

    Camera(){
        //default camera
        _eye = glm::vec3(0,0.6,0.);
        _rot.w = 1.f;
        _viewMatrix = glm::mat4(1.0f);
        r_oc.w = 1.;
        r_oc.x = 0.;
        r_oc.y = 0.;
        r_oc.z = 0.;

        t_oc.x = 0.;
        t_oc.y = 0.;
        t_oc.z = 0.;
    }

    void setSimCamera(const SimulateCamera &sim_cam_i){
        _mouse_pos.H = sim_cam_i.height_;
        _mouse_pos.W = sim_cam_i.width_;

        _mouse_pos.xpos = sim_cam_i.height_/2.;
        _mouse_pos.ypos = sim_cam_i.width_/2.;


        _height = sim_cam_i.height_;
        _width = sim_cam_i.width_;


        _focal = sim_cam_i.focal_;


        _z_near = sim_cam_i.near_;
        _z_far = sim_cam_i.far_;
        _fovy = sim_cam_i.fovy_;

    }

    void update(unsigned long dt){


        glm::vec3 forward;
        glm::vec3 up;
        glm::vec3 right;
        get_forward_vec(forward);
        get_up_vec(up);
        right = glm::cross(forward,up);


        float cosPitch = cos(_pitch);
        float sinPitch = sin(_pitch);
        float cosYaw = cos(_yaw);
        float sinYaw = sin(_yaw);
        float cosRoll = cos(_roll);
        float sinRoll = sin(_roll);

        glm::vec3 xaxis = { cosRoll*cosYaw, sinRoll*cosYaw, -sinYaw };
        glm::vec3 yaxis = { cosRoll* sinYaw * sinPitch-sinRoll*cosPitch, sinRoll*sinYaw*sinPitch + cosRoll*cosPitch, cosYaw * sinPitch };
        glm::vec3 zaxis = { cosRoll* sinYaw * cosPitch + sinRoll*sinPitch, sinRoll*sinYaw*cosPitch-cosRoll*sinPitch, cosPitch * cosYaw };

        float speed = _speed * dt/1000.;
        if(_keys.left){
            _eye += xaxis * speed;
        }

        if(_keys.right){

            _eye -= xaxis * speed;
        }

        if(_keys.up){
            _eye += yaxis * speed;
        }

        if(_keys.down){
            _eye -= yaxis * speed;
        }

        if(_keys.forward){
            _eye += zaxis * speed;
        }

        if(_keys.backward){

            _eye -= zaxis * speed;
        }


        _yaw -= _mul_ang_speed * clip(_mouse_pos.xpos - static_cast<int>(_mouse_pos.W/2),-1,1)*dt/1000.;
        _pitch += _mul_ang_speed * clip(_mouse_pos.ypos-static_cast<int>(_mouse_pos.H/2),-1,1)*dt/1000.;
        _pitch = clip(_pitch,-1.5708f,1.5708f);
        if(_yaw > M_PI*2){
            _yaw = std::fmod(_yaw,M_PI*2);

        }
        if(_yaw < M_PI*2){
            _yaw = std::fmod(_yaw,M_PI*2);
        }
        cosPitch = cos(_pitch);
        sinPitch = sin(_pitch);
        cosYaw = cos(_yaw);
        sinYaw = sin(_yaw);

        xaxis = { cosRoll*cosYaw, sinRoll*cosYaw, -sinYaw };
        yaxis = { cosRoll* sinYaw * sinPitch-sinRoll*cosPitch, sinRoll*sinYaw*sinPitch + cosRoll*cosPitch, cosYaw * sinPitch };
        zaxis = { cosRoll* sinYaw * cosPitch + sinRoll*sinPitch, sinRoll*sinYaw*cosPitch-cosRoll*sinPitch, cosPitch * cosYaw };

        _viewMatrix = glm::lookAtRH(_eye,_eye+zaxis,yaxis);
        _rot = glm::quat(glm::vec3(_pitch,_yaw,_roll));
        reset_cursor();
    }

    void reset_move(){
        _keys = {false,false,false,false,false,false};
    }

    void reset_cursor(){
        _mouse_pos.xpos = _mouse_pos.W/2;
        _mouse_pos.ypos = _mouse_pos.H/2;
    }

    glm::mat4 _viewMatrix;

    glm::vec3 _eye;
    glm::quat _rot;


    int _height = 480;
    int _width = 640;


    float _pitch = 0.f;
    float _yaw = 0.f;
    float _roll = 0.0f;

    float _z_near = 0.01f;
    float _z_far = 5.f;
    float _fovy = 60.0f;

    float _focal =1;

    float _speed = 4.f;
    float _mul_ang_speed = 2.f;

    glm::quat r_oc;
    glm::vec3 t_oc;
    void get_forward_vec(glm::vec3 & _out3){
        _out3.x = 2 * (_rot.x*_rot.z + _rot.w*_rot.y);
        _out3.y = 2 * (_rot.y*_rot.z - _rot.w*_rot.x);
        _out3.z = 1 - 2 * (_rot.x*_rot.x + _rot.y*_rot.y);
    }

    void get_up_vec(glm::vec3 & _out3){
        _out3.x = 2 * (_rot.x*_rot.y - _rot.w*_rot.z);
        _out3.y = 1- 2 * (_rot.x*_rot.x - _rot.z*_rot.z);
        _out3.z = 2 * (_rot.y*_rot.z + _rot.w*_rot.x);
    }

};

#endif //VULKANVOXEL_CAMERA_HPP

