#pragma once
#include <vector>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <ros/ros.h>
#include <glm/gtc/matrix_access.hpp>
#include <limits>
class CameraFrustum
{


public:
    enum side { LEFT = 0, RIGHT = 1, TOP = 2, BOTTOM = 3, BACK = 4, FRONT = 5 };
    enum side_corner { FTOPLEFT = 0, FTOPRIGHT = 1, FBOTTOMLEFT = 2, FBOTTOMRIGHT = 3,
                 NTOPLEFT = 4, NTOPRIGHT = 5, NBOTTOMLEFT = 6, NBOTTOMRIGHT = 7};

    std::array<glm::vec4, 6> planes;
    std::array<glm::vec3, 8> corners_;

    struct box{
        float minx_=std::numeric_limits<float>::max(),miny_=std::numeric_limits<float>::max(),minz_=std::numeric_limits<float>::max();
        float maxx_=-std::numeric_limits<float>::max(),maxy_=-std::numeric_limits<float>::max(),maxz_=-std::numeric_limits<float>::max();


    };

    CameraFrustum();
    void init(ros::NodeHandle &nh);

    void update(const nav_msgs::Odometry &odom);
    void updateTransform(const nav_msgs::Odometry &odom);
    void updateFrustum();

    void setTransform();

    bool isOccluded(const Eigen::Vector3f &pts_i, uint16_t *depth_i, float resolution_i, uint8_t &flag_i);
    bool isOccluded(const Eigen::Vector3f &pts_i, uint16_t *depth_i, float resolution_i, uint8_t &flag_i, float sigma3, float &pixel_depth);

    bool retrieveClosestDepth(const Eigen::Vector3f &pts_i, uint16_t *depth_i, float resolution_i, float &pixel_depth);


    void reproj(const Eigen::Vector3f &pts, Eigen::Vector2f &pxl);
    void deproj(const Eigen::Vector2f &pxl, Eigen::Vector3f &pts, float dist);
    void deproj_relative(const Eigen::Vector2f &pxl, Eigen::Vector3f &pts, float dist);

    void back_transform(Eigen::Vector3f &pts);

    int findSide(const Eigen::Vector2f &p1,const Eigen::Vector2f &p2,const Eigen::Vector2f &p);
    float lineDist(const Eigen::Vector2f &p1,const Eigen::Vector2f &p2,const Eigen::Vector2f &p) ;
    void quick_hull(const Eigen::Vector2f * pxls, int n, const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, int side, std::list<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &hull);
    void convex_hull(const Eigen::Vector2f * pxls);

    box computeBBox();


    inline glm::vec3 get_forward()const {
        return glm::column(rotM_,0);
    }

    inline glm::vec3 get_up()const {
        return glm::column(rotM_,2);
    }


    inline glm::vec3 get_left()const {
        return glm::column(rotM_,1);
    }

    // PLANES:
    // Ordering: [0] = Left, [1] = Right, [2] = Down, [3] = Up, [4] = Near, [5] = Far
    std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > normals_;

    glm::mat4 view_;
    glm::mat4 proj_;


    glm::vec3 eye_;
    glm::quat rot_;
    glm::mat3 rotM_;
    Eigen::Vector3f t_;
    Eigen::Matrix3f R_;
    float max_distance_ = 5.f;

    float near_ = 0.01f;
    float far_ = 5.f;


    float farHeight_;
    float farWidth_;
    float nearHeight_;
    float nearWidth_;

    int width_ = 640;
    int height_ = 480;
    float ppx_;
    float ppy_;
    float fovy_ = 60;
    float fovx_ = 75.f;
    float fovRadians_ = glm::radians(fovy_);

    float f_;//simulate focal
    float fx_,fy_; // for future realistic test

};


