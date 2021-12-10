#include "../include/map_core/camerafrustum.h"

#include <glm/gtc/matrix_transform.hpp>
#include <map_core/camerafrustum.h>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <Eigen/Geometry>
CameraFrustum::CameraFrustum()
{

}

void CameraFrustum::init(ros::NodeHandle &nh)
{

    bool useFov,useRealSensor;
    nh.param("map_core/useFov",useFov,false);
    nh.param("map_core/useRealSensor",useRealSensor,false);
    nh.param("map_core/max_range",max_distance_,5.f);

    if(useFov){
        std::cout << "[MapCore] Use FOV simulate camera: " <<std::endl;

        nh.param("map_core/width",width_,640);
        nh.param("map_core/fovx",fovx_,75.f);
        nh.param("map_core/fovy",fovy_,60.f);
        float tanx = tanf((fovx_*M_PI/180.f/2.));
        //        if(fovx_ > 90.f){
        //            tanx = M_PI-tanx;
        //        }

        height_ = (width_/((tanx)/tan(fovy_*M_PI/180/2.)))+0.5;
        float ratio = (float) width_ / (float) height_;
        ppx_ = width_/2.f;
        ppy_ = height_/2.f;
        fovRadians_ = glm::radians(fovy_);
        f_ = (float)height_/(2.*tanf(fovRadians_/2.));
        fx_ =f_;
        fy_ = f_;

        far_ = max_distance_;
        proj_ = glm::perspective(fovRadians_, ratio , near_, far_);
        proj_[1][1] *= -1;
        view_ = glm::mat4(1.0f);
        nearHeight_ = 2. * tan(fovRadians_/ 2.) * near_;
        farHeight_ = 2. * tan(fovRadians_ / 2.) * far_;
        nearWidth_ = nearHeight_ * ratio;
        farWidth_ = farHeight_ * ratio;
        rot_ = glm::quat(1.0f,0.f,0.f,0.f);
        rotM_ = glm::toMat3(rot_);
        eye_ = glm::vec3(0,0,0);
    }
    else if(useRealSensor){
        std::cout << "[MapCore] Use real camera: " <<std::endl;
        nh.param("map_core/width",width_,640);
        nh.param("map_core/heigh",height_,480);
        nh.param("map_core/fx",fx_,461.453f);
        nh.param("map_core/fy",fy_,460.859f);
        nh.param("map_core/ppx",ppx_,344.191f);
        nh.param("map_core/ppy",ppy_,245.387f);

        far_ = max_distance_;
        fovy_ =2.*atanf(0.5*height_/(fy_));
        fovx_ =2.*atanf(0.5*width_/(fx_));

    }else{
        std::cout << "[MapCore] Use default simulate camera: " <<std::endl;
        float ratio = (float) width_ / (float) height_;
        ppx_ = width_/2.f;
        ppy_ = height_/2.f;
        fovRadians_ = glm::radians(fovy_);
        f_ = (float)height_/(2.*tanf(fovRadians_/2.));
        fx_ =f_;
        fy_ = f_;

        far_ = max_distance_;
        proj_ = glm::perspective(fovRadians_, ratio , near_, far_);
        proj_[1][1] *= -1;
        view_ = glm::mat4(1.0f);
        nearHeight_ = 2. * tan(fovRadians_/ 2.) * near_;
        farHeight_ = 2. * tan(fovRadians_ / 2.) * far_;
        nearWidth_ = nearHeight_ * ratio;
        farWidth_ = farHeight_ * ratio;
        rot_ = glm::quat(1.0f,0.f,0.f,0.f);
        rotM_ = glm::toMat3(rot_);
        eye_ = glm::vec3(0,0,0);
    }




    std::cout << "[MapCore-CameraFrustum] fovx;fovy: " << fovx_ << ";" << fovy_  <<std::endl
     << "[MapCore-CameraFrustum] Res [width,height]: " << width_ <<"x" << height_ <<std::endl
     << "[MapCore-CameraFrustum] Max range: " << far_ <<std::endl
     << "[MapCore-CameraFrustum] Focal (fx,fy): " << fx_ << "," << fy_ <<std::endl
    << "[MapCore-CameraFrustum] Center (ppx,ppy): " << ppx_ << "," << ppy_ <<std::endl;

}

void CameraFrustum::update(const nav_msgs::Odometry &odom_i)
{
    updateTransform(odom_i);
    //    glm::mat4 mat = proj_;
    updateFrustum();
}

void CameraFrustum::updateTransform(const nav_msgs::Odometry &odom_i)
{
    rot_.w = odom_i.pose.pose.orientation.w;
    rot_.x = odom_i.pose.pose.orientation.x;
    rot_.y = odom_i.pose.pose.orientation.y;
    rot_.z = odom_i.pose.pose.orientation.z;

    R_ = Eigen::Quaternionf(rot_.w,rot_.x,rot_.y,rot_.z).toRotationMatrix();
    t_ = Eigen::Vector3f(odom_i.pose.pose.position.x,
                         odom_i.pose.pose.position.y,
                         odom_i.pose.pose.position.z);
    rotM_ = glm::toMat3(rot_);

    eye_.x = odom_i.pose.pose.position.x;
    eye_.y = odom_i.pose.pose.position.y;
    eye_.z = odom_i.pose.pose.position.z;



    view_ = glm::lookAtRH(eye_,get_forward()+eye_,get_up());


}

void CameraFrustum::updateFrustum()
{



    for (auto i = 0; i < planes.size(); i++)
    {
        float length = sqrtf(planes[i].x * planes[i].x + planes[i].y * planes[i].y + planes[i].z * planes[i].z);
        planes[i].x /= length;
        planes[i].y /= length;
        planes[i].z /= length;
        planes[i].w /= length;
    }

    glm::vec3 nearCenter = eye_ + get_forward() * near_;
    glm::vec3 farCenter = eye_ + get_forward() * far_;

    corners_[FTOPLEFT] = farCenter + get_up() * (farHeight_*0.5f) + get_left() * (farWidth_*0.5f);
    corners_[FTOPRIGHT] = farCenter + get_up() * (farHeight_*0.5f) - get_left() * (farWidth_*0.5f);
    corners_[FBOTTOMLEFT] = farCenter - get_up() * (farHeight_*0.5f) + get_left() * (farWidth_*0.5f);
    corners_[FBOTTOMRIGHT] = farCenter - get_up() * (farHeight_*0.5f) - get_left() * (farWidth_*0.5f);

    corners_[NTOPLEFT] = nearCenter + get_up() * (nearHeight_*0.5f) + get_left() * (nearWidth_*0.5f);
    corners_[NTOPRIGHT] = nearCenter + get_up() * (nearHeight_*0.5f) - get_left() * (nearWidth_*0.5f);
    corners_[NBOTTOMLEFT] = nearCenter - get_up() * (nearHeight_*0.5f) + get_left() * (nearWidth_*0.5f);
    corners_[NBOTTOMRIGHT] = nearCenter - get_up() * (nearHeight_*0.5f) - get_left() * (nearWidth_*0.5f);


    planes[BACK] = glm::vec4(-get_forward(),glm::dot(-get_forward(),nearCenter));
    planes[FRONT] = glm::vec4(get_forward(),glm::dot(get_forward(),farCenter));

    glm::vec3 leftv = glm::normalize(glm::cross(get_up(),(corners_[FTOPLEFT] - corners_[NTOPLEFT])));
    planes[LEFT] = glm::vec4(leftv,0);

    glm::vec3 rightv = glm::normalize(glm::cross((corners_[FTOPRIGHT] - corners_[NTOPRIGHT]),get_up()));
    planes[RIGHT] = glm::vec4(rightv,0);

    glm::vec3 botv = glm::normalize(glm::cross(get_left(),(corners_[FBOTTOMRIGHT] - corners_[NBOTTOMRIGHT])));
    planes[BOTTOM] = glm::vec4(botv,0);

    glm::vec3 topv = glm::normalize(glm::cross(corners_[FTOPRIGHT] - corners_[NTOPRIGHT],get_left()));
    planes[TOP] = glm::vec4(topv,0);



}

bool CameraFrustum::isOccluded(const Eigen::Vector3f &pts_i, uint16_t * depth_i, float resolution_i, uint8_t &flag_i)
{
    Eigen::Vector2f pxl;
    //    resolution_i += 0.2;
    Eigen::Vector3f pts = R_.transpose()*pts_i-R_.transpose()*t_;//rotate translate

    //invert to camera system coord
    Eigen::Vector3f pts_cam(-pts.y(),-pts.z(),pts.x());
    if(pts_cam.x() > 0){
        pts_cam.x() -= resolution_i;
    }else{
        pts_cam.x() += resolution_i;
    }
    if(pts_cam.y() > 0){
        pts_cam.y() -= resolution_i;
    }else{
        pts_cam.y() += resolution_i;
    }
    float dist = pts_cam.norm();
    if(dist > max_distance_ || dist < 0.f || pts_cam.z() < 0)
        return true;

    reproj(pts_cam,pxl);
    pxl += Eigen::Vector2f(0.5f,0.5f);
    if(pxl.x() < width_ && pxl.x() >= 0 && pxl.y() < height_ && pxl.y() >= 0){
        uint16_t d = static_cast<uint16_t>(std::floor(dist*1000.f));
        int idx = static_cast<int>(pxl.y())*width_+static_cast<int>(pxl.x());
        if(depth_i[idx] <= 250){
            flag_i = 2;
            return false;
        }
        if(d < depth_i[idx]-50.f){
            flag_i = 2;
            return false;
        }
    }


    return true;
}

bool CameraFrustum::isOccluded(const Eigen::Vector3f &pts_i, uint16_t *depth_i,
                               float resolution_i, uint8_t &flag_i, float sigma3, float &pixel_depth)
{
    Eigen::Vector2f pxl;
    //    resolution_i += 0.2;

    Eigen::Vector3f pts_cam(-pts_i.y(),-pts_i.z(),pts_i.x());
    if(pts_cam.x() > 0){
        pts_cam.x() -= resolution_i;
    }else{
        pts_cam.x() += resolution_i;
    }
    if(pts_cam.y() > 0){
        pts_cam.y() -= resolution_i;
    }else{
        pts_cam.y() += resolution_i;
    }
    float dist = pts_cam.norm();
    //dist to voxel center to close / behind cam or to far is considered occluded
    if(dist > max_distance_-sqrt(2*resolution_i*resolution_i) || dist <= 0.f || pts_cam.z() < 0)
        return true;

    reproj(pts_cam,pxl);
    //    pxl += Eigen::Vector2f(0.5f,0.5f);
    if(pxl.x() < width_ && pxl.x() >= 0 && pxl.y() < height_  && pxl.y() >= 0){
        //        uint16_t d = static_cast<uint16_t>(std::floor(dist));
        int idx = static_cast<int>(pxl.y())*width_+static_cast<int>(pxl.x());

        // if depth < 25cm consider free TODO check mask to avoid
        if(depth_i[idx] <= 10){
            pixel_depth = 0;
            flag_i = 2;
            return false;
        }
        pixel_depth = static_cast<float>(depth_i[idx])/1000.f;

        if(pts_cam.z() < pixel_depth){
            //            Eigen::Vector3f p;
            //            p.x() = 1.0f;
            //            p.y() = -(pxl.x() - ppx_)/fx_;
            //            p.z() = -(pxl.y() - ppy_)/fy_;
            //            p *= pixel_depth;
            //            pixel_depth = p.x();
            flag_i = 2;
            return false;
        }
    }


    return true;
}

bool CameraFrustum::retrieveClosestDepth(const Eigen::Vector3f &pts_i, uint16_t *depth_i, float resolution_i, float &pixel_depth)
{
    Eigen::Vector2f pxl;
    //    resolution_i += 0.2;




    Eigen::Vector3f pts_cam(-pts_i.y(),-pts_i.z(),pts_i.x());





    float dist = pts_cam.norm();


    if(pts_cam.z() > max_distance_-sqrt(3*resolution_i*resolution_i) || pts_cam.z() < 0.f || pts_cam.z() < 0)
        return false;


    reproj(pts_cam,pxl);
    // if depth < 25cm consider free TODO check mask to avoid
    if(pxl.x() < width_ && pxl.x() >= 0 && pxl.y() < height_  && pxl.y() >= 0){
        int idx = static_cast<int>(pxl.y())*width_+static_cast<int>(pxl.x());

        if(depth_i[idx] <= 10){
            pixel_depth = 0;
            return true;
        }
    }

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> list_pts;
    list_pts.reserve(100);
    float step = resolution_i/2.f;
    for(float x = -resolution_i; x <=resolution_i; x+=step){
        for(float y = -resolution_i; y <=resolution_i; y+=step){
            for(float z = -resolution_i; z <=resolution_i; z+=step){
                list_pts.push_back(pts_cam+Eigen::Vector3f(x,y,z));
            }
        }
    }

    float best_depth = std::numeric_limits<float>::max();
    int best_idx = -1;

    for(int i = 0; i < list_pts.size();++i){
        reproj(list_pts[i],pxl);
        if(pxl.x() < width_ && pxl.x() >= 0 && pxl.y() < height_  && pxl.y() >= 0){
            int idx = static_cast<int>(pxl.y())*width_+static_cast<int>(pxl.x());
            if(depth_i[idx] >= 10){
                float d = static_cast<float>(depth_i[idx])/1000.f;
                float tmp = fabs(d-list_pts[i].z());

                if (tmp < best_depth){
                    best_depth = d;
                    best_idx = idx;
                }
            }

        }
    }

    //    if(pts_cam.x() > 0){
    //        pts_cam.x() -= resolution_i;
    //    }else{
    //        pts_cam.x() += resolution_i;
    //    }
    //    if(pts_cam.y() > 0){
    //        pts_cam.y() -= resolution_i;
    //    }else{
    //        pts_cam.y() += resolution_i;
    //    }
    //dist to voxel center to close / behind cam or to far is considered occluded
    if(best_idx == -1)
        return false;

    reproj(pts_cam,pxl);
    pixel_depth = best_depth;
    // if depth < 25cm consider free TODO check mask to avoid
    if(depth_i[best_idx] <= 10){
        pixel_depth = 0;
        return true;
    }

    if(pts_cam.z() < best_depth+resolution_i*3){
        return true;
    }


    return false;

}

void CameraFrustum::reproj(const Eigen::Vector3f &pts, Eigen::Vector2f &pxl)
{
    //undistorted depth image, on real depth image  we must use fx fy and distortion model
    pxl = pts.head<2>();
    pxl.noalias() = pxl / pts.z();
    pxl.x() =  pxl.x() * fx_ + ppx_;
    pxl.y() = pxl.y() *fy_ +ppy_;
}

//deproj & apply transform ros system coordinate
void CameraFrustum::deproj(const Eigen::Vector2f &pxl, Eigen::Vector3f &pts, float dist)
{
    pts.x() = 1.0f;
    pts.y() = -(pxl.x() - ppx_)/fx_;
    pts.z() = -(pxl.y() - ppy_)/fy_;
    //    pts.normalize();
    pts *= dist;

    pts = R_*pts + t_;
}

void CameraFrustum::deproj_relative(const Eigen::Vector2f &pxl, Eigen::Vector3f &pts, float dist)
{
    pts.z() = 1.0f;
    pts.x() = (pxl.x() - ppx_)/fx_;
    pts.y() = (pxl.y() - ppy_)/fy_;
    //    pts.normalize();
    pts *= dist;
}

void CameraFrustum::back_transform(Eigen::Vector3f &pts)
{
    //    Eigen::Vector3f pts = R_.transpose()*pts_i-R_.transpose()*t_;//rotate translate
    pts.noalias() = R_.transpose()*(pts - t_);
}

int CameraFrustum::findSide(const Eigen::Vector2f &p1,const Eigen::Vector2f &p2,const Eigen::Vector2f &p)
{
    float val = (p.y() - p1.y()) * (p2.x() - p1.x()) -
            (p2.y() - p1.y()) * (p.x() - p1.x());

    if (val > 0)
        return 1;
    if (val < 0)
        return -1;
    return 0;
}

float CameraFrustum::lineDist(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p)
{
    return std::fabs((p.y() - p1.y()) * (p2.x() - p1.x()) -
                     (p2.y() - p1.y()) * (p.x() - p1.x()));
}

void CameraFrustum::convex_hull(const Eigen::Vector2f *pxls)
{
    int n = 8;
    std::list<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> hull;
    // Finding the point with minimum and
    // maximum x-coordinate
    int min_x = 0, max_x = 0;
    for (int i=1; i<n; i++)
    {
        if (pxls[i].x() < pxls[min_x].x())
            min_x = i;
        if (pxls[i].x() > pxls[max_x].x())
            max_x = i;
    }

    // Recursively find convex hull points on
    // one side of line joining a[min_x] and
    // a[max_x]
    quick_hull(pxls, n, pxls[min_x], pxls[max_x], 1,hull);

    // Recursively find convex hull points on
    // other side of line joining a[min_x] and
    // a[max_x]
    quick_hull(pxls, n, pxls[min_x], pxls[max_x], -1,hull);

    //    while (!hull.empty())
    //    {
    //        std::cout << "(" <<( *hull.begin()).y() << ", "
    //             << (*hull.begin()).x() << ") \n";
    //        hull.erase(hull.begin());
    //    }
}

void CameraFrustum::quick_hull(const Eigen::Vector2f *pxls, int n,const Eigen::Vector2f &p1,const Eigen::Vector2f &p2, int side, std::list<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &hull)
{
    int ind = -1;
    float max_dist = 0;

    // finding the point with maximum distance
    // from L and also on the specified side of L.
    for (int i=0; i<n; i++)
    {
        float temp = lineDist(p1, p2, pxls[i]);
        if (findSide(p1, p2, pxls[i]) == side && temp > max_dist)
        {
            ind = i;
            max_dist = temp;
        }
    }

    // If no point is found, add the end points
    // of L to the convex hull.
    if (ind == -1)
    {
        //        hull.push_back(p1);
        //        hull.push_back(p2);
        return;
    }

    // Recur for the two parts divided by a[ind]
    quick_hull(pxls, n, pxls[ind], p1, -findSide(pxls[ind], p1, p2),hull);
    quick_hull(pxls, n, pxls[ind], p2, -findSide(pxls[ind], p2, p1),hull);
}



CameraFrustum::box CameraFrustum::computeBBox()
{
    box b;


    for(auto &v: corners_){
        b.maxx_ = std::max(v.x,b.maxx_);
        b.minx_ = std::min(v.x,b.minx_);

        b.maxy_ = std::max(v.y,b.maxy_);
        b.miny_ = std::min(v.y,b.miny_);


        b.maxz_ = std::max(v.z,b.maxz_);
        b.minz_ = std::min(v.z,b.minz_);

    }

    return b;

}
