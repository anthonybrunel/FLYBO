#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE


#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/quaternion.hpp>
#define STB_IMAGE_IMPLEMENTATION
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <set>
#include <array>
#include <chrono>
#include <thread>
#include "camera.hpp"
#include "scenemanager.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>
#include "simulatecamera.h"

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


static int WIDTH = 640;
static int HEIGHT = 480;

const int MAX_FRAMES_IN_FLIGHT = 2;

const std::vector<const char*> validationLayers = {
    "VK_LAYER_KHRONOS_validation"
};

const std::vector<const char*> deviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
    //    VK_EXT_DEPTH_RANGE_UNRESTRICTED_EXTENSION_NAME
};

#ifdef NDEBUG
const bool enableValidationLayers = false;
#else
const bool enableValidationLayers = true;
#endif

VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger) {
    auto func = (PFN_vkCreateDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
    if (func != nullptr) {
        return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
    } else {
        return VK_ERROR_EXTENSION_NOT_PRESENT;
    }
}

void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
    auto func = (PFN_vkDestroyDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
    if (func != nullptr) {
        func(instance, debugMessenger, pAllocator);
    }
}

struct Optional{
    uint32_t _value;
    bool _has_value = false;

    bool has_value(){return _has_value;}
    uint32_t value(){return _value;}

    bool operator !=(const Optional& v)
    {
        return this->_value != v._value;
    }
    Optional& operator =(const uint32_t v)
    {
        this->_value = v;
        _has_value = true;
        return *this;
    }
};

struct QueueFamilyIndices {
    Optional graphicsFamily;
    Optional presentFamily;

    bool isComplete() {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapChainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

struct UniformBufferObject {
    alignas(4) float i;
    alignas(16) glm::mat4 transform;
};




static bool send_pos = false;




#include <random>
class DepthSimApp {
public:
    SceneManager scene;
    std::string map_path;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> randomWalkDist;
    std::uniform_real_distribution<double> random;
    std::uniform_int_distribution<> randomInt;


    //    message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Odometry>* _sync_depth_odom;

    //    message_filters::Subscriber<nav_msgs::Odometry>* _gt_odom_sub;
    //    message_filters::Subscriber<nav_msgs::Odometry>* _noisy_odom_sub;

    //    message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry>* _sync_odom ;
    DepthSimApp(const ros::NodeHandle &nh_i,const ros::NodeHandle &nh_private_i):nh_(nh_i),nh_private_(nh_private_i){

        //        scene.scene_from_pcd();

        nh_private_.param("/depth_cam_node/camera/model_path", map_path, std::string(""));
        nh_private_.param("/depth_cam_node/shaders_path", shaders_path, std::string(""));


        if(map_path.size()<2){
            std::cerr << "[VulkanVizu] error model path not provided : (" << map_path <<")"<<std::endl;
            exit(1);
        }

        if(shaders_path.size()<2){
            std::cerr << "[VulkanVizu] error shaders path not provided : (" << shaders_path <<")"<<std::endl;
            exit(1);
        }



        std::cout << "[VulkanVizu] Model Map loaded: " << map_path <<std::endl;
        scene.scene_from_obj(map_path);
        //        scene.init_rtcScene();
        nh_private_.param("/depth_cam_node/camera/useUavOdom",useUavOdom_,true);
        nh_private_.param("/depth_cam_node/camera/applyDepthNoise",applyDepthNoise,true);
        nh_private_.param("/depth_cam_node/camera/applyOdomNoise",applyOdomNoise,true);
        nh_private_.param("/depth_cam_node/camera/coeffOdomNoise",coeff_odom_noise,1.);


        if(applyDepthNoise){
            std::cout << "[VulkanVizu] Depth noise enabled" <<std::endl;
        }
        if(applyOdomNoise){
            std::cout << "[VulkanVizu] Odom noise (randomwalk) enabled " << coeff_odom_noise <<std::endl;

            std::cout << "[VulkanVizu] position uncertainty " << position_uncertainty <<std::endl;
            std::cout << "[VulkanVizu] roll pitch uncertainty " << roll_pitch_uncertainty <<std::endl;
            std::cout << "[VulkanVizu] yaw uncertainty " << yaw_uncertainty <<std::endl;
        }

        uavOdomIsReady_ = !useUavOdom_;
        if(!useUavOdom_){
            std::cout << "[VulkanVizu]  Vizualizer will not use uav odometry" <<std::endl;
        }else{
            std::cout << "[VulkanVizu]  Vizualizer will use uav odometry" <<std::endl;
        }
        depth_pub_ = nh_private_.advertise<sensor_msgs::Image>("/camera/depth",1);
        z_depth_pub_ = nh_private_.advertise<sensor_msgs::Image>("/camera/z_depth",1);
        waypts_pub = nh_private_.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 50);
        uav_odom_ = nh_private_.subscribe<nav_msgs::Odometry>("/camera/ground_truth/odometry",1,&DepthSimApp::uavOdomCallback, this);
        cloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("/camera/cloud",1);
        cloud_transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>("/camera/cloud_transform",1);
        pub_noisy_odom = nh_private_.advertise<nav_msgs::Odometry>("/camera/noisy_odom", 1);
        pub_odom = nh_private_.advertise<nav_msgs::Odometry>("/camera/odom", 1);

        sim_cam_ = SimulateCamera(nh_i,nh_private_i);

        cam.setSimCamera(sim_cam_);

        nh_private_.param("/depth_cam_node/camera/oc_x",cam._eye.x,0.f);
        nh_private_.param("/depth_cam_node/camera/oc_y",cam._eye.y,0.f);
        nh_private_.param("/depth_cam_node/camera/oc_z",cam._eye.z,0.f);
        nh_private_.param("/depth_cam_node/camera/oc_qx",cam.r_oc.x,0.f);
        nh_private_.param("/depth_cam_node/camera/oc_qy",cam.r_oc.y,0.f);
        nh_private_.param("/depth_cam_node/camera/oc_qz",cam.r_oc.z,0.f);
        nh_private_.param("/depth_cam_node/camera/oc_qw",cam.r_oc.w,1.f);

        WIDTH = sim_cam_.width_;
        HEIGHT = sim_cam_.height_;
        odom_.header.stamp = ros::Time::now();
        generator = std::default_random_engine(std::random_device()());
        randomInt = (std::uniform_int_distribution<>(0,1));
        randomWalkDist = (std::uniform_real_distribution<double>(0.2,0.4));
        random = (std::uniform_real_distribution<double>(0.,1.));



        //        _gt_odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh_private_,"/camera/sync_gt_odom",1);
        //        _noisy_odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh_private_,"/camera/sync_noisy_odom",1);

        //       _sync_odom =
        //                new message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry>(*_gt_odom_sub,*_noisy_odom_sub,1);
        //        //        sync_depth_odom_->registerCallback(boost::bind(&MapCore::depthOdomCalbackFrustum,this, _1, _2));
        //        _sync_depth_odom->registerCallback(boost::bind(&DepthSimApp::syncOdometryGtNoise,this, _1, _2));



        std::cout << "[VulkanVizu] Visualization and Depth sensor setup" <<std::endl;
    }
    ~DepthSimApp(){
        std::cout << "Clear renderer" << std::endl;

        //        if(!useUavOdom_){
        //            std::string save_path = map_path.substr(0, map_path.size()-4);
        //            save_path+= "_gt.obj";
        //            scene.saveTaggedObj(save_path);
        //            std::cout << "save gt mesh:" << save_path << std::endl;
        //        }
    }
    void loadVertices(std::vector<std::pair<float,float[6]>> &cubes){
        //        std::cout << "Number voxel: " << cubes.size() <<std::endl;

        //        int cpt = 0;




        //        for(auto c: cubes){
        //            float size = c.first;
        //            float x = c.second[0];
        //            float y = c.second[1];
        //            float z = c.second[2];
        //            float r = c.second[3];
        //            float g = c.second[4];
        //            float b = c.second[5];
        //            vertices.push_back({{-0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
        //            vertices.push_back({{0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
        //            vertices.push_back({{0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});
        //            vertices.push_back({{-0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});

        //            vertices.push_back({{-0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
        //            vertices.push_back({{0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
        //            vertices.push_back({{0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});
        //            vertices.push_back({{-0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});

        //            int arr[] =
        //                    {0+cpt*8, 2+cpt*8, 1+cpt*8, 3+cpt*8, 2+cpt*8, 0+cpt*8,
        //                     4+cpt*8, 5+cpt*8, 6+cpt*8, 6+cpt*8, 7+cpt*8, 4+cpt*8,
        //                     5+cpt*8, 0+cpt*8, 1+cpt*8, 0+cpt*8, 5+cpt*8, 4+cpt*8,
        //                     5+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 6+cpt*8, 5+cpt*8,
        //                     4+cpt*8, 7+cpt*8, 0+cpt*8, 7+cpt*8, 3+cpt*8, 0+cpt*8,
        //                     2+cpt*8, 3+cpt*8, 7+cpt*8, 7+cpt*8, 6+cpt*8, 2+cpt*8};
        //            indices.insert(indices.end(),arr,arr+6*6);
        //            cpt++;
        //        }

        //        float x = cubes[0].second[0];
        //        float y = cubes[0].second[1];
        //        float z = cubes[0].second[2];
        //        float x1 = cubes[1].second[3];
        //        float y1 = cubes[1].second[4];
        //        float z1 = cubes[1].second[5];
        //        vertices.push_back({{x, y, z}, {1.,1.f,0.f}});
        //        vertices.push_back({{x+0.0000001f, y+0.0000001f,z+0.0000001f}, {1.,1.f,0.f}});
        //        vertices.push_back({{x1, y1, z1}, {1.,1.f,0.f}});
        //        vertices.push_back({{x1+0.0000001f, y1+0.0000001f, z1+0.0000001f}, {1.,1.f,0.f}});
        //        vertices.push_back({{x, y, z}, {1.,1.f,0.f}});
        //        vertices.push_back({{x+0.0000001f, y+0.0001f,z+0.0000001f}, {1.,1.f,0.f}});
        //        vertices.push_back({{x1, y1, z1}, {1.,1.f,0.f}});
        //        vertices.push_back({{x1+0.0000001f, y1+0.0001f, z1+0.0000001f}, {1.,1.f,0.f}});

        //        int arr[] =
        //                {0+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 3+cpt*8, 1+cpt*8,
        //                 0+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 3+cpt*8, 1+cpt*8,
        //                 0+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 3+cpt*8, 1+cpt*8,
        //                 0+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 3+cpt*8, 1+cpt*8,
        //                 0+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 3+cpt*8, 1+cpt*8,
        //                 0+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 3+cpt*8, 1+cpt*8};
        //        indices.insert(indices.end(),arr,arr+6*6);
        //        cpt++;

    }


    template <class Clock, class Duration>
    bool
    refresh(std::chrono::time_point<Clock, Duration> tp)
    {
        using namespace std::chrono;
        return (tp < Clock::now())
                ;
    }

    void run() {

        ros::Rate loop(500);
        std::chrono::time_point<std::chrono::system_clock> t1;
        t1 = std::chrono::high_resolution_clock::now();
        double elapsed_time;

        //sleep mav is setup
        while(nh_.ok()){

            ros::spinOnce();
            loop.sleep();

            elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1).count();
            if(elapsed_time > 5000)//5sec
                break;
        }

        initWindow();
        initVulkan();



        using namespace std;
        using namespace std::chrono;

        using framerate = duration<int, ratio<1, 20>>;
        auto prev = system_clock::now();
        auto next = prev + framerate{1};
        system_clock::duration sum{0};

        while(nh_.ok()){
            ros::spinOnce();
            elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1).count();

            if(refresh(next)){
                glfwPollEvents();
                mainLoop(50);
                next += framerate{1};

                auto now = system_clock::now();
                sum += now - prev;
            }

            loop.sleep();
        }
        cleanup();

    }

private:
    GLFWwindow* window;

    VkInstance instance;
    VkDebugUtilsMessengerEXT debugMessenger;
    VkSurfaceKHR surface;

    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device;

    VkQueue graphicsQueue;
    VkQueue presentQueue;

    VkSwapchainKHR swapChain;
    std::vector<VkImage> swapChainImages;
    VkFormat swapChainImageFormat;
    VkExtent2D swapChainExtent;
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;

    VkRenderPass renderPass;
    VkDescriptorSetLayout descriptorSetLayout;
    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;

    VkCommandPool commandPool;

    VkImage depthImage;
    VkDeviceMemory depthImageMemory;
    VkImageView depthImageView;

    VkImage textureImage;
    VkDeviceMemory textureImageMemory;
    VkImageView textureImageView;
    VkSampler textureSampler;

    VkBuffer vertexBuffer;
    VkDeviceMemory vertexBufferMemory;
    VkBuffer indexBuffer;
    VkDeviceMemory indexBufferMemory;

    std::string shaders_path;

    std::vector<VkBuffer> uniformBuffers;
    std::vector<VkDeviceMemory> uniformBuffersMemory;

    VkDescriptorPool descriptorPool;
    std::vector<VkDescriptorSet> descriptorSets;

    std::vector<VkCommandBuffer> commandBuffers;

    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;


    std::vector<std::string> supportedExtensions;

    size_t currentFrame = 0;


    Camera cam;

    bool uavOdomIsReady_ = false;
    bool useUavOdom_ = true;
    bool applyDepthNoise = true;
    bool applyOdomNoise = true;
    double coeff_odom_noise = 1.;
    SimulateCamera sim_cam_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher pub_odom;
    ros::Publisher depth_pub_;
    ros::Publisher z_depth_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher cloud_transform_pub_;

    ros::Publisher pub_noisy_odom;

    ros::Subscriber uav_odom_;


    ros::Publisher waypts_pub;
    bool framebufferResized = false;
    nav_msgs::Odometry odom_;
    nav_msgs::Odometry odom_noisy_;

    double position_uncertainty = 0.025 * coeff_odom_noise;
    double roll_pitch_uncertainty = 0.5 * coeff_odom_noise;
    double yaw_uncertainty = 0.5 * coeff_odom_noise;


    Eigen::Matrix<double,6,1> rw_offset = Eigen::Matrix<double,6,1>::Zero();

    void initWindow() {
        glfwInit();

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

        window = glfwCreateWindow(sim_cam_.width_, sim_cam_.height_, "Vulkan", nullptr, nullptr);
        glfwSetWindowUserPointer(window, this);
        glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);

        if(!useUavOdom_){
            glfwSetKeyCallback(window, key_callback);
            glfwSetCursorPosCallback(window, cursor_position_callback);
            glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
        }

    }

    Eigen::Vector3d quatToEuler(const geometry_msgs::Quaternion &q){
        Eigen::Vector3d angles;

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles(0) = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles(1) = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles(2) = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }



    geometry_msgs::Quaternion eulerToQuat(Eigen::Vector3d angles) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        double cy = cos(angles(2) * 0.5);
        double sy = sin(angles(2) * 0.5);
        double cp = cos(angles(1) * 0.5);
        double sp = sin(angles(1) * 0.5);
        double cr = cos(angles(0) * 0.5);
        double sr = sin(angles(0) * 0.5);

        geometry_msgs::Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    double add_angle(double angle, double to_add){
        angle += to_add;
        if (angle > 2.0 * M_PI)
            angle -= 2.0 * M_PI;
        else if(angle < 0)
            angle += 2.0 * M_PI;
        return angle;
    }

    //Lukas Schmid, Michael Pantic, Raghav Khanna, Lionel Ott, Roland Siegwart, and Juan Nieto,
    //"An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments",
    //in IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 1500-1507, April 2020
    //https://github.com/ethz-asl/unreal_cv_ros
    void applyRandomWalk(const nav_msgs::OdometryConstPtr & odom_msg_i){

        ros::Duration diff = odom_msg_i->header.stamp - odom_noisy_.header.stamp;

        double max_tries = 20;

        int count = 0;
        while (count < max_tries){
            double distance = randomWalkDist(generator) * diff.toSec() * position_uncertainty;

            double theta = 2.0 * M_PI * random(generator);
            double phi = acos(1 - 2.0 * random(generator));
            Eigen::Vector3d direction(
                        sin(phi) * cos(theta),
                        sin(phi) * sin(theta),
                        cos(phi)
                        );

            if ((rw_offset.head(3) + direction*distance).norm() <= position_uncertainty){
                rw_offset.head(3) = rw_offset.head(3) + direction * distance;
                break;
            }else{
                count += 1;
            }
        }

        count = 0;
        while (count < max_tries){
            double offset = randomWalkDist(generator) * diff.toSec() * roll_pitch_uncertainty;
            offset *= (1.0 - 2.0 * randomInt(generator));
            if (abs(rw_offset[3] +
                    offset) < roll_pitch_uncertainty){
                rw_offset[3] = rw_offset[3] + offset;
                break;
            }
            else
                count += 1;
        }
        count = 0;

        while (count < max_tries){
            double offset = randomWalkDist(generator) * diff.toSec() * roll_pitch_uncertainty;
            offset *= (1.0 - 2.0 * randomInt(generator));
            if (abs(rw_offset[4] +
                    offset) < roll_pitch_uncertainty){
                rw_offset[4] = rw_offset[4] + offset;
                break;
            }
            else
                count += 1;
        }
        count = 0;

        while (count < max_tries){
            double offset = randomWalkDist(generator) * diff.toSec() * yaw_uncertainty;
            offset *= (1.0 - 2.0 * randomInt(generator));
            if (abs(rw_offset[5] +
                    offset) < yaw_uncertainty){
                rw_offset[5] = rw_offset[5] + offset;
                break;
            }
            else
                count += 1;
        }
        odom_noisy_ = *odom_msg_i;
        odom_noisy_.pose.pose.position.x = (*odom_msg_i).pose.pose.position.x+rw_offset.x();
        odom_noisy_.pose.pose.position.y = (*odom_msg_i).pose.pose.position.y+rw_offset.y();
        odom_noisy_.pose.pose.position.z = (*odom_msg_i).pose.pose.position.z+rw_offset.z();
        Eigen::Vector3d angles = quatToEuler(odom_noisy_.pose.pose.orientation);
        angles(0) = add_angle(angles(0), rw_offset[3] / 180.0 * M_PI);
        angles(1) = add_angle(angles(1), rw_offset[4] / 180.0 * M_PI);
        angles(2) = add_angle(angles(2), rw_offset[5] / 180.0 * M_PI);

        odom_noisy_.pose.pose.orientation = eulerToQuat(angles);

    }

    void syncOdometryGtNoise(const nav_msgs::OdometryConstPtr &gt_odom_msg_i, const nav_msgs::OdometryConstPtr &noisy_odommsg_i){

    }

    void uavOdomCallback(const nav_msgs::OdometryConstPtr & odom_msg_i){

        odom_ = *odom_msg_i;
        //        time_diff = (rospy.Time.now() - self.previous_time).to_sec()
        //        odom_.header.stamp = ros::Time::now(); //
        if(uavOdomIsReady_){
            if(applyOdomNoise){
                applyRandomWalk(odom_msg_i);

            }else{
                odom_noisy_ = *odom_msg_i;
            }

        }else{
            odom_noisy_ = *odom_msg_i;
        }
        pub_noisy_odom.publish(odom_noisy_);
        glm::quat q;
        q.w = odom_msg_i->pose.pose.orientation.w;
        q.x = odom_msg_i->pose.pose.orientation.x;
        q.y = odom_msg_i->pose.pose.orientation.y;
        q.z = odom_msg_i->pose.pose.orientation.z;

        q = q*cam.r_oc;

        //ros coordinate system: z up, x front y left
        //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        cam._roll = std::fmod(std::atan2(sinr_cosp, cosr_cosp),M_PI*2);
        //        cam._roll = M_PI/4.;
        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            cam._pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            cam._pitch = std::asin(sinp);
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        cam._yaw = std::atan2(siny_cosp, cosy_cosp);
        cam._eye.z = odom_msg_i->pose.pose.position.x;
        cam._eye.x = odom_msg_i->pose.pose.position.y;
        cam._eye.y = odom_msg_i->pose.pose.position.z;

        cam._eye += cam.t_oc;
        uavOdomIsReady_  = true;

    }


    static void framebufferResizeCallback(GLFWwindow* window, int width, int height) {
        auto app = reinterpret_cast<DepthSimApp*>(glfwGetWindowUserPointer(window));
        WIDTH = width;
        HEIGHT = height;

        _mouse_pos.H = height;
        _mouse_pos.W = width;
        app->framebufferResized = true;
    }

    void initVulkan() {
        createInstance();
        setupDebugMessenger();
        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();
        createSwapChain();
        createImageViews();
        createRenderPass();
        createDescriptorSetLayout();
        createGraphicsPipeline();
        createCommandPool();
        createDepthResources();
        createFramebuffers();
        //        createTextureImage();
        //        createTextureImageView();
        //        createTextureSampler();
        createVertexBuffer();
        createIndexBuffer();
        createUniformBuffers();
        createDescriptorPool();
        createDescriptorSets();
        createCommandBuffers();
        createSyncObjects();

    }


    float depthSample(float linearDepth)
    {
        float nonLinearDepth = (100. + 0.01 - 2.0 * 0.01 * 100. / linearDepth) / (100. - 0.01);
        nonLinearDepth = (nonLinearDepth + 1.0) / 2.0;
        return nonLinearDepth;
    }

    float linearize_depth(float d,float zNear,float zFar)
    {
        return zNear * zFar / (zFar + d * (zNear - zFar));
    }
    int count = 0;
    void mainLoop(ulong dt) {


        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ pt;

        if (!glfwWindowShouldClose(window)) {
            if(uavOdomIsReady_ || !useUavOdom_){
                cloud.clear();
                drawFrame(dt);

                if(count<=1){
                    //skip the first frame
                    count++;
                    return;
                }
                float depth[ sim_cam_.width_*sim_cam_.height_];
                retrieveDepth(depth);
                sim_cam_.linearize_depth_buffer(depth);

                //to use for generate GT visible mesh
                //                scene.triangleTagging(depth,cam);
//                float depth2[ sim_cam_.width_*sim_cam_.height_];
//                memcpy(depth2, depth, sim_cam_.width_*sim_cam_.height_*sizeof(float));
                if(applyDepthNoise)
                    sim_cam_.add_z_noise(depth);

                cv::Mat im_z(sim_cam_.height_, sim_cam_.width_, CV_32F, depth);


//                cv::Mat im(sim_cam_.height_, sim_cam_.width_, CV_32F, depth2);

                cv::Mat im = im_z.clone();

                cloud.points.reserve(im.rows*im.cols);
                std::random_device rd;
                std::mt19937 mt(rd());
                std::uniform_real_distribution<float> dist(0.f, 1.f);

                for(int i = 0; i < im.rows; i++)
                {
                    float* Mi = im.ptr<float>(i);
                    float *Mi_zim = im_z.ptr<float>(i);
                    for(int j = 0; j < im.cols; j++){

                        Eigen::Vector3f pos;
                        float z = Mi[j];
                        pos.x() = ((j-sim_cam_.ppx_)*z)/sim_cam_.focal_;
                        pos.y() = ((i-sim_cam_.ppy_)*z)/sim_cam_.focal_;
                        pos.z() = z; //avoid border effect with noise
                        float d = pos.norm();
                        if(d < sim_cam_.near_ || d > sim_cam_.far_){//fake pts
                            Mi[j] = 0.;//convert [mm]
                            Mi_zim[j]=0.;
                            pos.x() = ((j-sim_cam_.ppx_))/sim_cam_.focal_;
                            pos.y() = ((i-sim_cam_.ppy_))/sim_cam_.focal_;
                            pos.z() = 1;
                            pos.normalize();
                            pos*=8;
                            pt.getVector3fMap() = pos;
                            cloud.points.push_back(pt);
                        }else{
                            pos.z() = Mi[j];
                            d = pos.norm();
//                            d=pos.z();
                            Mi[j] = std::floor(d*1000.f+0.5);//convert [mm]
                            Mi_zim[j] = std::floor(Mi_zim[j]*1000.f+0.5);//convert [mm]
                            pt.getVector3fMap() = pos;
                            cloud.points.push_back(pt);


                        }
                    }
                }

                im.convertTo(im, CV_16UC1);
                im_z.convertTo(im_z, CV_16UC1);
                ros::Time odom_ts;
                odom_ts = ros::Time::now();

                if(!useUavOdom_){
                    odom_ts = ros::Time::now();

                }else{
                    odom_ts = ros::Time(odom_.header.stamp);

                }




                //            cv::normalize(im, im, 0,255, cv::NORM_MINMAX, CV_8UC1);
                //            cv::normalize(imtmp, imtmp, 0,255, cv::NORM_MINMAX, CV_8UC1);
                //            cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
                //            cv::imshow("Display Image", im);
                //            cv::imshow("ORtho", imtmp);
                //            cv::waitKey(1);

                sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", im).toImageMsg();
                sensor_msgs::ImagePtr z_depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", im_z).toImageMsg();




                nav_msgs::Odometry odom;
                odom = odom_noisy_;
                odom.header.stamp  = odom_ts;
                odom.header.frame_id = "world";
                depth_msg->header.stamp = odom_ts;
                depth_msg->header.frame_id = "world";

                z_depth_msg->header.stamp = odom_ts;
                z_depth_msg->header.frame_id = "world";

                //                double cy = cos(cam._yaw * 0.5);
                //                double sy = sin(cam._yaw   * 0.5);
                //                double cp = cos(cam._pitch * 0.5);
                //                double sp = sin(cam._pitch * 0.5);

                //                double cr = cos(cam._roll * 0.5);
                //                double sr = sin(cam._roll * 0.5);
                //                glm::quat q;
                //                q.w = cy * cp * cr + sy * sp * sr;
                //                q.x = cy * cp * sr - sy * sp * cr;
                //                q.y = sy * cp * sr + cy * sp * cr;
                //                q.z = sy * cp * cr - cy * sp * sr;

                glm::quat qRoll = glm::angleAxis(cam._roll, glm::vec3(1, 0, 0));
                glm::quat qPitch =  glm::angleAxis(cam._pitch, glm::vec3(0, 1, 0));
                glm::quat qYaw = glm::angleAxis(cam._yaw, glm::vec3(0, 0, 1));

                ///x,y,z are in radians
                glm::quat q =  qRoll*qYaw*qPitch;

                Eigen::Isometry3f T;
                Eigen::Quaterniond r =  Eigen::Quaterniond(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);

                Eigen::Matrix3f R;
                Eigen::Matrix3f newR;
                Eigen::Vector3f t;

                if(useUavOdom_){
                    //                    newR.col(0) = -1.*R.col(1);
                    //                    newR.col(1) = -1.*R.col(2);
                    //                    newR.col(2) = R.col(0);
                    geometry_msgs::Quaternion qg;
                    qg.w = r.w();
                    qg.x = r.x();
                    qg.y = r.y();
                    qg.z = r.z();
                    Eigen::Vector3d angles = quatToEuler(qg).transpose();
                    glm::quat qRoll = glm::angleAxis((float)angles(0), glm::vec3(1, 0, 0));
                    glm::quat qPitch =  glm::angleAxis((float)angles(1), glm::vec3(0, 1, 0));
                    glm::quat qYaw = glm::angleAxis((float)angles(2), glm::vec3(0, 0, 1));

                    ///x,y,z are in radians
                    q =  qRoll*qYaw*qPitch;
                    R =  Eigen::Quaternionf(q.w,q.x,q.y,q.z).toRotationMatrix();
                    //                    std::cout<<cam._roll<<" "<< cam._pitch << " " << cam._yaw << std::endl;
                    //                    std::cout<<angles.transpose() << std::endl;
                    newR = R;
                    odom.pose.pose.orientation.w = q.w;
                    odom.pose.pose.orientation.x = q.x;
                    odom.pose.pose.orientation.y = q.y;
                    odom.pose.pose.orientation.z = q.z;
                    t=Eigen::Vector3f(odom.pose.pose.position.x,
                                      odom.pose.pose.position.y,
                                      odom.pose.pose.position.z);
                }else{

                    R =  Eigen::Quaternionf(q.w,q.x,q.y,q.z).toRotationMatrix();
                    newR = R;

                    t=Eigen::Vector3f(cam._eye.z,cam._eye.x,cam._eye.y);
                    odom.pose.pose.position.x = cam._eye.z;
                    odom.pose.pose.position.y = cam._eye.x;
                    odom.pose.pose.position.z = cam._eye.y;

                    odom.pose.pose.orientation.w = q.w;
                    odom.pose.pose.orientation.x = q.x;
                    odom.pose.pose.orientation.y = q.y;
                    odom.pose.pose.orientation.z = q.z;

                }


                bool inWorldCoordinate = true;
                //apply Rotation to avoid large error due to tf
                if(inWorldCoordinate){
                    for(size_t i = 0; i < cloud.points.size(); ++i){
                        Eigen::Vector3f tmp_pt(cloud.points[i].z,-cloud.points[i].x,-cloud.points[i].y);
                        //                        tmp_pt = newR*tmp_pt+t;
                        tmp_pt = newR*tmp_pt;
                        cloud.points[i].getVector3fMap() = tmp_pt;
                    }
                }
                cloud.width = cloud.points.size();
                cloud.height = 1;
                cloud.is_dense = true;
                cloud.header.frame_id = "vulkan_camera_link";


                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(cloud, cloud_msg);
                cloud_msg.header.stamp = odom_ts;

                //                odom.pose.pose.position.x = cam._eye.z;
                //                odom.pose.pose.position.y = cam._eye.x;
                //                odom.pose.pose.position.z = cam._eye.y;

                //point cloud already in world frame
                static tf2_ros::TransformBroadcaster br;

                geometry_msgs::TransformStamped transformStamped;

                transformStamped.header.stamp = odom_ts;
                transformStamped.header.frame_id = "world";
                transformStamped.child_frame_id = "vulkan_camera_link";

                transformStamped.transform.translation.x = cam._eye.z;
                transformStamped.transform.translation.y = cam._eye.x;
                transformStamped.transform.translation.z = cam._eye.y;
                //                transformStamped.transform.rotation.x = r.x();
                //                transformStamped.transform.rotation.y = r.y();
                //                transformStamped.transform.rotation.z = r.z();
                //                transformStamped.transform.rotation.w = r.w();
                //                transformStamped.transform.translation.x = 0;
                //                transformStamped.transform.translation.y = 0;
                //                transformStamped.transform.translation.z = 0;


                transformStamped.transform.rotation.x = 0;
                transformStamped.transform.rotation.y = 0;
                transformStamped.transform.rotation.z = 0;
                transformStamped.transform.rotation.w = 1;

                br.sendTransform(transformStamped);
                //                cloud_transform_pub_.publish(transformStamped);
                cloud_pub_.publish(cloud_msg);

                pub_odom.publish(odom);//odom with identic camera timestamp

                //                if(send_pos){
                //                    nav_msgs::Path waypoints;
                //                    waypoints.header.frame_id = std::string("world");
                //                    waypoints.header.stamp = ros::Time::now();
                //                    geometry_msgs::PoseStamped init_pose;
                //                    init_pose.pose.position.x = cam._eye.z;
                //                    init_pose.pose.position.y = cam._eye.x;
                //                    init_pose.pose.position.z = cam._eye.y;
                //                    waypoints.poses.insert(waypoints.poses.begin(), init_pose);
                //                    // pub2.publish(waypoints);
                //                    waypts_pub.publish(waypoints);
                //                    send_pos = false;
                //                }

                //                sim_cam_.send_depth(depth,odom);
                depth_pub_.publish(depth_msg);
                z_depth_pub_.publish(z_depth_msg);
            }
        }

        vkDeviceWaitIdle(device);
    }

    void cleanupSwapChain() {
        vkDestroyImageView(device, depthImageView, nullptr);
        vkDestroyImage(device, depthImage, nullptr);
        vkFreeMemory(device, depthImageMemory, nullptr);

        for (auto framebuffer : swapChainFramebuffers) {
            vkDestroyFramebuffer(device, framebuffer, nullptr);
        }

        vkFreeCommandBuffers(device, commandPool, static_cast<uint32_t>(commandBuffers.size()), commandBuffers.data());

        vkDestroyPipeline(device, graphicsPipeline, nullptr);
        vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
        vkDestroyRenderPass(device, renderPass, nullptr);

        for (auto imageView : swapChainImageViews) {
            vkDestroyImageView(device, imageView, nullptr);
        }

        vkDestroySwapchainKHR(device, swapChain, nullptr);

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            vkDestroyBuffer(device, uniformBuffers[i], nullptr);
            vkFreeMemory(device, uniformBuffersMemory[i], nullptr);
        }

        vkDestroyDescriptorPool(device, descriptorPool, nullptr);

    }

    void cleanup() {
        cleanupSwapChain();

        //        vkDestroySampler(device, textureSampler, nullptr);
        //        vkDestroyImageView(device, textureImageView, nullptr);
        //
        //        vkDestroyImage(device, textureImage, nullptr);
        //        vkFreeMemory(device, textureImageMemory, nullptr);

        vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

        vkDestroyBuffer(device, indexBuffer, nullptr);
        vkFreeMemory(device, indexBufferMemory, nullptr);

        vkDestroyBuffer(device, vertexBuffer, nullptr);
        vkFreeMemory(device, vertexBufferMemory, nullptr);

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
            vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
            vkDestroyFence(device, inFlightFences[i], nullptr);
        }

        vkDestroyCommandPool(device, commandPool, nullptr);

        vkDestroyDevice(device, nullptr);

        if (enableValidationLayers) {
            DestroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
        }

        vkDestroySurfaceKHR(instance, surface, nullptr);
        vkDestroyInstance(instance, nullptr);

        glfwDestroyWindow(window);

        glfwTerminate();
    }

    void recreateSwapChain() {
        int width = 0, height = 0;
        while (width == 0 || height == 0) {
            glfwGetFramebufferSize(window, &width, &height);
            glfwWaitEvents();
        }

        vkDeviceWaitIdle(device);

        cleanupSwapChain();

        createSwapChain();
        createImageViews();
        createRenderPass();
        createGraphicsPipeline();
        createDepthResources();
        createFramebuffers();
        createUniformBuffers();
        createDescriptorPool();
        createDescriptorSets();
        createCommandBuffers();
    }

    void createInstance() {
        if (enableValidationLayers && !checkValidationLayerSupport()) {
            throw std::runtime_error("validation layers requested, but not available!");
        }

        VkApplicationInfo appInfo = {};
        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "Hello Triangle";
        appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.pEngineName = "No Engine";
        appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.apiVersion = VK_API_VERSION_1_0;

        VkInstanceCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;

        auto extensions = getRequiredExtensions();
        createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();

        VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo;
        if (enableValidationLayers) {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
            createInfo.ppEnabledLayerNames = validationLayers.data();

            populateDebugMessengerCreateInfo(debugCreateInfo);
            createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*) &debugCreateInfo;
        } else {
            createInfo.enabledLayerCount = 0;

            createInfo.pNext = nullptr;
        }

        if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
            throw std::runtime_error("failed to create instance!");
        }
    }

    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo) {
        createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        createInfo.pfnUserCallback = debugCallback;
    }

    void setupDebugMessenger() {
        if (!enableValidationLayers) return;

        VkDebugUtilsMessengerCreateInfoEXT createInfo;
        populateDebugMessengerCreateInfo(createInfo);

        if (CreateDebugUtilsMessengerEXT(instance, &createInfo, nullptr, &debugMessenger) != VK_SUCCESS) {
            throw std::runtime_error("failed to set up debug messenger!");
        }
    }

    void createSurface() {
        if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
            throw std::runtime_error("failed to create window surface!");
        }
    }
    std::string physicalDeviceTypeString(VkPhysicalDeviceType type)
    {
        switch (type)
        {
#define STR(r) case VK_PHYSICAL_DEVICE_TYPE_ ##r: return #r
        STR(OTHER);
        STR(INTEGRATED_GPU);
        STR(DISCRETE_GPU);
        STR(VIRTUAL_GPU);
        STR(CPU);
#undef STR
        default: return "UNKNOWN_DEVICE_TYPE";
        }
    }

    void pickPhysicalDevice() {
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);
        if (deviceCount == 0) {
            throw std::runtime_error("failed to find GPUs with Vulkan support!");
        }

        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());
        int i=0;
        for (const auto& device : devices) {

            VkPhysicalDeviceProperties deviceProperties;
            vkGetPhysicalDeviceProperties(device, &deviceProperties);

            std::cout << "Device [" << i++ << "] : " << deviceProperties.deviceName << std::endl;
            std::cout << " Type: " << physicalDeviceTypeString(deviceProperties.deviceType) << "\n";
            std::cout << " API: " << (deviceProperties.apiVersion >> 22) << "." << ((deviceProperties.apiVersion >> 12) & 0x3ff) << "." << (deviceProperties.apiVersion & 0xfff) << "\n";

        }
        for (const auto& device : devices) {

            VkPhysicalDeviceProperties deviceProperties;
            vkGetPhysicalDeviceProperties(device, &deviceProperties);

            if (isDeviceSuitable(device) && deviceProperties.deviceType == VkPhysicalDeviceType::VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
                physicalDevice = device;

                break;
            }
        }

        if (physicalDevice == VK_NULL_HANDLE) {
            throw std::runtime_error("failed to find a suitable GPU!");
        }
    }

    void createLogicalDevice() {
        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);

        std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
        std::set<uint32_t> uniqueQueueFamilies = {indices.graphicsFamily.value(), indices.presentFamily.value()};

        float queuePriority = 1.0f;
        for (uint32_t queueFamily : uniqueQueueFamilies) {
            VkDeviceQueueCreateInfo queueCreateInfo = {};
            queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
            queueCreateInfo.queueFamilyIndex = queueFamily;
            queueCreateInfo.queueCount = 1;
            queueCreateInfo.pQueuePriorities = &queuePriority;
            queueCreateInfos.push_back(queueCreateInfo);
        }

        VkPhysicalDeviceFeatures deviceFeatures = {};
        deviceFeatures.fillModeNonSolid = VK_TRUE;


        VkDeviceCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;

        createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
        createInfo.pQueueCreateInfos = queueCreateInfos.data();

        createInfo.pEnabledFeatures = &deviceFeatures;

        createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();

        if (enableValidationLayers) {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
            createInfo.ppEnabledLayerNames = validationLayers.data();
        } else {
            createInfo.enabledLayerCount = 0;
        }

        if (vkCreateDevice(physicalDevice, &createInfo, nullptr, &device) != VK_SUCCESS) {
            throw std::runtime_error("failed to create logical device!");
        }

        vkGetDeviceQueue(device, indices.graphicsFamily.value(), 0, &graphicsQueue);
        vkGetDeviceQueue(device, indices.presentFamily.value(), 0, &presentQueue);
    }

    void createSwapChain() {
        SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);

        VkSurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.formats);
        VkPresentModeKHR presentMode = chooseSwapPresentMode(swapChainSupport.presentModes);
        VkExtent2D extent = chooseSwapExtent(swapChainSupport.capabilities);

        uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
        if (swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount) {
            imageCount = swapChainSupport.capabilities.maxImageCount;
        }

        VkSwapchainCreateInfoKHR createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = surface;

        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
        uint32_t queueFamilyIndices[] = {indices.graphicsFamily.value(), indices.presentFamily.value()};

        if (indices.graphicsFamily != indices.presentFamily) {
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            createInfo.queueFamilyIndexCount = 2;
            createInfo.pQueueFamilyIndices = queueFamilyIndices;
        } else {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }

        createInfo.preTransform = swapChainSupport.capabilities.currentTransform;
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;

        if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS) {
            throw std::runtime_error("failed to create swap chain!");
        }

        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
        swapChainImages.resize(imageCount);
        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());

        swapChainImageFormat = surfaceFormat.format;
        swapChainExtent = extent;
    }


    void createImageViews() {
        swapChainImageViews.resize(swapChainImages.size());

        for (uint32_t i = 0; i < swapChainImages.size(); i++) {
            swapChainImageViews[i] = createImageView(swapChainImages[i], swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);
        }
    }

    void createRenderPass() {
        VkAttachmentDescription colorAttachment = {};
        colorAttachment.format = swapChainImageFormat;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;


        VkAttachmentDescription depthAttachment = {};
        depthAttachment.format = findDepthFormat();
        depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorAttachmentRef = {};
        colorAttachmentRef.attachment = 0;
        colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference depthAttachmentRef = {};
        depthAttachmentRef.attachment = 1;
        depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass = {};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorAttachmentRef;
        subpass.pDepthStencilAttachment = &depthAttachmentRef;

        VkSubpassDependency dependency = {};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.srcAccessMask = 0;
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        std::array<VkAttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};
        VkRenderPassCreateInfo renderPassInfo = {};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        renderPassInfo.pAttachments = attachments.data();
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        renderPassInfo.dependencyCount = 1;
        renderPassInfo.pDependencies = &dependency;

        if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS) {
            throw std::runtime_error("failed to create render pass!");
        }
    }

    void createDescriptorSetLayout() {
        VkDescriptorSetLayoutBinding uboLayoutBinding = {};
        uboLayoutBinding.binding = 0;
        uboLayoutBinding.descriptorCount = 1;
        uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        uboLayoutBinding.pImmutableSamplers = nullptr;
        uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;


        //        VkDescriptorSetLayoutBinding samplerLayoutBinding = {};
        //        samplerLayoutBinding.binding = 1;
        //        samplerLayoutBinding.descriptorCount = 1;
        //        samplerLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        //        samplerLayoutBinding.pImmutableSamplers = nullptr;
        //        samplerLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        //        std::array<VkDescriptorSetLayoutBinding, 2> bindings = {uboLayoutBinding, samplerLayoutBinding};
        std::array<VkDescriptorSetLayoutBinding, 1> bindings = {uboLayoutBinding};
        VkDescriptorSetLayoutCreateInfo layoutInfo = {};
        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
        layoutInfo.pBindings = bindings.data();

        if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) {
            throw std::runtime_error("failed to create descriptor set layout!");
        }
    }

    void createGraphicsPipeline() {
        auto vertShaderCode = readFile(shaders_path+"/vert.spv");
        auto fragShaderCode = readFile(shaders_path+"/frag.spv");

        VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
        VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

        VkPipelineShaderStageCreateInfo vertShaderStageInfo = {};
        vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertShaderStageInfo.module = vertShaderModule;
        vertShaderStageInfo.pName = "main";

        VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
        fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragShaderStageInfo.module = fragShaderModule;
        fragShaderStageInfo.pName = "main";

        VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};

        VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

        auto bindingDescription = Vertex::getBindingDescription();
        auto attributeDescriptions = Vertex::getAttributeDescriptions();

        vertexInputInfo.vertexBindingDescriptionCount = 1;
        vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
        vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
        vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();


        VkPipelineInputAssemblyStateCreateInfo inputAssembly = {};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        inputAssembly.primitiveRestartEnable = VK_FALSE;

        VkViewport viewport = {};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = (float) swapChainExtent.width;
        viewport.height = (float) swapChainExtent.height;
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;

        VkRect2D scissor = {};
        scissor.offset = {0, 0};
        scissor.extent = swapChainExtent;

        VkPipelineViewportStateCreateInfo viewportState = {};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.pViewports = &viewport;
        viewportState.scissorCount = 1;
        viewportState.pScissors = &scissor;

        VkPipelineRasterizationStateCreateInfo rasterizer = {};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_TRUE;
        rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.depthBiasEnable = VK_FALSE;

        VkPipelineMultisampleStateCreateInfo multisampling = {};
        multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.sampleShadingEnable = VK_TRUE;
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depthStencil = {};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_TRUE;
        depthStencil.depthWriteEnable = VK_TRUE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
        depthStencil.depthBoundsTestEnable = VK_FALSE;
        depthStencil.minDepthBounds = 0.0f; // Optional
        depthStencil.maxDepthBounds = 15.0f; // Optional
        depthStencil.stencilTestEnable = VK_FALSE;
        depthStencil.front = {}; // Optional
        depthStencil.back = {}; // Optional

        VkPipelineColorBlendAttachmentState colorBlendAttachment = {};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        colorBlendAttachment.blendEnable = VK_FALSE;//TRYUE FOR ALPHA AND MODIFY SHADER
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;


        VkPipelineColorBlendStateCreateInfo colorBlending = {};
        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.logicOpEnable = VK_FALSE;
        colorBlending.logicOp = VK_LOGIC_OP_COPY;
        colorBlending.attachmentCount = 1;
        colorBlending.pAttachments = &colorBlendAttachment;
        colorBlending.blendConstants[0] = 0.0f;
        colorBlending.blendConstants[1] = 0.0f;
        colorBlending.blendConstants[2] = 0.0f;
        colorBlending.blendConstants[3] = 0.0f;


        VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;

        if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
            throw std::runtime_error("failed to create pipeline layout!");
        }

        VkGraphicsPipelineCreateInfo pipelineInfo = {};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 2;
        pipelineInfo.pStages = shaderStages;
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlending;
        pipelineInfo.layout = pipelineLayout;
        pipelineInfo.renderPass = renderPass;
        pipelineInfo.subpass = 0;
        pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;

        if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS) {
            throw std::runtime_error("failed to create graphics pipeline!");
        }

        vkDestroyShaderModule(device, fragShaderModule, nullptr);
        vkDestroyShaderModule(device, vertShaderModule, nullptr);
    }


    void createFramebuffers() {
        swapChainFramebuffers.resize(swapChainImageViews.size());

        for (size_t i = 0; i < swapChainImageViews.size(); i++) {
            std::array<VkImageView, 2> attachments = {
                swapChainImageViews[i],
                depthImageView
            };

            VkFramebufferCreateInfo framebufferInfo = {};
            framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            framebufferInfo.renderPass = renderPass;
            framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
            framebufferInfo.pAttachments = attachments.data();
            framebufferInfo.width = swapChainExtent.width;
            framebufferInfo.height = swapChainExtent.height;
            framebufferInfo.layers = 1;

            if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS) {
                throw std::runtime_error("failed to create framebuffer!");
            }
        }
    }
    void createCommandPool() {
        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);

        VkCommandPoolCreateInfo poolInfo = {};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

        if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS) {
            throw std::runtime_error("failed to create command pool!");
        }
    }



    VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags) {
        VkImageViewCreateInfo viewInfo = {};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspectFlags;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        VkImageView imageView;
        if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
            throw std::runtime_error("failed to create texture image view!");
        }

        return imageView;
    }

    void createImage(uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory) {
        VkImageCreateInfo imageInfo = {};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = width;
        imageInfo.extent.height = height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = format;
        imageInfo.tiling = tiling;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = usage;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(device, &imageInfo, nullptr, &image) != VK_SUCCESS) {
            throw std::runtime_error("failed to create image!");
        }

        VkMemoryRequirements memRequirements;
        vkGetImageMemoryRequirements(device, image, &memRequirements);

        VkMemoryAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(device, &allocInfo, nullptr, &imageMemory) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate image memory!");
        }

        vkBindImageMemory(device, image, imageMemory, 0);
    }

    void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout) {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkImageMemoryBarrier barrier = {};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;

        if (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

            if (hasStencilComponent(format)) {
                barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
            }
        } else {
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        }

        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        VkPipelineStageFlags sourceStage;
        VkPipelineStageFlags destinationStage;

        if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        } else {
            throw std::invalid_argument("unsupported layout transition!");
        }

        vkCmdPipelineBarrier(
                    commandBuffer,
                    sourceStage, destinationStage,
                    0,
                    0, nullptr,
                    0, nullptr,
                    1, &barrier
                    );

        endSingleTimeCommands(commandBuffer);
    }

    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height) {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkBufferImageCopy region = {};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;
        region.imageOffset = {0, 0, 0};
        region.imageExtent = {
            width,
            height,
            1
        };

        vkCmdCopyBufferToImage(commandBuffer, buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        endSingleTimeCommands(commandBuffer);
    }

    void createDepthResources() {
        VkFormat depthFormat = findDepthFormat();

        createImage(swapChainExtent.width, swapChainExtent.height, depthFormat, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, depthImage, depthImageMemory);
        depthImageView = createImageView(depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

        transitionImageLayout(depthImage, depthFormat, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
    }

    VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features) {
        for (VkFormat format : candidates) {
            VkFormatProperties props;
            vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);

            if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features) {
                return format;
            } else if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features) {
                return format;
            }
        }

        throw std::runtime_error("failed to find supported format!");
    }

    VkFormat findDepthFormat() {
        return findSupportedFormat(
        {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT},
                    VK_IMAGE_TILING_OPTIMAL,
                    VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
                    );
    }

    bool hasStencilComponent(VkFormat format) {
        return format == VK_FORMAT_D32_SFLOAT_S8_UINT || format == VK_FORMAT_D24_UNORM_S8_UINT;
    }

    void createVertexBuffer() {
        {
            VkDeviceSize bufferSize = sizeof(scene.getStatic_vertices()[0]) * scene.getStatic_vertices().size();

            VkBuffer stagingBuffer;
            VkDeviceMemory stagingBufferMemory;

            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer,
                         stagingBufferMemory);

            void *data;
            vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
            memcpy(data, scene.getStatic_vertices().data(), (size_t) bufferSize);
            vkUnmapMemory(device, stagingBufferMemory);

            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

            copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

            vkDestroyBuffer(device, stagingBuffer, nullptr);
            vkFreeMemory(device, stagingBufferMemory, nullptr);

        }



    }

    void createUniformBuffers() {
        VkDeviceSize bufferSize = sizeof(UniformBufferObject);

        uniformBuffers.resize(swapChainImages.size());
        uniformBuffersMemory.resize(swapChainImages.size());

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            createBuffer(bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                         uniformBuffers[i], uniformBuffersMemory[i]);
        }
    }

    void createDescriptorPool() {
        std::array<VkDescriptorPoolSize, 1> poolSizes = {};
        poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        poolSizes[0].descriptorCount = static_cast<uint32_t>(swapChainImages.size());
        //        poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        //        poolSizes[1].descriptorCount = static_cast<uint32_t>(swapChainImages.size());

        VkDescriptorPoolCreateInfo poolInfo = {};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();
        poolInfo.maxSets = static_cast<uint32_t>(swapChainImages.size());

        if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
            throw std::runtime_error("failed to create descriptor pool!");
        }
    }

    void createDescriptorSets() {
        std::vector<VkDescriptorSetLayout> layouts(swapChainImages.size(), descriptorSetLayout);
        VkDescriptorSetAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = descriptorPool;
        allocInfo.descriptorSetCount = static_cast<uint32_t>(swapChainImages.size());
        allocInfo.pSetLayouts = layouts.data();

        descriptorSets.resize(swapChainImages.size());
        if (vkAllocateDescriptorSets(device, &allocInfo, descriptorSets.data()) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate descriptor sets!");
        }

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            VkDescriptorBufferInfo bufferInfo = {};
            bufferInfo.buffer = uniformBuffers[i];
            bufferInfo.offset = 0;
            bufferInfo.range = sizeof(UniformBufferObject);

            //            VkDescriptorImageInfo imageInfo = {};
            //            imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            //            imageInfo.imageView = textureImageView;
            //            imageInfo.sampler = textureSampler;

            //            std::array<VkWriteDescriptorSet, 2> descriptorWrites = {};
            std::array<VkWriteDescriptorSet, 1> descriptorWrites = {};

            descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptorWrites[0].dstSet = descriptorSets[i];
            descriptorWrites[0].dstBinding = 0;
            descriptorWrites[0].dstArrayElement = 0;
            descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            descriptorWrites[0].descriptorCount = 1;
            descriptorWrites[0].pBufferInfo = &bufferInfo;

            //            descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            //            descriptorWrites[1].dstSet = descriptorSets[i];
            //            descriptorWrites[1].dstBinding = 1;
            //            descriptorWrites[1].dstArrayElement = 0;
            //            descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            //            descriptorWrites[1].descriptorCount = 1;
            //            descriptorWrites[1].pImageInfo = &imageInfo;

            vkUpdateDescriptorSets(device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
        }
    }
    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory) {
        VkBufferCreateInfo bufferInfo = {};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to create buffer!");
        }

        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

        VkMemoryAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);
        if ((vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory)) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate buffer memory!");
        }

        vkBindBufferMemory(device, buffer, bufferMemory, 0);
    }

    VkCommandBuffer beginSingleTimeCommands() {
        VkCommandBufferAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

        VkCommandBufferBeginInfo beginInfo = {};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(commandBuffer, &beginInfo);

        return commandBuffer;
    }

    void endSingleTimeCommands(VkCommandBuffer commandBuffer) {
        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo = {};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);

        vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    }

    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size) {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkBufferCopy copyRegion = {};
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);

        endSingleTimeCommands(commandBuffer);
    }

    void createIndexBuffer() {
        {
            VkDeviceSize bufferSize = sizeof(scene.getStatic_indices()[0]) * scene.getStatic_indices().size();

            VkBuffer stagingBuffer;
            VkDeviceMemory stagingBufferMemory;
            std::cout << (int) sizeof(scene.getStatic_indices()[0]) << " " << scene.getStatic_indices().size()<<std::endl;

            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer,
                         stagingBufferMemory);

            void *data;
            vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
            memcpy(data, scene.getStatic_indices().data(), (size_t) bufferSize);
            vkUnmapMemory(device, stagingBufferMemory);

            createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

            copyBuffer(stagingBuffer, indexBuffer, bufferSize);

            vkDestroyBuffer(device, stagingBuffer, nullptr);
            vkFreeMemory(device, stagingBufferMemory, nullptr);
        }

    }



    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }

        throw std::runtime_error("failed to find suitable memory type!");
    }

    void createCommandBuffers() {
        commandBuffers.resize(swapChainFramebuffers.size());

        VkCommandBufferAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = commandPool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = (uint32_t) commandBuffers.size();

        if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate command buffers!");
        }

        for (size_t i = 0; i < commandBuffers.size(); i++) {
            VkCommandBufferBeginInfo beginInfo = {};
            beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;

            if (vkBeginCommandBuffer(commandBuffers[i], &beginInfo) != VK_SUCCESS) {
                throw std::runtime_error("failed to begin recording command buffer!");
            }

            VkRenderPassBeginInfo renderPassInfo = {};
            renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            renderPassInfo.renderPass = renderPass;
            renderPassInfo.framebuffer = swapChainFramebuffers[i];
            renderPassInfo.renderArea.offset = {0, 0};
            renderPassInfo.renderArea.extent = swapChainExtent;

            std::array<VkClearValue, 2> clearValues = {};
            clearValues[0].color = {0.0f, 0.0f, 0.0f, 1.0f};
            clearValues[1].depthStencil = {1.f, 0};

            renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
            renderPassInfo.pClearValues = clearValues.data();

            vkCmdBeginRenderPass(commandBuffers[i], &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

            vkCmdBindPipeline(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);

            VkBuffer vertexBuffers[] = {vertexBuffer};
            VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(commandBuffers[i], 0, 1, vertexBuffers, offsets);

            vkCmdBindIndexBuffer(commandBuffers[i], indexBuffer, 0, VK_INDEX_TYPE_UINT32);

            vkCmdBindDescriptorSets(commandBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[i], 0, nullptr);

            vkCmdDrawIndexed(commandBuffers[i], static_cast<uint32_t>(scene.getStatic_indices().size()), 1, 0, 0, 0);

            vkCmdEndRenderPass(commandBuffers[i]);
            if (vkEndCommandBuffer(commandBuffers[i]) != VK_SUCCESS) {
                throw std::runtime_error("failed to record command buffer!");
            }
        }
    }


    void createSyncObjects() {
        imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

        VkSemaphoreCreateInfo semaphoreInfo = {};
        semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

        VkFenceCreateInfo fenceInfo = {};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
                    vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
                    vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS) {
                throw std::runtime_error("failed to create synchronization objects for a frame!");
            }
        }
    }


    void updateUniformBuffer(uint32_t currentImage) {

        UniformBufferObject ubo = {};

        glm::mat4 proj = glm::perspective(glm::radians(sim_cam_.fovy_), (float) swapChainExtent.width / (float) swapChainExtent.height, sim_cam_.near_, sim_cam_.far_);
        proj[1][1] *= -1.;
        //        proj *= 1/5.0;
        ubo.transform = proj*cam._viewMatrix;

        void* data;
        vkMapMemory(device, uniformBuffersMemory[currentImage], 0, sizeof(ubo), 0, &data);
        memcpy(data, &ubo, sizeof(ubo));
        vkUnmapMemory(device, uniformBuffersMemory[currentImage]);
    }

    void drawFrame(unsigned long dt) {
        vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, std::numeric_limits<uint64_t>::max());

        uint32_t imageIndex;
        VkResult result = vkAcquireNextImageKHR(device, swapChain, std::numeric_limits<uint64_t>::max(), imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);

        if (result == VK_ERROR_OUT_OF_DATE_KHR) {
            recreateSwapChain();
            return;
        } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
            throw std::runtime_error("failed to acquire swap chain image!");
        }

        cam.update(dt);
        updateUniformBuffer(imageIndex);

        VkSubmitInfo submitInfo = {};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

        VkSemaphore waitSemaphores[] = {imageAvailableSemaphores[currentFrame]};
        VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
        submitInfo.waitSemaphoreCount = 1;
        submitInfo.pWaitSemaphores = waitSemaphores;
        submitInfo.pWaitDstStageMask = waitStages;

        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffers[imageIndex];

        VkSemaphore signalSemaphores[] = {renderFinishedSemaphores[currentFrame]};
        submitInfo.signalSemaphoreCount = 1;
        submitInfo.pSignalSemaphores = signalSemaphores;

        vkResetFences(device, 1, &inFlightFences[currentFrame]);

        if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS) {
            throw std::runtime_error("failed to submit draw command buffer!");
        }

        VkPresentInfoKHR presentInfo = {};
        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

        presentInfo.waitSemaphoreCount = 1;
        presentInfo.pWaitSemaphores = signalSemaphores;

        VkSwapchainKHR swapChains[] = {swapChain};
        presentInfo.swapchainCount = 1;
        presentInfo.pSwapchains = swapChains;

        presentInfo.pImageIndices = &imageIndex;

        result = vkQueuePresentKHR(presentQueue, &presentInfo);

        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized) {
            framebufferResized = false;
            recreateSwapChain();
        } else if (result != VK_SUCCESS) {
            throw std::runtime_error("failed to present swap chain image!");
        }

        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
    }

    VkShaderModule createShaderModule(const std::vector<char>& code) {
        VkShaderModuleCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

        VkShaderModule shaderModule;
        if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
            throw std::runtime_error("failed to create shader module!");
        }

        return shaderModule;
    }

    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats) {
        if (availableFormats.size() == 1 && availableFormats[0].format == VK_FORMAT_UNDEFINED) {
            return {VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
        }

        for (const auto& availableFormat : availableFormats) {
            if (availableFormat.format == VK_FORMAT_B8G8R8A8_UNORM && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
                return availableFormat;
            }
        }

        return availableFormats[0];
    }

    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes) {
        VkPresentModeKHR bestMode = VK_PRESENT_MODE_FIFO_KHR;

        for (const auto& availablePresentMode : availablePresentModes) {
            if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
                return availablePresentMode;
            } else if (availablePresentMode == VK_PRESENT_MODE_IMMEDIATE_KHR) {
                bestMode = availablePresentMode;
            }
        }

        return bestMode;
    }

    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities) {
        if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
            return capabilities.currentExtent;
        } else {
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);

            VkExtent2D actualExtent = {
                static_cast<uint32_t>(width),
                static_cast<uint32_t>(height)
            };

            actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
            actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));

            return actualExtent;
        }
    }

    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device) {
        SwapChainSupportDetails details;

        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &details.capabilities);

        uint32_t formatCount;
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);

        if (formatCount != 0) {
            details.formats.resize(formatCount);
            vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, details.formats.data());
        }

        uint32_t presentModeCount;
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);

        if (presentModeCount != 0) {
            details.presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, details.presentModes.data());
        }

        return details;
    }

    bool isDeviceSuitable(VkPhysicalDevice device) {
        QueueFamilyIndices indices = findQueueFamilies(device);

        bool extensionsSupported = checkDeviceExtensionSupport(device);

        bool swapChainAdequate = false;
        if (extensionsSupported) {
            SwapChainSupportDetails swapChainSupport = querySwapChainSupport(device);
            swapChainAdequate = !swapChainSupport.formats.empty() && !swapChainSupport.presentModes.empty();
        }

        VkPhysicalDeviceFeatures supportedFeatures;
        vkGetPhysicalDeviceFeatures(device, &supportedFeatures);
        return indices.isComplete() && extensionsSupported && swapChainAdequate && supportedFeatures.samplerAnisotropy;
    }

    bool checkDeviceExtensionSupport(VkPhysicalDevice device) {
        uint32_t extensionCount;
        vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

        std::vector<VkExtensionProperties> availableExtensions(extensionCount);
        vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());



        std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());

        for (const auto& extension : availableExtensions) {
            requiredExtensions.erase(extension.extensionName);
        }

        return requiredExtensions.empty();
    }

    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device) {
        QueueFamilyIndices indices;

        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

        int i = 0;
        for (const auto& queueFamily : queueFamilies) {

            if (queueFamily.queueCount > 0 && queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                indices.graphicsFamily = i;
            }

            VkBool32 presentSupport = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);

            if (queueFamily.queueCount > 0 && presentSupport) {
                indices.presentFamily = i;
            }

            if (indices.isComplete()) {
                break;
            }

            i++;
        }

        return indices;
    }

    std::vector<const char*> getRequiredExtensions() {
        uint32_t glfwExtensionCount = 0;
        const char** glfwExtensions;
        glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);



        std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);




        return extensions;
    }

    bool extensionSupported(std::string extension)
    {
        return (std::find(supportedExtensions.begin(), supportedExtensions.end(), extension) != supportedExtensions.end());
    }

    bool checkValidationLayerSupport() {
        uint32_t layerCount;
        vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

        std::vector<VkLayerProperties> availableLayers(layerCount);
        vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

        for (const char* layerName : validationLayers) {
            bool layerFound = false;

            for (const auto& layerProperties : availableLayers) {
                if (strcmp(layerName, layerProperties.layerName) == 0) {
                    layerFound = true;
                    break;
                }
            }

            if (!layerFound) {
                return false;
            }
        }

        return true;
    }

    void retrieveDepth(float *outData){


        VkBuffer stagingBuffer;
        VkDeviceMemory stagingBufferMemory;

        VkDeviceSize bufferSize = sim_cam_.width_ * sim_cam_.height_* sizeof(float);

        createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     stagingBuffer, stagingBufferMemory);

        //create staging buffer as src for image
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkBufferImageCopy region = {};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;
        region.imageOffset = (VkOffset3D){0, 0, 0};
        region.imageExtent = (VkExtent3D){
                (uint32_t)sim_cam_.width_,
                (uint32_t)sim_cam_.height_,
                1
    };


        vkCmdCopyImageToBuffer(commandBuffer, depthImage,
                               VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               stagingBuffer, 1, &region);

        endSingleTimeCommands(commandBuffer);

        void *data;
        vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
        memcpy(outData, data, bufferSize);
        vkUnmapMemory(device, stagingBufferMemory);

        vkDestroyBuffer(device, stagingBuffer, nullptr);
        vkFreeMemory(device, stagingBufferMemory, nullptr);
    }

    static std::vector<char> readFile(const std::string& filename) {
        std::ifstream file(filename, std::ios::ate | std::ios::binary);

        if (!file.is_open()) {
            throw std::runtime_error("failed to open file!");
        }

        size_t fileSize = (size_t) file.tellg();
        std::vector<char> buffer(fileSize);

        file.seekg(0);
        file.read(buffer.data(), fileSize);

        file.close();

        return buffer;
    }

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData) {
        std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;

        return VK_FALSE;
    }



    //input


    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        if (key == GLFW_KEY_D && (action == GLFW_PRESS))
            _keys.right = true;
        if(key == GLFW_KEY_D && action == GLFW_RELEASE)
            _keys.right = false;

        if (key == GLFW_KEY_A && (action == GLFW_PRESS ))
            _keys.left = true;
        if(key == GLFW_KEY_A && action == GLFW_RELEASE)
            _keys.left = false;

        if (key == GLFW_KEY_W && (action == GLFW_PRESS ))
            _keys.forward = true;
        if(key == GLFW_KEY_W && action == GLFW_RELEASE)
            _keys.forward = false;

        if (key == GLFW_KEY_S && (action == GLFW_PRESS ))
            _keys.backward = true;
        if(key == GLFW_KEY_S && action == GLFW_RELEASE)
            _keys.backward = false;

        if (key == GLFW_KEY_X && (action == GLFW_PRESS ))
            _keys.up = true;
        if(key == GLFW_KEY_X && action == GLFW_RELEASE)
            _keys.up = false;

        if (key == GLFW_KEY_C && (action == GLFW_PRESS ))
            _keys.down = true;
        if(key == GLFW_KEY_C && action == GLFW_RELEASE)
            _keys.down = false;
        if(key==GLFW_KEY_G && action == GLFW_RELEASE){
            send_pos = true;
        }
    }


    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
    {

        glfwSetCursorPos(window, static_cast<int>(WIDTH/2),static_cast<int>(HEIGHT/2));

        _mouse_pos.xpos = xpos;
        _mouse_pos.ypos = ypos;

    }
};
