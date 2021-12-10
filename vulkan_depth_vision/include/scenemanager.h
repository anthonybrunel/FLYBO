#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H
#include <vector>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <set>
#include <array>
#include <pcl/point_types.h>

#include<Eigen/Core>
#include <random>


#include <ros/ros.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "camera.hpp"

struct Vertex {
    glm::vec3 pos;
    glm::vec3 color;

    static VkVertexInputBindingDescription getBindingDescription() {
        VkVertexInputBindingDescription bindingDescription = {};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(Vertex);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        return bindingDescription;
    }
    static std::array<VkVertexInputAttributeDescription, 2> getAttributeDescriptions() {
        std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {};

        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[0].offset = offsetof(Vertex, pos);

        attributeDescriptions[1].binding = 0;
        attributeDescriptions[1].location = 1;
        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[1].offset = offsetof(Vertex, color);

//        attributeDescriptions[2].binding = 0;
//        attributeDescriptions[2].location = 2;
//        attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
//        attributeDescriptions[2].offset = offsetof(Vertex, texCoord);
        return attributeDescriptions;
    }

};

class SceneManager
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SceneManager();
    ~SceneManager();

    void loader();


    void generate_custom_scene();
    void scene_from_pcd();

    void scene_from_obj(const std::string &path);

    std::vector<Vertex>& getStatic_vertices();
    void setStatic_vertices(const std::vector<Vertex> &value);
    std::vector<uint32_t>& getStatic_indices();
    void setStatic_indices(const std::vector<uint32_t> &value);




    double interpolate( double val, double y0, double x0, double y1, double x1 ) {
        return (val-x0)*(y1-y0)/(x1-x0) + y0;
    }

    double base( double val ) {
        if ( val <= -0.75 ) return 0;
        else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
        else if ( val <= 0.25 ) return 1.0;
        else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
        else return 0.0;
    }

    double red( double gray ) {
        return base( gray - 0.5 );
    }
    double green( double gray ) {
        return base( gray );
    }
    double blue( double gray ) {
        return base( gray + 0.5 );
    }




private:


    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxels;

    std::vector<Vertex> static_vertices;
    std::vector<uint32_t> static_indices;





};

#endif // SCENEMANAGER_H
