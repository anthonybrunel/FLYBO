#include "scenemanager.h"
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include<Eigen/StdVector>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include "../utils/timer.hpp"
#include "obj_io.h"

SceneManager::SceneManager()
{

}

SceneManager::~SceneManager()
{

}

void SceneManager::generate_custom_scene()
{
    int cpt = 0;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(0, 1);
    //ground

    float size = 0.02f;
    for(int i = -0; i < 50; ++i){
        for(int k = -0; k < 50; ++k){
            float x = size*i;
            float y = size*0;
            float z = size*k;
            int color = dis(gen);
            float r=0.f,g=0.f,b=0.f;
            if(color == 0){
                r = 1.f;
            }else{
                g = 1.f;
            }
            static_vertices.push_back({{-0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{-0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});

            static_vertices.push_back({{-0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{-0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});

            int arr[] =
            {0+cpt*8, 2+cpt*8, 1+cpt*8, 3+cpt*8, 2+cpt*8, 0+cpt*8,
             4+cpt*8, 5+cpt*8, 6+cpt*8, 6+cpt*8, 7+cpt*8, 4+cpt*8,
             5+cpt*8, 0+cpt*8, 1+cpt*8, 0+cpt*8, 5+cpt*8, 4+cpt*8,
             5+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 6+cpt*8, 5+cpt*8,
             4+cpt*8, 7+cpt*8, 0+cpt*8, 7+cpt*8, 3+cpt*8, 0+cpt*8,
             2+cpt*8, 3+cpt*8, 7+cpt*8, 7+cpt*8, 6+cpt*8, 2+cpt*8};
            static_indices.insert(static_indices.end(),arr,arr+6*6);
            cpt++;
        }
    }
    //wall
    for(int i = 0; i < 25; ++i){
        for(int k = -0; k < 50; ++k){

            float x = size*-1;
            float y = size*i;
            float z = size*k;
            int color = dis(gen);
            float r=0.f,g=0.f,b=0.f;
            if(color == 0){
                r = 1.f;
            }else{
                g = 1.f;
            }
            static_vertices.push_back({{-0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{-0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});

            static_vertices.push_back({{-0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});
            static_vertices.push_back({{-0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});

            int arr[] =
            {0+cpt*8, 2+cpt*8, 1+cpt*8, 3+cpt*8, 2+cpt*8, 0+cpt*8,
             4+cpt*8, 5+cpt*8, 6+cpt*8, 6+cpt*8, 7+cpt*8, 4+cpt*8,
             5+cpt*8, 0+cpt*8, 1+cpt*8, 0+cpt*8, 5+cpt*8, 4+cpt*8,
             5+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 6+cpt*8, 5+cpt*8,
             4+cpt*8, 7+cpt*8, 0+cpt*8, 7+cpt*8, 3+cpt*8, 0+cpt*8,
             2+cpt*8, 3+cpt*8, 7+cpt*8, 7+cpt*8, 6+cpt*8, 2+cpt*8};
            static_indices.insert(static_indices.end(),arr,arr+6*6);
            cpt++;
        }
    }






}

void SceneManager::scene_from_pcd()
{

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/anthony/OSCAR/data/Berret5cm.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    transform_2.translation() << 0., 0.0, 0.0;
    transform_2.rotate (Eigen::AngleAxisf (-M_PI/2., Eigen::Vector3f::UnitY())*Eigen::AngleAxisf (-M_PI/2., Eigen::Vector3f::UnitX()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *cloud, transform_2);
    float res = 0.05;


    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (res);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud, minPt, maxPt);

    int a = octree.getOccupiedVoxelCenters(voxels);
    int cpt = 0;
    for(auto c: voxels){
        float size = res;
        float x =  c.x;
        float y =  c.y;
        float z =  c.z;
        float r = red((c.y-minPt.y)/(maxPt.y - minPt.y));
        float g = green((c.y-minPt.y)/(maxPt.y - minPt.y));
        float b = blue((c.y-minPt.y)/(maxPt.y - minPt.y));
        static_vertices.push_back({{-0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
        static_vertices.push_back({{0.5f*size+x, -0.5f*size+y, -0.5f*size+z}, {r,g,b}});
        static_vertices.push_back({{0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});
        static_vertices.push_back({{-0.5f*size+x, 0.5f*size+y, -0.5f*size+z}, {r,g,b}});

        static_vertices.push_back({{-0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
        static_vertices.push_back({{0.5f*size+x, -0.5f*size+y, 0.5f*size+z}, {r,g,b}});
        static_vertices.push_back({{0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});
        static_vertices.push_back({{-0.5f*size+x, 0.5f*size+y, 0.5f*size+z}, {r,g,b}});

        int arr[] =
        {0+cpt*8, 2+cpt*8, 1+cpt*8, 3+cpt*8, 2+cpt*8, 0+cpt*8,
         4+cpt*8, 5+cpt*8, 6+cpt*8, 6+cpt*8, 7+cpt*8, 4+cpt*8,
         5+cpt*8, 0+cpt*8, 1+cpt*8, 0+cpt*8, 5+cpt*8, 4+cpt*8,
         5+cpt*8, 1+cpt*8, 2+cpt*8, 2+cpt*8, 6+cpt*8, 5+cpt*8,
         4+cpt*8, 7+cpt*8, 0+cpt*8, 7+cpt*8, 3+cpt*8, 0+cpt*8,
         2+cpt*8, 3+cpt*8, 7+cpt*8, 7+cpt*8, 6+cpt*8, 2+cpt*8};
        static_indices.insert(static_indices.end(),arr,arr+6*6);




        cpt++;
    }


}

void SceneManager::scene_from_obj(const std::string &path)
{

    class vec3{
    public:
        float x,y,z;
    };


    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str());

//    if (!warn.empty()) {
//        std::cout << warn << std::endl;
//    }

//    if (!err.empty()) {
//        std::cerr << err << std::endl;
//    }

    if (!ret) {
        std::cerr << "[VulkanVizu] error model path not provided\n";
        exit(1);
    }

    int count = 0;
    // Loop over shapes
    size_t index_shape_offset = 0;
    static_vertices.resize(attrib.vertices.size()/3);
    unsigned int geomID;
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];




            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex


                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
                tinyobj::real_t nx = attrib.normals[3*idx.normal_index+0];
                tinyobj::real_t ny = attrib.normals[3*idx.normal_index+1];
                tinyobj::real_t nz = attrib.normals[3*idx.normal_index+2];
                tinyobj::real_t tx = attrib.texcoords[2*idx.texcoord_index+0];
                tinyobj::real_t ty = attrib.texcoords[2*idx.texcoord_index+1];
                static_vertices[idx.vertex_index] = {{vx,vy,vz}, {0,125,125}};
                //                std::cout << index_offset+v <<std::endl;
                static_indices.push_back(idx.vertex_index);
                //                std::cout << vx << " " << vy <<" " << vz << " & ";
            }



            index_offset += fv;
        }
        index_shape_offset += shapes[s].mesh.indices.size();
    }




}


std::vector<Vertex> &SceneManager::getStatic_vertices()
{
    return static_vertices;
}

void SceneManager::setStatic_vertices(const std::vector<Vertex> &value)
{
    static_vertices = value;
}

std::vector<uint32_t>& SceneManager::getStatic_indices()
{
    return static_indices;
}

void SceneManager::setStatic_indices(const std::vector<uint32_t> &value)
{
    static_indices = value;
}

