cmake_minimum_required(VERSION 3.0.2)
project(flybo_utils)

add_compile_options(-std=c++14 -O3  -g -msse -msse2 -msse3 -msse4.2)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}

        src/flybo_utils.cpp
        )




set(SOURCE_FILES
    src/cloudmanager.cpp
 )

set(HEADER_FILES
    include/cloud_extraction/cloudmanager.h
    )

cs_add_library(cloud_extraction
    ${SOURCE_FILES}
    ${HEADER_FILES}
    )
    
    
add_dependencies(cloud_extraction  ${open3d_ros_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_extraction Eigen3::Eigen ${Open3D_LIBRARIES})

target_include_directories(cloud_extraction PUBLIC ${Open3D_INCLUDE_DIRS})
set_target_properties(cloud_extraction  PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


link_directories(${Open3D_LIBRARY_DIRS})


cs_add_executable(cloud_extraction_node nodes/cloud_extraction_node.cpp)
target_link_libraries(cloud_extraction_node  Eigen3::Eigen ${catkin_LIBRARIES} cloud_extraction
)
target_include_directories(cloud_extraction_node PUBLIC ${Open3D_INCLUDE_DIRS} )



cs_install()
cs_export()
