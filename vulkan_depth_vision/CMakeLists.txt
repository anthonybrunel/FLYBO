cmake_minimum_required(VERSION 3.5)
project(vulkan_depth_vision)


add_compile_options(-std=c++14 -O3 -march=native -g -msse -msse2 -msse3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


find_package (Threads REQUIRED)
find_package(Vulkan REQUIRED)

find_package( glm REQUIRED )
find_package(glfw3 3.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package( OpenCV REQUIRED )
catkin_package()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)


cs_add_library(Vizualizer
    include/tiny_obj_loader.h
    include/obj_io.h
    src/scenemanager.cpp
    include/scenemanager.h
    utils/timer.hpp
    include/simulatecamera.h
    src/simulatecamera.cpp
    )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/shader
  ${GLM_INCLUDE_DIRS}
)
INCLUDE_DIRECTORIES(${EMBREE_INCLUDE_DIRS})

cs_add_executable(depth_cam_node
  nodes/depth_cam_node.cpp include/VoxelApp.hpp  include/camera.hpp

)
target_link_libraries(depth_cam_node Vizualizer ${catkin_LIBRARIES} ${Vulkan_LIBRARIES} glfw ${PCL_LIBRARIES}  Eigen3::Eigen  ${OpenCV_LIBS} )
cs_install()
cs_export()

