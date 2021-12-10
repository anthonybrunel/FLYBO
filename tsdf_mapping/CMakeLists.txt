cmake_minimum_required(VERSION 3.15)
project(tsdf_mapping LANGUAGES CUDA CXX)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

catkin_package()

#https://github.com/intel-isl/Open3D/issues/2286

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    )

find_package(Open3D REQUIRED)

set(SOURCE_FILES
    src/tsdfrosinterface.cpp
    src/tsdfvoxelgridevaluator.cpp
 )

set(HEADER_FILES
    include/tsdfrosinterface.h
    include/tsdfvoxelgridevaluator.h
    utils/timer.hpp
    )

cs_add_library(${PROJECT_NAME}_core
    ${SOURCE_FILES}
    ${HEADER_FILES}
    )

add_dependencies(${PROJECT_NAME}_core ${open3d_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen ${Open3D_LIBRARIES})
target_include_directories(${PROJECT_NAME}_core PUBLIC ${Open3D_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME}_core PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


link_directories(${Open3D_LIBRARY_DIRS})


cs_add_executable(tsdf_node nodes/tsdf_node.cpp)
target_link_libraries(tsdf_node  Eigen3::Eigen ${catkin_LIBRARIES} ${PROJECT_NAME}_core
)
target_include_directories(tsdf_node PUBLIC ${Open3D_INCLUDE_DIRS} )





cs_install()
cs_export()






