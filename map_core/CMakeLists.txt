cmake_minimum_required(VERSION 3.15)
project(map_core)

add_compile_options(-std=c++14 -O3 -march=native -g -msse -msse2 -msse3)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL  REQUIRED)
find_package(glm REQUIRED )



find_package(TBB REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
)

cs_add_library(map_core
    include/map_core/mapcore.h
    include/map_core/esdfmap.h
    src/mapcore.cpp
    src/esdfmap.cpp

    include/map_core/mapcoreserver.h
    src/mapcoreserver.cpp

    include/map_core/voxelbuffer.h
    src/voxelbuffer.cpp
    utils/timer.hpp
    utils/plf_list.h
    utils/boundingmap.hpp


    include/map_core/camerafrustum.h
    src/camerafrustum.cpp
    include/map_core/frontiermanager.h
    src/frontiermanager.cpp

    include/map_core/gridhash.h
    src/gridhash.cpp
    )
target_link_libraries(map_core ${catkin_LIBRARIES} Eigen3::Eigen ${PCL_LIBRARIES} tbb)

link_directories(${PCL_LIBRARY_DIRS})
cs_add_executable(map_test
  node/map_test.cpp
)
target_link_libraries(map_test map_core ${catkin_LIBRARIES} Eigen3::Eigen ${PCL_LIBRARIES})


link_directories(${PCL_LIBRARY_DIRS})
cs_add_executable(map_server_test
  node/map_server_test.cpp
)
target_link_libraries(map_server_test map_core ${catkin_LIBRARIES} Eigen3::Eigen ${PCL_LIBRARIES})


cs_install()
cs_export()
