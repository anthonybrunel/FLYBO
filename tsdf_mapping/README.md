# Online TSDF Mapping using Open3D

This package is part of the [FLYBO](https://github.com/anthonybrunel/FLYBO/) simulation framework.
<!-- ![Farmers Market Finder - Animated gif demo](ressouces/visu_demo.gif) -->

## System requirements

Ubuntu (20.04) -  CMake: 3.15+ - C++14 compiler

[ROS](http://wiki.ros.org/ROS/Installation) Desktop-Full Install Recommended

[Open3D](https://github.com/isl-org/Open3D) version [0.13](https://github.com/isl-org/Open3D/releases/tag/v0.13.0) build from source ([docs](http://www.open3d.org/docs/release/compilation.html)) with ```cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA_MODULE=ON  -D CMAKE_C_COMPILER=gcc-8 -D CMAKE_CXX_COMPILER=g++-8 -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_JSONCPP=ON```

[Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

```rosdep install --from-paths WORKSPACE --ignore-src --rosdistro=ROSDISTRO```


## Demo


