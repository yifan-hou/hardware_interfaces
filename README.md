# hardware_interface
The purpose of this package is to provide standard interface for robot hardwares. Controllers using those standard interfaces do not need to assume any particular robot hardware. For example, our [force_control](https://github.com/yifan-hou/force_control) package can be used with either an ABB robot or a UR robot.
Currently supporting the following interfaces:
* Position-controlled robot arm
* 6-dof force torque sensor

Current implementations includes (see `robots/`):
* UR robot rtde communication
* ARX robot arm via CAN bus
* ATI force torque sensor via netft;
* Robotiq FT series force torque sensor via modbus
* Realsense cameras via USB.

Author: Yifan Hou
yifanhou@stanford.edu

# Install
## Dependency
This package depends on [cpplibrary](https://github.com/yifan-hou/cpplibrary) and [yaml-cpp](https://github.com/jbeder/yaml-cpp).


Additionally, each hardware interface has there own dependencies. You can disable the hardware that you don't need by comment out the corresponding "add_subdirectory" lines in the base CMakeLists.txt.
* The ur_rtde module is dependent on [UR rtde c++ library](https://gitlab.com/sdurobotics/ur_rtde)
* The arx_can module is dependent on the [arx c++ sdk](https://github.com/yihuai-gao/arx5-sdk).
* The realsense module is dependent on the [official lib realsense package](https://github.com/IntelRealSense/librealsense/blob/master/examples/readme.md)
* The GoPro module is dependent on opencv.
* The ati_netft and robotiq_ft_modbus module contains a copy of the respective drivers and thus do not have additional dependencies. 

Example installation procedure with CMake:

``` sh
cd workspace
# install yaml-cpp
git clone git@github.com:jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake -DCMAKE_CXX_STANDARD=17 ..  # need to build with c++ 17
make -j
make install

# install cpplibrary
cd ../..
git clone git@github.com:yifan-hou/cpplibrary.git
cd cpplibrary
mkdir build && cd build
cmake ..
make -j
make install

# install ur-rtde
cd ../..
git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
mkdir build && cd build
cmake ..
make -j
make install

# build hardware_interface
git clone git@github.com:yifan-hou/hardware_interfaces.git
cd hardware_interfaces
mkdir build && cd build
cmake ..
make -j
make install
```
