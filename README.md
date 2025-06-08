# hardware_interface

**Note: Release versions**
The previous release is in branch v1.0. If you come here from [Adaptive Compliance Policy](https://github.com/yifan-hou/adaptive_compliance_policy), please use branch v1.0 instead of main.

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
* Oak camera via USB.

Author: Yifan Hou
yifanhou@stanford.edu

# Install
## Dependency
This package depends on
* [cpplibrary](https://github.com/yifan-hou/cpplibrary)
* [force_control](https://github.com/yifan-hou/force_control)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp).


Additionally, each hardware interface has there own dependencies. You can disable the hardware that you don't need by comment out the corresponding "add_subdirectory" lines in the base CMakeLists.txt.
* The ur_rtde module is dependent on [UR rtde c++ library](https://gitlab.com/sdurobotics/ur_rtde)
* The arx_can module is dependent on the [arx c++ sdk](https://github.com/yihuai-gao/arx5-sdk).
* The realsense module is dependent on the [official lib realsense package](https://github.com/IntelRealSense/librealsense/blob/master/examples/readme.md)
* The GoPro module is dependent on opencv.
* The ati_netft and robotiq_ft_modbus module contains a copy of the respective drivers and thus do not have additional dependencies. 
* The Oak camera is dependent on [depthai-core](https://github.com/luxonis/depthai-core).

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

# install force_control
cd ../..
git clone git@github.com:yifan-hou/force_control.git
cd force_control
mkdir build && cd build
cmake ..
make -j
make install

# install ur-rtde (Needed by UR)
cd ../..
git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
mkdir build && cd build
cmake ..
make -j
make install

# install depthai-core (Needed by OAK)
cd ../..
git clone https://github.com/luxonis/depthai-core.git
cd depthai-core
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local -DDEPTHAI_BUILD_EXAMPLES=ON -DBUILD_SHARED_LIBS=ON ..
make -j
make install

# Install Realsense dev (Needed by realsense)
# Checkout their official documentation

# Install onnx runtime (Needed by CoinFT)
# Download the latest version with the right architecture here:
#    https://github.com/microsoft/onnxruntime/releases
# Extract file, and place it in /opt/onnxruntime
# You can change the path from CMakeLists.txt


# build hardware_interface
git clone git@github.com:yifan-hou/hardware_interfaces.git
cd hardware_interfaces
mkdir build && cd build
cmake ..
make -j
make install

# Install the python bindings for ManipServer
cd hardware_interfaces
pip install -e .
```

## Install to a local path
The above instruction will install the libraries to /usr, which affects all users on your system.
It is recommended to install to a local folder instead.

### Configure folders and environmental variables
In your $HOME, create the following file structure:
```
$HOME/
    .local/
        bin/
        include/
        lib/
```
In .bashrc or .zshrc, add the following:
```
# For home installations
export PATH=$HOME/.local/bin:$PATH
export C_INCLUDE_PATH=$HOME/.local/include/:$C_INCLUDE_PATH
export CPLUS_INCLUDE_PATH=$HOME/.local/include/:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib/:$LD_LIBRARY_PATH
```

### Compile and install hardware_interface to the local path
Note that all install targets in hardware_interfaces are written with `CMAKE_INSTALL_PREFIX` like this:
```
install(TARGETS Utilities
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
    )
```

To install to the local path we specified before, just change the line
``` sh
cmake ..
```
to
``` sh
cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
```
in the instructions above, then `make`, `make install` as usual.


## Troubleshooting
### cmake failed with wrong python interp version
```
CMake Error at cmake/FindPythonLibsNew.cmake:96 (message): Python config failure: <string>:1: DeprecationWarning: The distutils package is deprecated and slated for removal in Python 3.12. Use setuptools or check PEP 632 for potential alternatives
```
Solution: explicitly specify the full path to the python interp you want to use.
```
cmake .. -DPYTHON_EXECUTABLE=$(which python)
```