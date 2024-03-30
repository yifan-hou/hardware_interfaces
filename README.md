# hardware_interface
The purpose of this package is to provide standard interface for robot hardwares. Controllers using those standard interfaces do not need to assume any particular robot hardware. For example, our [force_control](https://github.com/yifan-hou/force_control) package can be used with either an ABB robot or a UR robot.
Currently supporting the following interfaces:
* Position controlled robot arm
* 6-dof force torque sensor

Current implementations includes (see `robots/`):
* UR robot rtde communication;
* ATI force torque sensor via netft;

Author: Yifan Hou
yifanhou@stanford.edu

# Install
## Dependency
This package depends on [cpplibrary](https://github.com/yifan-hou/cpplibrary).

## Build
Build and install with cmake.
``` sh
cd hardware_interfaces
mkdir build && cd build
cmake ..
make -j
sudo make install
```
