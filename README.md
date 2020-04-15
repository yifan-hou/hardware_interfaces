# hardware_interface
The purpose of this package is to provide standard interface for robot hardwares. Controllers using those standard interfaces do not need to assume any particular robot hardware. For example, our [force_control](https://github.com/yifan-hou/force_control) package can be used with either an ABB robot or a UR robot.
Currently supporting the following interfaces:
* Position controlled robot arm
* 6-dof force torque sensor
* Delta robot

Current implementations includes (see `robots/`):
* ABB robot, EGM Cartesian mode;
* UR robot socket communication;
* ATI force torque sensor via netft;
* Delta platform with DJI BLDC module (under development)

Author: Yifan Hou
yifanh at cmu dot edu

# Install
## Dependency
This package depends on [cpplibrary](https://github.com/yifan-hou/cpplibrary).
Note that [cpplibrary](https://github.com/yifan-hou/cpplibrary) is not a ROS package. Please follow its instruction to install.

## Build
This is a ROS package. clone this repo into your source folder, then compile your catkin workspace.

## Usage
(TODO) Add detailed instructions.
Please refer to [force_control](https://github.com/yifan-hou/force_control) for an example of using this package.
