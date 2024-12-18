Made from https://github.com/yifan-hou/wsg50_cpp
=========================================
A c++ driver library for the [Schunk WSG50-110 Gripper](https://www.weiss-robotics.com/en/produkte/gripping-systems/performance-line-en/wsg-50-en/).

This repo is made by peeling off all ROS dependencies from this WSG ROS driver [ipa325_wsg50](https://github.com/ipa320/ipa325_wsg50).

Tested on WSG50.

## Installation
This package is dependent on boost and cmake.

``` sh
git clone git@github.com:yifan-hou/wsg50_cpp.git
cd wsg50_cpp
mkdir build && cd build
cmake ..
make -j
# run the test program
./wsg_test
```

## Usage
Check out src/test.cpp.

## Documentation
You can find some documentation from the original ROS repo [ipa325_wsg50](https://github.com/ipa320/ipa325_wsg50). I did not modify any functionality of it.