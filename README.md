# egm
C++ Interface with ABB EGM module

The library help you write a server that talks with the robot.
The server has a few tasks to do:
1. Open&listen to the udp port for the handshake signal from EGM (the robot side)
2. Receive & unwrap messages from EGM
3. After receiving a message, send a wrapped message to EGM.

## Features:
* Support Cartesian Pose reading/writting
* Provide basic safety checking (bounding box & displacement).

## Dependances & supported platform
* Google Protobuf.
* Eigen3.
* cmake

The repo can be compiled as a ROS package, or compiled independently with cmake.



# Installation
## Install the google protobuf.
* Download code from: (Tested on 3.1 and 3.4, you can try newer version)
https://github.com/google/protobuf/tree/v3.1.0
* Then follow the instructions:
https://github.com/google/protobuf/blob/master/src/README.md

## Download this repo
If use in ROS, download it as a ROS package.
```
cd path_to_catkin_workspace/src
git clone git@mlab.ri.cmu.edu:mlab-infra/egm.git
```
If use independently, just download it to wherever your want.
```
cd a_good_place/
git clone git@mlab.ri.cmu.edu:mlab-infra/egm.git
```

## Compile & Install
### Build as a ROS package
Rename ```CMakeLists.txt.egm``` to ```CMakeLists.txt``` (replace the origin ```CMakeLists.txt```. 

Then just run ```catkin_make```.
```
cd path_to_catkin_workspace
catkin_make
```

### Build independently
```
cd egm
mkdir build & cd build
cmake ..
make
sudo make install
```
The last step will install egm headers to /usr/local/include/egm/, *shared* libraries to /usr/local/lib/egm/.


## Compile EGM Proto
Normally you DO NOT need to do this. The ```CMakeLists.txt``` already handles compiling of the proto. However, if you encouter problems with this part and have to compile proto file manually, here is how:
(remember to install the protobuf compiler first)

``` 
$ cd egm/proto
$ protoc  --cpp_out=. egm.proto 
```
There will be a warning saying default syntax is used, and that is fine.

This generates the following files, which shall be used in your CMakeLists.txt.
- egm.pb.h, the header which declares your generated classes.
- egm.pb.cc, which contains the implementation of your classes.


# Usage
## Prepare RAPID code on Robot
There are a set of RAPID commands just for EGM. You can control a variety of parameters from that. For details please refer to the ABB official manual listed in *Reference* section.

For MLab people:
There are already programs with EGM on ABB R120 robot.
You can just load one of them.

## Example Code (ROS)
Make sure you have the following in ```CMakeLists.txt``` of your package:
```
find_package(catkin REQUIRED COMPONENTS
  egm_hardware
)

include_directories(${egm_INCLUDE_DIRS})

target_link_libraries(targetname egm)
```

For writing C++ code, refer to the next section.
## Example Code (Independent)
Create a repo. In command line:
```
cd somewhere_clean
mkdir test & cd test
touch CMakeLists.txt
touch main.cpp
```
Write the following into ```main.cpp```
```
#include <egm/EGMClass.h>

using namespace std;


int main(void)
{
	// get current time
	std::chrono::high_resolution_clock::time_point time0 = std::chrono::high_resolution_clock::now();
	unsigned short portnum   = 6510;
	float max_dist_tran      = 10;
	float max_dist_rot       = 1;
	float egm_safety[6]      = {-100, 100, -100, 100, -100, 100};
	EGMSafetyMode sf_mode    = SAFETY_MODE_STOP;
	EGMOperationMode op_mode = OPERATION_MODE_CARTESIAN;
	bool print_flag          = true;
	string filefullpath      = "/home/mlab/data/egmdata.txt";

	EGMClass *egm = EGMClass::Instance();
	// create thread to communite with EGM at 250Hz
	egm->init(time0, portnum, max_dist_tran, max_dist_rot, egm_safety, sf_mode, op_mode, print_flag, filefullpath);


	// write your control loop here
	float pose[7];
	float pose_set[7];
	while (true)
	{
		// read feedback
		egm->GetCartesian(pose);

		// compute command
		pose_set[0] = pose[0]; // x
		pose_set[1] = pose[1]; // y
		pose_set[2] = pose[2]; // z
		pose_set[3] = pose[3]; // q0
		pose_set[4] = pose[4]; // q1
		pose_set[5] = pose[5]; // q2
		pose_set[6] = pose[6]; // q3

		// send command
		egm->SetCartesian(pose_set);

		// spin once
	}


	return 0;
}
```

Put the following into ```CMakeLists.txt```
```
cmake_minimum_required(VERSION 2.8.3)
#######################
project(egmtest)
set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

# find_package(Boost REQUIRED COMPONENTS system)

include_directories(/usr/local/include/egm) # so as to find egm.pb.h

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} libegm.so)
```
Then compile your code in command line: (you are at root of the repo)
```
mkdir build & cd build
cmake ..
make
```

If can not receive message, run 'sudo ufw allow 6510'\n";

# Experiment with Robot
To test EGM, follow the steps below.
1. Power on the robot.
2. In RoboStudio, choose the correct RAPID program.
3. On your computer, run the server program. The process will be blocked at egm->init, waiting for connection.
4. On robot Teach Pedant, click "start" button. A signal will be sent to server, and the 4ms communication shall begin. 


# About EGM Communication
ABB EGM use ```Google Protocol Buffers``` to serialize/de-serialize data.

ABB provide the ```.proto``` file (included in this repo), which describes the message structures used by EGM.

The ```.proto``` file will automatically get compiled with this project, so as to obtain the library files in C++.

The library contains serialized/de-serialized code which is then used by the application. The application reads a message from the network, runs the de-serialization, creates a message, calls serialization method, and then sends the message.

More about ```protobuf```:
https://developers.google.com/protocol-buffers/docs/cpptutorial

# About EGM Position Guidance
*Official Manual*
Application manual-Controller software IRC5

*EGM Position Guidance does not contain interpolator functionality. It only does filtering, supervision of references, and state handling.
*The inner controller works as:
	speed = k * (pos_ref – pos) + speed_ref
where:
** k - factor
** pos_ref - reference position
** pos - desired position
** speed_ref - reference speed
* EGM can be done every 4 ms with a control lag of 10–20 ms depending on the robot type.
The time between writing a new position until that given position starts to affect the actual robot position, is usually around 20 ms.
* Set Parameters
** Do it in RoboStudio. Edit in configuration/motion/External Motion Interface Data
** Default proportional Position Gain: The parameter Position gain in the figure influences the responsiveness moving to the target position, given by the sensor, in relation to the current robot position. The higher the value, the faster the response.
** Default Low Pass Filter Bandwith: The parameter LP Filter in the figure is the default value used to filter the speed contribution from EGM.
** The list of parameters can be found in Page 343, section 9.3.4, Application Manual - Controller software IRC5



