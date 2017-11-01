# egm
C++ Interface with ABB EGM module

## Features:
* Support Cartesian Pose reading/writting
* Provide basic safety checking (bounding box & displacement).

## Dependances & supported platform
Require Google Protobuf.
Require Eigen3.
Can be compiled as a ROS package, or independently with CMake.




# Installation
## Install the google protobuf.
* Download code from: (Tested on 3.1 and 3.4, you can try newer version)
https://github.com/google/protobuf/tree/v3.1.0
* Then follow the instructions:
https://github.com/google/protobuf/blob/master/src/README.md

## Download this repo
If use in ROS, download it as a package.
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


# Code Example
## Prepare RAPID code on Robot
There are a set of RAPID commands just for EGM. You can control a variety of parameters from that. For details please refer to the ABB official manual listed in *Reference* section.

For MLab people:
There are already programs with EGM on ABB R120 robot.
You can just load one of them.

## Writting your application
In EGM, the server is the thread that talks with the robot.
The server has a few tasks to do:
1. Open&listen to the udp port for the handshake signal from EGM (the robot side)
2. Receive & unwrap messages from EGM
3. After receiving a message, send a wrapped message to EGM.

Here *wrap* means package into Google ProtocolBuf.
 
There are two code examples for you to choose from: 
1. egm-server: A minimal server example. Everything you need to write is in one file.
2. egm-comm: A C++ class wrapping up all the EGM function with easy to use interface. Also can be used with ROS.


# About EGM Communication
ABB EGM use ```Google Protocol Buffers``` to serialize/de-serialize data.

ABB provide the ```.proto``` file (included in this repo), which describes the message structures used by EGM.

The ```.proto``` file will automatically get compiled with this project, so as to obtain the library files in C++.

The library contains serialized/de-serialized code which is then used by the application. The application reads a message from the network, runs the de-serialization, creates a message, calls serialization method, and then sends the message.

More about ```protobuf```:
https://developers.google.com/protocol-buffers/docs/cpptutorial

# About EGM Position Guidance
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



