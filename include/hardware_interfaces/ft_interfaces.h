/*
  FTInterfaces: virtual class with common interfaces for a force-torque sensor.
  Provides reading interfaces and tool-sensor transformations for 6D
  force-torque.

  Author:
      Yifan Hou <yifanhou@stanford.edu>
*/

#ifndef _FT_INTERFACE_CLASS_HEADER_
#define _FT_INTERFACE_CLASS_HEADER_

#include <RobotUtilities/spatial_utilities.h>

class FTInterfaces {
 public:
  /**
   * Check if initialization is finished and data is ready to read.
   *
   * @return  True if data is ready.
   */
  bool is_data_ready() { return _flag_started; }

  /**
   * Get the sensor reading.
   *
   * @param  wrench  The wrench
   *
   * @return  0 if no error.
   */
  virtual int getWrenchSensor(RUT::VectorXd& wrench,
                              int num_of_sensors = 1) = 0;
  /**
   * Get the wrench in tool frame.
   *
   * @param      wrench  The wrench
   *
   * @return     0 if no error.
   */
  virtual int getWrenchTool(RUT::VectorXd& wrench, int num_of_sensors = 1) = 0;
  /**
   * Get the tool wrench after tool weight compensation.
   *
   * @param[in]  pose    The Cartesian pose of the robot tool
   * @param      wrench  The wrench
   *
   * @return     0 if no error.
   */
  virtual int getWrenchNetTool(const RUT::Vector7d& pose, RUT::VectorXd& wrench,
                               int num_of_sensors = 1) = 0;

  /**
   * Get the number of sensors. If there are two sensors, the wrench feedback will be 12 dimensional.
   */
  virtual int getNumSensors() = 0;

  // Update the tool sensor transformation online
  virtual int setToolSensor(const RUT::Vector7d& pose_TS) {
    _adj_sensor_tool = RUT::SE32Adj(RUT::SE3Inv(RUT::pose2SE3(pose_TS)));
    return true;
  }

  // pre-allocated internal variables
  RUT::Vector3d _force, _force_old;
  RUT::Vector3d _torque, _torque_old;
  RUT::VectorXd _wrench_sensor_temp, _wrench_tool_temp;
  RUT::Matrix3d _R_WT;
  RUT::Vector3d _GinF, _GinT;

  // monitor pausing of the data stream.
  // if the data is the same in 50 frames, the stream is considered dead.
  int _stall_counts;

  RUT::Vector6d _WrenchSafety;
  RUT::Matrix6d _adj_sensor_tool;
  bool _flag_started{false};  // Whether readings are available.
};

#endif
