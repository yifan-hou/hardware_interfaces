/*
  FTInterfaces: virtual class with common interfaces for a force-torque sensor.
  Provides reading interfaces and tool-sensor transformations for 6D
  force-torque.

  Author:
      Yifan Hou <yifanhou@stanford.edu>
*/

#ifndef _FT_INTERFACE_CLASS_HEADER_
#define _FT_INTERFACE_CLASS_HEADER_

#include <RobotUtilities/utilities.h>

class FTInterfaces {
 public:
  /**
   * Get the sensor reading.
   *
   * @param  wrench  The wrench
   *
   * @return  0 if no error.
   */
  virtual int getWrenchSensor(RUT::Vector6d& wrench) = 0;
  /**
   * Get the wrench in tool frame.
   *
   * @param      wrench  The wrench
   *
   * @return     0 if no error.
   */
  virtual int getWrenchTool(RUT::Vector6d& wrench) = 0;
  /**
   * Get the tool wrench after tool weight compensation.
   *
   * @param[in]  pose    The Cartesian pose of the robot tool
   * @param      wrench  The wrench
   *
   * @return     0 if no error.
   */
  virtual int getWrenchNetTool(const RUT::Vector7d& pose,
                               RUT::Vector6d& wrench) = 0;

  RUT::Vector6d _WrenchSafety;
  RUT::Matrix6d _adj_sensor_tool;

 protected:
  /**
   * for singleton implementation
   */
  static FTInterfaces* pinstance;
  FTInterfaces() {}
  FTInterfaces(const FTInterfaces&) {}
  FTInterfaces& operator=(const FTInterfaces&) { return *this; }
  ~FTInterfaces() {}
};

#endif
