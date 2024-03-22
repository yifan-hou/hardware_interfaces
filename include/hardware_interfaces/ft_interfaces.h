/*
  FTInterfaces: virtual class with common interfaces for a force-torque sensor.
  Provides reading interfaces and tool-sensor transformations for 6D
  force-torque.

  Author:
      Yifan Hou <yifanh@cmu.edu>
*/

#ifndef _FT_INTERFACE_CLASS_HEADER_
#define _FT_INTERFACE_CLASS_HEADER_

#include <Eigen/Dense>

class FTInterfaces {
 public:
  /**
   * Get the sensor reading.
   *
   * @param  wrench  The wrench
   *
   * @return  0 if no error.
   */
  virtual int getWrenchSensor(double *wrench) = 0;
  /**
   * Get the wrench in tool frame.
   *
   * @param      wrench  The wrench
   *
   * @return     0 if no error.
   */
  virtual int getWrenchTool(double *wrench) = 0;
  /**
   * Get the tool wrench after tool weight compensation.
   *
   * @param[in]  pose    The Cartesian pose of the robot tool
   * @param      wrench  The wrench
   *
   * @return     0 if no error.
   */
  virtual int getWrenchNetTool(const double *pose, double *wrench) = 0;

  double *_WrenchSafety;
  Eigen::Vector3d _Foffset;
  Eigen::Vector3d _Toffset;
  Eigen::Vector3d _Gravity;
  Eigen::Vector3d _Pcom;
  Eigen::Matrix<double, 6, 6> _adj_sensor_tool;

 protected:
  /**
   * for singleton implementation
   */
  static FTInterfaces *pinstance;
  FTInterfaces() {}
  FTInterfaces(const FTInterfaces &) {}
  FTInterfaces &operator=(const FTInterfaces &) { return *this; }
  ~FTInterfaces() {}
};

#endif
