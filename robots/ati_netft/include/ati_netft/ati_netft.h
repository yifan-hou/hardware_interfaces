/*
    Two methods to use this interface:
    1. use it as a hardware_interface. It provides
   force_torque_sensor_interface.
    2. read the public member _force and _torque, or call getWrench()
*/
#pragma once
#include <RobotUtilities/utilities.h>
#include <RobotUtilities/TimerLinux.h>
#include <pthread.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "hardware_interfaces/ft_interfaces.h"
#include "netft_rdt_driver/netft_rdt_driver.h"

typedef std::chrono::high_resolution_clock Clock;

class ATINetft : public FTInterfaces {
 public:
  struct ATINetftConfig {
    std::string ip_address{"192.168.1.1"};
    std::string sensor_name{"netft"};
    std::string fullpath{};
    bool print_flag{false};
    double publish_rate{100.0};
    RUT::Vector3d Foffset{};
    RUT::Vector3d Toffset{};
    RUT::Vector3d Gravity{};
    RUT::Vector3d Pcom{};
    RUT::Vector6d WrenchSafety{};
    RUT::Vector7d PoseSensorTool{};
  };

  ATINetft();
  ~ATINetft();
  bool init(RUT::TimePoint time0, const ATINetftConfig &ati_netft_config);
  /**
   * Get the sensor reading.
   *
   * @param  wrench  The wrench
   *
   * @return  0: no error.
   *   1: still waiting for new data. 2: dead stream.
   */
  int getWrenchSensor(double *wrench) override;
  /**
   * Get the wrench in tool frame.
   *
   * @param      wrench_T  The wrench in tool frame
   *
   * @return     0: no error. 1: still waiting for new data. 2: dead stream.
   */
  int getWrenchTool(double *wrench_T) override;
  /**
   * Get the tool wrench after tool weight compensation.
   *
   * @param[in]  pose          The Cartesian pose of the robot tool
   * @param      wrench_net_T  The net wrench in tool frame
   *
   * @return     0: no error. 1: still waiting for new data. 2: dead stream.
   *             3: force is too big.
   */
  int getWrenchNetTool(const double *pose, double *wrench_net_T) override;

  double *_force, *_force_old;
  double *_torque, *_torque_old;
  double _publish_rate;

  RUT::TimePoint _time0;  ///< high resolution timer.
  std::ofstream _file;
  bool _print_flag;

  // netft
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> _netft;

  // monitor pausing of the data stream.
  // if the data is the same in 50 frames, the stream is considered dead.
  int _stall_counts;

 private:
  // thread
  pthread_t _thread;
};
