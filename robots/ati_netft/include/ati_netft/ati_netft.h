/*
    Two methods to use this interface:
    1. use it as a hardware_interface. It provides
   force_torque_sensor_interface.
    2. read the public member _force and _torque, or call getWrench()
*/
#pragma once
#include <RobotUtilities/TimerLinux.h>
#include <RobotUtilities/utilities.h>
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
    // If the force change is smaller than noise_level for more than stall_threshold frames,
    // the stream is considered dead.
    double noise_level{0.0};
    int stall_threshold{50};
    RUT::Vector3d Foffset{};
    RUT::Vector3d Toffset{};
    RUT::Vector3d Gravity{};
    RUT::Vector3d Pcom{};
    RUT::Vector6d WrenchSafety{};
    RUT::Vector7d PoseSensorTool{};
  };

  ATINetft();
  ~ATINetft();
  bool init(RUT::TimePoint time0, const ATINetftConfig& ati_netft_config);
  /**
   * Get the sensor reading.
   *
   * @param  wrench  The wrench
   *
   * @return  0: no error.
   *   1: still waiting for new data. 2: dead stream.
   */
  int getWrenchSensor(RUT::Vector6d& wrench) override;
  /**
   * Get the wrench in tool frame.
   *
   * @param      wrench_T  The wrench in tool frame
   *
   * @return     0: no error. 1: still waiting for new data. 2: dead stream.
   */
  int getWrenchTool(RUT::Vector6d& wrench_T) override;
  /**
   * Get the tool wrench after tool weight compensation.
   *
   * @param[in]  pose          The Cartesian pose of the robot tool
   * @param      wrench_net_T  The net wrench in tool frame
   *
   * @return     0: no error. 1: still waiting for new data. 2: dead stream.
   *             3: force is too big.
   */
  int getWrenchNetTool(const RUT::Vector7d& pose,
                       RUT::Vector6d& wrench_net_T) override;

  // pre-allocated internal variables
  RUT::Vector3d _force, _force_old;
  RUT::Vector3d _torque, _torque_old;
  RUT::Vector6d _wrench_sensor_temp, _wrench_tool_temp;
  RUT::Matrix3d _R_WT;
  RUT::Vector3d _GinF, _GinT;

  RUT::TimePoint _time0;  ///< high resolution timer.
  std::ofstream _file;
  ATINetftConfig _config;
  // netft
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> _netft;

  // monitor pausing of the data stream.
  // if the data is the same in 50 frames, the stream is considered dead.
  int _stall_counts;

 private:
  // thread
  pthread_t _thread;
};
