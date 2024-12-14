/*
    Two methods to use this interface:
    1. use it as a hardware_interface. It provides
   force_torque_sensor_interface.
    2. read the public member _force and _torque, or call getWrench()
*/
#pragma once
#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <pthread.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>
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
    // counts per force and torque are found from 'configuration' tab in the ati web interface.
    double counts_per_force{1000000.0};
    double counts_per_torque{1000000.0};
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

    bool deserialize(const YAML::Node& node) {
      try {
        ip_address = node["ip_address"].as<std::string>();
        counts_per_force = node["counts_per_force"].as<double>();
        counts_per_torque = node["counts_per_torque"].as<double>();
        sensor_name = node["sensor_name"].as<std::string>();
        fullpath = node["fullpath"].as<std::string>();
        print_flag = node["print_flag"].as<bool>();
        publish_rate = node["publish_rate"].as<double>();
        noise_level = node["noise_level"].as<double>();
        stall_threshold = node["stall_threshold"].as<int>();

        Foffset = RUT::deserialize_vector<RUT::Vector3d>(node["Foffset"]);
        Toffset = RUT::deserialize_vector<RUT::Vector3d>(node["Toffset"]);
        Gravity = RUT::deserialize_vector<RUT::Vector3d>(node["Gravity"]);
        Pcom = RUT::deserialize_vector<RUT::Vector3d>(node["Pcom"]);
        WrenchSafety =
            RUT::deserialize_vector<RUT::Vector6d>(node["WrenchSafety"]);
        PoseSensorTool =
            RUT::deserialize_vector<RUT::Vector7d>(node["PoseSensorTool"]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
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
  int getWrenchSensor(RUT::VectorXd& wrench, int num_of_sensors = 1) override;
  /**
   * Get the wrench in tool frame.
   *
   * @param      wrench_T  The wrench in tool frame
   *
   * @return     0: no error. 1: still waiting for new data. 2: dead stream.
   */
  int getWrenchTool(RUT::VectorXd& wrench_T, int num_of_sensors = 1) override;
  /**
   * Get the tool wrench after tool weight compensation.
   *
   * @param[in]  pose          The Cartesian pose of the robot tool
   * @param      wrench_net_T  The net wrench in tool frame
   *
   * @return     0: no error. 1: still waiting for new data. 2: dead stream.
   *             3: force is too big.
   */
  int getWrenchNetTool(const RUT::Vector7d& pose, RUT::VectorXd& wrench_net_T,
                       int num_of_sensors = 1) override;

  int getNumSensors() override { return 1; }

  RUT::TimePoint _time0;  ///< high resolution timer.
  std::ofstream _file;
  ATINetftConfig _config;
  // netft
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> _netft;

 private:
  // thread
  pthread_t _thread;
};
