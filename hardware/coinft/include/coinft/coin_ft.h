/***************************************************************************************************
 * File:        CoinFT.h
 * Author:      Hojung Choi
 * Email:       hjchoi92@stanford.edu
 * Institution: Stanford University
 * Date:        2024-11-19
 * Description: Header file for the CoinFT class.
 *
 * Usage:       
 ***************************************************************************************************/

#ifndef COINFT_H
#define COINFT_H
#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <atomic>  // For atomic flags
#include <boost/asio.hpp>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>  // For mutex
#include <string>
#include <thread>
#include <vector>

#include "CoinFTBus.h"
#include "hardware_interfaces/ft_interfaces.h"

typedef std::chrono::high_resolution_clock Clock;

class CoinFT : public FTInterfaces {
 public:
  struct CoinFTConfig {
    std::string port{"/dev/tty.usbmodem101"};
    unsigned int baud_rate{115200};
    std::string left_calibration_file{"netft"};
    std::string right_calibration_file{"netft"};
    // If the force change is smaller than noise_level for more than stall_threshold frames,
    // the stream is considered dead.
    double noise_level{0.0};
    int stall_threshold{50};
    RUT::Vector3d Foffset{};
    RUT::Vector3d Toffset{};
    RUT::Vector3d Gravity{};
    RUT::Vector3d Pcom{};
    RUT::Vector6d WrenchSafety{};
    RUT::Vector7d PoseSensorToolLeft{};
    RUT::Vector7d PoseSensorToolRight{};

    bool deserialize(const YAML::Node& node) {
      try {
        port = node["port"].as<std::string>();
        baud_rate = node["baud_rate"].as<unsigned int>();
        left_calibration_file = node["left_calibration_file"].as<std::string>();
        right_calibration_file =
            node["right_calibration_file"].as<std::string>();
        noise_level = node["noise_level"].as<double>();
        stall_threshold = node["stall_threshold"].as<int>();
        Foffset = RUT::deserialize_vector<RUT::Vector3d>(node["Foffset"]);
        Toffset = RUT::deserialize_vector<RUT::Vector3d>(node["Toffset"]);
        Gravity = RUT::deserialize_vector<RUT::Vector3d>(node["Gravity"]);
        Pcom = RUT::deserialize_vector<RUT::Vector3d>(node["Pcom"]);
        WrenchSafety =
            RUT::deserialize_vector<RUT::Vector6d>(node["WrenchSafety"]);
        PoseSensorToolLeft =
            RUT::deserialize_vector<RUT::Vector7d>(node["PoseSensorToolLeft"]);
        PoseSensorToolRight =
            RUT::deserialize_vector<RUT::Vector7d>(node["PoseSensorToolRight"]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  CoinFT();
  ~CoinFT();
  bool init(RUT::TimePoint time0, const CoinFTConfig& coinft_config);
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

  int getNumSensors() override { return 2; }

 private:
  RUT::TimePoint _time0;
  CoinFTConfig _config;

  std::vector<double> _latest_data_left;
  std::vector<double> _latest_data_right;

  RUT::Matrix6d _adj_sensor_tool_left;
  RUT::Matrix6d _adj_sensor_tool_right;

  std::shared_ptr<CoinFTBus> _coinft_bus_ptr;
};

#endif  // COINFT_H
