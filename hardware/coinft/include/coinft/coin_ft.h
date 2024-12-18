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

#include "hardware_interfaces/ft_interfaces.h"

typedef std::chrono::high_resolution_clock Clock;

class CoinFT : public FTInterfaces {
 public:
  struct CoinFTConfig {
    std::string port{"/dev/ttyACM0"};
    unsigned int baud_rate{115200};
    std::string calibration_file{"netft"};
    int num_sensors{1};
    int publish_rate{360};
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
        port = node["port"].as<std::string>();
        baud_rate = node["baud_rate"].as<unsigned int>();
        calibration_file = node["calibration_file"].as<std::string>();
        num_sensors = node["num_sensors"].as<int>();
        publish_rate = node["publish_rate"].as<int>();
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

  int getNumSensors() override { return _config.num_sensors; }

  void startStreaming();
  void stopStreaming();
  void closePort();
  void tare();

 private:
  RUT::TimePoint _time0;
  CoinFTConfig _config;

  // CoinFT internal variables
  enum Commands {
    IDLE = 0x69,    // 'i'
    QUERY = 0x71,   // 'q'
    STREAM = 0x73,  // 's'
  };
  std::atomic<uint64_t> reading_counter;

  static const uint8_t STX = 0x02;
  static const uint8_t ETX = 0x03;

  std::vector<double> getLatestData();
  void initializeSensor();
  void sendChar(uint8_t cmd);
  std::vector<uint8_t> readData(size_t length);
  bool readRawData(std::vector<uint16_t>& rawData);
  void dataAcquisitionLoop();  // The function run by the background thread

  std::shared_ptr<boost::asio::io_service> io_ptr;
  std::shared_ptr<boost::asio::serial_port> serial_ptr;
  int packet_size;
  int num_sensor_units;  // CoinFT internal parameter

  std::thread data_thread;
  std::mutex data_mutex;
  std::vector<double> latest_data;
  std::atomic<bool> running;
  std::condition_variable data_cond;

  std::atomic<bool> tareFlag;
  std::vector<Eigen::VectorXd> tareSamples;

  size_t tareSampleCount;
  size_t tareSampleTarget;

  Eigen::MatrixXd calibrationMatrix;
  Eigen::VectorXd tareOffset;
};

#endif  // COINFT_H
