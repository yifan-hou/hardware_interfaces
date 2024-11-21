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

class CoinFT {
 public:
  CoinFT(const std::string& port, unsigned int baud_rate,
         const std::string& calibration_file);
  ~CoinFT();

  void startStreaming();
  void stopStreaming();
  void closePort();
  void tare();

  std::vector<double> getLatestData();
  std::atomic<uint64_t> reading_counter;

 private:
  enum Commands {
    IDLE = 0x69,    // 'i'
    QUERY = 0x71,   // 'q'
    STREAM = 0x73,  // 's'
  };

  static const uint8_t STX = 0x02;
  static const uint8_t ETX = 0x03;

  void initializeSensor();
  void sendChar(uint8_t cmd);
  std::vector<uint8_t> readData(size_t length);
  std::vector<uint16_t> readRawData();
  void dataAcquisitionLoop();  // The function run by the background thread

  boost::asio::io_service io;
  boost::asio::serial_port serial;
  int packet_size;
  int num_sensors;

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
