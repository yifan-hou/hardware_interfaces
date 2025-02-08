#ifndef COINFTBUS_H
#define COINFTBUS_H

// #include <cpu_provider_factory.h>
#include <onnxruntime_cxx_api.h>
#include <Eigen/Dense>
#include <boost/asio.hpp>
#include <csignal>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct CoinFTSensor {
  int address;
  bool tareInProgress = true;
  std::string input_name;
  std::string output_name;
  std::unique_ptr<Ort::Session>
      session;  // ONNX Runtime session for the neural network
  int tareSampleCount = 0;
  int tareSampleTarget = 100;
  std::vector<Eigen::VectorXd> tareSamples;
  Eigen::VectorXd tareOffset;
  std::vector<double> latestData;
  std::mutex mutex;

  // Move constructor
  CoinFTSensor(CoinFTSensor&& other) noexcept
      : address(other.address),
        tareInProgress(other.tareInProgress),
        input_name(std::move(other.input_name)),
        output_name(std::move(other.output_name)),
        session(std::move(other.session)),
        tareSampleCount(other.tareSampleCount),
        tareSampleTarget(other.tareSampleTarget),
        tareSamples(std::move(other.tareSamples)),
        tareOffset(std::move(other.tareOffset)),
        latestData(std::move(other.latestData)) {}

  // Move assignment operator
  CoinFTSensor& operator=(CoinFTSensor&& other) noexcept {
    if (this != &other) {
      address = other.address;
      tareInProgress = other.tareInProgress;
      input_name = std::move(other.input_name);
      output_name = std::move(other.output_name);
      session = std::move(other.session);
      tareSampleCount = other.tareSampleCount;
      tareSampleTarget = other.tareSampleTarget;
      tareSamples = std::move(other.tareSamples);
      tareOffset = std::move(other.tareOffset);
      latestData = std::move(other.latestData);
    }
    return *this;
  }

  // Disable copy constructor and copy assignment operator
  CoinFTSensor(const CoinFTSensor&) = delete;
  CoinFTSensor& operator=(const CoinFTSensor&) = delete;

  // Default constructor
  CoinFTSensor() = default;
};

class CoinFTBus {
 public:
  // Enum for command values (moved inside the class)
  enum Commands {
    IDLE = 0x69,   // 'i'
    QUERY = 0x71,  // 'q'
    STREAM = 0x73  // 's'
  };

  static constexpr uint8_t STX = 0x02;  // Start of Transmission byte
  static constexpr uint8_t ETX = 0x03;  // End of Transmission byte

  // Force and moment scaling factors
  static constexpr double FORCE_SCALING = 1.0;
  static constexpr double MOMENT_SCALING = 30.0;

  CoinFTBus(const std::string& port, unsigned int baud_rate,
            const std::vector<std::pair<int, std::string>>& sensorConfigs);
  ~CoinFTBus();

  void startStreaming();
  void stopStreaming();
  void idle();
  void tareSensor(int address);
  std::vector<double> getLatestData(int address);

  static constexpr int LEFT = 8;
  static constexpr int RIGHT = 9;

 private:
  void initializeBus();
  void sendIdle();
  void sendChar(uint8_t cmd);
  std::vector<uint8_t> readData(size_t length);
  void dataAcquisitionLoop();

  static void signalHandler(int signum);
  static CoinFTBus* instance;

  std::map<int, CoinFTSensor> sensors;
  std::mutex sensors_mutex;
  boost::asio::io_service io;
  boost::asio::serial_port serial;
  std::atomic<bool> running;
  std::thread data_thread;
  int packet_size;
  int num_raw_channels;
};

#endif  // COINFTBUS_H
