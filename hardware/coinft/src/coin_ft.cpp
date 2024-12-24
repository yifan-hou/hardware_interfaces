/***************************************************************************************************
 * File:        CoinFT.cpp
 * Author:      Hojung Choi
 * Email:       hjchoi92@stanford.edu
 * Institution: Stanford University
 * Date:        2024-11-19
 * Description: Implementation of the CoinFT class.
 *
 * Usage:       The variable 'latest_data' is where the latest wrench is stored.
 *              ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
 *              Force in [N]. Moment in [Nm].
 ***************************************************************************************************/

#include "coinft/coin_ft.h"
#include <fstream>
#include <sstream>

// Function to read matrix from CSV
Eigen::MatrixXd readMatrixFromCSV(const std::string& filename) {
  std::ifstream file(filename);
  std::string line;
  std::vector<double> values;  // To store all the values
  size_t rows = 0;

  while (std::getline(file, line)) {
    if (line.empty())
      continue;  // Skip empty lines
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      if (!cell.empty()) {
        values.push_back(std::stod(cell));
      }
    }
    ++rows;
  }

  // Assume that all rows have the same number of columns
  size_t cols = values.size() / rows;

  // Create the Eigen matrix
  Eigen::MatrixXd mat(rows, cols);
  for (size_t i = 0; i < values.size(); ++i) {
    mat(i / cols, i % cols) = values[i];
  }

  return mat;
}

CoinFT::CoinFT() {
  _force = RUT::Vector3d::Zero();
  _force_old = RUT::Vector3d::Zero();
  _torque = RUT::Vector3d::Zero();
  _torque_old = RUT::Vector3d::Zero();
  _stall_counts = 0;
}

CoinFT::~CoinFT() {
  stopStreaming();
  closePort();
}

bool CoinFT::init(RUT::TimePoint time0, const CoinFTConfig& config) {
  std::cout << "[CoinFT] initializing connection to " << config.port
            << std::endl;
  _time0 = time0;
  _config = config;
  _flag_started = false;

  _adj_sensor_tool = RUT::SE32Adj(RUT::pose2SE3(config.PoseSensorTool));

  try {
    io_ptr = std::make_shared<boost::asio::io_service>();
    serial_ptr =
        std::make_shared<boost::asio::serial_port>(*io_ptr, config.port);
  } catch (const std::exception& e) {
    std::cerr << "[CoinFT] Failed to open port " << config.port
              << ". Error: " << e.what() << '\n';
  }

  running = false;
  reading_counter = 0;
  tareFlag = false;
  tareSampleCount = 0;
  tareSampleTarget = 100;

  serial_ptr->set_option(
      boost::asio::serial_port_base::baud_rate(_config.baud_rate));
  std::cout << "Connected to Comport" << std::endl;
  initializeSensor();

  // Load the calibration matrix
  try {
    calibrationMatrix = readMatrixFromCSV(_config.calibration_file);
    std::cout << "Calibration matrix loaded. Dimensions: "
              << calibrationMatrix.rows() << " x " << calibrationMatrix.cols()
              << std::endl;

    // Check that calibration matrix is 6x24
    if (calibrationMatrix.rows() != 6 || calibrationMatrix.cols() != 24) {
      std::cerr << "Error: Calibration matrix must be 6x24." << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error loading calibration matrix: " << e.what() << std::endl;
  }

  // Initialize tareOffset to zero
  tareOffset = Eigen::VectorXd::Zero(num_sensor_units);

  // Initialize tareSamples
  tareSamples.clear();

  startStreaming();

  return true;
}

int CoinFT::getWrenchSensor(RUT::VectorXd& wrench, int num_of_sensors) {
  assert(wrench.size() == 6 * num_of_sensors);

  wrench.head(3) = _force;
  wrench.tail(3) = _torque;

  double data_change = (wrench.head(3) - _force_old).norm() +
                       10 * (wrench.tail(3) - _torque_old).norm();

  _force_old = _force;
  _torque_old = _torque;

  if (data_change > _config.noise_level) {
    _stall_counts = 0;
  } else {
    _stall_counts++;
    if (_stall_counts >= _config.stall_threshold) {
      std::cout << "\033[1;31m[CoinFT] Dead Stream\033[0m\n";
      return 2;
    }
  }

  // safety
  for (int i = 0; i < 6; ++i) {
    if (abs(wrench[i]) > _config.WrenchSafety[i]) {
      std::cout << "\033[1;31m[CoinFT] Force magnitude is above the safety "
                   "threshold:\033[0m\n";
      std::cout << "  feedback:" << wrench.transpose() << std::endl;
      std::cout << "  safety limit: " << _config.WrenchSafety.transpose()
                << std::endl;
      return -1;
    }
  }

  return 0;
}

int CoinFT::getWrenchTool(RUT::VectorXd& wrench_T, int num_of_sensors) {
  int flag = this->getWrenchSensor(_wrench_sensor_temp);
  wrench_T = _adj_sensor_tool.transpose() * _wrench_sensor_temp;
  return flag;
}

int CoinFT::getWrenchNetTool(const RUT::Vector7d& pose,
                             RUT::VectorXd& wrench_net_T, int num_of_sensors) {
  int flag = this->getWrenchTool(_wrench_tool_temp);

  // compensate for the weight of object
  _R_WT = RUT::quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  _GinF = _R_WT.transpose() * _config.Gravity;
  _GinT = _config.Pcom.cross(_GinF);
  wrench_net_T.head(3) = _wrench_tool_temp.head(3) + _config.Foffset - _GinF;
  wrench_net_T.tail(3) = _wrench_tool_temp.tail(3) + _config.Toffset - _GinT;

  return flag;
}

void CoinFT::initializeSensor() {
  // Send IDLE command
  sendChar(IDLE);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Send QUERY command to get packet size
  sendChar(QUERY);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Read packet size
  std::vector<uint8_t> data = readData(1);
  if (!data.empty()) {
    packet_size = static_cast<int>(data[0]) - 1;
  } else {
    throw std::runtime_error("Failed to read packet size.");
  }

  num_sensor_units = (packet_size - 1) / 2;
  std::cout << "Number of sensor units per CoinFT: " << num_sensor_units
            << std::endl;
}

void CoinFT::sendChar(uint8_t cmd) {
  boost::asio::write(*serial_ptr, boost::asio::buffer(&cmd, 1));
}

std::vector<uint8_t> CoinFT::readData(size_t length) {
  std::vector<uint8_t> buffer(length);
  boost::asio::read(*serial_ptr, boost::asio::buffer(buffer));
  return buffer;
}

void CoinFT::startStreaming() {
  sendChar(STREAM);
  running = true;
  tareFlag.store(true);  // Set tareFlag to true for initial taring
  data_thread = std::thread(&CoinFT::dataAcquisitionLoop, this);
}

void CoinFT::stopStreaming() {
  sendChar(IDLE);
  if (running) {
    running = false;
    if (data_thread.joinable()) {
      data_thread.join();
    }
  }
  tareFlag.store(true);  // Reset tareFlag so taring occurs next time
}

void CoinFT::tare() {
  tareFlag.store(true);
}

void CoinFT::dataAcquisitionLoop() {
  while (running) {
    std::vector<uint16_t> processedPacket(num_sensor_units, 0);
    Eigen::VectorXd rawInput(num_sensor_units);
    Eigen::VectorXd sum(num_sensor_units);
    Eigen::VectorXd newTareOffset(num_sensor_units);
    Eigen::VectorXd adjustedInput(num_sensor_units);
    Eigen::VectorXd extendedInput(num_sensor_units * 2);
    Eigen::VectorXd FT;
    try {
      // Read raw data
      readRawData(processedPacket);

      // Convert to Eigen vector
      for (int i = 0; i < num_sensor_units; ++i) {
        rawInput(i) = static_cast<double>(processedPacket[i]);
      }

      // If tareFlag is true, collect samples for taring
      if (tareFlag.load()) {
        // Collect samples
        tareSamples.push_back(rawInput);
        tareSampleCount++;

        // When enough samples are collected, compute tareOffset
        if (tareSampleCount >= tareSampleTarget) {
          // Compute average tareOffset
          sum.setZero();
          for (const auto& sample : tareSamples) {
            sum += sample;
          }
          newTareOffset = sum / static_cast<double>(tareSampleCount);
          {
            std::lock_guard<std::mutex> lock(data_mutex);
            tareOffset = newTareOffset;
          }
          tareSamples.clear();
          tareSampleCount = 0;
          tareFlag.store(false);
          std::cout << "Taring completed." << std::endl;
        }

        // Do not update latest_data during taring
        continue;  // Skip the rest of the loop and continue to next iteration
      }

      // Adjust rawInput by subtracting tareOffset
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        adjustedInput = rawInput - tareOffset;
      }

      // Create the extended vector [adjustedInput, adjustedInput.^2]
      for (int i = 0; i < num_sensor_units; ++i) {
        double value = adjustedInput(i);
        extendedInput(i) = value;
        extendedInput(i + num_sensor_units) = value * value;
      }

      // Perform the multiplication
      if (calibrationMatrix.cols() != extendedInput.size()) {
        std::cerr << "Error: Calibration matrix column count ("
                  << calibrationMatrix.cols()
                  << ") does not match extended vector size ("
                  << extendedInput.size() << ")." << std::endl;
        continue;
      }

      FT = calibrationMatrix * extendedInput;

      // Store the latest data thread-safely
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_data.assign(FT.data(), FT.data() + FT.size());

        _force[0] = latest_data[0];
        _force[1] = latest_data[1];
        _force[2] = latest_data[2];
        _torque[0] = latest_data[3];
        _torque[1] = latest_data[4];
        _torque[2] = latest_data[5];
      }
      data_cond
          .notify_one();  // Notify any waiting threads that new data is available
      reading_counter.fetch_add(1, std::memory_order_relaxed);
      _flag_started = true;
    } catch (const std::exception& e) {
      std::cerr << "Error in data acquisition loop: " << e.what() << std::endl;
      running = false;
    }
  }
}

bool CoinFT::readRawData(std::vector<uint16_t>& rawData) {
  // Wait for STX byte
  uint8_t byte = 0;
  do {
    byte = readData(1)[0];
  } while (byte != STX && running);

  if (!running)
    throw std::runtime_error("Data acquisition stopped.");

  // Read the rest of the packet
  std::vector<uint8_t> buffer(packet_size);
  boost::asio::read(*serial_ptr, boost::asio::buffer(buffer));

  // Check for ETX byte
  if (buffer[packet_size - 1] == ETX) {
    for (int i = 0; i < num_sensor_units; ++i) {
      rawData[i] = buffer[2 * i] + 256 * buffer[2 * i + 1];
    }
  } else {
    throw std::runtime_error("Bad end framing byte.");
  }
  return true;
}

std::vector<double> CoinFT::getLatestData() {
  std::lock_guard<std::mutex> lock(data_mutex);
  return latest_data;
}

void CoinFT::closePort() {
  if (serial_ptr->is_open()) {
    sendChar(IDLE);  // Ensure sensor is idle before closing
    serial_ptr->close();
    std::cout << "Port closed" << std::endl;
  }
}
