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

CoinFT::CoinFT(const std::string& port, unsigned int baud_rate,
               const std::string& calibration_file)
    : io(),
      serial(io, port),
      running(false),
      reading_counter(0),
      tareFlag(false),
      tareSampleCount(0),
      tareSampleTarget(100) {
  serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  std::cout << "Connected to Comport" << std::endl;
  initializeSensor();

  // Load the calibration matrix
  try {
    calibrationMatrix = readMatrixFromCSV(calibration_file);
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
  tareOffset = Eigen::VectorXd::Zero(num_sensors);

  // Initialize tareSamples
  tareSamples.clear();
}

CoinFT::~CoinFT() {
  stopStreaming();
  closePort();
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

  num_sensors = (packet_size - 1) / 2;
  std::cout << "Number of sensors: " << num_sensors << std::endl;
}

void CoinFT::sendChar(uint8_t cmd) {
  boost::asio::write(serial, boost::asio::buffer(&cmd, 1));
}

std::vector<uint8_t> CoinFT::readData(size_t length) {
  std::vector<uint8_t> buffer(length);
  boost::asio::read(serial, boost::asio::buffer(buffer));
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
    try {
      // Read raw data
      std::vector<uint16_t> processedPacket = readRawData();

      // Convert to Eigen vector
      Eigen::VectorXd rawInput(num_sensors);
      for (int i = 0; i < num_sensors; ++i) {
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
          Eigen::VectorXd sum = Eigen::VectorXd::Zero(num_sensors);
          for (const auto& sample : tareSamples) {
            sum += sample;
          }
          Eigen::VectorXd newTareOffset =
              sum / static_cast<double>(tareSampleCount);
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
      Eigen::VectorXd adjustedInput(num_sensors);
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        adjustedInput = rawInput - tareOffset;
      }

      // Create the extended vector [adjustedInput, adjustedInput.^2]
      Eigen::VectorXd extendedInput(num_sensors * 2);
      for (int i = 0; i < num_sensors; ++i) {
        double value = adjustedInput(i);
        extendedInput(i) = value;
        extendedInput(i + num_sensors) = value * value;
      }

      // Perform the multiplication
      if (calibrationMatrix.cols() != extendedInput.size()) {
        std::cerr << "Error: Calibration matrix column count ("
                  << calibrationMatrix.cols()
                  << ") does not match extended vector size ("
                  << extendedInput.size() << ")." << std::endl;
        continue;
      }

      Eigen::VectorXd FT = calibrationMatrix * extendedInput;

      // Store the latest data thread-safely
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_data.assign(FT.data(), FT.data() + FT.size());
      }
      data_cond
          .notify_one();  // Notify any waiting threads that new data is available
      reading_counter.fetch_add(1, std::memory_order_relaxed);

    } catch (const std::exception& e) {
      std::cerr << "Error in data acquisition loop: " << e.what() << std::endl;
      running = false;
    }
  }
}

std::vector<uint16_t> CoinFT::readRawData() {
  // Wait for STX byte
  uint8_t byte = 0;
  do {
    byte = readData(1)[0];
  } while (byte != STX && running);

  if (!running)
    throw std::runtime_error("Data acquisition stopped.");

  // Read the rest of the packet
  std::vector<uint8_t> buffer(packet_size);
  boost::asio::read(serial, boost::asio::buffer(buffer));

  // Check for ETX byte
  if (buffer[packet_size - 1] == ETX) {
    std::vector<uint16_t> rawData(num_sensors, 0);
    for (int i = 0; i < num_sensors; ++i) {
      rawData[i] = buffer[2 * i] + 256 * buffer[2 * i + 1];
    }
    return rawData;
  } else {
    throw std::runtime_error("Bad end framing byte.");
  }
}

std::vector<double> CoinFT::getLatestData() {
  std::lock_guard<std::mutex> lock(data_mutex);
  return latest_data;
}

void CoinFT::closePort() {
  if (serial.is_open()) {
    sendChar(IDLE);  // Ensure sensor is idle before closing
    serial.close();
    std::cout << "Port closed" << std::endl;
  }
}
