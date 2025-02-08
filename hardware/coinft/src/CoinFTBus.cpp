#include "coinft/CoinFTBus.h"
#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

// Global ONNX Runtime environment (only one per process is recommended)
static Ort::Env ort_env(ORT_LOGGING_LEVEL_WARNING, "CoinFTBus");
Ort::AllocatorWithDefaultOptions allocator;

std::vector<CoinFTBus*> activeInstances;
std::mutex instanceMutex;

//----------------------------------------------------------------
// Constructor
//----------------------------------------------------------------
CoinFTBus::CoinFTBus(
    const std::string& port, unsigned int baud_rate,
    const std::vector<std::pair<int, std::string>>& sensorConfigs)
    : serial(io), running(false), packet_size(0), num_raw_channels(0) {
  {
    std::lock_guard<std::mutex> lock(instanceMutex);
    activeInstances.push_back(this);
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGTSTP, signalHandler);

  // **Close lingering serial connections first**
  if (serial.is_open()) {
    std::cout << "[DEBUG] Closing previous serial connection..." << std::endl;
    serial.close();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(10));  // Allow OS time to release port
  }

  // Open the serial port
  serial.open(port);
  serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  std::cout << "Opened serial port: " << port << std::endl;

  // Initialize the bus
  initializeBus();

  // Load sensor configurations
  for (const auto& config : sensorConfigs) {
    int address = config.first;
    const std::string& model_file = config.second;

    CoinFTSensor sensor;
    sensor.address = address;
    sensor.tareInProgress = true;
    sensor.tareSampleCount = 0;
    sensor.tareSampleTarget = 100;
    sensor.tareOffset = Eigen::VectorXd::Zero(num_raw_channels);
    sensor.latestData = std::vector<double>(6, 0.0);

    // Create ONNX session options. TODO: what does this do, line by line?
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Create ONNX session
    sensor.session = std::make_unique<Ort::Session>(ort_env, model_file.c_str(),
                                                    session_options);

    // Retrieve input and output node names
    sensor.input_name =
        sensor.session->GetInputNameAllocated(0, allocator).get();
    sensor.output_name =
        sensor.session->GetOutputNameAllocated(0, allocator).get();

    // Store the sensor configuration
    {
      std::lock_guard<std::mutex> lock(sensors_mutex);
      sensors[address] = std::move(sensor);
    }

    std::cout << "Configured sensor at address " << address
              << " with ONNX model: " << model_file << std::endl;
  }

  // Start streaming
  startStreaming();

  // Wait until all sensors are tared
  std::cout << "Waiting for sensors to tare..." << std::endl;
  bool allTared = false;
  while (!allTared) {
    allTared = true;
    {
      std::lock_guard<std::mutex> lock(sensors_mutex);
      for (auto& kv : sensors) {
        std::lock_guard<std::mutex> sensor_lock(kv.second.mutex);
        if (kv.second.tareInProgress) {
          allTared = false;
          break;
        }
      }
    }
    if (!allTared) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  std::cout << "All sensors are tared." << std::endl;
}

//----------------------------------------------------------------
// Destructor
//----------------------------------------------------------------
CoinFTBus::~CoinFTBus() {
  stopStreaming();
  if (serial.is_open()) {
    sendIdle();
    serial.close();
  }

  // Remove instance from activeInstances
  {
    std::lock_guard<std::mutex> lock(instanceMutex);
    activeInstances.erase(
        std::remove(activeInstances.begin(), activeInstances.end(), this),
        activeInstances.end());
  }
}

//----------------------------------------------------------------
// Initialize Bus
//----------------------------------------------------------------

void CoinFTBus::initializeBus() {
  std::cout << "[DEBUG] Attempting to reset device to IDLE mode..."
            << std::endl;

  // Send IDLE multiple times to ensure sensor resets from an unknown state
  for (int i = 0; i < 3; i++) {
    sendChar(IDLE);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

// Clear any unread data in the serial buffer
#if defined(__unix__) || defined(__APPLE__)
  serial.cancel();
  std::cout << "[DEBUG] Flushing serial buffer..." << std::endl;
  int fd = serial.native_handle();
  tcflush(fd, TCIFLUSH);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
#endif

  std::cout << "[DEBUG] Sending QUERY command to get packet size..."
            << std::endl;
  sendChar(QUERY);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Read response
  std::vector<uint8_t> data = readData(1);
  if (data.empty()) {
    throw std::runtime_error(
        "[ERROR] Failed to read packet size from sensor bus.");
  }

  packet_size = static_cast<int>(data[0]) - 1;
  num_raw_channels = (packet_size - 1) / 2;
  std::cout << "[DEBUG] Packet size: " << packet_size
            << ", Number of raw channels: " << num_raw_channels << std::endl;
}

//----------------------------------------------------------------
// Start Streaming
//----------------------------------------------------------------
void CoinFTBus::startStreaming() {
  if (running.load())
    return;

  sendChar(STREAM);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  running.store(true);
  data_thread = std::thread(&CoinFTBus::dataAcquisitionLoop, this);
  std::cout << "Started streaming on sensor bus." << std::endl;
}

//----------------------------------------------------------------
// Stop Streaming
//----------------------------------------------------------------
/*void CoinFTBus::stopStreaming() {
    running.store(false);
    sendIdle();  // Ensure sensors are set to idle
    serial.cancel();  // Cancel any ongoing read operations

    if (data_thread.joinable()) {
        std::cerr << "[DEBUG] Waiting for data thread to stop..." << std::endl;
        data_thread.join();  // Wait for the thread to finish
        std::cerr << "[DEBUG] Data thread successfully stopped." << std::endl;
    }

    std::cout << "Stopped streaming on sensor bus." << std::endl;
}*/
void CoinFTBus::stopStreaming() {
  if (!running.exchange(false))
    return;  // Prevent multiple calls

  std::cerr << "[DEBUG] Stopping data acquisition thread..." << std::endl;

  sendIdle();       // Ensure sensors are set to idle
  serial.cancel();  // Cancel ongoing read operations

  // Give the thread some time to exit gracefully
  if (data_thread.joinable()) {
    std::thread killer([&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // If still joinable, force detach
      if (data_thread.joinable()) {
        std::cerr << "[WARNING] Data thread did not stop in time, forcefully "
                     "detaching!"
                  << std::endl;
        data_thread.detach();
      }
    });

    data_thread.join();
    killer.detach();
  }

  io.stop();   // Ensure io_service stops processing events
  io.reset();  // Reset for potential reuse

  std::cout << "[DEBUG] Data thread successfully stopped." << std::endl;
  std::cout << "Stopped streaming on sensor bus." << std::endl;
}

//----------------------------------------------------------------
// Idle
//----------------------------------------------------------------
void CoinFTBus::idle() {
  sendChar(IDLE);
  std::cout << "Sensor bus set to idle mode." << std::endl;
}

//----------------------------------------------------------------
// Tare Sensor
//----------------------------------------------------------------
void CoinFTBus::tareSensor(int address) {
  std::lock_guard<std::mutex> lock(sensors_mutex);
  auto it = sensors.find(address);
  if (it != sensors.end()) {
    std::lock_guard<std::mutex> sensor_lock(it->second.mutex);
    it->second.tareInProgress = true;
    it->second.tareSampleCount = 0;
    it->second.tareSamples.clear();
    it->second.tareOffset = Eigen::VectorXd::Zero(num_raw_channels);
    std::cout << "Taring initiated for sensor at address: " << address
              << std::endl;
  } else {
    std::cerr << "Sensor with address " << address << " not found."
              << std::endl;
  }
}

//----------------------------------------------------------------
// Get Latest Data
//----------------------------------------------------------------
std::vector<double> CoinFTBus::getLatestData(int address) {
  std::lock_guard<std::mutex> lock(sensors_mutex);
  auto it = sensors.find(address);
  if (it != sensors.end()) {
    std::lock_guard<std::mutex> sensor_lock(it->second.mutex);
    return it->second.latestData;
  } else {
    std::cerr << "Sensor with address " << address << " not found."
              << std::endl;
    return std::vector<double>();
  }
}

//----------------------------------------------------------------
// Send Idle Command
//----------------------------------------------------------------
void CoinFTBus::sendIdle() {
  sendChar(IDLE);
}

//----------------------------------------------------------------
// Send Character Command
//----------------------------------------------------------------
void CoinFTBus::sendChar(uint8_t cmd) {
  boost::asio::write(serial, boost::asio::buffer(&cmd, 1));
}

//----------------------------------------------------------------
// Read Data from Sensor Bus
//----------------------------------------------------------------

std::vector<uint8_t> CoinFTBus::readData(size_t length) {
  std::vector<uint8_t> buffer(length);
  boost::system::error_code ec;

  // Ensure io_service is reset before running async operations
  io.restart();

  boost::asio::deadline_timer timer(io, boost::posix_time::milliseconds(500));

  bool timeout_flag = false;
  bool read_complete = false;

  //std::cout << "[DEBUG] Starting async read of " << length << " bytes..." << std::endl;

  // Setup timeout handling
  timer.async_wait([&](const boost::system::error_code& error) {
    if (!error && !read_complete) {  // Timer expired
      timeout_flag = true;
      std::cerr << "[ERROR] Serial read timeout! Cancelling operation."
                << std::endl;
      serial.cancel();
    }
  });

  // Start async read operation
  boost::asio::async_read(serial, boost::asio::buffer(buffer, length),
                          [&](const boost::system::error_code& error,
                              std::size_t bytes_transferred) {
                            ec = error;
                            read_complete = true;
                            timer.cancel();  // Stop the timeout timer

                            if (error) {
                              std::cerr << "[ERROR] Serial read failed: "
                                        << error.message() << std::endl;
                            } else {
                              //std::cout << "[DEBUG] Successfully read " << bytes_transferred << " bytes." << std::endl;
                            }
                          });

  // Process IO events until read completes or timeout occurs
  while (!read_complete && !timeout_flag) {
    io.run_one();
  }

  // If a timeout occurred, return empty buffer
  if (timeout_flag) {
    std::cerr << "[ERROR] Read operation timed out. Returning empty buffer."
              << std::endl;
    return {};
  }

  // Handle errors
  if (ec && ec != boost::asio::error::operation_aborted) {
    std::cerr << "[ERROR] Serial read error: " << ec.message() << std::endl;
    return {};
  }

  return buffer;
}

//----------------------------------------------------------------
// Data Acquisition Loop
//----------------------------------------------------------------
void CoinFTBus::dataAcquisitionLoop() {

  while (running.load()) {
    try {
      // Wait for the STX byte.
      uint8_t byte = 0;
      do {
        std::vector<uint8_t> b = readData(1);
        if (b.empty())
          continue;
        byte = b[0];
      } while (byte != STX && running.load());

      if (!running.load())
        break;

      // Read the rest of the packet.
      std::vector<uint8_t> packet = readData(packet_size);
      if (packet.size() != static_cast<size_t>(packet_size)) {
        std::cerr << "Incomplete packet received." << std::endl;
        continue;
      }

      // Extract the sensor address.
      uint8_t end_byte = packet[packet_size - 1];

      if ((end_byte & 0x0F) != ETX) {
        std::cerr
            << "[ERROR] Bad end framing byte. Expected ETX (0x03), but got: 0x"
            << std::hex << static_cast<int>(end_byte & 0x0F) << std::dec
            << std::endl;
        continue;  // Skip processing this packet
      }

      int sensorAddress = end_byte >> 4;

      // Extract raw sensor values.
      std::vector<uint16_t> rawValues;
      for (int i = 0; i < num_raw_channels; ++i) {
        int index = 2 * i;
        uint16_t value = packet[index] + 256 * packet[index + 1];
        rawValues.push_back(value);
      }

      // Process the sensor's data.
      {
        std::lock_guard<std::mutex> lock(sensors_mutex);
        auto it = sensors.find(sensorAddress);
        //TODO: what does sensos.end() mean?
        if (it == sensors.end()) {
          std::cerr << "Received data for unknown sensor address: "
                    << sensorAddress << std::endl;
          continue;
        }
        CoinFTSensor& sensor = it->second;
        std::lock_guard<std::mutex> sensor_lock(sensor.mutex);

        // Convert raw values to an Eigen vector.
        Eigen::VectorXd rawInput(num_raw_channels);
        for (int i = 0; i < num_raw_channels; ++i) {
          rawInput(i) = static_cast<double>(rawValues[i]);
        }

        // If taring is active, accumulate samples.
        if (sensor.tareInProgress) {
          sensor.tareSamples.push_back(rawInput);
          sensor.tareSampleCount++;
          if (sensor.tareSampleCount >= sensor.tareSampleTarget) {
            // Compute the average tare offset.
            Eigen::VectorXd sum = Eigen::VectorXd::Zero(num_raw_channels);
            for (const auto& sample : sensor.tareSamples) {
              sum += sample;
            }
            sensor.tareOffset =
                sum / static_cast<double>(sensor.tareSampleCount);
            sensor.tareSamples.clear();
            sensor.tareSampleCount = 0;
            sensor.tareInProgress = false;
            std::cout << "Taring completed for sensor " << sensorAddress
                      << std::endl;
          }
          // Skip further processing until taring is complete.
          continue;
        }

        // Process the reading if taring is complete.
        Eigen::VectorXd adjustedInput = rawInput - sensor.tareOffset;

        // Build the input tensor for ONNX inference.
        std::vector<float> input_tensor_values(num_raw_channels);
        for (int i = 0; i < num_raw_channels; ++i) {
          input_tensor_values[i] = static_cast<float>(adjustedInput(i));
        }
        std::array<int64_t, 2> input_shape = {1, num_raw_channels};

        Ort::MemoryInfo memory_info =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_tensor_values.data(), input_tensor_values.size(),
            input_shape.data(), input_shape.size());

        std::vector<const char*> input_names = {sensor.input_name.c_str()};
        std::vector<const char*> output_names = {sensor.output_name.c_str()};

        // Run the ONNX inference.
        auto output_tensors =
            sensor.session->Run(Ort::RunOptions{nullptr}, input_names.data(),
                                &input_tensor, 1, output_names.data(), 1);
        float* float_array = output_tensors[0].GetTensorMutableData<float>();

        // Apply scaling factors.
        std::vector<double> ft(6);
        ft[0] = static_cast<double>(float_array[0]) / FORCE_SCALING;
        ft[1] = static_cast<double>(float_array[1]) / FORCE_SCALING;
        ft[2] = static_cast<double>(float_array[2]) / FORCE_SCALING;
        ft[3] = static_cast<double>(float_array[3]) / MOMENT_SCALING;
        ft[4] = static_cast<double>(float_array[4]) / MOMENT_SCALING;
        ft[5] = static_cast<double>(float_array[5]) / MOMENT_SCALING;

        // Update the sensor's latest data.
        sensor.latestData = ft;
      }  // end lock on sensors_mutex

    } catch (const std::exception& e) {
      std::cerr << "Exception in data acquisition loop: " << e.what()
                << std::endl;
      running.store(false);
    }
  }
}

//----------------------------------------------------------------
// Signal Handler
//----------------------------------------------------------------
void CoinFTBus::signalHandler(int signum) {
  if (signum == SIGINT || signum == SIGTERM) {
    std::cout << "\n[DEBUG] Signal (" << signum
              << ") received, shutting down all active instances..."
              << std::endl;
    {
      std::lock_guard<std::mutex> lock(instanceMutex);
      for (auto* instance : activeInstances) {
        if (instance) {
          instance->stopStreaming();
          instance->idle();
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::exit(signum);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  } else if (signum == SIGTSTP) {  // Handle `Ctrl+Z` separately
    std::cout
        << "\n[DEBUG] Signal (SIGTSTP) received, suspending all instances..."
        << std::endl;

    {
      std::lock_guard<std::mutex> lock(instanceMutex);
      for (auto* instance : activeInstances) {
        if (instance) {
          instance->stopStreaming();
          instance->idle();
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Send SIGSTOP to suspend process
    kill(getpid(), SIGSTOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}
