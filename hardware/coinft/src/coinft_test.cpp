/***************************************************************************************************
 * File:        coinft_interface.cpp
 * Author:      Hojung Choi
 * Email:       hjchoi92@stanford.edu
 * Institution: Stanford University
 * Date:        2024-11-19
 * Description: This script shows an example of how might the coinft class be used to acquire data.
 *
 * Usage:       ./coinft_interface <RunDurationInSeconds>
 ***************************************************************************************************/

#include <chrono>
#include <iomanip>  // For std::setprecision
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "coinft/coin_ft.h"  // Include the CoinFT class

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: ./coinft_interface <RunDurationInSeconds>"
              << std::endl;
    return 1;
  }

  double run_duration = std::stod(argv[1]);  // Convert duration to double

  try {
    // Provide the serial port, baud rate, and calibration matrix file name
    std::cout << "Creating CoinFT object..." << std::endl;
    CoinFT sensor("/dev/ttyACM0", 115200,
                  "/home/yifanhou/git/hardware_interfaces/hardware/coinft/"
                  "config/calMat_UFT6.csv");
    std::cout << "CoinFT object created." << std::endl;
    std::cout << "Starting streaming..." << std::endl;
    sensor.startStreaming();
    std::cout << "Streaming started." << std::endl;

    // Start the timer
    auto start_time = std::chrono::steady_clock::now();

    // Main loop
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         start_time)
               .count() < run_duration) {
      std::cout << "debug 1" << std::endl;
      // Retrieve the force/torque data
      std::vector<double> ftData = sensor.getLatestData();

      // Output the data
      if (!ftData.empty()) {
        std::cout << "[" << std::fixed << std::setprecision(3)
                  << std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_time)
                         .count()
                  << " s] "
                  << "Force/Torque Data: ";
        for (const auto& value : ftData) {
          std::cout << value << " ";
        }
        std::cout << std::endl;
      } else {
        std::cout << "No data available yet." << std::endl;
      }

      // Sleep for a short duration
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop streaming and clean up
    sensor.stopStreaming();
  } catch (const std::exception& e) {
    std::cerr << "An error occurred: " << e.what() << std::endl;
  }

  return 0;
}
