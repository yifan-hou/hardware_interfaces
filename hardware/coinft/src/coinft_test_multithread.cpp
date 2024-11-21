/***************************************************************************************************
 * File:        coinft_test_multithread.cpp
 * Author:      Hojung Choi
 * Email:       hjchoi92@stanford.edu
 * Institution: Stanford University
 * Date:        2024-11-19
 * Description: This script shows an example of how might the coinft class be used to acquire data.
 *              It demonstrates the multi-threading capabilities. Note that with longer sleep time in 
 *              the main loop, the more data you acquire.
 *
 * Usage:       ./coinft_interface <RunDurationInSeconds>
 ***************************************************************************************************/

#include <chrono>
#include <iomanip>  // For std::setprecision
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "coinft/coin_ft.h"  // Include the CoinFT class

// Simulate the robot_control function with variable delay
void robot_control(int& delay_ms, int& increment, const int min_delay,
                   const int max_delay) {
  auto start_time = std::chrono::steady_clock::now();

  // Simulate the delay
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

  // Adjust the delay for next iteration
  delay_ms += increment;
  if (delay_ms >= max_delay || delay_ms <= min_delay) {
    increment = -increment;  // Reverse the direction of change
  }

  auto end_time = std::chrono::steady_clock::now();
  double actual_delay =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

  std::cout << "Robot control delay requested: " << delay_ms << " ms, "
            << "actual delay: " << actual_delay
            << " ms on thread ID: " << std::this_thread::get_id() << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: ./coinft_test <RunDurationInSeconds>" << std::endl;
    return 1;
  }

  double run_duration = std::stod(argv[1]);  // Convert duration to double

  try {
    // Initialize and start the CoinFT sensor
    CoinFT sensor("/dev/tty.usbmodem2102", 115200, "../src/calMat_UFT6.csv");
    sensor.startStreaming();

    // Variables for robot_control()
    int delay_ms = 50;   // Start at 50 ms
    int increment = 10;  // Increment by 10 ms
    const int min_delay = 50;
    const int max_delay = 100;
    uint64_t last_reading_counter =
        sensor.reading_counter.load(std::memory_order_relaxed);
    auto last_check_time = std::chrono::steady_clock::now();

    // Start the timer
    auto start_time = std::chrono::steady_clock::now();

    // Main control loop
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         start_time)
               .count() < run_duration) {
      // Perform the robot control operation with variable delay
      robot_control(delay_ms, increment, min_delay, max_delay);

      // Retrieve the current reading counter
      uint64_t current_counter =
          sensor.reading_counter.load(std::memory_order_relaxed);
      auto current_time = std::chrono::steady_clock::now();

      // Calculate the number of readings since last check
      uint64_t readings_since_last = current_counter - last_reading_counter;

      // Calculate the time elapsed since last check in seconds
      double time_elapsed =
          std::chrono::duration<double>(current_time - last_check_time).count();

      // Calculate the sensor reading rate (readings per second)
      double reading_rate = 0.0;
      if (time_elapsed > 0.0) {
        reading_rate = readings_since_last / time_elapsed;
      }

      // Output the results
      std::cout
          << "[" << std::fixed << std::setprecision(3)
          << std::chrono::duration<double>(current_time - start_time).count()
          << " s] "
          << "Robot control delay: " << delay_ms << " ms, "
          << "Readings since last check: " << readings_since_last << ", "
          << "Time elapsed: " << time_elapsed << " s, "
          << "Reading rate: " << reading_rate << " readings/s" << std::endl;

      // Update last_reading_counter and last_check_time
      last_reading_counter = current_counter;
      last_check_time = current_time;
    }

    // Stop streaming and clean up
    sensor.stopStreaming();
  } catch (const std::exception& e) {
    std::cerr << "An error occurred: " << e.what() << std::endl;
  }
  return 0;
}
