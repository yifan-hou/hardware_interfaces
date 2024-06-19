#include "realsense/realsense.h"

#include <iostream>
#include <librealsense2/rs.hpp>

Realsense* Realsense::pinstance = 0;

struct Realsense::Implementation {
  Realsense::RealsenseConfig config{};
  RUT::TimePoint time0;

  rs2::pipeline pipe;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0,
                  const Realsense::RealsenseConfig& config);
};

Realsense::Implementation::Implementation() {}

Realsense::Implementation::~Implementation() {
  std::cout << "[Realsense] finishing.." << std::endl;
  delete pinstance;
}

bool Realsense::Implementation::initialize(
    RUT::TimePoint time0, const Realsense::RealsenseConfig& realsense_config) {
  time0 = time0;
  config = realsense_config;

  // Start streaming with default recommended configuration
  // The default video configuration contains Depth and Color streams
  // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
  pipe.start();

  std::cout << "[Realsense] Pipeline started.\n";
  return true;
}

Realsense::Realsense() : m_impl{std::make_unique<Implementation>()} {}

Realsense::~Realsense() {}

Realsense* Realsense::Instance() {
  if (pinstance == 0) {
    pinstance = new Realsense();
  }
  return pinstance;
}

bool Realsense::init(RUT::TimePoint time0,
                     const RealsenseConfig& realsense_config) {
  return m_impl->initialize(time0, realsense_config);
}

rs2::frameset Realsense::wait_for_frames() {
  try {
    return m_impl->pipe.wait_for_frames();
  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return rs2::frameset();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return rs2::frameset();
  }
}