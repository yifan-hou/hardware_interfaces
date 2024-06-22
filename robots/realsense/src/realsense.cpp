#include "realsense/realsense.h"
#include "realsense/cv-helpers.hpp"

#include <iostream>
#include <librealsense2/rs.hpp>

struct Realsense::Implementation {
  Realsense::RealsenseConfig config{};
  RUT::TimePoint time0;

  rs2::pipeline pipe;
  std::shared_ptr<rs2::align> align_to_ptr;
  rs2::frameset frames;
  rs2::frame color_frame;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0,
                  const Realsense::RealsenseConfig& config);
  cv::Mat next_rgb_frame_blocking();
};

Realsense::Implementation::Implementation() {}

Realsense::Implementation::~Implementation() {
  std::cout << "[Realsense] finishing.." << std::endl;
}

bool Realsense::Implementation::initialize(
    RUT::TimePoint time0, const Realsense::RealsenseConfig& realsense_config) {
  time0 = time0;
  config = realsense_config;

  align_to_ptr = std::make_shared<rs2::align>(rs2_stream::RS2_STREAM_COLOR);

  // stream: https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
  // format: https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#ae04b7887ce35d16dbd9d2d295d23aac7
  // also see this issue for acceptable formats: https://github.com/IntelRealSense/librealsense/issues/6341
  rs2::config rs_cfg;
  if (config.enable_color && !config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, config.width,
                         config.height, rs2_format::RS2_FORMAT_ANY,
                         config.framerate);
  } else if (!config.enable_color && config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, config.width,
                         config.height, rs2_format::RS2_FORMAT_Z16,
                         config.framerate);
  } else if (config.enable_color && config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_ANY, config.width,
                         config.height, rs2_format::RS2_FORMAT_ANY,
                         config.framerate);
  } else {
    std::cerr << "[Realsense] Error: no stream enabled." << std::endl;
    return false;
  }
  pipe.start(rs_cfg);

  std::cout << "[Realsense] Pipeline started.\n";
  return true;
}

cv::Mat Realsense::Implementation::next_rgb_frame_blocking() {
  try {
    while (true) {
      frames = pipe.wait_for_frames();
      if (config.align_depth_to_color) {
        frames = align_to_ptr->process(frames);
      }
      color_frame = frames.get_color_frame();
      // If color frame did not update, continue
      static int last_frame_number = 0;
      if (color_frame.get_frame_number() == last_frame_number)
        continue;
      last_frame_number = static_cast<int>(color_frame.get_frame_number());
      break;
    }
    return frame_to_mat(color_frame);
  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return cv::Mat();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return cv::Mat();
  }
}

Realsense::Realsense() : m_impl{std::make_unique<Implementation>()} {}

Realsense::~Realsense() {}

bool Realsense::init(RUT::TimePoint time0,
                     const RealsenseConfig& realsense_config) {
  return m_impl->initialize(time0, realsense_config);
}

cv::Mat Realsense::next_rgb_frame_blocking() {
  return m_impl->next_rgb_frame_blocking();
}