#include "gopro/gopro.h"

#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

struct GoPro::Implementation {
  GoPro::GoProConfig config{};
  RUT::TimePoint time0;

  cv::Mat image;
  std::shared_ptr<cv::VideoCapture> cap;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0, const GoPro::GoProConfig& config);
  cv::Mat next_rgb_frame_blocking();
};

GoPro::Implementation::Implementation() {}

GoPro::Implementation::~Implementation() {
  std::cout << "[GoPro] finishing.." << std::endl;
}

bool GoPro::Implementation::initialize(RUT::TimePoint time0,
                                       const GoPro::GoProConfig& GoPro_config) {
  std::cout << "[GoPro] Initializing GoPro pipeline.." << std::endl;
  time0 = time0;
  config = GoPro_config;

  char* device_name_char = new char[config.device_name.length() + 1];
  strcpy(device_name_char, config.device_name.c_str());
  cap = std::make_shared<cv::VideoCapture>(device_name_char);
  if (!cap->isOpened()) {  //This section prompt an error message if no video stream is found//
    std::cout << "No video stream detected. Check your device name config. "
                 "Current device name:"
              << config.device_name << std::endl;
    return false;
  }
  // config the video capture
  cap->set(cv::CAP_PROP_FRAME_WIDTH, config.frame_width);
  cap->set(cv::CAP_PROP_FRAME_HEIGHT, config.frame_height);
  cap->set(cv::CAP_PROP_FPS, config.fps);

  // try reading one frame
  std::cout << "Test reading a frame" << std::endl;
  *cap >> image;
  if (image.empty()) {  //Breaking the loop if no video frame is detected//
    std::cout << "Test reading failed. Please reset USB device." << std::endl;
    return false;
  }

  std::cout << "[GoPro] Pipeline started.\n";
  return true;
}

cv::Mat GoPro::Implementation::next_rgb_frame_blocking() {
  *cap >> image;
  if (image.empty()) {  //Breaking the loop if no video frame is detected//
    std::cerr << "[GoPro] Empty frame. Terminate" << std::endl;
    return cv::Mat();
  }
  return image;
}

GoPro::GoPro() : m_impl{std::make_unique<Implementation>()} {}

GoPro::~GoPro() {}

bool GoPro::init(RUT::TimePoint time0, const GoProConfig& GoPro_config) {
  return m_impl->initialize(time0, GoPro_config);
}

cv::Mat GoPro::next_rgb_frame_blocking() {
  return m_impl->next_rgb_frame_blocking();
}