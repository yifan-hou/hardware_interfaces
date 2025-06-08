#include "gopro/gopro.h"

#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

struct GoPro::Implementation {
  GoPro::GoProConfig config{};
  RUT::TimePoint time0;

  cv::Mat image, image_cropped;
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
    std::cerr << "\033[1;31mNo video stream detected. Check your device name "
                 "config\033[0m\n";
    std::cerr << "Current device name:" << config.device_name << std::endl;
    return false;
  }

  // config the video capture
  cap->set(cv::CAP_PROP_FRAME_WIDTH, config.frame_width);
  cap->set(cv::CAP_PROP_FRAME_HEIGHT, config.frame_height);
  cap->set(cv::CAP_PROP_FPS, config.fps);

  // try reading one frame
  std::cout << "Test reading a frame" << std::endl;
  *cap >> image;
  if (image.empty()) {
    std::cout << "\033[1;31mTest reading failed\033[0m\n";
    std::cout << "  Possibility one: GoPro is not connected. " << std::endl;
    std::cout << "  Possibility two: Need to reset USB device. " << std::endl;
    std::cout << "    To do so, run 'lsusb | grep Elgato', which should give "
                 "something like\n";
    std::cout << "      Bus 010 Device 005: ID 0fd9:008a Elgato Systems GmbH "
                 "Elgato HD60 X\n";
    std::cout << "    Then run 'sudo "
                 "hardware_interfaces/build/hardware/gopro/USBRESET "
                 "/dev/bus/usb/010/005'.\n";
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
  if (config.crop_rows[0] >= 0 && config.crop_cols[0] >= 0) {

    image_cropped = image(cv::Range(config.crop_rows[0], config.crop_rows[1]),
                          cv::Range(config.crop_cols[0], config.crop_cols[1]));
    return image_cropped;
  } else {
    return image;
  }
}

GoPro::GoPro() : m_impl{std::make_unique<Implementation>()} {}

GoPro::~GoPro() {}

bool GoPro::init(RUT::TimePoint time0, const GoProConfig& GoPro_config) {
  return m_impl->initialize(time0, GoPro_config);
}

cv::Mat GoPro::next_rgb_frame_blocking() {
  return m_impl->next_rgb_frame_blocking();
}