#include "oak/oak.h"

#include <depthai/depthai.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

struct OAK::Implementation {
  OAK::OAKConfig config{};
  RUT::TimePoint time0;

  dai::Pipeline pipeline;
  std::shared_ptr<dai::node::ColorCamera> camRgb;
  std::shared_ptr<dai::node::XLinkOut> xoutVideo;
  std::shared_ptr<dai::node::XLinkIn> controlIn;
  std::shared_ptr<dai::DataInputQueue> controlQueue;

  std::shared_ptr<dai::Device> device;
  std::shared_ptr<dai::DataOutputQueue> video;

  cv::Mat image, image_cropped;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0, const OAK::OAKConfig& config);
  cv::Mat next_rgb_frame_blocking();
};

OAK::Implementation::Implementation() {}

OAK::Implementation::~Implementation() {
  std::cout << "[OAK] finishing.." << std::endl;
}

bool OAK::Implementation::initialize(RUT::TimePoint time0,
                                     const OAK::OAKConfig& OAK_config) {
  std::cout << "[OAK] Initializing OAK pipeline.." << std::endl;
  time0 = time0;
  config = OAK_config;

  if (config.cv_num_threads > 0) {
    std::cout << "[OAK] Setting OpenCV threads to " << config.cv_num_threads
              << std::endl;
    cv::setNumThreads(config.cv_num_threads);
  }

  camRgb = pipeline.create<dai::node::ColorCamera>();
  xoutVideo = pipeline.create<dai::node::XLinkOut>();
  controlIn = pipeline.create<dai::node::XLinkIn>();

  xoutVideo->setStreamName("video");
  controlIn->setStreamName("control");

  // Properties
  camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  camRgb->setResolution(
      dai::ColorCameraProperties::SensorResolution::THE_800_P);
  camRgb->setVideoSize(config.frame_width, config.frame_height);
  camRgb->setFps(config.fps);

  xoutVideo->input.setBlocking(false);
  xoutVideo->input.setQueueSize(1);

  // Linking
  camRgb->video.link(xoutVideo->input);
  controlIn->out.link(camRgb->inputControl);

  // Connect to device and start pipeline
  device = std::make_shared<dai::Device>(pipeline);

  video = device->getOutputQueue("video");
  controlQueue = device->getInputQueue("control");
  dai::CameraControl ctrl;
  // ctrl.setManualFocus(lensPos);
  ctrl.setManualExposure(config.exposure_time, config.iso);
  controlQueue->send(ctrl);

  std::cout << "[OAK] Pipeline started.\n";
  return true;
}

cv::Mat OAK::Implementation::next_rgb_frame_blocking() {
  auto videoIn = video->get<dai::ImgFrame>();
  image = videoIn->getCvFrame();

  if (image.empty()) {  //Breaking the loop if no video frame is detected//
    std::cerr << "[OAK] Empty frame. Terminate" << std::endl;
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

OAK::OAK() : m_impl{std::make_unique<Implementation>()} {}

OAK::~OAK() {}

bool OAK::init(RUT::TimePoint time0, const OAKConfig& OAK_config) {
  return m_impl->initialize(time0, OAK_config);
}

cv::Mat OAK::next_rgb_frame_blocking() {
  return m_impl->next_rgb_frame_blocking();
}