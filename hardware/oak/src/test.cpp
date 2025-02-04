#include <RobotUtilities/timer_linux.h>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
  using namespace std;

  cv::setNumThreads(5);
  // Create pipeline
  dai::Pipeline pipeline;

  // Define source and output
  auto camRgb = pipeline.create<dai::node::ColorCamera>();
  auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
  auto controlIn = pipeline.create<dai::node::XLinkIn>();

  xoutRgb->setStreamName("rgb");
  controlIn->setStreamName("control");

  // Properties
  camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  camRgb->setResolution(
      dai::ColorCameraProperties::SensorResolution::THE_800_P);
  camRgb->setInterleaved(false);
  camRgb->setPreviewSize(1280, 800);
  camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
  camRgb->setFps(60);
  // Linking
  camRgb->preview.link(xoutRgb->input);
  controlIn->out.link(camRgb->inputControl);

  // Connect to device and start pipeline
  dai::Device device(pipeline, dai::UsbSpeed::SUPER);

  cout << "Connected cameras: " << device.getConnectedCameraFeatures() << endl;

  // Print USB speed
  cout << "Usb speed: " << device.getUsbSpeed() << endl;

  // Bootloader version
  if (device.getBootloaderVersion()) {
    cout << "Bootloader version: " << device.getBootloaderVersion()->toString()
         << endl;
  }

  // Device name
  cout << "Device name: " << device.getDeviceName()
       << " Product name: " << device.getProductName() << endl;

  // Output queue will be used to get the rgb frames from the output defined above
  auto qRgb = device.getOutputQueue("rgb", 4, false);

  // fixed exposure
  //   // Defaults and limits for manual focus/exposure controls
  // int lensPos = 150;
  // int lensMin = 0;
  // int lensMax = 255;

  int expTime = 2500;
  // int expMin = 1;
  // int expMax = 33000;

  int sensIso = 350;
  // int sensMin = 100;
  // int sensMax = 1600;
  auto controlQueue = device.getInputQueue("control");
  dai::CameraControl ctrl;
  // ctrl.setManualFocus(lensPos);
  ctrl.setManualExposure(expTime, sensIso);
  controlQueue->send(ctrl);

  RUT::Timer timer;
  timer.tic();
  int count = 0;
  while (true) {
    auto inRgb = qRgb->get<dai::ImgFrame>();

    count++;
    std::cout << "FPS: " << 1000 * count / timer.toc_ms() << std::endl;

    // Retrieve 'bgr' (opencv format) frame
    auto frame = inRgb->getCvFrame();
    cv::imshow("rgb", frame);

    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q') {
      break;
    }
  }
  return 0;
}

// int main() {
//   // Create pipeline
//   dai::Pipeline pipeline;

//   // Define source and output
//   auto camRgb = pipeline.create<dai::node::ColorCamera>();
//   // Properties
//   camRgb->setInterleaved(false);
//   camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
//   camRgb->setResolution(
//       dai::ColorCameraProperties::SensorResolution::THE_720_P);
//   // camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
//   camRgb->setVideoSize(224, 224);
//   camRgb->setFps(30);

//   auto xoutISP = pipeline.create<dai::node::XLinkOut>();
//   xoutISP->setStreamName("isp");

//   // xoutISP->input.setBlocking(false);
//   // xoutISP->input.setQueueSize(1);

//   // Linking
//   camRgb->isp.link(xoutISP->input);

//   // Connect to device and start pipeline
//   dai::Device device(pipeline);

//   auto video = device.getOutputQueue("isp", 1, false);

//   RUT::Timer timer;
//   timer.tic();
//   int count = 0;
//   while (true) {
//     auto videoIn = video->get<dai::ImgFrame>();

//     // Get BGR frame from NV12 encoded video frame to show with opencv
//     // Visualizing the frame on slower hosts might have overhead
//     auto frame = videoIn->getCvFrame();
//     cv::imshow("video", frame);

//     count++;
//     std::cout << "FPS: " << 1000 * count / timer.toc_ms() << std::endl;

//     int key = cv::waitKey(1);
//     if (key == 'q' || key == 'Q') {
//       return 0;
//     }
//   }
//   return 0;
// }
