/**
 * GoPro: video capture interface for GoPro with a video capture card.
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _GOPRO_HEADER_
#define _GOPRO_HEADER_

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <RobotUtilities/TimerLinux.h>

#include "hardware_interfaces/camera_interfaces.h"

class GoPro : public CameraInterfaces {
 public:
  struct GoProConfig {
    std::string device_name;
    double frame_width{1280};
    double frame_height{720};
    int fps{30};

    bool deserialize(const YAML::Node& node) {
      try {
        device_name = node["device_name"].as<std::string>();
        frame_width = node["frame_width"].as<double>();
        frame_height = node["frame_height"].as<double>();
        fps = node["fps"].as<int>();
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  GoPro();
  ~GoPro();

  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const GoProConfig& config);
  cv::Mat next_rgb_frame_blocking() override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif