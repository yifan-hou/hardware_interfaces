/**
 * OAK: interface for OAK USB cameras
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _OAK_HEADER_
#define _OAK_HEADER_

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <RobotUtilities/timer_linux.h>

#include "hardware_interfaces/camera_interfaces.h"

class OAK : public CameraInterfaces {
 public:
  struct OAKConfig {
    double frame_width{1280};
    double frame_height{720};
    std::vector<int> crop_rows{-1, -1};
    std::vector<int> crop_cols{-1, -1};
    int fps{30};
    int cv_num_threads{0};
    int exposure_time{2500};
    int iso{350};

    bool deserialize(const YAML::Node& node) {
      try {
        frame_width = node["frame_width"].as<double>();
        frame_height = node["frame_height"].as<double>();
        crop_rows = node["crop_rows"].as<std::vector<int>>();
        crop_cols = node["crop_cols"].as<std::vector<int>>();
        fps = node["fps"].as<int>();
        cv_num_threads = node["cv_num_threads"].as<int>();
        exposure_time = node["exposure_time"].as<int>();
        iso = node["iso"].as<int>();
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  OAK();
  ~OAK();

  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const OAKConfig& config);
  cv::Mat next_rgb_frame_blocking() override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif