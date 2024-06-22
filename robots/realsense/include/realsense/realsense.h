/**
 * Realsense: wrapper around librealsense
 * https://github.com/IntelRealSense/librealsense
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _REALSENSE_HEADER_
#define _REALSENSE_HEADER_

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <RobotUtilities/TimerLinux.h>

class Realsense {
 public:
  struct RealsenseConfig {
    // Not all sizes and frame rates are supported by the camera.
    // Use realsense viewer to check what is supported.
    int width{1920};
    int height{1080};
    int framerate{30};
    bool enable_color{true};
    bool enable_depth{true};
    bool align_depth_to_color{false};
  };

  Realsense();
  ~Realsense();

  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const RealsenseConfig& config);
  cv::Mat next_rgb_frame_blocking();

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif