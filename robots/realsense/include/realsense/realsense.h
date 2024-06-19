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

#include <RobotUtilities/TimerLinux.h>

class Realsense {
 public:
  struct RealsenseConfig {
  };

  // for singleton implementation
  static Realsense* Instance();

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
  rs2::frameset wait_for_frames();

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;

  /**
   * For singleton implementation
   */
  static Realsense* pinstance;
  Realsense();
  Realsense(const Realsense&) {}
  Realsense& operator=(const Realsense&) { return *this; }
  ~Realsense();
};

#endif