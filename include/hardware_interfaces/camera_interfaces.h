/*
  CameraInterfaces: virtual class with common interfaces for a camera.

  Author:
      Yifan Hou <yifanhou@stanford.edu>
*/

#ifndef _CAMERA_INTERFACE_CLASS_HEADER_
#define _CAMERA_INTERFACE_CLASS_HEADER_

#include <RobotUtilities/spatial_utilities.h>
#include <opencv2/opencv.hpp>

class CameraInterfaces {
 public:
  /**
   * A blocking call to get the next rgb image..
   *
   * @return  the image.
   */
  virtual cv::Mat next_rgb_frame_blocking() = 0;
};

#endif
