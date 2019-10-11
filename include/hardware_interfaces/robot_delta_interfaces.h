#ifndef _ROBOT_DELTA_INTERFACES_
#define _ROBOT_DELTA_INTERFACES_

#include "hardware_interfaces/delta_interfaces.h"
#include "hardware_interfaces/robot_interfaces.h"

class RobotDeltaInterfaces {
public:
  RobotDeltaInterfaces();
  ~RobotDeltaInterfaces();
  /**
   * Gets the Cartesian pose of the delta platform in the world frame. The pose
   * is determined by both the robot arm and the delta legs.
   *
   * @param      pose  The Cartesian pose. [x y z qw qx qy qz]. Translation is
   *                   in mm. Rotation is represented by unit quaternion.
   *
   * @return     True if success.
   */
  virtual bool getCartesian(double *pose) = 0;
  /**
   * Sets the Cartesian pose of the delta platform. Distances are in mm.
   *
   * @param[in]  pose  The Cartesian pose. [x y z qw qx qy qz].
   *
   * @return     True if success.
   */
  virtual bool setCartesian(const double *pose) = 0;

  RobotInterfaces *robot;
  DeltaInterfaces *delta;
};

#endif
