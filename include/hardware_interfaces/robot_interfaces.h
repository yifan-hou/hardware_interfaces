/*
    RobotInterfaces: virtual class with common interfaces with any robot arm.
    Provides basic read/write interfaces for robot poses and joints.

    Provides two safety mechanism:
    1. Incremental limit: the commanded configuration should not be too far
        away from the current configuration.
    2. Safety zone limit: the commanded configuration should not be outside of
        the safety zone.

    Author:
        Yifan Hou <yifanh@cmu.edu>
*/

#ifndef _ROBOT_INTERFACE_CLASS_HEADER_
#define _ROBOT_INTERFACE_CLASS_HEADER_

/**
 * Safety mode. Determines what to do when the commanded pose is too far away
 * from the current pose.
 *  SAFETY_MODE_NONE: Do not perform safety checking.
 *  SAFETY_MODE_TRUNCATE: Truncate the commanded pose based on _max_dist_tran and _max_dist_rot
 *  SAFETY_MODE_STOP: Throw an error and stop.
 */
typedef enum
{
  SAFETY_MODE_NONE,
  SAFETY_MODE_TRUNCATE,
  SAFETY_MODE_STOP
} RobotSafetyMode;

typedef enum
{
  OPERATION_MODE_CARTESIAN,
  OPERATION_MODE_JOINT
} RobotOperationMode;

class RobotInterfaces {
public:
  // ----------------------------------------
  //  user interfaces
  // ----------------------------------------

  /**
   * Gets the Cartesian pose of the robot tool. Distances are in mm.
   *
   * @param      pose  The Cartesian pose. [x y z qw qx qy qz]
   *
   * @return     True if success.
   */
  virtual bool getCartesian(double *pose) = 0;
  /**
   * Sets the Cartesian pose of the robot tool. Distances are in mm.
   *
   * @param[in]  pose  The Cartesian pose. [x y z qw qx qy qz].
   *
   * @return     True if success.
   */
  virtual bool setCartesian(const double *pose) = 0;
  /**
   * Gets the joint angles in rad.
   *
   * @param      joints  The joints.
   *
   * @return     True if success.
   */
  virtual bool getJoints(double *joints) = 0;
  /**
   * Sets the joint angles in rad.
   *
   * @param[in]  joints  The joints.
   *
   * @return     True if success.
   */
  virtual bool setJoints(const double *joints) = 0;

  // ----------------------------------------
  //  public state and parameters
  // ----------------------------------------

  struct RobotInterfaceConfig {

    /**
     * Current safety mode. see RobotSafetyMode
     */
    RobotSafetyMode safetyMode;
    /**
     * Joint mode or Cartesian mode.
     */
    RobotOperationMode operationMode;
    /**
     * Safety increments. Maximum distance between current configuration
     * and the goal.
     */
    double max_dist_tran;
    double max_dist_rot;
    double max_dist_joint;
    /**
     * Safety zone. Robot will stop if the commanded pose is out of the zone.
     * [xmin,xmax,ymin,ymax,zmin,zmax]
     */
    double *safe_zone;
  };

  RobotInterfaceConfig robot_interface_config;
};

#endif
