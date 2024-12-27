/*
    RobotInterfaces: virtual class with common interfaces with any robot arm.
    Provides basic read/write interfaces for robot poses and joints.

    Provides two safety mechanism:
    1. Incremental limit: the commanded configuration should not be too far
        away from the current configuration.
    2. Safety zone limit: the commanded configuration should not be outside of
        the safety zone.

    Author:
        Yifan Hou <yifanhou@stanford.edu>
*/

#ifndef _ROBOT_INTERFACE_CLASS_HEADER_
#define _ROBOT_INTERFACE_CLASS_HEADER_

#include <RobotUtilities/spatial_utilities.h>
#include <yaml-cpp/yaml.h>

#include "hardware_interfaces/types.h"

/**
 * Check if pose is within the safety zone, and truncate if not.
 *
 * @param[in]  pose       The commanded pose.
 * @param[in]  safe_zone  The safety zone. [xmin,xmax,ymin,ymax,zmin,zmax]
 * @param[out] safe_pose  The safe pose truncated to be within the safety zone.
 *
 * @return     True if the commanded pose is within the safety zone.
 */
inline bool zone_safety_check(const Eigen::VectorXd& pose,
                              const Eigen::VectorXd& safe_zone,
                              RUT::Vector7d& safe_pose) {
  if (safe_zone.size() != 6) {
    std::cerr
        << "[robot_interface] Error: safe_zone must have dim = 6. Getting "
        << safe_zone.size() << std::endl;
    return false;
  }
  bool is_safe = true;
  safe_pose = pose;
  for (int i = 0; i < 3; i++) {
    if (pose[i] < safe_zone[i * 2] || pose[i] > safe_zone[i * 2 + 1]) {
      is_safe = false;
      safe_pose[i] =
          std::max(safe_zone[i * 2], std::min(safe_zone[i * 2 + 1], pose[i]));
    }
  }
  return is_safe;
}

/**
 * Check if the incremental change is within the safety limit.
 *
 * @param[in]  pose         The commanded pose.
 * @param[in]  pose_prev    The previous pose.
 * @param[in]  max_incre_m  The maximum incremental distance in meters.
 * @param[in]  max_incre_rad  The maximum incremental angle in radians.
 *
 * @return     True if the incremental change is within the safety limit.
 */
inline bool incre_safety_check(const Eigen::VectorXd& pose,
                               const Eigen::VectorXd& pose_prev,
                               double max_incre_m, double max_incre_rad) {
  for (int i = 0; i < 3; i++) {
    if (std::abs(pose[i] - pose_prev[i]) > max_incre_m) {
      return false;
    }
  }
  if (RUT::angBTquat(pose.tail(4), pose_prev.tail(4)) > max_incre_rad) {
    return false;
  }
  return true;
}

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
  virtual bool getCartesian(RUT::Vector7d& pose) = 0;
  /**
   * Sets the Cartesian pose of the robot tool. Distances are in mm.
   *
   * @param[in]  pose  The Cartesian pose. [x y z qw qx qy qz].
   *
   * @return     True if success.
   */
  virtual bool setCartesian(const RUT::Vector7d& pose) = 0;
  /**
   * Gets the joint angles in rad.
   *
   * @param      joints  The joints.
   *
   * @return     True if success.
   */
  virtual bool getJoints(Eigen::VectorXd& joints) = 0;
  /**
   * Sets the joint angles in rad.
   *
   * @param[in]  joints  The joints.
   *
   * @return     True if success.
   */
  virtual bool setJoints(const Eigen::VectorXd& joints) = 0;

  // ----------------------------------------
  //  Optional interfaces
  // ----------------------------------------
  virtual bool getCartesianVelocity(RUT::Vector6d& velocity) {
    std::cerr << "[RobotInterfaces] getCartesianVelocity not implemented yet"
              << std::endl;
    return false;
  }

  // ----------------------------------------
  //  public state and parameters
  // ----------------------------------------

  struct RobotInterfaceConfig {
    /**
     * Joint mode or Cartesian mode.
     */
    RobotOperationMode operation_mode;
    /**
     * Safety mode for safety zone.
     */
    RobotSafetyMode zone_safety_mode;
    /**
     * Safety mode for incremental limit.
     */
    RobotSafetyMode incre_safety_mode;
    /**
     * Safe increments. Maximum distance between current configuration
     * and the command. If the robot has multiple control interfaces,
     * incremental safety is usually only applied to the highest frequency interface.
     */
    double max_incre_m;
    double max_incre_rad;
    double max_incre_joint_rad;
    /**
     * Safety zone.
     * [xmin,xmax,ymin,ymax,zmin,zmax]
     */
    RUT::Vector6d safe_zone;

    bool deserialize(const YAML::Node& node) {
      try {
        operation_mode = string_to_enum<RobotOperationMode>(
            node["operation_mode"].as<std::string>());
        zone_safety_mode = string_to_enum<RobotSafetyMode>(
            node["zone_safety_mode"].as<std::string>());
        incre_safety_mode = string_to_enum<RobotSafetyMode>(
            node["incre_safety_mode"].as<std::string>());
        max_incre_m = node["max_incre_m"].as<double>();
        max_incre_rad = node["max_incre_rad"].as<double>();
        max_incre_joint_rad = node["max_incre_joint_rad"].as<double>();
        safe_zone = RUT::deserialize_vector<RUT::Vector6d>(node["safe_zone"]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };
};

#endif
