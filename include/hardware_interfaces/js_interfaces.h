/*
    JSInterfaces: virtual class with common interfaces for joint space controllers,
    e.g. an arm, a gripper.
    Provides basic read/write interfaces for joints.

    Provides two safety mechanism:
    1. Incremental limit: the commanded joint position should not be too far
        away from the current joint position.
    2. Range limit: the commanded joint position should not be outside of
        the joint limit ranges.

    Author:
        Yifan Hou <yifanhou@stanford.edu>
*/

#ifndef _JS_INTERFACE_CLASS_HEADER_
#define _JS_INTERFACE_CLASS_HEADER_

#include <RobotUtilities/spatial_utilities.h>
#include <yaml-cpp/yaml.h>

#include "hardware_interfaces/types.h"

/**
 * Check if the given target is within the safety limit, and truncate if not.
 *
 * @param[in]  pos       The commanded position vector.
 * @param[in]  safe_zone  The safety zone. [j1_min,j1_max, j2_min, j2_max, ...]
 * @param[out] safe_pos  The safe pos truncated to be within the safety zone.
 *
 * @return     True if the commanded pos is within the safety zone.
 */
inline bool range_safety_check(const Eigen::VectorXd& pos,
                               const Eigen::VectorXd& safe_zone,
                               Eigen::VectorXd& safe_pos) {
  if (safe_zone.size() != 2 * pos.size()) {
    std::cerr << "[js_interface] Error: we must have dim(safe_zone) = "
                 "2*dim(pos). Getting "
              << safe_zone.size() << std::endl;
    return false;
  }
  bool is_safe = true;
  safe_pos = pos;
  for (int i = 0; i < pos.size(); i++) {
    if (pos[i] < safe_zone[i * 2] || pos[i] > safe_zone[i * 2 + 1]) {
      is_safe = false;
      safe_pos[i] =
          std::max(safe_zone[i * 2], std::min(safe_zone[i * 2 + 1], pos[i]));
    }
  }
  return is_safe;
}

/**
 * Check if the incremental change is within the safety limit.
 *
 * @param[in]  pos         The commanded position.
 * @param[in]  pos_prev    The previous position.
 * @param[in]  max_incre   The maximum allowed incremental distance.
 *
 * @return     True if the incremental change is within the safety limit.
 */
inline bool incre_safety_check(const Eigen::VectorXd& pos,
                               const Eigen::VectorXd& pos_prev,
                               double max_incre) {
  for (int i = 0; i < pos.size(); i++) {
    if (std::abs(pos[i] - pos_prev[i]) > max_incre) {
      return false;
    }
  }
  return true;
}

class JSInterfaces {
 public:
  // ----------------------------------------
  //  user interfaces
  // ----------------------------------------
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
  /**
   * Sets the joint forces.
   *
   * @param      forces  The forces.
   *
   * @return     True if success.
   */
  virtual bool setJointsPosForce(const Eigen::VectorXd& pos,
                                 const Eigen::VectorXd& forces) = 0;
  // ----------------------------------------
  //  public state and parameters
  // ----------------------------------------

  struct JSInterfaceConfig {
    /**
     * Number of joints.
     */
    int num_joints;
    /**
     * Safety mode for safety range.
     */
    RobotSafetyMode range_safety_mode;
    /**
     * Safety mode for incremental limit.
     */
    RobotSafetyMode incre_safety_mode;
    /**
     * Safe increments. Maximum distance between current configuration
     * and the command.
     */
    double max_incre;
    /**
     * Safety zone.
     */
    RUT::VectorXd safe_zone;

    bool deserialize(const YAML::Node& node) {
      try {
        num_joints = node["num_joints"].as<int>();
        range_safety_mode = string_to_enum<RobotSafetyMode>(
            node["range_safety_mode"].as<std::string>());
        incre_safety_mode = string_to_enum<RobotSafetyMode>(
            node["incre_safety_mode"].as<std::string>());
        max_incre = node["max_incre"].as<double>();
        safe_zone = RUT::deserialize_vector<RUT::VectorXd>(node["safe_zone"]);
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
