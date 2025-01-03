/**
 * WSGGripper: wrapper around WSG gripper controller
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _WSG_GRIPPER_HEADER_
#define _WSG_GRIPPER_HEADER_

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include "hardware_interfaces/js_interfaces.h"
#include "wsg_gripper/wsg_gripper_driver.h"

class WSGGripper : public JSInterfaces {
 public:
  struct WSGGripperConfig {
    std::string robot_ip{};
    std::string port{};
    float velResControl_kp{0.0};
    float velResControl_kf{0.0};
    float PDControl_kp{0.0};
    float PDControl_kd{0.0};
    JSInterfaceConfig js_interface_config{};

    bool deserialize(const YAML::Node& node) {
      try {
        robot_ip = node["robot_ip"].as<std::string>();
        port = node["port"].as<std::string>();
        velResControl_kp = node["velResControl_kp"].as<float>();
        velResControl_kf = node["velResControl_kf"].as<float>();
        PDControl_kp = node["PDControl_kp"].as<float>();
        PDControl_kd = node["PDControl_kd"].as<float>();

        js_interface_config.deserialize(node["js_interface_config"]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  WSGGripper();
  ~WSGGripper();

  /**
   * Initialize the wsg controller
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const WSGGripperConfig& config);

  // only read the stored feedback
  bool getJoints(RUT::VectorXd& joints) override;
  bool setJoints(const RUT::VectorXd& joints) override;
  // set the target position and force, and update feedback internally
  bool setJointsPosForce(const RUT::VectorXd& joints,
                         const RUT::VectorXd& forces) override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif