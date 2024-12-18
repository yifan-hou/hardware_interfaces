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
#include "wsg_gripper/WSG50Controller.h"

class WSGGripper : public JSInterfaces {
 public:
  struct WSGGripperConfig {
    std::string robot_ip{};
    std::string port{};
    float velResControl_stiffness{0.0};
    float velResControl_damping{0.0};
    float PDControl_kp{0.0};
    float PDControl_kd{0.0};
    JSInterfaceConfig js_interface_config{};

    bool deserialize(const YAML::Node& node) {
      try {
        robot_ip = node["robot_ip"].as<std::string>();
        port = node["port"].as<std::string>();
        velResControl_stiffness = node["velResControl_stiffness"].as<float>();
        velResControl_damping = node["velResControl_damping"].as<float>();
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
  bool checkJointTarget(RUT::Vector7d& pose_xyzq_set);

  bool getJoints(RUT::VectorXd& joints) override;
  bool setJoints(const RUT::VectorXd& joints) override;
  bool setJointsPosForceForce(const RUT::VectorXd& joints,
                              const RUT::VectorXd& forces) override;

 private:
  RUT::TimePoint _time0;
  WSGGripperConfig _config;
  std::shared_ptr<WSG50Controller> _wsg_ptr;

  // internal variables
  Eigen::VectorXd _joints_set_prev;
  Eigen::VectorXd _joints_set_truncated;
  Eigen::VectorXd _joints_set_processed;
};

#endif