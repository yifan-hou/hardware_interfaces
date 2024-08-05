/**
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _ARX_CAN_HEADER_
#define _ARX_CAN_HEADER_

#include <arx5-sdk/app/joint_controller.h>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include "hardware_interfaces/robot_interfaces.h"

class ARXCAN : public RobotInterfaces {
 public:
  struct ARXCANConfig {
    // controller configs
    std::string can_interface = "can0";
    std::string urdf_path = "";  // for gravity compensation
    // send_receive_in_background determines whether the controller runs in synchonized mode.
    // If true, the controller will run in background and the user does not need to call step().
    // If false, the user needs to call update() every dt seconds.
    bool send_receive_in_background = true;
    bool enable_gravity_compensation = true;
    bool reset_to_home_upon_start = true;

    RobotInterfaceConfig robot_interface_config{};

    bool deserialize(const YAML::Node& node) {
      try {
        can_interface = node["can_interface"].as<std::string>();
        urdf_path = node["urdf_path"].as<std::string>();
        send_receive_in_background =
            node["send_receive_in_background"].as<bool>();
        enable_gravity_compensation =
            node["enable_gravity_compensation"].as<bool>();
        reset_to_home_upon_start = node["reset_to_home_upon_start"].as<bool>();

        robot_interface_config.deserialize(node["robot_interface_config"]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  ARXCAN();
  ~ARXCAN();

  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const ARXCANConfig& config);

  bool getJoints(RUT::VectorXd& joints) override;
  bool setJoints(const RUT::VectorXd& joints) override;

  bool getCartesian(RUT::Vector7d& pose) override;
  bool setCartesian(const RUT::Vector7d& pose) override;  // not implemented yet

  // interfaces unique to ARXCAN (no override)
  bool setGains(const RUT::VectorXd& kp, const RUT::VectorXd& kd);
  void getGains(RUT::VectorXd& kp, RUT::VectorXd& kd);
  double get_dt_s();
  bool step();

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif