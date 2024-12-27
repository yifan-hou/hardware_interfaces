/**
 * URRTDE: wrapper around ur rtde implementation
 * https://sdurobotics.gitlab.io/ur_rtde/index.html
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _UR_RTDE_HEADER_
#define _UR_RTDE_HEADER_

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include "hardware_interfaces/robot_interfaces.h"

class URRTDE : public RobotInterfaces {
 public:
  struct URRTDEConfig {
    std::string robot_ip{};
    double rtde_frequency{500};
    // The following three parameters set the thread priority of the receiver,
    // controller, and this interface itself.
    int rt_receive_priority{90};
    int rt_control_priority{85};
    int interface_priority{80};
    // The following two are only used for waypoint move (moveL), not used by
    // streaming (servoL). see
    // https://sdurobotics.gitlab.io/ur_rtde/api/api.html#rtde-control-api
    double linear_vel{0.5};
    double linear_acc{0.5};
    double servoL_lookahead_time{0.1};
    double servoL_gain{600};
    RobotInterfaceConfig robot_interface_config{};

    bool deserialize(const YAML::Node& node) {
      try {
        robot_ip = node["robot_ip"].as<std::string>();
        rtde_frequency = node["rtde_frequency"].as<double>();
        rt_receive_priority = node["rt_receive_priority"].as<int>();
        rt_control_priority = node["rt_control_priority"].as<int>();
        interface_priority = node["interface_priority"].as<int>();
        linear_vel = node["linear_vel"].as<double>();
        linear_acc = node["linear_acc"].as<double>();
        servoL_lookahead_time = node["servoL_lookahead_time"].as<double>();
        servoL_gain = node["servoL_gain"].as<double>();

        robot_interface_config.deserialize(node["robot_interface_config"]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  URRTDE();
  ~URRTDE();
  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const URRTDEConfig& config);

  bool getCartesian(RUT::Vector7d& pose) override;
  bool getCartesianVelocity(RUT::Vector6d& velocity) override;

  bool setCartesian(const RUT::Vector7d& pose) override;

  // interfaces unique to URRTDE

  /**
   * Get wrench measurement from the UR internal FT sensor.
   * The wrench is described in a frame located at TCP, but with
   * axes aligned with the robot base frame.
   *
   * @param[out] wrench 6D wrench data
   *
   * @return true if success
   */
  bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
  /**
   * Get wrench measurement from the UR internal FT sensor.
   * The wrench is described in the tool frame.
   * This function calls getWrenchBaseOnTool() internally and 
   * transforms the result to the tool frame.
   */
  bool getWrenchTool(RUT::Vector6d& wrench);
  bool streamCartesian(const RUT::Vector7d& pose);
  /**
   * rtde_init_period and rtde_wait_period are used together to maintain a timed loop.
   * Example:
   * for (unsigned int i=0; i<1000; i++) {
   *    RUT::TimePoint t_start = ur_rtde->rtde_init_period();
   *    ur_rtde->streamCartesian(pose);
   *    pose[0] += 0.001;
   *    pose[1] += 0.001;
   *    ur_rtde->rtde_wait_period(t_start);
   *  }
   */
  RUT::TimePoint rtde_init_period();
  void rtde_wait_period(RUT::TimePoint time_point);

  /**
   * Not implemented yet. Keep empty implementation to not make an abstract
   * class
   */
  bool getJoints(RUT::VectorXd& joints) override;
  bool setJoints(const RUT::VectorXd& joints) override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif