/**
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _ARX_CAN_HEADER_
#define _ARX_CAN_HEADER_

#include <arx5-sdk/app/joint_controller.h>

#include <RobotUtilities/TimerLinux.h>
#include <RobotUtilities/utilities.h>

#include "hardware_interfaces/robot_interfaces.h"

class ARXCAN : public RobotInterfaces {
 public:
  struct ARXCANConfig {
    // controller configs
    std::string can_interface = "can0";
    std::string urdf_path = ""; // for gravity compensation
    // send_receive_in_background determines whether the controller runs in synchonized mode.
    // If true, the controller will run in background and the user does not need to call step().
    // If false, the user needs to call update() every dt seconds.
    bool send_receive_in_background = true;
    bool enable_gravity_compensation = true;
    bool reset_to_home_upon_start = true;

    RobotInterfaceConfig robot_interface_config{};
  };

  // for singleton implementation
  static ARXCAN* Instance();

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

  bool getJoints(double* joints) override;
  bool setJoints(const double* joints) override;

  bool getCartesian(double* pose) override;
  bool setCartesian(const double* pose) override; // not implemented yet

  // interfaces unique to ARXCAN (no override)
  bool setGains(const double* kp, const double* kd);
  double get_dt_s();
  bool step();

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;

  /**
   * For singleton implementation
   */
  static ARXCAN* pinstance;
  ARXCAN();
  ARXCAN(const ARXCAN&) {}
  ARXCAN& operator=(const ARXCAN&) { return *this; }
  ~ARXCAN();
};

#endif