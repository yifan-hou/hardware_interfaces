/**
 * URRTDE: wrapper around ur rtde implementation
 * https://sdurobotics.gitlab.io/ur_rtde/index.html
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _UR_SOCKET_HEADER_
#define _UR_SOCKET_HEADER_

#include <RobotUtilities/TimerLinux.h>
#include <RobotUtilities/utilities.h>

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
    RUT::Vector6d safe_zone{};
    RobotInterfaceConfig robot_interface_config{};
  };

  // for singleton implementation
  static URRTDE* Instance();

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

  bool getCartesian(double* pose) override;
  bool setCartesian(const double* pose) override;

  // interfaces unique to URRTDE
  bool streamCartesian(const double* pose);
  RUT::TimePoint rtde_init_period();
  void rtde_wait_period(RUT::TimePoint time_point);

  /**
   * Not implemented yet. Keep empty implementation to not make an abstract
   * class
   */
  bool getJoints(double* joints) override;
  bool setJoints(const double* joints) override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;

  /**
   * For singleton implementation
   */
  static URRTDE* pinstance;
  URRTDE();
  URRTDE(const URRTDE&) {}
  URRTDE& operator=(const URRTDE&) { return *this; }
  ~URRTDE();
};

#endif