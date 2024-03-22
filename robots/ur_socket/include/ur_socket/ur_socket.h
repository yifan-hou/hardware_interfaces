/**
 * URSocket: interface for socket communication with UR e robot. Realtime socket
 * format is according to e5.4. Maintains a 500Hz communication with the robot.
 *
 * Author:
 *      Yifan Hou <yifanh@cmu.edu>
 */

#ifndef _UR_SOCKET_HEADER_
#define _UR_SOCKET_HEADER_

#include <RobotUtilities/utilities.h>
#include <yaml-cpp/yaml.h>

#include <chrono>

#include "hardware_interfaces/robot_interfaces.h"

typedef std::chrono::high_resolution_clock Clock;

class URSocket : public RobotInterfaces {
 public:
  struct URSocketConfig {
    int ur_portnum{};
    std::string ur_ip{};
    double move_para_t{};
    double move_para_lookahead{};
    double move_para_gain{};
    double max_dist_tran{};
    double max_dist_rot{};
    int safety_mode_int{};
    int operation_mode_int{};
    RUT::Vector6d safe_zone{};

    RobotInterfaceConfig robot_interface_config{};
  };

  // for singleton implementation
  static URSocket* Instance();

  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(Clock::time_point time0, const URSocketConfig& config);

  bool getCartesian(double* pose) override;
  bool setCartesian(const double* pose) override;

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
  static URSocket* pinstance;
  URSocket();
  URSocket(const URSocket&) {}
  URSocket& operator=(const URSocket&) { return *this; }
  ~URSocket();
};

#endif