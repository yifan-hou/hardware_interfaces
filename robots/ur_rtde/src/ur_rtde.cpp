#include "ur_rtde/ur_rtde.h"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <Eigen/Dense>
#include <memory>

#include <RobotUtilities/utilities.h>

URRTDE* URRTDE::pinstance = 0;

struct URRTDE::Implementation {
  std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control_ptr;
  std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_ptr;

  URRTDE::URRTDEConfig config{};

  std::vector<double>
      tcp_pose_feedback{};  // std vector to be compatible with ur_rtde lib
  std::vector<double> tcp_pose_command{};
  RUT::Vector7d pose_xyzq_set_prev{};

  // pre-allocated variables that are always assigned before use
  RUT::Vector3d v_axis_receive{};
  RUT::Vector3d v_axis_control{};
  RUT::Vector7d pose_xyzq_set_truncated{};
  RUT::Vector7d pose_xyzq_set_processed{};

  double dt_s{};

  RUT::TimePoint time0;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0, const URRTDE::URRTDEConfig& config);
  bool getCartesian(RUT::Vector7d& pose_xyzq);
  bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
  bool getWrenchTool(RUT::Vector6d& wrench);
  bool checkCartesianTarget(RUT::Vector7d& pose_xyzq_set);
  bool setCartesian(const RUT::Vector7d& pose_xyzq);
  bool streamCartesian(const RUT::Vector7d& pose_xyzq);
  RUT::TimePoint rtde_init_period();
  void rtde_wait_period(RUT::TimePoint time_point);
};

URRTDE::Implementation::Implementation() {
  tcp_pose_feedback.resize(6);
  tcp_pose_command.resize(6);
}

URRTDE::Implementation::~Implementation() {
  std::cout << "[URRTDE] finishing.." << std::endl;
  delete pinstance;
}

bool URRTDE::Implementation::initialize(
    RUT::TimePoint time0, const URRTDE::URRTDEConfig& ur_rtde_config) {
  time0 = time0;
  config = ur_rtde_config;
  dt_s = 1. / config.rtde_frequency;

  /* Establish connection with UR */
  std::cout << "[URRTDE] Connecting to robot at " << config.robot_ip
            << std::endl;
  std::cout << "[URRTDE] Creating control interface at: "
            << config.rtde_frequency << " Hz\n";
  while (true) {
    try {
      rtde_control_ptr = std::shared_ptr<ur_rtde::RTDEControlInterface>(
          new ur_rtde::RTDEControlInterface(config.robot_ip));
      break;
    } catch (const std::exception& e) {
      std::cerr << "[URRTDE] Failed to create control interface. Retrying.."
                << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // rtde_control_ptr = std::shared_ptr<ur_rtde::RTDEControlInterface>(
  //     new ur_rtde::RTDEControlInterface(config.robot_ip, config.rtde_frequency,
  //                                       {}, {}, config.rt_control_priority));
  std::cout << "[URRTDE] Creating receive interface at: "
            << config.rtde_frequency << " Hz\n";
  rtde_receive_ptr = std::shared_ptr<ur_rtde::RTDEReceiveInterface>(
      new ur_rtde::RTDEReceiveInterface(config.robot_ip));
  // rtde_receive_ptr = std::shared_ptr<ur_rtde::RTDEReceiveInterface>(
  //     new ur_rtde::RTDEReceiveInterface(config.robot_ip, config.rtde_frequency,
  //                                       {}, {}, {},
  //                                       config.rt_receive_priority));
  std::cout << "[URRTDE] RTDE interfaces created. Setting realtime priority:\n";
  // Set application realtime priority
  ur_rtde::RTDEUtility::setRealtimePriority(config.interface_priority);
  std::cout << "[URRTDE] UR socket connection established.\n";

  // read current state
  assert(getCartesian(pose_xyzq_set_prev));
  tcp_pose_command = tcp_pose_feedback;

  return true;
}

bool URRTDE::Implementation::getCartesian(RUT::Vector7d& pose_xyzq) {
  tcp_pose_feedback = rtde_receive_ptr->getActualTCPPose();  // std vector

  pose_xyzq[0] = tcp_pose_feedback[0];
  pose_xyzq[1] = tcp_pose_feedback[1];
  pose_xyzq[2] = tcp_pose_feedback[2];

  // convert from Euler to quaternion
  v_axis_receive[0] = tcp_pose_feedback[3];  // rx
  v_axis_receive[1] = tcp_pose_feedback[4];  // ry
  v_axis_receive[2] = tcp_pose_feedback[5];  // rz
  double angle = v_axis_receive.norm();

  pose_xyzq.tail<4>() = RUT::aa2quat(angle, v_axis_receive);

  return true;
}

bool URRTDE::Implementation::getWrenchBaseOnTool(RUT::Vector6d& wrench) {
  std::vector<double> wrench_bot_vec = rtde_receive_ptr->getActualTCPForce();
  wrench = RUT::Vector6d::Map(wrench_bot_vec.data(), 6);
  return true;
}

bool URRTDE::Implementation::getWrenchTool(RUT::Vector6d& wrench_T) {
  std::vector<double> wrench_bot_vec = rtde_receive_ptr->getActualTCPForce();
  RUT::Vector6d wrench_bot = RUT::Vector6d::Map(wrench_bot_vec.data(), 6);

  // get the current pose
  RUT::Vector7d pose_WT;
  getCartesian(pose_WT);
  RUT::Matrix4d SE3_botT = RUT::Matrix4d::Identity();
  SE3_botT.block<3, 3>(0, 0) =
      RUT::quat2SO3(pose_WT.tail(4));  // bot orientation = world orientation
  RUT::Matrix6d Adj_botT = RUT::SE32Adj(SE3_botT);

  wrench_T = Adj_botT.transpose() * wrench_bot;

  return true;
}

bool URRTDE::Implementation::checkCartesianTarget(
    RUT::Vector7d& pose_xyzq_set) {
  if (config.robot_interface_config.incre_safety_mode !=
      RobotSafetyMode::SAFETY_MODE_NONE) {
    bool incre_safe =
        incre_safety_check(pose_xyzq_set, pose_xyzq_set_prev,
                           config.robot_interface_config.max_incre_m,
                           config.robot_interface_config.max_incre_rad);
    if (!incre_safe) {
      std::cerr
          << "[URRTDE][checkCartesianTarget] Incremental safety check failed. "
          << "set pose: " << pose_xyzq_set.transpose()
          << ", prev pose: " << pose_xyzq_set_prev.transpose()
          << ", max_incre_m: " << config.robot_interface_config.max_incre_m
          << ", max_incre_rad: " << config.robot_interface_config.max_incre_rad
          << std::endl;
      if (config.robot_interface_config.incre_safety_mode ==
          RobotSafetyMode::SAFETY_MODE_STOP) {
        std::cerr << "[URRTDE][checkCartesianTarget] Returning false."
                  << std::endl;
        return false;
      } else if (config.robot_interface_config.incre_safety_mode ==
                 RobotSafetyMode::SAFETY_MODE_TRUNCATE) {
        std::cerr
            << "[URRTDE][checkCartesianTarget] Truncating is not implemented. "
            << std::endl;
        return false;
      }
    }
  }

  bool zone_safe =
      zone_safety_check(pose_xyzq_set, config.robot_interface_config.safe_zone,
                        pose_xyzq_set_truncated);
  if (!zone_safe) {
    if (config.robot_interface_config.zone_safety_mode ==
        RobotSafetyMode::SAFETY_MODE_STOP) {
      std::cerr << "[URRTDE][checkCartesianTarget] Zone safety check failed."
                << std::endl;
      std::cerr << "[URRTDE][checkCartesianTarget] target pose: "
                << pose_xyzq_set.transpose() << std::endl;
      std::cerr << "[URRTDE][checkCartesianTarget] safe zone: "
                << config.robot_interface_config.safe_zone.transpose()
                << std::endl;
      return false;
    } else if (config.robot_interface_config.zone_safety_mode ==
               RobotSafetyMode::SAFETY_MODE_TRUNCATE) {
      std::cerr << "[URRTDE][checkCartesianTarget] Zone safety check failed. "
                   "Using truncated pose."
                << std::endl;
      pose_xyzq_set = pose_xyzq_set_truncated;
    }
  }
  return true;
}

bool URRTDE::Implementation::setCartesian(const RUT::Vector7d& pose_xyzq_set) {
  assert(config.robot_interface_config.operation_mode ==
         RobotOperationMode::OPERATION_MODE_CARTESIAN);

  // safety checks
  pose_xyzq_set_processed = pose_xyzq_set;
  if (!checkCartesianTarget(pose_xyzq_set_processed)) {
    return false;
  }
  pose_xyzq_set_prev = pose_xyzq_set_processed;

  // convert quaternion to Euler
  RUT::quat2aa(pose_xyzq_set_processed.tail(4), v_axis_control);
  tcp_pose_command[0] = pose_xyzq_set_processed[0];
  tcp_pose_command[1] = pose_xyzq_set_processed[1];
  tcp_pose_command[2] = pose_xyzq_set_processed[2];
  tcp_pose_command[3] = v_axis_control[0];
  tcp_pose_command[4] = v_axis_control[1];
  tcp_pose_command[5] = v_axis_control[2];

  return rtde_control_ptr->moveL(tcp_pose_command, config.linear_vel,
                                 config.linear_acc);
}

bool URRTDE::Implementation::streamCartesian(
    const RUT::Vector7d& pose_xyzq_set) {
  assert(config.robot_interface_config.operation_mode ==
         RobotOperationMode::OPERATION_MODE_CARTESIAN);
  // safety checks
  pose_xyzq_set_processed = pose_xyzq_set;
  if (!checkCartesianTarget(pose_xyzq_set_processed)) {
    return false;
  }
  pose_xyzq_set_prev = pose_xyzq_set_processed;

  // convert quaternion to Euler
  RUT::quat2aa(pose_xyzq_set.tail(4), v_axis_control);
  tcp_pose_command[0] = pose_xyzq_set[0];
  tcp_pose_command[1] = pose_xyzq_set[1];
  tcp_pose_command[2] = pose_xyzq_set[2];
  tcp_pose_command[3] = v_axis_control[0];
  tcp_pose_command[4] = v_axis_control[1];
  tcp_pose_command[5] = v_axis_control[2];
  return rtde_control_ptr->servoL(
      tcp_pose_command, config.linear_vel, config.linear_acc, dt_s,
      config.servoL_lookahead_time, config.servoL_gain);
}

RUT::TimePoint URRTDE::Implementation::rtde_init_period() {
  return rtde_control_ptr->initPeriod();
}

void URRTDE::Implementation::rtde_wait_period(RUT::TimePoint time_point) {
  rtde_control_ptr->waitPeriod(time_point);
}

URRTDE::URRTDE() : m_impl{std::make_unique<Implementation>()} {}

URRTDE::~URRTDE() {}

URRTDE* URRTDE::Instance() {
  if (pinstance == 0) {
    pinstance = new URRTDE();
  }
  return pinstance;
}

bool URRTDE::init(RUT::TimePoint time0, const URRTDEConfig& ur_rtde_config) {
  return m_impl->initialize(time0, ur_rtde_config);
}

bool URRTDE::getCartesian(RUT::Vector7d& pose_xyzq) {
  return m_impl->getCartesian(pose_xyzq);
}

bool URRTDE::getWrenchBaseOnTool(RUT::Vector6d& wrench) {
  return m_impl->getWrenchBaseOnTool(wrench);
}

bool URRTDE::getWrenchTool(RUT::Vector6d& wrench) {
  return m_impl->getWrenchTool(wrench);
}

bool URRTDE::setCartesian(const RUT::Vector7d& pose_xyzq) {
  return m_impl->setCartesian(pose_xyzq);
}

bool URRTDE::streamCartesian(const RUT::Vector7d& pose_xyzq) {
  return m_impl->streamCartesian(pose_xyzq);
}

RUT::TimePoint URRTDE::rtde_init_period() {
  return m_impl->rtde_init_period();
}

void URRTDE::rtde_wait_period(RUT::TimePoint time_point) {
  m_impl->rtde_wait_period(time_point);
}

bool URRTDE::getJoints(RUT::VectorXd& joints) {
  std::cerr << "[URRTDE] not implemented yet" << std::endl;
  return false;
}

bool URRTDE::setJoints(const RUT::VectorXd& joints) {
  std::cerr << "[URRTDE] not implemented yet" << std::endl;
  return false;
}