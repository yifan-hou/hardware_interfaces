#include "wsg_gripper/wsg_gripper.h"

#include <Eigen/Dense>
#include <memory>

#include <RobotUtilities/spatial_utilities.h>

bool WSGGripper::initialize(
    RUT::TimePoint time0,
    const WSGGripper::WSGGripperConfig& wsg_gripper_config) {
  _time0 = time0;
  _config = wsg_gripper_config;

  /* Establish connection with WSG gripper */
  std::cout << "[WSGGripper] Connecting to gripper at " << config.robot_ip
            << ", port " << config.port << std::endl;
  _wsg_ptr = std::make_shared<WSG50Controller>(_config.robot_ip, _config.port);

  while (!_wsg_ptr->ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "[WSGGripper] Connection established." << std::endl;

  // read current state
  assert(getJoints(_joints_set_prev));

  return true;
}

bool WSGGripper::checkJointTarget(RUT::VectorXd& joints_set) {
  if (config.js_interface_config.incre_safety_mode !=
      RobotSafetyMode::SAFETY_MODE_NONE) {
    bool incre_safe = incre_safety_check(joints_set, _joints_set_prev,
                                         config.js_interface_config.max_incre);
    if (!incre_safe) {
      std::cerr << "\033[1;33m[WSGGripper][checkJointTarget] Incremental "
                   "safety check failed.\033[0m\n";
      std::cerr << "set joint: " << joints_set.transpose()
                << "\nprev set joint: " << _joints_set_prev.transpose()
                << ", max_incre: " << config.js_interface_config.max_incre
                << std::endl;
      if (config.js_interface_config.incre_safety_mode ==
          RobotSafetyMode::SAFETY_MODE_STOP) {
        std::cerr << "[WSGGripper][checkJointTarget] Returning false."
                  << std::endl;
        return false;
      } else if (config.js_interface_config.incre_safety_mode ==
                 RobotSafetyMode::SAFETY_MODE_TRUNCATE) {
        // clip the joint set around the previous joint set
        joints_set = _joints_set_prev +
                     (joints_set - _joints_set_prev)
                         .cwiseMin(config.js_interface_config.max_incre)
                         .cwiseMax(-config.js_interface_config.max_incre);
        return false;
      }
    }
  }

  bool zone_safe = range_safety_check(
      joints_set, config.js_interface_config.safe_zone, _joints_set_truncated);
  if (!zone_safe) {
    if (config.js_interface_config.range_safety_mode ==
        RobotSafetyMode::SAFETY_MODE_STOP) {
      std::cerr
          << "\033[1;33m[WSGGripper][checkJointTarget] Range safety check "
             "failed.\033[0m\n";
      std::cerr << "[WSGGripper][checkJointTarget] target joints: "
                << joints_set.transpose() << std::endl;
      std::cerr << "[WSGGripper][checkJointTarget] safe range: "
                << config.js_interface_config.safe_zone.transpose()
                << std::endl;
      return false;
    } else if (config.js_interface_config.range_safety_mode ==
               RobotSafetyMode::SAFETY_MODE_TRUNCATE) {
      std::cerr << "[WSGGripper][checkJointTarget] Zone safety check failed. "
                   "Using truncated pose."
                << std::endl;
      joints_set = _joints_set_truncated;
    }
  }
  return true;
}

bool WSGGripper::getJoints(RUT::VectorXd& joints) {
  while (!_wsg_ptr->ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  joints[0] = _wsg_ptr->getWidth();
  return true;
}

bool WSGGripper::setJoints(const RUT::VectorXd& joints) {
  // safety checks
  _joints_set_processed = joints;
  if (!checkJointTarget(_joints_set_processed)) {
    return false;
  }
  _joints_set_prev = _joints_set_processed;

  while (!wsgController.ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  _wsg_ptr->prePositionFingers(false, _joints_set_processed[0]);
  return true;
}

bool WSGGripper::setJointsPosForce(const RUT::VectorXd& joints,
                                   const RUT::VectorXd& forces) {
  // safety checks
  _joints_set_processed = joints;
  if (!checkJointTarget(_joints_set_processed)) {
    return false;
  }
  _joints_set_prev = _joints_set_processed;

  while (!wsgController.ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  _wsg_ptr->setVelResolvedControl(_joints_set_processed[0], forces[0],
                                  _config.velResControl_stiffness,
                                  _config.velResControl_damping);
  // _wsg_ptr->setPDControl(_joints_set_processed[0], _config.PDControl_kp,
  //                        _config.PDControl_kd, forces[0]);
  return true;
}

WSGGripper::WSGGripper() : m_impl{std::make_unique<Implementation>()} {}

WSGGripper::~WSGGripper() {}
