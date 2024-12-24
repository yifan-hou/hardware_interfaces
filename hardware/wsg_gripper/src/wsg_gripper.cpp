#include "wsg_gripper/wsg_gripper.h"

#include <Eigen/Dense>
#include <memory>

#include <RobotUtilities/spatial_utilities.h>

WSGGripper::WSGGripper() {
  _joints_set_prev = RUT::VectorXd::Zero(1);
  _joints_set_truncated = RUT::VectorXd::Zero(1);
  _joints_set_processed = RUT::VectorXd::Zero(1);
}

WSGGripper::~WSGGripper() {}

bool WSGGripper::init(RUT::TimePoint time0,
                      const WSGGripper::WSGGripperConfig& wsg_gripper_config) {
  _time0 = time0;
  _config = wsg_gripper_config;

  /* Establish connection with WSG gripper */
  std::cout << "[WSGGripper] Connecting to gripper at " << _config.robot_ip
            << ", port " << _config.port << std::endl;
  _wsg_ptr = std::make_shared<WSGGripperDriver>(_config.robot_ip, _config.port);
  std::cout << "[WSGGripper] Connection established." << std::endl;

  // read current state
  assert(getJoints(_joints_set_prev));
  return true;
}

bool WSGGripper::checkJointTarget(RUT::VectorXd& joints_set) {
  if (_config.js_interface_config.incre_safety_mode !=
      RobotSafetyMode::SAFETY_MODE_NONE) {
    bool incre_safe = incre_safety_check(joints_set, _joints_set_prev,
                                         _config.js_interface_config.max_incre);
    if (!incre_safe) {
      std::cerr << "\033[1;33m[WSGGripper][checkJointTarget] Incremental "
                   "safety check failed.\033[0m\n";
      std::cerr << "set joint: " << joints_set.transpose()
                << "\nprev set joint: " << _joints_set_prev.transpose()
                << ", max_incre: " << _config.js_interface_config.max_incre
                << std::endl;
      if (_config.js_interface_config.incre_safety_mode ==
          RobotSafetyMode::SAFETY_MODE_STOP) {
        std::cerr << "[WSGGripper][checkJointTarget] Returning false."
                  << std::endl;
        return false;
      } else if (_config.js_interface_config.incre_safety_mode ==
                 RobotSafetyMode::SAFETY_MODE_TRUNCATE) {
        // clip the joint set around the previous joint set
        joints_set = _joints_set_prev +
                     (joints_set - _joints_set_prev)
                         .cwiseMin(_config.js_interface_config.max_incre)
                         .cwiseMax(-_config.js_interface_config.max_incre);
        return false;
      }
    }
  }

  bool zone_safe = range_safety_check(
      joints_set, _config.js_interface_config.safe_zone, _joints_set_truncated);
  if (!zone_safe) {
    if (_config.js_interface_config.range_safety_mode ==
        RobotSafetyMode::SAFETY_MODE_STOP) {
      std::cerr
          << "\033[1;33m[WSGGripper][checkJointTarget] Range safety check "
             "failed.\033[0m\n";
      std::cerr << "[WSGGripper][checkJointTarget] target joints: "
                << joints_set.transpose() << std::endl;
      std::cerr << "[WSGGripper][checkJointTarget] safe range: "
                << _config.js_interface_config.safe_zone.transpose()
                << std::endl;
      return false;
    } else if (_config.js_interface_config.range_safety_mode ==
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
  // joints[0] = _wsg_ptr->getWidth();
  return true;
}

bool WSGGripper::setJoints(const RUT::VectorXd& joints) {
  // safety checks
  _joints_set_processed = joints;
  if (!checkJointTarget(_joints_set_processed)) {
    return false;
  }
  _joints_set_prev = _joints_set_processed;

  // _wsg_ptr->prePositionFingers(false, _joints_set_processed[0]);
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

  // _wsg_ptr->setPDControl(static_cast<float>(_joints_set_processed[0]),
  //                        static_cast<float>(_config.PDControl_kp),
  //                        static_cast<float>(_config.PDControl_kd),
  //                        static_cast<float>(forces[0]));
  // _wsg_ptr->setVelResolvedControl(
  //     static_cast<float>(_joints_set_processed[0]),
  //     static_cast<float>(forces[0]),
  //     static_cast<float>(_config.velResControl_stiffness),
  //     static_cast<float>(_config.velResControl_damping));
  return true;
}
