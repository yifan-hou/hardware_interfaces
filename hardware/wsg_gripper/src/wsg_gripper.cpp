#include "wsg_gripper/wsg_gripper.h"

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <RobotUtilities/spatial_utilities.h>

struct WSGGripper::Implementation {
  WSGGripper::WSGGripperConfig _config{};
  RUT::TimePoint _time0;

  // internal variables
  std::shared_ptr<WSGGripperDriver> _wsg_ptr;
  Eigen::VectorXd _joints_set_prev{Eigen::VectorXd::Zero(1)};
  Eigen::VectorXd _joints_set_truncated{Eigen::VectorXd::Zero(1)};
  Eigen::VectorXd _joints_set_processed{Eigen::VectorXd::Zero(1)};

  // control
  Eigen::VectorXd _cmd_pos{Eigen::VectorXd::Zero(1)};
  Eigen::VectorXd _cmd_force{Eigen::VectorXd::Zero(1)};
  std::mutex _cmd_mtx;

  // feedback
  WSGState _wsg_state;
  std::mutex _wsg_state_mtx;

  // thread
  std::thread _thread;

  // thread control
  bool _thread_should_be_running{false};
  std::mutex _thread_should_be_running_mtx;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0,
                  const WSGGripper::WSGGripperConfig& config);
  bool checkJointTarget(RUT::VectorXd& joints_set);
  void controlLoop();
};

WSGGripper::Implementation::Implementation() {}

WSGGripper::Implementation::~Implementation() {
  std::cout << "[WSGGripper] finishing.." << std::endl;
  {
    std::lock_guard<std::mutex> lock(_thread_should_be_running_mtx);
    _thread_should_be_running = false;
  }
  while (!_thread.joinable()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  _thread.join();
}

bool WSGGripper::Implementation::initialize(
    RUT::TimePoint time0,
    const WSGGripper::WSGGripperConfig& wsg_gripper_config) {
  _time0 = time0;
  _config = wsg_gripper_config;

  /* Establish connection with WSG gripper */
  std::cout << "[WSGGripper] Connecting to gripper at " << _config.robot_ip
            << ", port " << _config.port << std::endl;
  _wsg_ptr = std::make_shared<WSGGripperDriver>(_config.robot_ip, _config.port);
  std::cout << "[WSGGripper] Connection established. Clearing up error state."
            << std::endl;

  // initialize
  unsigned char cmd_id = _wsg_ptr->ackFastStop();
  _wsg_ptr->getAck(cmd_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // TODO (Yifan): After Homing, the next command does not work.
  //    Could be that homing generates a certain nonempty response.
  // std::cout << "[WSGGripper] Homing the gripper." << std::endl;
  // cmd_id = _wsg_ptr->homing();
  // _wsg_ptr->getAck(cmd_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "[WSGGripper] Read gripper state." << std::endl;
  // read current state, initialize _joints_set_prev
  cmd_id = _wsg_ptr->askForState();
  _wsg_state = _wsg_ptr->getState(cmd_id);
  _joints_set_prev[0] = static_cast<double>(_wsg_state.position);
  _cmd_pos = _joints_set_prev;

  // start control loop
  _thread = std::thread(&WSGGripper::Implementation::controlLoop, this);

  while (!_thread_should_be_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "[WSGGripper] Finished initialization." << std::endl;

  return true;
}

bool WSGGripper::Implementation::checkJointTarget(RUT::VectorXd& joints_set) {
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

void WSGGripper::Implementation::controlLoop() {
  _thread_should_be_running = true;
  WSGState new_state;
  Eigen::VectorXd cmd_pos = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd cmd_force = Eigen::VectorXd::Zero(1);

  std::cout << "[WSGGripper] Control loop started." << std::endl;
  int count = 0;
  RUT::Timer timer;
  timer.tic();
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_thread_should_be_running_mtx);
      if (!_thread_should_be_running) {
        break;
      }
    }

    // _wsg_ptr->setPDControl(static_cast<float>(_joints_set_processed[0]),
    //                        static_cast<float>(_config.PDControl_kp),
    //                        static_cast<float>(_config.PDControl_kd),
    //                        static_cast<float>(forces[0]));

    // Step one: send control
    {
      std::lock_guard<std::mutex> lock(_cmd_mtx);
      cmd_pos = _cmd_pos;
      cmd_force = _cmd_force;
    }
    unsigned char cmd_id = _wsg_ptr->setVelResolvedControl(
        static_cast<float>(cmd_pos[0]), static_cast<float>(cmd_force[0]),
        static_cast<float>(_config.velResControl_kp),
        static_cast<float>(_config.velResControl_kf));

    // Step two: get state
    new_state = _wsg_ptr->getState(cmd_id);
    {
      std::lock_guard<std::mutex> lock(_wsg_state_mtx);
      _wsg_state = new_state;
    }

    // count++;
    // if (count % 100 == 0) {
    //   std::cout << "[WSGGripper] Control loop running at "
    //             << count * 1000.0 / timer.toc_ms() << " Hz." << std::endl;
    //   timer.tic();
    //   count = 0;
    // }
  }
  _wsg_ptr->disconnect();
  std::cout << "[WSGGripper] Control loop finished." << std::endl;
}

WSGGripper::WSGGripper() : m_impl{std::make_unique<Implementation>()} {}
WSGGripper::~WSGGripper() {}

bool WSGGripper::init(RUT::TimePoint time0,
                      const WSGGripper::WSGGripperConfig& wsg_gripper_config) {
  return m_impl->initialize(time0, wsg_gripper_config);
}

bool WSGGripper::getJoints(RUT::VectorXd& joints) {
  assert(joints.size() == 1);
  if (m_impl->_thread_should_be_running == false) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(m_impl->_wsg_state_mtx);
    joints[0] = static_cast<double>(m_impl->_wsg_state.position);
  }
  return true;
}

bool WSGGripper::setJoints(const RUT::VectorXd& joints) {
  // TODO: Implement setJoints command in wsg_gripper_driver
  throw std::runtime_error("Not implemented.");
  // safety checks
  m_impl->_joints_set_processed = joints;
  if (!m_impl->checkJointTarget(m_impl->_joints_set_processed)) {
    return false;
  }
  m_impl->_joints_set_prev = m_impl->_joints_set_processed;

  // _wsg_ptr->setPDControl(static_cast<float>(_joints_set_processed[0]),
  //                        static_cast<float>(_config.PDControl_kp),
  //                        static_cast<float>(_config.PDControl_kd),
  //                        static_cast<float>(forces[0]));
  return true;
}

bool WSGGripper::setJointsPosForce(const RUT::VectorXd& joints,
                                   const RUT::VectorXd& forces) {
  // safety checks
  m_impl->_joints_set_processed = joints;
  if (!m_impl->checkJointTarget(m_impl->_joints_set_processed)) {
    return false;
  }
  m_impl->_joints_set_prev = m_impl->_joints_set_processed;

  {
    std::lock_guard<std::mutex> lock(m_impl->_cmd_mtx);
    m_impl->_cmd_pos = m_impl->_joints_set_processed;
    m_impl->_cmd_force = forces;
  }
  return true;
}
