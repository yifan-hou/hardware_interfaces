#include "arx_can/arx_can.h"

#include <Eigen/Dense>
#include <memory>
#include <mutex>

#include <RobotUtilities/utilities.h>

#include <arx5-sdk/app/joint_controller.h>

using namespace arx;

ARXCAN *ARXCAN::pinstance = 0;

struct ARXCAN::Implementation {
  std::shared_ptr<Arx5JointController> arx_controller_ptr;

  ARXCAN::ARXCANConfig config{};

  arx::JointState js_state;
  arx::Gain js_gain;
  Vec6d tool_pose;

  std::mutex js_state_mutex;
  // double dt_s{};
  RUT::TimePoint time0;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0, const ARXCAN::ARXCANConfig &config);
  bool getCartesian(double *pose_xyzq);
  bool setCartesian(const double *pose_xyzq);
  bool getJoints(double *joints);
  bool setJoints(const double *joints);
  bool setGains(const double *kp, const double *kd);
};

ARXCAN::Implementation::Implementation() {
}

ARXCAN::Implementation::~Implementation() {
  std::cout << "[ARXCAN] finishing.." << std::endl;
  delete pinstance;
}

bool ARXCAN::Implementation::initialize(
    RUT::TimePoint time0, const ARXCAN::ARXCANConfig &arx_can_config) {
  time0 = time0;
  config = arx_can_config;

  std::cout << "[ARXCAN] Connecting to robot at " << config.can_interface
            << std::endl;

  arx_controller_ptr = std::shared_ptr<Arx5JointController>(
      new Arx5JointController(config.can_interface));

  std::cout << "[ARXCAN] CAN connection established.\n";
  arx_controller_ptr->set_log_level(spdlog::level::level_enum::debug);
  arx_controller_ptr->enable_background_send_recv();

  if (config.enable_gravity_compensation) {
    arx_controller_ptr->enable_gravity_compensation(config.urdf_path);
  }

  // setting default gains
  js_gain.kp = {100.0, 100.0, 100.0, 30.0, 30, 5.0};
  js_gain.kd = {1.5, 1.5, 1.5, 2.0, 1.0, 1.0};
  js_gain.gripper_kp = 5.0;
  js_gain.gripper_kd = 0.1;
  arx_controller_ptr->set_gain(js_gain);

  if (config.reset_to_home_upon_start) {
    arx_controller_ptr->reset_to_home();
  }

  js_state = arx_controller_ptr->get_state();

  std::cout << "[ARXCAN] Controller initialized.\n";
  return true;
}

bool ARXCAN::Implementation::getCartesian(double *pose_xyzq) {
  tool_pose = arx_controller_ptr->get_tool_pose();
  pose_xyzq[0] = tool_pose[0];
  pose_xyzq[1] = tool_pose[1];
  pose_xyzq[2] = tool_pose[2];
  RUT::rpy2quat(tool_pose[3], tool_pose[4], tool_pose[5], pose_xyzq + 3);

  return true;
}

bool ARXCAN::Implementation::setCartesian(const double *pose_xyzq_set) {
  assert(config.robot_interface_config.operationMode ==
         OPERATION_MODE_CARTESIAN);
  throw std::runtime_error("[ARXCAN] setCartesian not implemented yet");
}

bool ARXCAN::Implementation::getJoints(double *joints) {
  js_state_mutex.lock();
  js_state = arx_controller_ptr->get_state();
  for (int i = 0; i < 6; i++) {
    joints[i] = js_state.pos[i];
  }
  js_state_mutex.unlock();
  return true;
}

bool ARXCAN::Implementation::setJoints(const double *joints) {
  assert(config.robot_interface_config.operationMode == OPERATION_MODE_JOINT);
  js_state_mutex.lock();
  for (int i = 0; i < 6; i++) {
    js_state.pos[i] = joints[i];
  }
  arx_controller_ptr->set_joint_cmd(js_state);
  js_state_mutex.unlock();
  return true;
}

bool ARXCAN::Implementation::setGains(const double *kp, const double *kd) {
  for (int i = 0; i < 6; i++) {
    js_gain.kp[i] = kp[i];
    js_gain.kd[i] = kd[i];
  }
  arx_controller_ptr->set_gain(js_gain);
  return true;
}

ARXCAN::ARXCAN() : m_impl{std::make_unique<Implementation>()} {}

ARXCAN::~ARXCAN() {}

ARXCAN *ARXCAN::Instance() {
  if (pinstance == 0) {
    pinstance = new ARXCAN();
  }
  return pinstance;
}

bool ARXCAN::init(RUT::TimePoint time0, const ARXCANConfig &ur_rtde_config) {
  return m_impl->initialize(time0, ur_rtde_config);
}

bool ARXCAN::getCartesian(double *pose_xyzq) {
  return m_impl->getCartesian(pose_xyzq);
}

bool ARXCAN::setCartesian(const double *pose_xyzq) {
  return m_impl->setCartesian(pose_xyzq);
}

bool ARXCAN::getJoints(double *joints) {
  return m_impl->getJoints(joints);
}

bool ARXCAN::setJoints(const double *joints) {
  return m_impl->setJoints(joints);
}

bool ARXCAN::setGains(const double *kp, const double *kd) {
  return m_impl->setGains(kp, kd);
}