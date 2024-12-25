#include <time.h>
#include <exception>
#include <iostream>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include "wsg_gripper/wsg_gripper.h"
#include "wsg_gripper/wsg_gripper_driver.h"

// int main() {
//   WSGGripper::WSGGripperConfig config;
//   config.robot_ip = "192.168.1.101";
//   config.port = "1000";
//   config.velResControl_kp = 10.0;
//   config.velResControl_kf = 0.001;
//   config.PDControl_kp = 10.0;
//   config.PDControl_kd = 0.001;
//   config.js_interface_config.num_joints = 1;
//   config.js_interface_config.range_safety_mode =
//       RobotSafetyMode::SAFETY_MODE_STOP;
//   config.js_interface_config.incre_safety_mode =
//       RobotSafetyMode::SAFETY_MODE_STOP;
//   config.js_interface_config.max_incre = 80;
//   config.js_interface_config.safe_zone = {5, 90};

//   WSGGripper wsg_gripper;
//   RUT::Timer timer;
//   wsg_gripper.init(timer.tic(), config);

//   RUT::VectorXd fb_pos = RUT::VectorXd::Zero(1);
//   RUT::VectorXd target_pos = RUT::VectorXd::Zero(1);
//   RUT::VectorXd target_force = RUT::VectorXd::Zero(1);
//   target_pos[0] = 40;
//   target_force[0] = 10;

//   wsg_gripper.setJointsPosForce(target_pos, target_force);
//   // for (int i = 0; i < 200; i++) {
//   //   wsg_gripper.setJointsPosForce(target_pos, target_force);
//   //   std::cout << i << ", Time_ms: " << timer.toc_ms() << std::endl;
//   // }
//   std::cout << "Done" << std::endl;

//   return 0;
// }

int main() {
  std::string robot_ip = "192.168.2.111";
  std::string port = "1000";
  float pos_target = 20;
  float force_target = 0;
  float velResControl_kp = 3;
  float velResControl_kf = 10;
  float PDControl_kp = 10.0;
  float PDControl_kd = 0.001;

  std::cout << "[main] Starting connection" << std::endl;
  WSGGripperDriver wsg(robot_ip, port);
  std::cout << "[main] Connection established." << std::endl;

  // get states
  std::cout << "[main] Getting state" << std::endl;
  unsigned char cmd_id = wsg.setEmpty();
  WSGState state = wsg.getState(cmd_id);
  wsg.printState(state);

  // set vel resolved control
  RUT::Timer timer;
  timer.tic();
  std::cout << "[main] Setting vel resolved control" << std::endl;
  int count = 0;
  while (timer.toc_ms() < 2000) {
    cmd_id = wsg.setVelResolvedControl(pos_target, force_target,
                                       velResControl_kp, velResControl_kf);
    state = wsg.getState(cmd_id);
    // wsg.printState(state);

    std::cout << "Count: " << ++count << ", Time: " << timer.toc_ms()
              << ", State: " << state.state << ", Pos: " << state.position
              << ", Vel: " << state.velocity << ", Force: " << state.force_motor
              << std::endl;
  }

  wsg.disconnect();
  std::cout << "Done" << std::endl;

  return 0;
}