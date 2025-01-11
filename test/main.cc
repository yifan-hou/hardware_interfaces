#include <iostream>
#include <mutex>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <coinft/coin_ft.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#include <wsg_gripper/wsg_gripper.h>
#include <wsg_gripper/wsg_gripper_driver.h>

int main() {
  std::string robot_ip = "192.168.2.111";
  std::string port = "1000";
  float pos_target = 30;
  float force_target = 0;
  float velResControl_kp = 3;
  float velResControl_kf = 20;
  float PDControl_kp = 10.0;
  float PDControl_kd = 0.001;

  std::cout << "[main] Starting connection to gripper" << std::endl;
  WSGGripperDriver wsg(robot_ip, port);
  std::cout << "[main] Connection established with gripper." << std::endl;

  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();

  const std::string CONFIG_PATH =
      "/home/yifanhou/git/hardware_interfaces/workcell/table_top_manip/config/"
      "right_arm_coinft.yaml";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(CONFIG_PATH);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  CoinFT sensor;
  CoinFT::CoinFTConfig coinft_config;
  try {
    coinft_config.deserialize(config_node["coinft0"]);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the CoinFT config file: " << e.what()
              << std::endl;
    return false;
  }
  std::cout << "Creating CoinFT object..." << std::endl;
  try {
    sensor.init(time0, coinft_config);
    std::cout << "CoinFT object created." << std::endl;

    while (!sensor.is_data_ready()) {
      std::cout << "CoinFT Waiting for data..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& e) {
    std::cerr << "An error occurred with CoinFT: " << e.what() << std::endl;
    return -1;
  }

  RUT::VectorXd wrench = RUT::VectorXd::Zero(6);

  // get gripper states
  std::cout << "[main] Getting gripper state" << std::endl;
  unsigned char cmd_id = wsg.askForState();
  WSGState state = wsg.getState(cmd_id);
  wsg.printState(state);

  std::cout << "[main] Starting control loop." << std::endl;
  int count = 0;
  while (timer.toc_ms() < 10000) {
    // read force feedback
    sensor.getWrenchSensor(wrench, 1);
    std::cout << "Time: " << timer.toc_ms()
              << " ms, Wrench: " << wrench.transpose() << std::endl;
    force_target = wrench(2);

    // send command to gripper
    cmd_id = wsg.setVelResolvedControl(pos_target, force_target,
                                       velResControl_kp, velResControl_kf);
    state = wsg.getState(cmd_id);
    // wsg.printState(state);

    // std::cout << "Count: " << ++count << ", Time: " << timer.toc_ms()
    //           << ", State: " << state.state << ", Pos: " << state.position
    //           << ", Vel: " << state.velocity << ", Force: " << state.force_motor
    //           << std::endl;
  }

  std::cout << "Done" << std::endl;

  return 0;
}