#include <iostream>
#include <mutex>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <coinft/coin_ft.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#include <wsg_gripper/wsg_gripper.h>
#include <wsg_gripper/wsg_gripper_driver.h>

int main() {
  std::string robot_ip = "192.168.1.101";
  std::string port = "1000";
  float pos_target = 30;
  float force_target = 0;
  float velResControl_kp = 3;
  float velResControl_kf = 10;
  float PDControl_kp = 10.0;
  float PDControl_kd = 0.001;

  std::cout << "[main] Starting connection to gripper" << std::endl;
  WSGGripperDriver wsg(robot_ip, port);
  std::cout << "[main] Connection established with gripper." << std::endl;

  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();

  CoinFT::CoinFTConfig config;
  config.port = "/dev/ttyACM0";
  config.baud_rate = 115200;
  config.calibration_file =
      "/home/yifanhou/git/hardware_interfaces/hardware/coinft/config/"
      "calMat_UFT6.csv";

  std::cout << "Creating CoinFT object..." << std::endl;
  CoinFT sensor;
  try {
    // Provide the serial port, baud rate, and calibration matrix file name
    sensor.init(time0, config);
    std::cout << "CoinFT object created." << std::endl;

    while (!sensor.is_data_ready()) {
      std::cout << "CoinFT Waiting for data..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& e) {
    std::cerr << "An error occurred with CoinFT: " << e.what() << std::endl;
    return -1;
  }

  RUT::VectorXd wrench;

  // get gripper states
  std::cout << "[main] Getting gripper state" << std::endl;
  unsigned char cmd_id = wsg.setEmpty();
  WSGState state = wsg.getState(cmd_id);
  wsg.printState(state);

  std::cout << "[main] Starting control loop." << std::endl;
  int count = 0;
  while (timer.toc_ms() < 10000) {
    // read force feedback
    sensor.getWrenchSensor(wrench);
    std::cout << "Time: " << timer.toc_ms() << " ms, Wrench: " << wrench
              << std::endl;

    // send command to gripper
    cmd_id = wsg.setVelResolvedControl(pos_target, force_target,
                                       velResControl_kp, velResControl_kf);
    state = wsg.getState(cmd_id);
    // wsg.printState(state);

    std::cout << "Count: " << ++count << ", Time: " << timer.toc_ms()
              << ", State: " << state.state << ", Pos: " << state.position
              << ", Vel: " << state.velocity << ", Force: " << state.force_motor
              << std::endl;
  }

  std::cout << "Done" << std::endl;

  return 0;
}