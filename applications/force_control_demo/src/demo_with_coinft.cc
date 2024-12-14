#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <coinft/coin_ft.h>
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <unistd.h>
#include <ur_rtde/ur_rtde.h>
#include <yaml-cpp/yaml.h>

Eigen::MatrixXd deserialize_matrix(const YAML::Node& node) {
  int nr = node.size();
  int nc = node[0].size();
  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nr, nc);
  for (int r = 0; r < nr; ++r) {
    for (int c = 0; c < nc; ++c) {
      mat(r, c) = node[r][c].as<double>();
    }
  }
  return mat;
}

// load eigen
template <typename T>
T deserialize_vector(const YAML::Node& node) {
  std::vector<double> q = node.as<std::vector<double>>();
  return Eigen::Map<T, Eigen::Unaligned>(q.data(), q.size());
}

int main() {
  URRTDE::URRTDEConfig robot_config;
  CoinFT::CoinFTConfig coinft_config;
  AdmittanceController::AdmittanceControllerConfig admittance_config;

  // open file
  const std::string CONFIG_PATH =
      "/home/yifanhou/git/hardware_interfaces/applications/force_control_demo/"
      "config/force_control_demo.yaml";
  YAML::Node config{};
  // CoinFT configs
  // TODO: replace with CoinFT config serialization
  bool ft_use_coinft = false;
  std::string coinft_port;
  unsigned int coinft_baud_rate;
  std::string coinft_calibration_file;
  std::vector<double> coinft_PoseSensorTool;
  RUT::Matrix6d adj_sensor_tool = RUT::Matrix6d::Identity();
  try {
    config = YAML::LoadFile(CONFIG_PATH);
    robot_config.deserialize(config["ur_rtde"]);
    coinft_config.deserialize(config["coin_ft"]);
    deserialize(config["admittance_controller"], admittance_config);

  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return -1;
  }

  URRTDE robot;
  CoinFT sensor;
  AdmittanceController controller;
  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();
  RUT::Vector7d pose, pose_ref, pose_cmd;
  RUT::VectorXd wrench, wrench_WTr;
  wrench_WTr.setZero();

  robot.init(time0, robot_config);
  robot.getCartesian(pose);

  sensor.init(time0, coinft_config);

  std::cout << "Starting in 2 seconds ..." << std::endl;
  sleep(2.0);

  controller.init(time0, admittance_config, pose);

  // RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  RUT::Matrix6d Tr;
  // clang-format off
  Tr <<
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;
  // clang-format on
  int n_af = 3;
  controller.setForceControlledAxis(Tr, n_af);

  pose_ref = pose;

  while (!sensor.is_data_ready()) {
    std::cout << "Waiting for data..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  timer.tic();

  while (true) {
    RUT::TimePoint t_start = robot.rtde_init_period();
    // Update robot status
    robot.getCartesian(pose);

    // read wrench
    sensor.getWrenchTool(wrench);

    controller.setRobotStatus(pose, wrench);

    // Update robot reference
    controller.setRobotReference(pose_ref, wrench_WTr);

    // Compute the control output
    controller.step(pose_cmd);

    if (!robot.streamCartesian(pose_cmd)) {
      printf("streamCartesian failed\n");
      break;
    }

    double dt = timer.toc_ms();
    printf("t = %f, wrench: %f %f %f %f %f %f\n", dt, wrench[0], wrench[1],
           wrench[2], wrench[3], wrench[4], wrench[5]);

    if (dt > 3000)
      break;

    robot.rtde_wait_period(t_start);
  }
  std::cout << "Main loop stopped ..." << std::endl;

  sensor.stopStreaming();
  std::cout << "Done." << std::endl;

  return 0;
}