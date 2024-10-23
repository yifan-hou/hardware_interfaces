#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
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
  AdmittanceController::AdmittanceControllerConfig admittance_config;

  // open file
  const std::string CONFIG_PATH =
      "/path/to/hardware_interfaces/applications/force_control_demo/"
      "config/force_control_demo.yaml";
  YAML::Node config{};
  try {
    config = YAML::LoadFile(CONFIG_PATH);
    robot_config.deserialize(config["ur_rtde"]);
    deserialize(config["admittance_controller"], admittance_config);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return -1;
  }

  URRTDE robot;
  AdmittanceController controller;
  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();
  RUT::Vector7d pose, pose_ref, pose_cmd;
  RUT::Vector6d wrench, wrench0, wrench_WTr;

  robot.init(time0, robot_config);
  robot.getCartesian(pose);
  robot.getWrenchTool(wrench0);

  // get average wrench
  std::cout << "wrench0: " << wrench0.transpose() << std::endl;
  std::cout << "Starting in 2 seconds ..." << std::endl;
  wrench0.setZero();
  sleep(2.0);
  // int N = 200;
  // for (int i = 0; i < N; ++i) {
  //   RUT::Vector6d wrench_temp;
  //   robot.getWrenchTool(wrench_temp);
  //   wrench0 += wrench_temp;
  //   usleep(10000);
  // }
  // wrench0 /= N;
  // std::cout << "wrench0: " << wrench0.transpose() << std::endl;

  controller.init(time0, admittance_config, pose);

  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  // RUT::Matrix6d Tr;
  // // clang-format off
  // Tr << 0, 0, 0, 1, 0, 0,
  //       0, 0, 0, 0, 1, 0,
  //       0, 0, 0, 0, 0, 1,
  //       1, 0, 0, 0, 0, 0,
  //       0, 1, 0, 0, 0, 0,
  //       0, 0, 1, 0, 0, 0;
  // // clang-format on
  int n_af = 6;
  controller.setForceControlledAxis(Tr, n_af);

  pose_ref = pose;
  wrench_WTr.setZero();

  timer.tic();

  while (true) {
    RUT::TimePoint t_start = robot.rtde_init_period();
    // Update robot status
    robot.getCartesian(pose);
    robot.getWrenchTool(wrench);
    controller.setRobotStatus(pose, wrench - wrench0);

    // // generate wrench reference as a 1Hz sinusoidal signal
    // double t = timer.toc_ms() / 1000.0;
    // wrench_WTr[0] = 5 * sin(2 * M_PI * t);
    // wrench_WTr[1] = 5 * sin(2 * M_PI * t);
    // wrench_WTr[2] = 5 * sin(2 * M_PI * t);
    // wrench_WTr[3] = 0.5 * sin(2 * M_PI * t);
    // wrench_WTr[4] = 0.5 * sin(2 * M_PI * t);
    // wrench_WTr[5] = 0.5 * sin(2 * M_PI * t);

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

    if (dt > 50000)
      break;

    robot.rtde_wait_period(t_start);
  }

  return 0;
}