#include <unistd.h>
#include <memory>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <ati_netft/ati_netft.h>
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#include <ur_rtde/ur_rtde.h>

Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
                          "]");

class StaticCalibration {
 public:
  StaticCalibration(int n_samples = 1000) {
    wrench_data.reserve(n_samples);
    SO3_data.reserve(n_samples);
  }

  void addData(const RUT::Vector6d& wrench, const RUT::Vector7d& pose) {
    wrench_data.push_back(wrench);
    RUT::Matrix4d T = RUT::pose2SE3(pose);
    SO3_data.push_back(T.block<3, 3>(0, 0));
  }

  void solve() {
    int N = SO3_data.size();
    Eigen::MatrixXd A(N * 3, 6);
    Eigen::MatrixXd b(N * 3, 1);
    for (int i = 0; i < N; ++i) {
      A.block<3, 3>(3 * i, 0) = SO3_data[i].transpose();
      A.block<3, 3>(3 * i, 3) = -RUT::Matrix3d::Identity();
      b.block<3, 1>(3 * i, 0) << wrench_data[i][0], wrench_data[i][1],
          wrench_data[i][2];
    }

    std::cout << "A: " << std::endl << A << std::endl;
    std::cout << "b: " << std::endl << b << std::endl;
    std::cout << "Solving SVD..." << std::endl;

    RUT::Vector6d x_GF =
        A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::cout << "Done." << std::endl;

    RUT::Vector3d G, F;
    G << x_GF[0], x_GF[1], x_GF[2];
    F << x_GF[3], x_GF[4], x_GF[5];
    std::cout << "G: " << G << std::endl << "F: " << F << std::endl;

    /*
    Solve for P, T
    */
    for (int i = 0; i < N; ++i) {
      double F_hat0 = wrench_data[i][0] + F[0];
      double F_hat1 = wrench_data[i][1] + F[1];
      double F_hat2 = wrench_data[i][2] + F[2];
      A.block<3, 3>(3 * i, 0) << 0, F_hat2, -F_hat1, -F_hat2, 0, F_hat0, F_hat1,
          -F_hat0, 0;
      b.block<3, 1>(3 * i, 0) << wrench_data[i][3], wrench_data[i][4],
          wrench_data[i][5];
    }
    std::cout << "A: " << std::endl << A << std::endl;
    std::cout << "b: " << std::endl << b << std::endl;
    std::cout << "Solving SVD..." << std::endl;

    RUT::Vector6d x_PT =
        A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::cout << "Done." << std::endl;

    RUT::Vector3d P, T;
    P << x_PT[0], x_PT[1], x_PT[2];
    T << x_PT[3], x_PT[4], x_PT[5];

    Eigen::IOFormat vecFormat(3, 0, "", "\n", "", "", "[", "];");

    RUT::Vector6d Foffset;
    Foffset << F, T;

    std::cout << "\n\nCopy the following to your config yaml file:\n\n";
    std::cout << "------------------- begin of yaml -------------------\n";
    std::cout << "ftsensor:\n";
    std::cout << "  Foffset: [" << Foffset(0) << ", " << Foffset(1) << ", "
              << Foffset(2) << "]\n";
    std::cout << "  Toffset: [" << Foffset(3) << ", " << Foffset(4) << ", "
              << Foffset(5) << "]\n";
    std::cout << "  Gravity: [" << G(0) << ", " << G(1) << ", " << G(2)
              << "]\n";
    std::cout << "  Pcom: [" << P(0) << ", " << P(1) << ", " << P(2) << "]\n";
    std::cout << "------------------- end of yaml -------------------\n";
  }

 private:
  std::vector<RUT::Vector6d> wrench_data;
  std::vector<RUT::Matrix3d> SO3_data;
};

int main() {
  std::cout << "Force calibration demo\n";
  std::cout << "This program will collect wrench data from the FT sensor and "
               "solve for the offset, gravity and COM.\n";
  std::cout << "The robot will be in admittance control mode, where you are "
               "supposed to drag the robot slowly to different orientations "
               "within 10 seconds. The calibration result will be printed out "
               "in the end.\n";
  std::cout << "Note: 1. Make sure the FT sensor config has zeros for Foffset, "
               "Toffset, Gravity and PCom.\n";
  std::cout << "      2. Make sure the FT sensor config has the correct "
               "PoseSensorTool.\n";
  std::cout << "      3. Make sure you are calling the getWrenchTool() "
               "function to get wrench reading for calibration.\n";
  URRTDE::URRTDEConfig robot_config;

  RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
  ATINetft::ATINetftConfig ati_config;
  AdmittanceController::AdmittanceControllerConfig admittance_config;

  // open file
  const std::string CONFIG_PATH =
      "/home/xuxm/hardware_interfaces_internal/applications/"
      "ft_calibration/"
      "config/ft_calibration.yaml";
  YAML::Node config{};
  try {
    config = YAML::LoadFile(CONFIG_PATH);
    robot_config.deserialize(config["ur_rtde"]);
    robotiq_config.deserialize(config["robotiq_ft_modbus"]);
    ati_config.deserialize(config["ati_netft"]);
    deserialize(config["admittance_controller"], admittance_config);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return -1;
  }

  URRTDE robot;
  std::shared_ptr<FTInterfaces> force_sensor_ptr;

  AdmittanceController controller;
  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();
  RUT::Vector7d pose, pose_ref, pose_cmd;
  RUT::Vector6d wrench_6d, wrench_WTr;
  RUT::VectorXd wrench = RUT::VectorXd::Zero(6);

  robot.init(time0, robot_config);
  robot.getCartesian(pose);
  controller.init(time0, admittance_config, pose);

  // force sensor
  bool use_ati = config["use_ati"].as<bool>();
  if (use_ati) {
    force_sensor_ptr = std::shared_ptr<ATINetft>(new ATINetft);
    ATINetft* ati_ptr = static_cast<ATINetft*>(force_sensor_ptr.get());
    if (!ati_ptr->init(time0, ati_config)) {
      std::cerr << "Failed to initialize ATI Netft. Exiting." << std::endl;
      return false;
    }
  } else {
    force_sensor_ptr = std::shared_ptr<RobotiqFTModbus>(new RobotiqFTModbus);
    RobotiqFTModbus* robotiq_ptr =
        static_cast<RobotiqFTModbus*>(force_sensor_ptr.get());
    if (!robotiq_ptr->init(time0, robotiq_config)) {
      std::cerr << "Failed to initialize Robotiq FT Modbus. Exiting."
                << std::endl;
      return false;
    }
  }

  // wait for FT300 to be ready
  std::cout << "Waiting for FT sensor to start streaming.\n";
  while (!force_sensor_ptr->is_data_ready()) {
    usleep(100000);
  }

  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  int n_af = 6;
  controller.setForceControlledAxis(Tr, n_af);

  pose_ref = pose;
  wrench_WTr.setZero();

  timer.tic();

  double t_last_collect_ms = 0;
  const double collect_interval_ms = 500;
  double total_duration_ms = 20000;
  StaticCalibration calibrator(total_duration_ms / collect_interval_ms + 1);
  while (true) {
    RUT::TimePoint t_start = robot.rtde_init_period();
    // Update robot status
    robot.getCartesian(pose);
    robot.getWrenchTool(wrench_6d);
    controller.setRobotStatus(pose, wrench_6d);
    // std::cout << "pose_cmd: " << pose_cmd.transpose() << std::endl;

    // Update robot reference
    controller.setRobotReference(pose_ref, wrench_WTr);

    // Compute the control output
    controller.step(pose_cmd);

    // get wrench from the sensor to be calibrated
    double dt = timer.toc_ms();
    if (dt > t_last_collect_ms + collect_interval_ms) {
      force_sensor_ptr->getWrenchTool(wrench);
      calibrator.addData(wrench, pose);
      t_last_collect_ms = dt;
      printf(
          "t =\t%.1f, recorded wrench:\t%.1f\t%.1f\t%.1f\t%.2f\t%.2f\t%.2f\n",
          dt, wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);
    }

    if (!robot.streamCartesian(pose_cmd)) {
      printf("streamCartesian failed\n");
      break;
    }

    if (dt > total_duration_ms)
      break;

    robot.rtde_wait_period(t_start);
  }

  calibrator.solve();

  return 0;
}