#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <coinft/coin_ft.h>
#include <fcntl.h>
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <termios.h>
#include <unistd.h>
#include <ur_rtde/ur_rtde.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <iostream>
#include <thread>

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

// Function to initialize terminal for non-blocking input
void initTerminal() {
  struct termios term;
  tcgetattr(STDIN_FILENO, &term);
  term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
  tcsetattr(STDIN_FILENO, TCSANOW, &term);

  // Set stdin to non-blocking
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// Function to restore terminal settings
void resetTerminal() {
  struct termios term;
  tcgetattr(STDIN_FILENO, &term);
  term.c_lflag |= (ICANON | ECHO);  // Enable canonical mode and echo
  tcsetattr(STDIN_FILENO, TCSANOW, &term);

  // Restore blocking mode
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

// Function to check if a key is pressed and return the character
int kbhit() {
  unsigned char ch;
  int nread = read(STDIN_FILENO, &ch, 1);
  if (nread == 1) {
    return ch;
  }
  return -1;
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
      "/home/yifan/git/hardware_interfaces_internal/applications/"
      "force_control_demo/"
      "config/force_control_demo.yaml";
  YAML::Node config{};
  // CoinFT configs
  try {
    config = YAML::LoadFile(CONFIG_PATH);
    robot_config.deserialize(config["ur_rtde"]);
    coinft_config.deserialize(config["coinft"]);
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
  wrench_WTr = RUT::VectorXd::Zero(6);
  wrench = RUT::VectorXd::Zero(12);

  robot.init(time0, robot_config);
  robot.getCartesian(pose);

  sensor.init(time0, coinft_config);

  std::cout << "Starting in 2 seconds ..." << std::endl;
  sleep(2.0);

  controller.init(time0, admittance_config, pose);

  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  // RUT::Matrix6d Tr;
  // // clang-format off
  // Tr <<
  //       0, 0, 0, 1, 0, 0,
  //       0, 0, 0, 0, 1, 0,
  //       0, 0, 0, 0, 0, 1,
  //       1, 0, 0, 0, 0, 0,
  //       0, 1, 0, 0, 0, 0,
  //       0, 0, 1, 0, 0, 0;
  // // clang-format on
  int n_af = 3;
  controller.setForceControlledAxis(Tr, n_af);

  pose_ref = pose;

  while (!sensor.is_data_ready()) {
    std::cout << "Waiting for data..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  timer.tic();

  // Initialize terminal for non-blocking input
  initTerminal();

  std::cout << "Use arrow keys to move. Press q to exit." << std::endl;
  std::cout << "Current target position: " << pose_ref.head<3>().transpose()
            << std::endl;
  bool running = true;

  RUT::Vector6d wrench_fb;
  while (running) {
    RUT::TimePoint t_start = robot.rtde_init_period();
    // Update robot status
    robot.getCartesian(pose);

    // read wrench
    sensor.getWrenchTool(wrench, 2);

    wrench_fb = wrench.head<6>();

    wrench_fb[0] = -wrench_fb[1];  // flip x axis
    wrench_fb[1] = wrench_fb[0];   // flip x axis

    controller.setRobotStatus(pose, wrench_fb);

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

    // Check for keyboard input
    double delta = 0.005;
    int key = kbhit();
    if (key != -1) {
      // Handle escape sequences for arrow keys
      if (key == 27) {  // ESC character
        key = kbhit();
        if (key == -1) {
          // Just ESC was pressed
          running = false;
          std::cout << "Exiting program..." << std::endl;
        } else if (key == '[') {
          // This is an arrow key
          key = kbhit();
          switch (key) {
            case 'A':  // Up arrow
              pose_ref[2] += delta;
              std::cout << "Up arrow pressed. Current position: "
                        << pose_ref.head<3>().transpose() << std::endl;
              break;
            case 'B':  // Down arrow
              pose_ref[2] -= delta;
              std::cout << "Down arrow pressed. Current position: "
                        << pose_ref.head<3>().transpose() << std::endl;
              break;
            case 'D':  // Left arrow
              pose_ref[0] -= delta;
              std::cout << "Left arrow pressed. Current position: "
                        << pose_ref.head<3>().transpose() << std::endl;
              break;
            case 'C':  // Right arrow
              pose_ref[0] += delta;
              std::cout << "Right arrow pressed. Current position: "
                        << pose_ref.head<3>().transpose() << std::endl;
              break;
          }
        }
      } else if (key == 'q' || key == 'Q') {
        running = false;
        std::cout << "Exiting program..." << std::endl;
      }
    }

    robot.rtde_wait_period(t_start);
  }

  std::cout << "Main loop stopped ..." << std::endl;
  std::cout << "Done." << std::endl;

  // Restore terminal settings
  resetTerminal();
  return 0;
}
