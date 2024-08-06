#pragma once

#include <unistd.h>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <unsupported/Eigen/CXX11/Tensor>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <hardware_interfaces/robot_interfaces.h>
#include <hardware_interfaces/types.h>
// hardware used in this app
#include <ati_netft/ati_netft.h>
#include <gopro/gopro.h>
#include <realsense/realsense.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#include <ur_rtde/ur_rtde.h>

#include <RobotUtilities/data_buffer.h>

// typedef Eigen::TensorMap<Eigen::Tensor<float, 3, Eigen::RowMajor>> RGBTensor;

struct ManipServerConfig {
  std::string data_folder{""};
  std::string camera_selection{""};
  bool plot_rgb{false};
  bool run_low_dim_thread{false};
  int rgb_buffer_size{5};
  int pose_buffer_size{100};
  int wrench_buffer_size{100};
  bool mock_hardware{false};
  ForceSensingMode force_sensing_mode{ForceSensingMode::NONE};

  bool deserialize(const YAML::Node& node) {
    try {
      data_folder = node["data_folder"].as<std::string>();
      camera_selection = node["camera_selection"].as<std::string>();
      plot_rgb = node["plot_rgb"].as<bool>();
      run_low_dim_thread = node["run_low_dim_thread"].as<bool>();
      rgb_buffer_size = node["rgb_buffer_size"].as<int>();
      pose_buffer_size = node["pose_buffer_size"].as<int>();
      wrench_buffer_size = node["wrench_buffer_size"].as<int>();
      mock_hardware = node["mock_hardware"].as<bool>();
      force_sensing_mode = string_to_enum<ForceSensingMode>(
          node["force_sensing_mode"].as<std::string>());
    } catch (const std::exception& e) {
      std::cerr << "Failed to load the config file: " << e.what() << std::endl;
      return false;
    }
    return true;
  }
};

/// @brief ManipServer class
/// This class is the main interface to the hardware. It initializes the hardware
/// interfaces, starts the threads to collect data, and provides the most recent
/// data points to the user.
/// Usage:
///   ManipServer server;
///   server.initialize("config.yaml");
///   // wait until the server is ready
///   while (!server.is_ready()) {
///     usleep(1000);
///   }
///   while (server.is_running()) {
///     Eigen::MatrixXd camera_rgb = server.get_camera_rgb(5); // get the most recent 5 data points
///     Eigen::MatrixXd pose = server.get_pose(5);
///     Eigen::MatrixXd wrench = server.get_wrench(5);
///     // get the timestamps of the most recently fetched data points
///     Eigen::VectorXd camera_rgb_timestamps = server.get_camera_rgb_timestamps_ms();
///     Eigen::VectorXd pose_timestamps = server.get_pose_timestamps_ms();
///     Eigen::VectorXd wrench_timestamps = server.get_wrench_timestamps_ms();
///     // do something with the data
///   }
///   server.join_threads();
class ManipServer {
 public:
  ManipServer() {};
  ManipServer(const std::string&);
  ~ManipServer();

  bool initialize(const std::string& config_file);
  void join_threads();
  bool is_ready();  // check if all buffers are full
  bool is_running();

  // getters: get the most recent k data points in the buffer
  const Eigen::MatrixXd get_camera_rgb(int k);
  const Eigen::MatrixXd get_wrench(int k);
  const Eigen::MatrixXd get_pose(int k);

  // the following functions return the timestamps of
  //  the most recent getter call of the corresponding feedback
  //  So size is already know
  const Eigen::VectorXd get_camera_rgb_timestamps_ms();
  const Eigen::VectorXd get_wrench_timestamps_ms();
  const Eigen::VectorXd get_pose_timestamps_ms();

  double get_timestamp_now_ms();  // access the current hardware time

  void set_high_level_maintain_position();
  void set_high_level_free_jogging();

  void set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                       double dt_in_future_ms = 1000);
  void set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af);
  void set_stiffness_matrix(const RUT::Matrix6d& stiffness);

  void schedule_waypoints(const Eigen::MatrixXd& waypoints,
                          const Eigen::VectorXd& timepoints_ms);
  void schedule_stiffness(const Eigen::MatrixXd& stiffness,
                          const Eigen::VectorXd& timepoints_ms);

 private:
  // config
  ManipServerConfig _config;

  // data buffer
  RUT::DataBuffer<Eigen::MatrixXd> _camera_rgb_buffer;
  RUT::DataBuffer<Eigen::VectorXd> _pose_buffer;
  RUT::DataBuffer<Eigen::VectorXd> _wrench_buffer;
  // action buffer
  RUT::DataBuffer<Eigen::VectorXd> _waypoints_buffer;
  RUT::DataBuffer<Eigen::MatrixXd> _stiffness_buffer;

  RUT::DataBuffer<double> _camera_rgb_timestamp_ms_buffer;
  RUT::DataBuffer<double> _pose_timestamp_ms_buffer;
  RUT::DataBuffer<double> _wrench_timestamp_ms_buffer;
  RUT::DataBuffer<double> _waypoints_timestamp_ms_buffer;
  RUT::DataBuffer<double> _stiffness_timestamp_ms_buffer;

  std::mutex _camera_rgb_buffer_mtx;
  std::mutex _pose_buffer_mtx;
  std::mutex _wrench_buffer_mtx;
  std::mutex _waypoints_buffer_mtx;
  std::mutex _stiffness_buffer_mtx;

  // timing
  RUT::Timer _timer;

  // additional configs as local variables
  RUT::Matrix6d _stiffness_high{};
  RUT::Matrix6d _stiffness_low{};

  //  hardware interfaces
  std::shared_ptr<CameraInterfaces> camera_ptr;
  std::shared_ptr<FTInterfaces> force_sensor_ptr;
  URRTDE* robot_ptr;
  AdmittanceController controller;

  // threads
  std::thread _rgb_thread;
  std::thread _low_dim_thread;
  std::thread _rgb_plot_thread;

  // control variables to control the threads
  std::string _ctrl_rgb_folder;
  std::ofstream _ctrl_low_dim_data_stream;
  bool _ctrl_flag_running = false;  // flag to terminate the program
  bool _ctrl_flag_saving = false;   // flag for ongoing data collection
  std::mutex _ctrl_mtx;

  // controller variables to control the force controller
  std::mutex _controller_mtx;

  // state variable indicating the status of the threads
  bool _state_low_dim_thread_ready = false;
  bool _state_rgb_thread_ready = false;
  bool _state_low_dim_thread_saving = false;
  bool _state_rgb_thread_saving = false;
  int _state_low_dim_seq_id = 0;
  int _state_rgb_seq_id = 0;

  // pre-allocated variables for camera feedback
  cv::Mat* _bgr;  //destination array
  Eigen::MatrixXd _bm, _gm, _rm;
  Eigen::MatrixXd _rgb_row_combined;

  // temp variables storing timestamps of data just being fetched
  Eigen::VectorXd _camera_rgb_timestamps_ms;
  Eigen::VectorXd _pose_timestamps_ms;
  Eigen::VectorXd _wrench_timestamps_ms;

  void low_dim_loop(const RUT::TimePoint& time0);

  void rgb_loop(const RUT::TimePoint& time0);

  void rgb_plot_loop();
};