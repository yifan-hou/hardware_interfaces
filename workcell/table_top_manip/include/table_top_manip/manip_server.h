#pragma once

#include <unistd.h>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <unsupported/Eigen/CXX11/Tensor>

#include <RobotUtilities/butterworth.h>
#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <hardware_interfaces/js_interfaces.h>
#include <hardware_interfaces/robot_interfaces.h>
#include <hardware_interfaces/types.h>
// hardware used in this app
#include <ati_netft/ati_netft.h>
#include <coinft/coin_ft.h>
#include <gopro/gopro.h>
#include <realsense/realsense.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#include <table_top_manip/perturbation_generator.h>
#include <ur_rtde/ur_rtde.h>
#include <wsg_gripper/wsg_gripper.h>

#include <RobotUtilities/data_buffer.h>

struct ManipServerConfig {
  std::string data_folder{""};
  bool run_robot_thread{false};
  bool run_eoat_thread{false};
  bool run_wrench_thread{false};
  bool run_rgb_thread{false};
  bool plot_rgb{false};
  int rgb_buffer_size{5};
  int robot_buffer_size{100};
  int eoat_buffer_size{100};
  int wrench_buffer_size{100};
  bool mock_hardware{false};
  bool bimanual{false};
  bool use_perturbation_generator{false};
  CameraSelection camera_selection{CameraSelection::NONE};
  ForceSensingMode force_sensing_mode{ForceSensingMode::NONE};
  RUT::Matrix6d low_damping{};
  std::vector<int> output_rgb_hw{};
  std::vector<double>
      wrench_filter_parameters{};  // cutoff frequency, sampling time, order

  bool deserialize(const YAML::Node& node) {
    try {
      data_folder = node["data_folder"].as<std::string>();
      run_robot_thread = node["run_robot_thread"].as<bool>();
      run_eoat_thread = node["run_eoat_thread"].as<bool>();
      run_wrench_thread = node["run_wrench_thread"].as<bool>();
      run_rgb_thread = node["run_rgb_thread"].as<bool>();
      plot_rgb = node["plot_rgb"].as<bool>();
      rgb_buffer_size = node["rgb_buffer_size"].as<int>();
      robot_buffer_size = node["robot_buffer_size"].as<int>();
      eoat_buffer_size = node["eoat_buffer_size"].as<int>();
      wrench_buffer_size = node["wrench_buffer_size"].as<int>();
      mock_hardware = node["mock_hardware"].as<bool>();
      bimanual = node["bimanual"].as<bool>();
      use_perturbation_generator =
          node["use_perturbation_generator"].as<bool>();
      camera_selection = string_to_enum<CameraSelection>(
          node["camera_selection"].as<std::string>());
      force_sensing_mode = string_to_enum<ForceSensingMode>(
          node["force_sensing_mode"].as<std::string>());

      low_damping = RUT::deserialize_vector<RUT::Vector6d>(node["low_damping"])
                        .asDiagonal();
      output_rgb_hw = node["output_rgb_hw"].as<std::vector<int>>();
      wrench_filter_parameters =
          node["wrench_filter_parameters"].as<std::vector<double>>();

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
  bool is_bimanual() { return _config.bimanual; }

  // getters: get the most recent k data points in the buffer
  const Eigen::MatrixXd get_camera_rgb(int k, int camera_id = 0);
  const Eigen::MatrixXd get_wrench(int k, int sensor_id = 0);
  const Eigen::MatrixXd get_wrench_filtered(int k, int sensor_id = 0);
  const Eigen::MatrixXd get_robot_wrench(int k, int robot_id = 0);
  const Eigen::MatrixXd get_pose(int k, int robot_id = 0);
  const Eigen::MatrixXd get_vel(int k, int robot_id = 0);
  const Eigen::MatrixXd get_eoat(int k, int robot_id = 0);
  const int get_test();

  // the following functions return the timestamps of
  //  the most recent getter call of the corresponding feedback
  //  So size is already know
  const Eigen::VectorXd get_camera_rgb_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_wrench_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_wrench_filtered_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_robot_wrench_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_pose_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_vel_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_eoat_timestamps_ms(int id = 0);
  const double get_test_timestamp_ms();

  double get_timestamp_now_ms();  // access the current hardware time

  void set_high_level_maintain_position();
  void set_high_level_free_jogging();

  void set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                       double dt_in_future_ms = 1000, int robot_id = 0);
  void set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af,
                                 int robot_id = 0);
  void set_stiffness_matrix(const RUT::Matrix6d& stiffness, int robot_id = 0);

  void schedule_waypoints(const Eigen::MatrixXd& waypoints,
                          const Eigen::VectorXd& timepoints_ms,
                          int robot_id = 0);
  void schedule_eoat_waypoints(const Eigen::MatrixXd& waypoints,
                               const Eigen::VectorXd& timepoints_ms,
                               int robot_id = 0);
  void schedule_stiffness(const Eigen::MatrixXd& stiffness,
                          const Eigen::VectorXd& timepoints_ms,
                          int robot_id = 0);

  void clear_cmd_buffer();

  // data logging
  void start_saving_data_for_a_new_episode();
  void stop_saving_data();
  bool is_saving_data();
  void set_episode_start(bool start);
  void set_episode_end(bool end);
  bool is_episode_active();

 private:
  // config
  ManipServerConfig _config;

  // additional configs as local variables
  std::vector<RUT::Matrix6d> _stiffnesses_high{};
  std::vector<RUT::Matrix6d> _stiffnesses_low{};
  std::vector<RUT::Matrix6d> _dampings_high{};
  std::vector<RUT::Matrix6d> _dampings_low{};

  // list of id
  std::vector<int> _id_list;

  // data buffers
  std::vector<RUT::DataBuffer<Eigen::MatrixXd>> _camera_rgb_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _pose_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _vel_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _eoat_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>>
      _wrench_buffers;  // wrench from external sensors
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _wrench_filtered_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>>
      _robot_wrench_buffers;  // wrench from the robot itself
  // data buffer mutexes
  std::deque<std::mutex> _camera_rgb_buffer_mtxs;
  std::deque<std::mutex> _pose_buffer_mtxs;
  std::deque<std::mutex> _vel_buffer_mtxs;
  std::deque<std::mutex> _eoat_buffer_mtxs;
  std::deque<std::mutex> _wrench_buffer_mtxs;
  std::deque<std::mutex> _wrench_filtered_buffer_mtxs;
  std::deque<std::mutex> _robot_wrench_buffer_mtxs;
  // data buffer timestamps
  std::vector<RUT::DataBuffer<double>> _camera_rgb_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _pose_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _vel_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _eoat_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _wrench_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _wrench_filtered_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _robot_wrench_timestamp_ms_buffers;
  // data buffer timestamps just being fetched
  std::vector<Eigen::VectorXd> _camera_rgb_timestamps_ms;
  std::vector<Eigen::VectorXd> _pose_timestamps_ms;
  std::vector<Eigen::VectorXd> _vel_timestamps_ms;
  std::vector<Eigen::VectorXd> _eoat_timestamps_ms;
  std::vector<Eigen::VectorXd> _wrench_timestamps_ms;
  std::vector<Eigen::VectorXd> _wrench_filtered_timestamps_ms;
  std::vector<Eigen::VectorXd> _robot_wrench_timestamps_ms;

  // action buffers
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _waypoints_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _eoat_waypoints_buffers;
  std::vector<RUT::DataBuffer<Eigen::MatrixXd>> _stiffness_buffers;
  // action buffer mutexes
  std::deque<std::mutex> _waypoints_buffer_mtxs;
  std::deque<std::mutex> _eoat_waypoints_buffer_mtxs;
  std::deque<std::mutex> _stiffness_buffer_mtxs;
  // action buffer timestamps
  std::vector<RUT::DataBuffer<double>> _waypoints_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _eoat_waypoints_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _stiffness_timestamp_ms_buffers;
  double _test_timestamp_ms;

  // timing
  RUT::Timer _timer;

  // episode start and end flags
  bool _start_episode = false;
  bool _end_episode = false;
  std::mutex _flag_mtx;
  std::condition_variable _flag_cv;

  //  hardware interfaces
  std::vector<std::shared_ptr<CameraInterfaces>> camera_ptrs;
  std::vector<std::shared_ptr<FTInterfaces>> force_sensor_ptrs;
  std::vector<std::shared_ptr<RobotInterfaces>> robot_ptrs;
  std::vector<std::shared_ptr<JSInterfaces>> eoat_ptrs;

  // controllers
  std::vector<AdmittanceController> _controllers;
  std::vector<PerturbationGenerator> _perturbation_generators;
  std::deque<std::mutex> _controller_mtxs;

  // wrench filters
  std::vector<RUT::Butterworth> _wrench_filters;

  // threads
  std::vector<std::thread> _robot_threads;
  std::vector<std::thread> _eoat_threads;
  std::vector<std::thread> _wrench_threads;
  std::vector<std::thread> _rgb_threads;
  std::thread _rgb_plot_thread;
  std::thread _data_saving_thread;

  // control variables to control the threads
  std::vector<std::string> _ctrl_rgb_folders;
  std::vector<std::ofstream> _ctrl_robot_data_streams;
  std::vector<std::ofstream> _ctrl_eoat_data_streams;
  std::vector<std::ofstream> _ctrl_wrench_data_streams;
  bool _ctrl_flag_running = false;  // flag to terminate the program
  bool _ctrl_flag_saving = false;   // flag for ongoing data collection
  std::mutex _ctrl_mtx;

  // state variable indicating the status of the threads
  std::vector<bool> _states_robot_thread_ready{};
  std::vector<bool> _states_eoat_thread_ready{};
  std::vector<bool> _states_rgb_thread_ready{};
  std::vector<bool> _states_wrench_thread_ready{};
  bool _state_plot_thread_ready{false};
  bool _state_data_saving_thread_ready{false};

  std::vector<bool> _states_robot_thread_saving{};
  std::vector<bool> _states_eoat_thread_saving{};
  std::vector<bool> _states_rgb_thread_saving{};
  std::vector<bool> _states_wrench_thread_saving{};

  std::vector<int> _states_robot_seq_id{};
  std::vector<int> _states_eoat_seq_id{};
  std::vector<int> _states_rgb_seq_id{};
  std::vector<int> _states_wrench_seq_id{};

  // shared variables between camera thread and plot thread
  std::vector<cv::Mat> _color_mats;
  std::deque<std::mutex> _color_mat_mtxs;

  // shared variables between robot thread and wrench thread
  std::vector<Eigen::VectorXd> _poses_fb;
  std::vector<Eigen::VectorXd> _perturbation;
  std::deque<std::mutex> _poses_fb_mtxs;
  std::deque<std::mutex> _perturbation_mtxs;

  // shared variables between eoat thread and wrench thread
  std::vector<Eigen::VectorXd> _wrench_fb;
  std::deque<std::mutex> _wrench_fb_mtxs;

  // variables for data saving
  std::string _current_episode_folder;
  std::vector<std::ofstream> _data_streams;

  // loop functions
  void robot_loop(const RUT::TimePoint& time0, int robot_id);
  void eoat_loop(const RUT::TimePoint& time0, int robot_id);
  void rgb_loop(const RUT::TimePoint& time0, int camera_id);
  void wrench_loop(const RUT::TimePoint& time0, int publish_rate,
                   int sensor_id);
  void rgb_plot_loop();  // opencv plotting does not support multi-threading
  void data_saving_loop();
};