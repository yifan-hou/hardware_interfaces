#include "table_top_manip/manip_server.h"
#include "helpers.hpp"

ManipServer::ManipServer(const std::string& config_path) {
  initialize(config_path);
}

ManipServer::~ManipServer() {
  // clean up
  delete[] _bgr;
}

bool ManipServer::initialize(const std::string& config_path) {
  std::cout << "[ManipServer] Initializing.\n";

  RUT::TimePoint time0 = _timer.tic();

  // read config files
  std::cout << "[ManipServer] Reading config files.\n";

  URRTDE::URRTDEConfig robot_config;
  ATINetft::ATINetftConfig ati_config;
  RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
  Realsense::RealsenseConfig realsense_config;
  GoPro::GoProConfig gopro_config;
  AdmittanceController::AdmittanceControllerConfig admittance_config;

  YAML::Node config;

  try {
    config = YAML::LoadFile(config_path);

    _config.deserialize(config);
    robot_config.deserialize(config["ur_rtde"]);
    ati_config.deserialize(config["ati_netft"]);
    robotiq_config.deserialize(config["robotiq_ft_modbus"]);
    realsense_config.deserialize(config["realsense"]);
    gopro_config.deserialize(config["gopro"]);
    deserialize(config["admittance_controller"], admittance_config);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  _stiffness_high = admittance_config.compliance6d.stiffness;
  _stiffness_low = RUT::Matrix6d::Zero();

  // initialize hardwares
  std::cout << "[ManipServer] Initialize each hardware interface.\n";

  robot_ptr = URRTDE::Instance();

  _bgr = new cv::Mat[3];

  if (!_config.mock_hardware) {
    // robot
    if (!robot_ptr->init(time0, robot_config)) {
      std::cerr << "Failed to initialize UR RTDE. Exiting." << std::endl;
      return false;
    }

    // camera
    if (_config.camera_selection == CameraSelection::GOPRO) {
      camera_ptr = std::shared_ptr<GoPro>(new GoPro);
      GoPro* gopro_ptr = static_cast<GoPro*>(camera_ptr.get());
      if (!gopro_ptr->init(time0, gopro_config)) {
        std::cerr << "Failed to initialize GoPro. Exiting." << std::endl;
        return false;
      }
    } else if (_config.camera_selection == CameraSelection::REALSENSE) {
      camera_ptr = std::shared_ptr<Realsense>(new Realsense);
      Realsense* realsense_ptr = static_cast<Realsense*>(camera_ptr.get());
      if (!realsense_ptr->init(time0, realsense_config)) {
        std::cerr << "Failed to initialize realsense. Exiting." << std::endl;
        return false;
      }
    } else {
      std::cerr << "Invalid camera selection. Exiting." << std::endl;
      return false;
    }

    // force sensor
    if (_config.force_sensing_mode == ForceSensingMode::FORCE_MODE_ATI) {
      force_sensor_ptr = std::shared_ptr<ATINetft>(new ATINetft);
      ATINetft* ati_ptr = static_cast<ATINetft*>(force_sensor_ptr.get());
      if (!ati_ptr->init(time0, ati_config)) {
        std::cerr << "Failed to initialize ATI Netft. Exiting." << std::endl;
        return false;
      }
    } else if (_config.force_sensing_mode ==
               ForceSensingMode::FORCE_MODE_ROBOTIQ) {
      force_sensor_ptr = std::shared_ptr<RobotiqFTModbus>(new RobotiqFTModbus);
      RobotiqFTModbus* robotiq_ptr =
          static_cast<RobotiqFTModbus*>(force_sensor_ptr.get());
      if (!robotiq_ptr->init(time0, robotiq_config)) {
        std::cerr << "Failed to initialize Robotiq FT Modbus. Exiting."
                  << std::endl;
        return false;
      }
    } else {
      std::cerr << "Invalid force sensing mode. Exiting." << std::endl;
      return false;
    }
  }

  // initialize admittance controller
  std::cout << "[ManipServer] Initialize admittance controller.\n";

  RUT::Vector7d pose = RUT::Vector7d::Zero();
  if (!_config.mock_hardware) {
    robot_ptr->getCartesian(pose);
  }
  if (!controller.init(time0, admittance_config, pose)) {
    std::cerr << "Failed to initialize admittance controller. Exiting."
              << std::endl;
    return false;
  }
  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  // The robot should not behave with any compliance during initialization.
  // The user needs to set the desired compliance afterwards.
  int n_af = 0;
  controller.setForceControlledAxis(Tr, n_af);

  // create the data buffers
  std::cout << "[ManipServer] Creating data buffers.\n";
  int image_height = gopro_config.crop_rows[1] - gopro_config.crop_rows[0];
  int image_width = gopro_config.crop_cols[1] - gopro_config.crop_cols[0];
  _camera_rgb_buffer.initialize(_config.rgb_buffer_size, 3 * image_height,
                                image_width, "camera_rgb");
  _pose_buffer.initialize(_config.pose_buffer_size, 7, 1, "pose");
  _wrench_buffer.initialize(_config.wrench_buffer_size, 6, 1, "wrench");
  _waypoints_buffer.initialize(-1, 7, 1, "waypoints");
  _stiffness_buffer.initialize(-1, 6, 6, "stiffness");

  // create the timestamp buffers
  _camera_rgb_timestamp_ms_buffer.initialize(_config.rgb_buffer_size, 1, 1,
                                             "camera_rgb_timestamp_ms");
  _pose_timestamp_ms_buffer.initialize(_config.pose_buffer_size, 1, 1,
                                       "pose_timestamp_ms");
  _wrench_timestamp_ms_buffer.initialize(_config.wrench_buffer_size, 1, 1,
                                         "wrench_timestamp_ms");
  _waypoints_timestamp_ms_buffer.initialize(-1, 1, 1, "waypoints_timestamp_ms");
  _stiffness_timestamp_ms_buffer.initialize(-1, 1, 1, "stiffness_timestamp_ms");

  // kickstart the threads
  _ctrl_flag_running = true;
  std::cout << "[ManipServer] Starting the threads.\n";
  _rgb_thread = std::thread(&ManipServer::rgb_loop, this, std::ref(time0));
  if (_config.run_low_dim_thread) {
    _low_dim_thread =
        std::thread(&ManipServer::low_dim_loop, this, std::ref(time0));
  }

  if (_config.plot_rgb) {
    // pause 1s, then start the rgb plot thread
    std::this_thread::sleep_for(std::chrono::seconds(1));
    _rgb_plot_thread = std::thread(&ManipServer::rgb_plot_loop, this);
  }

  // wait for threads to be ready
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (_state_low_dim_thread_ready && _state_rgb_thread_ready) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "[ManipServer] All threads are ready." << std::endl;
  std::cout << "[ManipServer] Done initialization." << std::endl;
  return true;
}

void ManipServer::join_threads() {
  std::cout << "[ManipServer]: Waiting for threads to join." << std::endl;
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }

  // join the threads
  std::cout << "[ManipServer]: Waiting for rgb thread to join." << std::endl;
  _rgb_thread.join();
  std::cout << "[ManipServer]: Waiting for low dim thread to join."
            << std::endl;
  _low_dim_thread.join();

  if (_config.plot_rgb) {
    std::cout << "[ManipServer]: Waiting for plotting thread to join."
              << std::endl;
    _rgb_plot_thread.join();
  }

  std::cout << "[ManipServer]: Threads have joined. Exiting." << std::endl;
}

bool ManipServer::is_ready() {
  {
    std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtx);
    if (!_camera_rgb_buffer.is_full()) {
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock2(_pose_buffer_mtx);
    if (!_pose_buffer.is_full()) {
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock3(_wrench_buffer_mtx);
    if (!_wrench_buffer.is_full()) {
      return false;
    }
  }
  return true;
}

bool ManipServer::is_running() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  return _ctrl_flag_running;
}

const Eigen::MatrixXd ManipServer::get_camera_rgb(int k) {
  std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtx);
  _camera_rgb_timestamps_ms = _camera_rgb_timestamp_ms_buffer.get_last_k(k);
  return _camera_rgb_buffer.get_last_k(k);
}

const Eigen::MatrixXd ManipServer::get_wrench(int k) {
  std::lock_guard<std::mutex> lock(_wrench_buffer_mtx);
  _wrench_timestamps_ms = _wrench_timestamp_ms_buffer.get_last_k(k);
  return _wrench_buffer.get_last_k(k);
}

const Eigen::MatrixXd ManipServer::get_pose(int k) {
  std::lock_guard<std::mutex> lock(_pose_buffer_mtx);
  _pose_timestamps_ms = _pose_timestamp_ms_buffer.get_last_k(k);
  return _pose_buffer.get_last_k(k);
}

const Eigen::VectorXd ManipServer::get_camera_rgb_timestamps_ms() {
  return _camera_rgb_timestamps_ms;
}
const Eigen::VectorXd ManipServer::get_wrench_timestamps_ms() {
  return _wrench_timestamps_ms;
}
const Eigen::VectorXd ManipServer::get_pose_timestamps_ms() {
  return _pose_timestamps_ms;
}

double ManipServer::get_timestamp_now_ms() {
  return _timer.toc_ms();
}

void ManipServer::set_high_level_maintain_position() {
  // clear existing targets
  clear_cmd_buffer();

  // get the current pose as the only new target
  RUT::Vector7d pose_fb;
  if (!_config.mock_hardware) {
    robot_ptr->getCartesian(pose_fb);  // use the current pose as the reference
  }
  set_target_pose(pose_fb, 200);
  set_target_pose(pose_fb, 1000);

  // wait for > 100ms before turn on high stiffness
  // So that the internal target in the interpolation controller gets refreshed
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  {
    std::lock_guard<std::mutex> lock(_controller_mtx);
    // set the robot to have high stiffness, but still compliant
    controller.setStiffnessMatrix(_stiffness_high);
  }
}

void ManipServer::set_high_level_free_jogging() {
  std::lock_guard<std::mutex> lock(_controller_mtx);
  // set the robot to be compliant
  controller.setStiffnessMatrix(_stiffness_low);
}

void ManipServer::set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                                  double dt_in_future_ms) {
  {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtx);
    _waypoints_buffer.put(pose);
    _waypoints_timestamp_ms_buffer.put(_timer.toc_ms() +
                                       dt_in_future_ms);  // 1s in the future
  }
}

void ManipServer::set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af) {
  std::lock_guard<std::mutex> lock(_controller_mtx);
  controller.setForceControlledAxis(Tr, n_af);
}

void ManipServer::set_stiffness_matrix(const RUT::Matrix6d& stiffness) {
  std::lock_guard<std::mutex> lock(_controller_mtx);
  controller.setStiffnessMatrix(stiffness);
}

void ManipServer::clear_cmd_buffer() {
  {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtx);
    _waypoints_buffer.clear();
    _waypoints_timestamp_ms_buffer.clear();
  }
  {
    std::lock_guard<std::mutex> lock(_stiffness_buffer_mtx);
    _stiffness_buffer.clear();
    _stiffness_timestamp_ms_buffer.clear();
  }
}

/*
  1. Points in _waypoints_buffer are not yet scheduled to be executed. 
  2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
    and interpolate them to generate a trajectory.
  3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
    a. remove input waypoints that are in the past.
    b. remove existing waypoints that are newer than input waypoints.
    c. Adds remaining of a to the end of b.
*/
// #define DEBUG_WP_SCHEDULING
void ManipServer::schedule_waypoints(const Eigen::MatrixXd& waypoints,
                                     const Eigen::VectorXd& timepoints_ms) {
  double curr_time = _timer.toc_ms();
  // check the shape of inputs
  if (waypoints.rows() != 7) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints should have 7 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (timepoints_ms.size() != waypoints.cols()) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints and "
                 "timepoints_ms should have the same "
                 "number of columns. Exiting."
              << std::endl;
    return;
  }

#ifdef DEBUG_WP_SCHEDULING
  std::cout << "[ManipServer][schedule_waypoints] waypoints: \n"
            << waypoints << std::endl;
  std::cout << "[ManipServer][schedule_waypoints] timepoints_ms: \n"
            << timepoints_ms.transpose() << std::endl;
  std::cout << "[ManipServer][schedule_waypoints] curr_time: " << curr_time
            << std::endl;
#endif
  /*
   * a. Get rid of input waypoints that are in the past
   */
  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    // all input points are in the past. Do nothing.
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtx);
    /*
   * b. Get rid of existing waypoints that are newer than input waypoints
   */
    int existing_id_end = 0;
    for (int i = 0; i < _waypoints_timestamp_ms_buffer.size(); i++) {
      if (_waypoints_timestamp_ms_buffer[i] > timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _waypoints_buffer.remove_last_k(_waypoints_buffer.size() - existing_id_end);
    _waypoints_timestamp_ms_buffer.remove_last_k(
        _waypoints_timestamp_ms_buffer.size() - existing_id_end);
    assert(_waypoints_buffer.size() == _waypoints_timestamp_ms_buffer.size());

    /*
   * c. Add remaining of a to the end of b
   */
    int input_id_end = timepoints_ms.size();
#ifdef DEBUG_WP_SCHEDULING
    std::cout << "[ManipServer][schedule_waypoints] input_id_start: "
              << input_id_start << std::endl;
    std::cout << "[ManipServer][schedule_waypoints] input_id_end: "
              << input_id_end << std::endl;
#endif
    for (int i = input_id_start; i < input_id_end; i++) {
#ifdef DEBUG_WP_SCHEDULING
      std::cout << "[ManipServer][schedule_waypoints] Adding waypoint: "
                << waypoints.col(i).transpose()
                << " at time: " << timepoints_ms(i) << std::endl;
#endif
      _waypoints_buffer.put(waypoints.col(i));
      _waypoints_timestamp_ms_buffer.put(timepoints_ms(i));
    }
  }
}  // end function schedule_waypoints

/*
  1. Points in _waypoints_buffer are not yet scheduled to be executed. 
  2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
    and interpolate them to generate a trajectory.
  3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
    a. remove input waypoints that are in the past.
    b. remove existing waypoints that are newer than input waypoints.
    c. Adds remaining of a to the end of b.
*/
// #define DEBUG_STIFFNESS_SCHEDULING
void ManipServer::schedule_stiffness(const Eigen::MatrixXd& stiffnesses,
                                     const Eigen::VectorXd& timepoints_ms) {
  double curr_time = _timer.toc_ms();
  // check the shape of inputs
  if (stiffnesses.rows() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have 6 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (stiffnesses.cols() / timepoints_ms.size() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have "
                 "6x number of columns as timepoints_ms. Exiting."
              << std::endl;
    return;
  }

#ifdef DEBUG_STIFFNESS_SCHEDULING
  std::cout << "[ManipServer][schedule_stiffness] stiffnesses: \n"
            << stiffnesses << std::endl;
  std::cout << "[ManipServer][schedule_stiffness] timepoints_ms: \n"
            << timepoints_ms.transpose() << std::endl;
  std::cout << "[ManipServer][schedule_stiffness] curr_time: " << curr_time
            << std::endl;
#endif
  /*
   * a. Get rid of inputs that are in the past
   */
  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    // all input points are in the past. Do nothing.
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_stiffness_buffer_mtx);
    /*
     * b. Get rid of existing stiffness that are newer than input stiffness
     */
    int existing_id_end = 0;
    for (int i = 0; i < _stiffness_timestamp_ms_buffer.size(); i++) {
      if (_stiffness_timestamp_ms_buffer[i] > timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _stiffness_buffer.remove_last_k(_stiffness_buffer.size() - existing_id_end);
    _stiffness_timestamp_ms_buffer.remove_last_k(
        _stiffness_timestamp_ms_buffer.size() - existing_id_end);
    assert(_stiffness_buffer.size() == _stiffness_timestamp_ms_buffer.size());
    /*
     * c. Add remaining of a to the end of b
     */
    int input_id_end = timepoints_ms.size();
#ifdef DEBUG_STIFFNESS_SCHEDULING
    std::cout << "[ManipServer][schedule_stiffness] input_id_start: "
              << input_id_start << std::endl;
    std::cout << "[ManipServer][schedule_stiffness] input_id_end: "
              << input_id_end << std::endl;
#endif
    for (int i = input_id_start; i < input_id_end; i++) {
#ifdef DEBUG_STIFFNESS_SCHEDULING
      std::cout << "[ManipServer][schedule_stiffness] Adding stiffness:\n"
                << stiffnesses.middleCols<6>(6 * i)
                << " at time: " << timepoints_ms(i) << std::endl;
#endif
      _stiffness_buffer.put(stiffnesses.middleCols<6>(i * 6));
      _stiffness_timestamp_ms_buffer.put(timepoints_ms(i));
    }
  }
}  // end function schedule_stiffness

void ManipServer::start_saving_data_for_a_new_episode() {
  // create episode folders
  auto [rgb_folder_name, json_file_name] =
      create_folder_for_new_episode(_config.data_folder);
  std::cout << "[main] New episode. rgb_folder_name: " << rgb_folder_name
            << std::endl;

  // get rgb folder and low dim json file for saving data
  _ctrl_rgb_folder = rgb_folder_name;
  _ctrl_low_dim_data_stream.open(json_file_name);

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_saving = true;
  }
}

void ManipServer::stop_saving_data() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  _ctrl_flag_saving = false;
}

bool ManipServer::is_saving_data() {
  return _state_low_dim_thread_saving || _state_rgb_thread_saving;
}