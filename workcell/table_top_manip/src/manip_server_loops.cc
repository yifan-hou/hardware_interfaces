#include "table_top_manip/manip_server.h"

#include <RobotUtilities/interpolation_controller.h>
#include <opencv2/core/eigen.hpp>

#include "helpers.hpp"

void ManipServer::robot_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][Robot thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::Vector7d pose_fb;
  RUT::Vector7d pose_target_waypoint;
  RUT::Vector6d vel_fb;
  RUT::Vector7d force_control_ref_pose;
  RUT::Vector7d pose_rdte_cmd;
  // The following two initial values are used in mock hardware mode
  pose_fb << id, 0, 0, 1, 0, 0, 0;
  pose_rdte_cmd = pose_fb;
  vel_fb << 0, 0, 0, 0, 0, 0;

  RUT::Vector6d wrench_fb_ur, wrench_WTr;
  RUT::Matrix6d stiffness;

  // TODO: use base pointer robot_ptr instead of URRTDE
  //       Need to create interfaces for all used functions here in RobotInterfaces
  URRTDE* urrtde_ptr;

  if (!_config.mock_hardware) {
    urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
    urrtde_ptr->getCartesian(pose_fb);
  }

  force_control_ref_pose = pose_fb;
  wrench_WTr.setZero();

  bool ctrl_flag_saving = false;  // local copy

  RUT::InterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());
  std::cout << header << "intp_controller initialized with pose_fb: "
            << pose_fb.transpose() << std::endl;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_robot_thread_ready[id] = true;
  }

  RUT::Profiler loop_profiler;
  std::cout << header << "Loop started." << std::endl;

  RUT::Timer mock_loop_timer;
  mock_loop_timer.set_loop_rate_hz(500);
  mock_loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    loop_profiler.start();
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      // real hardware
      t_start = urrtde_ptr->rtde_init_period();
      urrtde_ptr->getCartesian(pose_fb);
      urrtde_ptr->getCartesianVelocity(vel_fb);
      urrtde_ptr->getWrenchTool(wrench_fb_ur);
      time_now_ms = timer.toc_ms();
      loop_profiler.stop("compute");
      loop_profiler.start();
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        _poses_fb[id] = pose_fb;
      }
      loop_profiler.stop("lock");
      loop_profiler.start();

    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pose_fb = pose_rdte_cmd;
      vel_fb.setZero();
      wrench_fb_ur.setZero();
    }
    // buffer robot pose
    loop_profiler.stop("compute");
    loop_profiler.start();
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      _pose_buffers[id].put(pose_fb);
      _pose_timestamp_ms_buffers[id].put(time_now_ms);
    }
    {
      std::lock_guard<std::mutex> lock(_vel_buffer_mtxs[id]);
      _vel_buffers[id].put(vel_fb);
      _vel_timestamp_ms_buffers[id].put(time_now_ms);
    }
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      _robot_wrench_buffers[id].put(wrench_fb_ur);
      _robot_wrench_timestamp_ms_buffers[id].put(time_now_ms);
    }
    loop_profiler.stop("lock");
    loop_profiler.start();

    // update control target from interpolation controller
    if (!intp_controller.get_control(time_now_ms, force_control_ref_pose)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
        while (!_waypoints_buffers[id].is_empty()) {
          // keep querying buffer until we get a target that is in the future
          pose_target_waypoint = _waypoints_buffers[id].pop();
          double target_time_ms = _waypoints_timestamp_ms_buffers[id].pop();
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(pose_target_waypoint,
                                           target_time_ms);
            new_wp_found = true;
            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
        //           << ", target_time_ms:" << target_time_ms
        //           << ", pose_target_waypoint: "
        //           << pose_target_waypoint.transpose() << std::endl;
        intp_controller.keep_the_last_target(time_now_ms);
      }
      intp_controller.get_control(time_now_ms, force_control_ref_pose);
    }

    loop_profiler.stop("intp_controller");
    loop_profiler.start();

    // update stiffness matrix from buffer

    // condition:
    //   time_now_ms < time[0], do nothing
    //   time_now_ms >= time[0], look for next
    bool new_stiffness_found = false;
    {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
      if (!_stiffness_buffers[id].is_empty()) {
        double next_available_time_ms = _stiffness_timestamp_ms_buffers[id][0];
        if (time_now_ms > next_available_time_ms) {
          new_stiffness_found = true;
          while ((!_stiffness_timestamp_ms_buffers[id].is_empty()) &&
                 (_stiffness_timestamp_ms_buffers[id][0] < time_now_ms)) {
            stiffness = _stiffness_buffers[id].pop();
            next_available_time_ms = _stiffness_timestamp_ms_buffers[id].pop();
          }
        }
      }
    }
    loop_profiler.stop("stiffness");
    loop_profiler.start();

    wrench_WTr.setZero();

    // std::cout << "[debug] time: " << time_now_ms
    //           << ", wrench_fb_ur: " << wrench_fb_ur.transpose()
    //           << ", wrench_WTr: " << wrench_WTr.transpose() << std::endl;

    // Update the compliance controller
    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      loop_profiler.stop("controller_lock");
      loop_profiler.start();
      _controllers[id].setRobotStatus(pose_fb, wrench_fb_ur);
      // Update robot reference
      _controllers[id].setRobotReference(force_control_ref_pose, wrench_WTr);

      // Update stiffness matrix
      if (new_stiffness_found) {
        _controllers[id].setStiffnessMatrix(stiffness);
      }
      loop_profiler.stop("controller_set");
      loop_profiler.start();
      // Compute the control output
      _controllers[id].step(pose_rdte_cmd);
      loop_profiler.stop("controller_step");
      loop_profiler.start();
    }

    // Send control command to the robot
    if ((!_config.mock_hardware) &&
        (!urrtde_ptr->streamCartesian(pose_rdte_cmd))) {
      std::cout << header << "streamCartesian failed. Ending thread."
                << std::endl;
      std::cout << header << "last pose_fb: " << pose_fb.transpose()
                << std::endl;
      std::cout << header << "last wrench_fb_ur: " << wrench_fb_ur.transpose()
                << std::endl;
      std::cout << header << "last force_control_ref_pose: "
                << force_control_ref_pose.transpose() << std::endl;
      std::cout << header << "last pose_rdte_cmd: " << pose_rdte_cmd.transpose()
                << std::endl;
      break;
    }

    // std::cout << "t = " << timer.toc_ms()
    //           << ", pose_rdte_cmd: " << pose_rdte_cmd.transpose() << std::endl;

    // logging
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << "[robot thread] Start saving low dim data." << std::endl;
        json_file_start(_ctrl_robot_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_robot_thread_saving[id] = true;
      save_robot_data_json(_ctrl_robot_data_streams[id],
                           _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                           false);
      json_frame_ending(_ctrl_robot_data_streams[id]);
      _states_robot_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << "[robot thread] Stop saving low dim data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_robot_data_json(_ctrl_robot_data_streams[id],
                             _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                             false);
        json_file_ending(_ctrl_robot_data_streams[id]);
        _ctrl_robot_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_robot_thread_saving[id] = false;
      }
    }

    loop_profiler.stop("logging");
    loop_profiler.start();

    // loop control
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[robot thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    loop_profiler.stop("lock");

    // loop timing and overrun check
    if (_config.mock_hardware) {
      mock_loop_timer.sleep_till_next();
    } else {
      double overrun_ms = mock_loop_timer.check_for_overrun_ms(false);
      if (overrun_ms > 0) {
        std::cout << "\033[33m";  // set color to bold yellow
        std::cout << header << "Overrun: " << overrun_ms << "ms" << std::endl;
        std::cout << "\033[0m";  // reset color to default
        loop_profiler.show();
      }
      urrtde_ptr->rtde_wait_period(t_start);
      mock_loop_timer.check_for_overrun_ms(
          false);  // just call it to reset the timer
    }
    loop_profiler.clear();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[robot thread] Joined." << std::endl;
}

void ManipServer::eoat_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][EoAT thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::VectorXd pos_fb = RUT::VectorXd::Zero(1);
  RUT::Vector2d eoat_target_waypoint;
  RUT::Vector2d eoat_cmd;
  if (!_config.mock_hardware) {
    eoat_ptrs[id]->getJoints(pos_fb);
  }
  eoat_cmd << pos_fb, 0;

  bool ctrl_flag_saving = false;  // local copy

  RUT::InterpolationController intp_controller;
  intp_controller.initialize(eoat_cmd, timer.toc_ms());
  std::cout << header
            << "intp_controller initialized with pos_fb: " << pos_fb.transpose()
            << std::endl;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_eoat_thread_ready[id] = true;
  }

  std::cout << header << "Loop started." << std::endl;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(100);
  loop_timer.start_timed_loop();
  while (true) {
    // Update EoAT status (for query and logging)
    double time_now_ms;
    if (!_config.mock_hardware) {
      // real hardware
      eoat_ptrs[id]->getJoints(pos_fb);
      time_now_ms = timer.toc_ms();
    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pos_fb[0] = eoat_cmd[0];
    }
    // save state to eoat fb buffer
    {
      std::lock_guard<std::mutex> lock(_eoat_buffer_mtxs[id]);
      _eoat_buffers[id].put(pos_fb);
      _eoat_timestamp_ms_buffers[id].put(time_now_ms);
    }

    // update control target from interpolation controller
    if (!intp_controller.get_control(time_now_ms, eoat_cmd)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_eoat_waypoints_buffer_mtxs[id]);
        while (!_eoat_waypoints_buffers[id].is_empty()) {
          // keep querying buffer until we get a target that is in the future
          eoat_target_waypoint = _eoat_waypoints_buffers[id].pop();
          double target_time_ms =
              _eoat_waypoints_timestamp_ms_buffers[id].pop();
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(eoat_target_waypoint,
                                           target_time_ms);
            new_wp_found = true;
            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
        //           << ", target_time_ms:" << target_time_ms
        //           << ", eoat_target_waypoint: "
        //           << eoat_target_waypoint.transpose() << std::endl;
        intp_controller.keep_the_last_target(time_now_ms);
      }
      intp_controller.get_control(time_now_ms, eoat_cmd);
    }

    // Send command to EoAT
    double force_fb = 0;
    {
      std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
      // TODO: currently, assuming the grasping force is captured by Z axis of the first wrench sensor.
      // Need to find a better way to specify it.
      force_fb = _wrench_fb[id][2];
    }
    eoat_cmd[1] -= force_fb;
    if ((!_config.mock_hardware) && (!eoat_ptrs[id]->setJointsPosForce(
                                        eoat_cmd.head(1), eoat_cmd.tail(1)))) {
      std::cout << header << "setJointsPosForce failed. Ending thread."
                << std::endl;
      std::cout << header << "last pos_fb: " << pos_fb.transpose() << std::endl;
      std::cout << header << "last eoat_cmd: " << eoat_cmd.transpose()
                << std::endl;
      break;
    }

    // std::cout << "t = " << timer.toc_ms()
    //           << ", eoat_cmd: " << eoat_cmd.transpose() << std::endl;

    // logging
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << header << "Start saving eoat data." << std::endl;
        json_file_start(_ctrl_eoat_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_eoat_thread_saving[id] = true;
      save_eoat_data_json(_ctrl_eoat_data_streams[id], _states_eoat_seq_id[id],
                          timer.toc_ms(), pos_fb);
      json_frame_ending(_ctrl_eoat_data_streams[id]);
      _states_eoat_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << header << "Stop saving eoat data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_eoat_data_json(_ctrl_eoat_data_streams[id],
                            _states_eoat_seq_id[id], timer.toc_ms(), pos_fb);
        json_file_ending(_ctrl_eoat_data_streams[id]);
        _ctrl_eoat_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_eoat_thread_saving[id] = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header
                  << "_ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    loop_timer.sleep_till_next();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[EoAT thread] Joined." << std::endl;
}

void ManipServer::wrench_loop(const RUT::TimePoint& time0, int publish_rate,
                              int id) {
  std::string header =
      "[ManipServer][Wrench thread] " + std::to_string(id) + ": ";
  std::cout << header << "thread starting." << std::endl;
  std::cout << header << "Rate at" << publish_rate << "Hz." << std::endl;
  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  int num_ft_sensors = force_sensor_ptrs[id]->getNumSensors();
  RUT::VectorXd wrench_fb;

  if (!_config.mock_hardware) {
    // wait for force sensor to be ready
    std::cout << header
              << "Waiting for force sensor to start "
                 "streaming.\n";
    while (!force_sensor_ptrs[id]->is_data_ready()) {
      usleep(100000);
    }
  }

  // wait for pose_fb to be ready
  std::cout << header
            << "Waiting for robot thread to "
               "populate pose_fb. \n";
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      if (_pose_buffers[id].size() > 0) {
        break;
      }
    }
    usleep(300 * 1000);  // 300ms
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_wrench_thread_ready[id] = true;
  }

  std::cout << header << "Loop started." << std::endl;
  bool ctrl_flag_saving = false;  // local copy

  RUT::Vector7d pose_fb;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(publish_rate);
  loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      // get the most recent tool pose (for static calibration)
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        pose_fb = _poses_fb[id];
      }
      int safety_flag =
          force_sensor_ptrs[id]->getWrenchNetTool(pose_fb, wrench_fb);
      if (safety_flag < 0) {
        std::cout << header
                  << "Wrench is above safety threshold. Ending thread."
                  << std::endl;
        break;
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
      {
        std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
        _wrench_fb[id] = wrench_fb;
      }
    } else {
      // mock hardware
      wrench_fb.setZero(num_ft_sensors * 6);
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
    }

    // logging
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << "[wrench thread] Start saving wrench data." << std::endl;
        json_file_start(_ctrl_wrench_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_wrench_thread_saving[id] = true;
      save_wrench_data_json(_ctrl_wrench_data_streams[id],
                            _states_wrench_seq_id[id], timer.toc_ms(),
                            wrench_fb);
      json_frame_ending(_ctrl_wrench_data_streams[id]);
      _states_wrench_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << "[wrench thread] Stop saving wrench data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_wrench_data_json(_ctrl_wrench_data_streams[id],
                              _states_wrench_seq_id[id], timer.toc_ms(),
                              wrench_fb);
        json_file_ending(_ctrl_wrench_data_streams[id]);
        _ctrl_wrench_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_wrench_thread_saving[id] = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[wrench thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }
    loop_timer.sleep_till_next();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[wrench thread] Joined." << std::endl;
}

void ManipServer::rgb_loop(const RUT::TimePoint& time0, int id) {
  std::string header = "[ManipServer][rgb thread] " + std::to_string(id) + ": ";
  std::cout << header << "starting thread" << std::endl;

  RUT::Timer timer;
  timer.tic(time0);
  double time_start = timer.toc_ms();
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_rgb_thread_ready[id] = true;
  }
  std::cout << header << "Loop started." << std::endl;

  cv::Mat resized_color_mat;
  cv::Mat bgr[3];  //destination array
  Eigen::MatrixXd bm, gm, rm;
  Eigen::MatrixXd rgb_row_combined;

  while (true) {
    double time_now_ms = 0;
    {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      if (!_config.mock_hardware) {
        _color_mats[id] = camera_ptrs[id]->next_rgb_frame_blocking();
      } else {
        // mock hardware
        _color_mats[id] = cv::Mat::zeros(1080, 1080, CV_8UC3);
        usleep(20 * 1000);  // 20ms, 50hz
      }
      time_now_ms = timer.toc_ms();
      cv::resize(_color_mats[id], resized_color_mat,
                 cv::Size(_config.output_rgb_hw[1], _config.output_rgb_hw[0]),
                 cv::INTER_LINEAR);
      cv::split(resized_color_mat, bgr);  //split source
    }

    cv::cv2eigen(bgr[0], bm);
    cv::cv2eigen(bgr[1], gm);
    cv::cv2eigen(bgr[2], rm);
    rgb_row_combined.resize(resized_color_mat.rows * 3, resized_color_mat.cols);
    rgb_row_combined << rm, gm, bm;
    {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
      _camera_rgb_buffers[id].put(rgb_row_combined);
      _camera_rgb_timestamp_ms_buffers[id].put(time_now_ms);
    }

    if (_ctrl_flag_saving) {
      _states_rgb_thread_saving[id] = true;
      {
        std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
        save_rgb_data(_ctrl_rgb_folders[id], _states_rgb_seq_id[id],
                      timer.toc_ms(), _color_mats[id]);
      }
      _states_rgb_seq_id[id]++;
    } else {
      _states_rgb_thread_saving[id] = false;
    }

    // std::cout << "t = " << timer.toc_ms() << ", get new rgb frame."
    //           << std::endl;
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[rgb thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;

        break;
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[rgb thread] Joined." << std::endl;
}

void ManipServer::rgb_plot_loop() {
  std::string header = "[ManipServer][plot thread]: ";
  std::cout << header << "starting thread." << std::endl;
  cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
  std::vector<cv::Mat> color_mat_copy;
  cv::Mat canvas;

  for (int id : _id_list) {
    color_mat_copy.push_back(cv::Mat());
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_plot_thread_ready = true;
  }

  std::cout << header << "Loop started." << std::endl;

  while (true) {
    for (int id : _id_list) {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      color_mat_copy[id] = _color_mats[id].clone();
    }

    cv::vconcat(color_mat_copy, canvas);

    cv::imshow("RGB", canvas);

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header
                  << "[rgb plot thread] _ctrl_flag_running is false. Shuting "
                     "down this thread"
                  << std::endl;
        break;
      }
    }

    if (cv::waitKey(30) >= 0)
      break;
  }
  std::cout << "[plot thread] Joined." << std::endl;
}
