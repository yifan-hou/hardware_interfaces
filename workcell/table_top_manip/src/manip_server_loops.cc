#include "table_top_manip/manip_server.h"

#include <RobotUtilities/interpolation_controller.h>
#include <opencv2/core/eigen.hpp>

#include "helpers.hpp"

void ManipServer::robot_loop(const RUT::TimePoint& time0, int id) {
  std::cout << "[ManipServer][Robot thread] starting thread for id " << id
            << ".\n";

  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::Vector7d pose_fb;
  RUT::Vector7d pose_target_waypoint;
  RUT::Vector7d force_control_ref_pose;
  RUT::Vector7d pose_rdte_cmd;
  // The following two initial values are used in mock hardware mode
  pose_fb << 0, 0, 0, 1, 0, 0, 0;
  pose_rdte_cmd = pose_fb;

  RUT::Vector6d wrench_fb_ur, wrench_WTr;
  RUT::Matrix6d stiffness;

  // TODO: use base pointer robot_ptr instead of URRTDE
  //       Need to create interfaces for all used functions here in RobotInterfaces
  URRTDE* urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());

  if (!_config.mock_hardware) {
    urrtde_ptr->getCartesian(pose_fb);
    // wait for FT300 to be ready
    std::cout << "[ManipServer][robot thread] Waiting for FT300 to start "
                 "streaming.\n";
  }

  force_control_ref_pose = pose_fb;
  wrench_WTr.setZero();

  bool ctrl_flag_saving = false;  // local copy

  bool perturbation_is_applied = false;
  RUT::Vector6d perturbation;

  RUT::InterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_robot_thread_ready[id] = true;
  }

  RUT::Timer mock_loop_timer;
  mock_loop_timer.set_loop_rate_hz(500);
  mock_loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      // real hardware
      t_start = urrtde_ptr->rtde_init_period();
      urrtde_ptr->getCartesian(pose_fb);
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        _poses_fb[id] = pose_fb;
      }
      urrtde_ptr->getWrenchTool(
          wrench_fb_ur);  // not used outside this loop, so no need to lock
    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pose_fb = pose_rdte_cmd;
      wrench_fb_ur.setZero();
    }
    // buffer robot pose
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      _pose_buffers[id].put(pose_fb);
      _pose_timestamp_ms_buffers[id].put(time_now_ms);
    }

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
    // std::cout << "[debug] time: " << time_now_ms << ", force_control_ref_pose: "
    //           << force_control_ref_pose.transpose() << std::endl;

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

    // apply perturbation
    wrench_WTr.setZero();
    perturbation.setZero();
    if (_config.use_perturbation_generator) {
      perturbation_is_applied =
          _perturbation_generators[id].generate_perturbation(perturbation);
      wrench_WTr += perturbation;
    }

    // Update the controller
    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      _controllers[id].setRobotStatus(pose_fb, wrench_fb_ur);
      // Update robot reference
      // std::cout << "debug: wrench_WTr: " << wrench_WTr.transpose() << std::endl;
      _controllers[id].setRobotReference(force_control_ref_pose, wrench_WTr);

      // Update stiffness matrix
      if (new_stiffness_found) {
        _controllers[id].setStiffnessMatrix(stiffness);
      }
      // Compute the control output
      _controllers[id].step(pose_rdte_cmd);
    }

    if ((!_config.mock_hardware) &&
        (!urrtde_ptr->streamCartesian(pose_rdte_cmd))) {
      std::cout << "[robot thread] streamCartesian failed. Ending thread."
                << std::endl;
      break;
    }

    // std::cout << "t = " << timer.toc_ms()
    //           << ", wrench: " << wrench_fb.transpose() << std::endl;

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
                           perturbation_is_applied);
      json_frame_ending(_ctrl_robot_data_streams[id]);
      _states_robot_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << "[robot thread] Stop saving low dim data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_robot_data_json(_ctrl_robot_data_streams[id],
                             _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                             perturbation_is_applied);
        json_file_ending(_ctrl_robot_data_streams[id]);
        _ctrl_robot_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_robot_thread_saving[id] = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[robot thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    if (_config.mock_hardware) {
      mock_loop_timer.sleep_till_next();
    } else {
      urrtde_ptr->rtde_wait_period(t_start);
    }
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[robot thread] Joined." << std::endl;
}

void ManipServer::wrench_loop(const RUT::TimePoint& time0, int publish_rate,
                              int id) {
  std::cout << "[ManipServer][wrench thread] starting thread for id " << id
            << std::endl;
  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::Vector6d wrench_fb;

  if (!_config.mock_hardware) {
    // wait for force sensor to be ready
    std::cout
        << "[ManipServer][wrench thread] Waiting for force sensor to start "
           "streaming.\n";
    while (!force_sensor_ptrs[id]->is_data_ready()) {
      usleep(100000);
    }
  }

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
      force_sensor_ptrs[id]->getWrenchNetTool(pose_fb, wrench_fb);
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
    } else {
      // mock hardware
      wrench_fb.setZero();
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
  std::cout << "[ManipServer][rgb thread] starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);
  double time_start = timer.toc_ms();

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_rgb_thread_ready[id] = true;
  }

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
      cv::split(_color_mats[id], bgr);  //split source
    }

    cv::cv2eigen(bgr[0], bm);
    cv::cv2eigen(bgr[1], gm);
    cv::cv2eigen(bgr[2], rm);
    rgb_row_combined.resize(_color_mats[id].rows * 3, _color_mats[id].cols);
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

void ManipServer::rgb_plot_loop(int id) {
  std::cout << "[ManipServer][plot thread] starting thread.\n";
  cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
  cv::Mat color_mat_copy;
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      color_mat_copy = _color_mats[id].clone();
    }
    cv::imshow("RGB", color_mat_copy);

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[rgb plot thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    if (cv::waitKey(30) >= 0)
      break;
  }
  std::cout << "[plot thread] Joined." << std::endl;
}
