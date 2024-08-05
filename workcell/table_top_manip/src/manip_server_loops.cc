#include "manip_server.h"

#include <RobotUtilities/interpolation_controller.h>
#include <opencv2/core/eigen.hpp>

#include "helpers.hpp"

void ManipServer::low_dim_loop(const RUT::TimePoint& time0) {
  std::cout << "[ManipServer][low dim thread] starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::Vector7d pose_fb;
  RUT::Vector7d pose_target_waypoint;
  RUT::Vector7d pose_force_control_ref;
  RUT::Vector7d pose_rdte_cmd;
  // The following two initial values are used in mock hardware mode
  pose_fb << 0, 0, 0, 1, 0, 0, 0;
  pose_rdte_cmd = pose_fb;

  RUT::Vector6d wrench_fb;
  RUT::Vector6d wrench_fb_ur, wrench_WTr;
  RUT::Matrix6d stiffness;

  if (!_config.mock_hardware) {
    robot_ptr->getCartesian(pose_fb);
    // wait for FT300 to be ready
    std::cout << "[ManipServer][low dim thread] Waiting for FT300 to start "
                 "streaming.\n";
    while (!robotiq.is_data_ready()) {
      usleep(100000);
    }
  }

  pose_force_control_ref = pose_fb;
  wrench_WTr.setZero();

  bool ctrl_flag_saving = false;

  RUT::InterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_low_dim_thread_ready = true;
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
      t_start = robot_ptr->rtde_init_period();
      robot_ptr->getCartesian(pose_fb);
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_pose_buffer_mtx);
        _pose_buffer.put(pose_fb);
        _pose_timestamp_ms_buffer.put(time_now_ms);
      }
      robotiq.getWrenchNetTool(pose_fb, wrench_fb);
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtx);
        _wrench_buffer.put(wrench_fb);
        _wrench_timestamp_ms_buffer.put(time_now_ms);
      }
      robot_ptr->getWrenchTool(
          wrench_fb_ur);  // not used outside this loop, so no need to lock
    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pose_fb = pose_rdte_cmd;
      {
        std::lock_guard<std::mutex> lock(_pose_buffer_mtx);
        _pose_buffer.put(pose_fb);
        _pose_timestamp_ms_buffer.put(time_now_ms);
      }
      wrench_fb.setZero();
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtx);
        _wrench_buffer.put(wrench_fb);
        _wrench_timestamp_ms_buffer.put(time_now_ms);
      }
      wrench_fb_ur = wrench_fb;
    }

    // update control target from interpolation controller
    if (!intp_controller.get_control(time_now_ms, pose_force_control_ref)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_waypoints_buffer_mtx);
        while (!_waypoints_buffer.is_empty()) {
          // keep querying buffer until we get a target that is in the future
          pose_target_waypoint = _waypoints_buffer.pop();
          double target_time_ms = _waypoints_timestamp_ms_buffer.pop();
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
      intp_controller.get_control(time_now_ms, pose_force_control_ref);
    }
    // std::cout << "[debug] time: " << time_now_ms << ", pose_force_control_ref: "
    //           << pose_force_control_ref.transpose() << std::endl;

    // update stiffness matrix from buffer

    // condition:
    //   time_now_ms < time[0], do nothing
    //   time_now_ms >= time[0], look for next
    bool new_stiffness_found = false;
    {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtx);
      if (!_stiffness_buffer.is_empty()) {
        double next_available_time_ms = _stiffness_timestamp_ms_buffer[0];
        if (time_now_ms > next_available_time_ms) {
          new_stiffness_found = true;
          while ((!_stiffness_timestamp_ms_buffer.is_empty()) &&
                 (_stiffness_timestamp_ms_buffer[0] < time_now_ms)) {
            stiffness = _stiffness_buffer.pop();
            next_available_time_ms = _stiffness_timestamp_ms_buffer.pop();
          }
        }
      }
    }

    // Update the controller
    {
      std::lock_guard<std::mutex> lock(_controller_mtx);
      controller.setRobotStatus(pose_fb, wrench_fb_ur);
      // Update robot reference
      controller.setRobotReference(pose_force_control_ref, wrench_WTr);

      // Update stiffness matrix
      if (new_stiffness_found) {
        controller.setStiffnessMatrix(stiffness);
      }
      // Compute the control output
      controller.step(pose_rdte_cmd);
    }

    if ((!_config.mock_hardware) &&
        (!robot_ptr->streamCartesian(pose_rdte_cmd))) {
      std::cout << "[low dim thread] streamCartesian failed. Ending thread."
                << std::endl;
      break;
    }

    // std::cout << "t = " << timer.toc_ms()
    //           << ", wrench: " << wrench_fb.transpose() << std::endl;

    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << "[low dim thread] Start saving low dim data." << std::endl;
        json_file_start(_ctrl_low_dim_data_stream);
        ctrl_flag_saving = true;
      }

      _state_low_dim_thread_saving = true;
      save_low_dim_data_json(_ctrl_low_dim_data_stream, _state_low_dim_seq_id,
                             timer.toc_ms(), pose_fb, wrench_fb);
      json_frame_ending(_ctrl_low_dim_data_stream);
      _state_low_dim_seq_id++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << "[low dim thread] Stop saving low dim data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_low_dim_data_json(_ctrl_low_dim_data_stream, _state_low_dim_seq_id,
                               timer.toc_ms(), pose_fb, wrench_fb);
        json_file_ending(_ctrl_low_dim_data_stream);
        _ctrl_low_dim_data_stream.close();
        ctrl_flag_saving = false;
        _state_low_dim_thread_saving = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[low dim thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    if (_config.mock_hardware) {
      mock_loop_timer.sleep_till_next();
    } else {
      robot_ptr->rtde_wait_period(t_start);
    }
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[low dim thread] Joined." << std::endl;
}

void ManipServer::rgb_loop(const RUT::TimePoint& time0) {
  std::cout << "[ManipServer][rgb thread] starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);
  double time_start = timer.toc_ms();
  cv::Mat color_mat;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_rgb_thread_ready = true;
  }

  while (true) {
    if (!_config.mock_hardware) {
      color_mat = camera_ptr->next_rgb_frame_blocking();
    } else {
      // mock hardware
      color_mat = cv::Mat::zeros(1080, 1080, CV_8UC3);
      usleep(20 * 1000);  // 20ms, 50hz
    }
    double time_now_ms = timer.toc_ms();
    cv::split(color_mat, _bgr);  //split source
    cv::cv2eigen(_bgr[0], _bm);
    cv::cv2eigen(_bgr[1], _gm);
    cv::cv2eigen(_bgr[2], _rm);
    _rgb_row_combined.resize(color_mat.rows * 3, color_mat.cols);
    _rgb_row_combined << _rm, _gm, _bm;
    {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtx);
      _camera_rgb_buffer.put(_rgb_row_combined);
      _camera_rgb_timestamp_ms_buffer.put(time_now_ms);
    }

    if (_ctrl_flag_saving) {
      _state_rgb_thread_saving = true;
      save_rgb_data(_ctrl_rgb_folder, _state_rgb_seq_id, timer.toc_ms(),
                    color_mat);
      _state_rgb_seq_id++;
    } else {
      _state_rgb_thread_saving = false;
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
  std::cout << "[ManipServer][plot thread] starting thread.\n";
  cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
  Eigen::MatrixXd rgb_separate_eigen;
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtx);
      rgb_separate_eigen = _camera_rgb_buffer.get_last_k(1);
    }
    // convert eigen matrix to cv::Mat
    cv::Mat color_mat;
    // TODO: finish this conversion
    std::cerr << "[rgb plot thread] Conversion from Eigen to cv::Mat is not "
                 "implemented yet."
              << std::endl;
    // cv::eigen2cv(rgb_separate_eigen, color_mat);
    cv::imshow("RGB", color_mat);

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
