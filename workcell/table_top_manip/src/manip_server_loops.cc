#include "table_top_manip/manip_server.h"

#include <RobotUtilities/interpolation_controller.h>
#include <opencv2/core/eigen.hpp>

#include <fcntl.h>        // for key loop
#include <linux/input.h>  // for key loop
#include <stdlib.h>       // for key loop
#include <unistd.h>       // for key loop

#include "helpers.hpp"

#include <cmath>
#include <vector>
#include <opencv2/imgproc.hpp>

// Key code for 'a' (use 'a' lowercase)
#define KEY_CODE_A 30

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

  int num_ft_sensors = 1;
  if (!_config.mock_hardware)
    num_ft_sensors = force_sensor_ptrs[id]->getNumSensors();

  RUT::Vector6d wrench_fb_ur;  // wrench feedback from ur
  RUT::VectorXd wrench_fb_sensor =
      RUT::VectorXd::Zero(6 * num_ft_sensors);  // wrench feedback from sensor
  RUT::Vector6d wrench_fb;                      // wrench feedback selected
  RUT::Vector6d wrench_WTr;                     // wrench command
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

  bool perturbation_is_applied = false;
  RUT::VectorXd perturbation = RUT::VectorXd::Zero(6);

  RUT::TaskSpaceInterpolationController intp_controller;
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
    mock_loop_timer.tic();
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      // real hardware
      // t_start = urrtde_ptr->rtde_init_period();
      urrtde_ptr->getCartesian(pose_fb);
      urrtde_ptr->getCartesianVelocity(vel_fb);
      urrtde_ptr->getWrenchToolCalibrated(wrench_fb_ur);

      {
        std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
        wrench_fb_sensor = _wrench_fb[id];
      }

      if (_config.compliance_control_force_source ==
          ComplianceControlForceSource::UR) {
        wrench_fb = wrench_fb_ur;
      } else if (_config.compliance_control_force_source ==
                 ComplianceControlForceSource::COINFT) {
        wrench_fb = wrench_fb_sensor.head<6>() + wrench_fb_sensor.tail<6>();
      } else if (_config.compliance_control_force_source ==
                 ComplianceControlForceSource::ATI) {
        wrench_fb = wrench_fb_sensor.head<6>();
      } else {
        std::cerr << header << "Invalid compliance control force source: "
                  << to_string(_config.compliance_control_force_source)
                  << std::endl;
        break;
      }

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
      wrench_fb.setZero();
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
      _robot_wrench_buffers[id].put(wrench_fb);
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
            // std::cout << "[debug] time_now_ms: " << time_now_ms
            //           << ", time now: " << timer.toc_ms()
            //           << ", target_time_ms:" << target_time_ms
            //           << ", pose_target_waypoint: "
            //           << pose_target_waypoint.transpose() << std::endl;

            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] no new wp found at time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
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

    // apply perturbation
    perturbation.setZero();
    if (_config.use_perturbation_generator) {
      perturbation_is_applied =
          _perturbation_generators[id].generate_perturbation(perturbation);
      wrench_fb += perturbation;
    }
    // record perturbation so wrench thread knows it
    {
      std::lock_guard<std::mutex> lock(_perturbation_mtxs[id]);
      _perturbation[id] = perturbation;
    }

    loop_profiler.stop("perturbation");
    loop_profiler.start();

    wrench_WTr.setZero();
    // std::cout << "[debug] time: " << time_now_ms
    //           << ", wrench_fb: " << wrench_fb.transpose()
    //           << ", wrench_WTr: " << wrench_WTr.transpose() << std::endl;

    // Update the compliance controller
    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      loop_profiler.stop("controller_lock");
      loop_profiler.start();
      _controllers[id].setRobotStatus(pose_fb, wrench_fb);
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
      std::cout << header << "last wrench_fb: " << wrench_fb.transpose()
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
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
      if (!ctrl_flag_saving) {
        std::cout << "[robot thread] Start saving low dim data." << std::endl;
        json_file_start(_ctrl_robot_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_robot_thread_saving[id] = true;
      save_robot_data_json(_ctrl_robot_data_streams[id],
                           _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                           wrench_fb_ur, perturbation_is_applied);
      json_frame_ending(_ctrl_robot_data_streams[id]);
      _states_robot_seq_id[id]++;
    } else {
      if (ctrl_flag_saving) {
        std::cout << "[robot thread] Stop saving low dim data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_robot_data_json(_ctrl_robot_data_streams[id],
                             _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                             wrench_fb_ur, perturbation_is_applied);
        json_file_ending(_ctrl_robot_data_streams[id]);
        _ctrl_robot_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_robot_thread_saving[id] = false;
        std::cout << "[robot thread] Low dim data saved." << std::endl;
      }
    }

    loop_profiler.stop("logging");
    loop_profiler.start();

    // loop control
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << "[robot thread] _ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;
      break;
    }

    loop_profiler.stop("lock");

    // loop timing and overrun check
    double overrun_ms = mock_loop_timer.check_for_overrun_ms(false);
    // TODO: this overrun check does not work. Needs to debug
    if (_config.check_robot_loop_overrun && (overrun_ms > 0)) {
      std::cout << "\033[33m";  // set color to bold yellow
      std::cout << header << "Overrun: " << overrun_ms << "ms" << std::endl;
      std::cout << "\033[0m";  // reset color to default
      loop_profiler.show();
    }
    loop_profiler.clear();
    mock_loop_timer.sleep_till_next();
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
  eoat_cmd << pos_fb[0], 0;

  bool ctrl_flag_saving = false;  // local copy

  RUT::JointSpaceInterpolationController intp_controller;
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

    // keyboard interuption of gripper control
    {
      std::lock_guard<std::mutex> lock(_key_mtx);
      if (_key_is_pressed == 2) {
        eoat_cmd[0] = 0;  // close the gripper
      }
      if (_key_is_pressed == 1) {
        eoat_cmd[0] = 110;  // open the gripper
      }
    }

    // Send command to EoAT
    double force_fb = 0;
    // {
    //   std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
    //   // TODO: currently, assuming the grasping force is captured by Z axis of the first wrench sensor.
    //   // Need to find a better way to specify it.
    //   force_fb = _wrench_fb[id][2];
    // }
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

    // std::cout << "deubg: t = " << timer.toc_ms()
    //           << ", eoat_cmd: " << eoat_cmd.transpose() << std::endl;

    // logging
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
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

    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << header
                << "_ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;
      break;
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
  std::cout << header << "Rate at " << publish_rate << "Hz." << std::endl;
  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  int num_ft_sensors = 1;
  if (!_config.mock_hardware) {
    num_ft_sensors = force_sensor_ptrs[id]->getNumSensors();

    std::cout << header << "Number of FT sensors: " << num_ft_sensors
              << std::endl;
    // wait for force sensor to be ready
    std::cout << header
              << "Waiting for force sensor to start "
                 "streaming.\n";
    while (!force_sensor_ptrs[id]->is_data_ready()) {
      usleep(100000);
    }
  }

  RUT::VectorXd wrench_fb = RUT::VectorXd::Zero(6 * num_ft_sensors);
  RUT::VectorXd wrench_fb_filtered = RUT::VectorXd::Zero(6 * num_ft_sensors);

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
  RUT::VectorXd perturbation;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(publish_rate);
  loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    RUT::TimePoint t_start;
    double time_now_ms;

    // read perturbations
    if (num_ft_sensors == 1) {
      std::lock_guard<std::mutex> lock(_perturbation_mtxs[id]);
      perturbation = _perturbation[id];
    }

    if (!_config.mock_hardware) {
      // get the most recent tool pose (for static calibration)
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        pose_fb = _poses_fb[id];
      }
      int safety_flag = force_sensor_ptrs[id]->getWrenchNetTool(
          pose_fb, wrench_fb, num_ft_sensors);
      if (safety_flag < 0) {
        std::cout << header
                  << "Wrench is above safety threshold. Ending thread."
                  << std::endl;
        break;
      }
      if (num_ft_sensors == 1) {
        wrench_fb += perturbation;  // apply perturbation
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_filtered_buffer_mtxs[id]);
        if (num_ft_sensors == 1) {
          wrench_fb_filtered = _wrench_filters[id].step(wrench_fb);
        } else {
          wrench_fb_filtered = wrench_fb;
        }
        _wrench_filtered_buffers[id].put(wrench_fb_filtered);
        _wrench_filtered_timestamp_ms_buffers[id].put(time_now_ms);
      }
      {
        std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
        _wrench_fb[id] = wrench_fb;
      }
    } else {
      // mock hardware
      wrench_fb.setZero(num_ft_sensors * 6);
      if (num_ft_sensors == 1) {
        wrench_fb += perturbation;  // apply perturbation
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
      time_now_ms = timer.toc_ms();
      wrench_fb_filtered.setZero();
      {
        std::lock_guard<std::mutex> lock(_wrench_filtered_buffer_mtxs[id]);
        if (num_ft_sensors == 1) {
          wrench_fb_filtered = _wrench_filters[id].step(wrench_fb);
        } else {
          wrench_fb_filtered = wrench_fb;
        }
        _wrench_filtered_buffers[id].put(wrench_fb_filtered);
        _wrench_filtered_timestamp_ms_buffers[id].put(time_now_ms);
      }
    }

    // logging
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
      if (!ctrl_flag_saving) {
        std::cout << "[wrench thread] Start saving wrench data." << std::endl;
        json_file_start(_ctrl_wrench_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_wrench_thread_saving[id] = true;
      save_wrench_data_json(_ctrl_wrench_data_streams[id],
                            _states_wrench_seq_id[id], timer.toc_ms(),
                            wrench_fb, wrench_fb_filtered);
      json_frame_ending(_ctrl_wrench_data_streams[id]);
      _states_wrench_seq_id[id]++;
    } else {
      if (ctrl_flag_saving) {
        std::cout << "[wrench thread] Stop saving wrench data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_wrench_data_json(_ctrl_wrench_data_streams[id],
                              _states_wrench_seq_id[id], timer.toc_ms(),
                              wrench_fb, wrench_fb_filtered);
        json_file_ending(_ctrl_wrench_data_streams[id]);
        _ctrl_wrench_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_wrench_thread_saving[id] = false;
      }
    }

    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << "[wrench thread] _ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;
      break;
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

  cv::Mat raw, tmp, resized_color_mat;
  cv::Mat bgr[3];
  Eigen::MatrixXd bm, gm, rm;
  Eigen::MatrixXd rgb_row_combined;

  const int ow = _config.output_rgb_hw[1];  // 224
  const int oh = _config.output_rgb_hw[0];  // 224
  // const std::string debug_dir = "/home/zhanyi/hardware_interfaces/debug_images";

  int frame_counter = 0;
  while (true) {
    double time_now_ms = 0;
    {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      if (!_config.mock_hardware) {
        raw = camera_ptrs[id]->next_rgb_frame_blocking();  // **already RGB**
      } else {
        raw = cv::Mat::zeros(oh, ow, CV_8UC3);
        usleep(20 * 1000);  // 20ms, 50hz
      }
      _color_mats[id] = raw;
      time_now_ms     = timer.toc_ms();
    }

    // === resize → center‐crop ===
    int iw = raw.cols, ih = raw.rows;
    int rw, rh;
    int interp = cv::INTER_AREA;
    if (float(iw)/ih >= float(ow)/oh) {
      rh = oh;
      rw = int(std::ceil(float(rh)/ih * iw));
      if (oh > ih) interp = cv::INTER_LINEAR;
    } else {
      rw = ow;
      rh = int(std::ceil(float(rw)/iw * ih));
      if (ow > iw) interp = cv::INTER_LINEAR;
    }
    cv::resize(raw, tmp, cv::Size(rw, rh), 0.0, 0.0, interp);
    int x0 = (rw - ow)/2, y0 = (rh - oh)/2;
    resized_color_mat = tmp(cv::Rect(x0, y0, ow, oh)).clone();

    // // === DEBUG save first 20 frames ===
    // if (frame_counter < 20) {
    //   std::ostringstream oss;
    //   oss << debug_dir
    //       << "/rgb_" << id << "_"
    //       << std::setw(5) << std::setfill('0')
    //       << frame_counter << ".png";
    //   cv::imwrite(oss.str(), resized_color_mat);
    // }
    // ++frame_counter;

    // === pipeline into Eigen buffer (unchanged) ===
    cv::split(resized_color_mat, bgr);
    cv::cv2eigen(bgr[0], bm);
    cv::cv2eigen(bgr[1], gm);
    cv::cv2eigen(bgr[2], rm);
    rgb_row_combined.resize(oh * 3, ow);
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
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << "[rgb thread] _ctrl_flag_running is false. Shuting "
                   "down this thread."
                << std::endl;

      break;
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

    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << header
                << "[rgb plot thread] _ctrl_flag_running is false. Shuting "
                   "down this thread"
                << std::endl;
      break;
    }

    if (cv::waitKey(30) >= 0)
      break;
  }
  std::cout << "[plot thread] Joined." << std::endl;
}

void ManipServer::key_loop(const RUT::TimePoint& time0) {
  std::string header = "[ManipServer][key thread]: ";
  std::cout << header << "starting thread." << std::endl;
  RUT::Timer timer;
  timer.tic(time0);

  struct input_event ev;
  int fd;

  const char* device = _config.key_event_device.c_str();
  std::cout << header << "Using device: " << device << std::endl;
  fd = open(device, O_RDONLY | O_NONBLOCK);
  if (fd == -1) {
    std::cerr << header << "Cannot open input device: " << device << std::endl;
    std::cerr << "Check your key_event_device parameter It should be something "
                 "like /dev/input/eventxx."
              << std::endl;
    std::cerr << "You can find the correct device by running 'ls -l "
                 "/dev/input/by-path/ | grep kbd'."
              << std::endl;
    std::cerr << "If you have permission issue, run 'sudo chmod 777 "
                 "/dev/input/eventxx'."
              << std::endl;
    throw std::runtime_error("Cannot open input device. Exiting key loop.");
  }

  std::cout << header << "Monitoring 'a' key ...\n" << std::endl;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_key_thread_ready = true;
  }

  // Set frame rate to 500Hz
  timer.set_loop_rate_hz(500);

  timer.start_timed_loop();
  std::cout << header << "Loop started." << std::endl;
  bool ctrl_flag_saving = false;  // local copy
  while (true) {
    int key_event =
        -1;  // -1: no event. 0: key is not pressed. 1: key is pressed
    if (std::lock_guard<std::mutex> lock(_ctrl_key_mtx);
        _ctrl_listen_key_event) {
      // Process input events
      while (read(fd, &ev, sizeof(ev)) > 0) {
        if (ev.type != EV_KEY)
          continue;  // Skip non-key events
        for (int kid = 0; kid < _config.keys_to_monitor.size(); kid++) {
          if (ev.code == _config.keys_to_monitor[kid]) {
            if (ev.value == 1) {
              // Key down event
              key_event = 1;
              // save current state
              {
                std::lock_guard<std::mutex> lock(_key_mtx);
                _key_is_pressed = kid + 1;
              }
              _key_is_pressed_delayed = key_event;
              std::cout << "\nKey DOWN detected for key # " << kid + 1
                        << std::endl;
            } else if (ev.value == 0) {
              // Key up event
              key_event = 0;
              // save current state
              {
                std::lock_guard<std::mutex> lock(_key_mtx);
                _key_is_pressed = key_event;
              }
              _last_key_released_time_ms = _key_delayed_timer.toc_ms();
              std::cout << "\nKey UP detected for key # " << kid + 1
                        << std::endl;
            }
            break;  // break when we detect a key to avoid getting values overwritten
          }
        }  // end for kid
      }
    }

    if (_config.take_over_mode) {
      if ((_key_is_pressed_delayed > 0) && (_key_is_pressed == 0) &&
          (_key_delayed_timer.toc_ms() - _last_key_released_time_ms > 1000)) {
        _key_is_pressed_delayed = 0;
      }
      if (key_event == 1) {
        std::cout << "\n===== taking over =====" << std::endl;
        clear_cmd_buffer();
        set_high_level_free_jogging();
      } else if (key_event == 0) {
        std::cout << "\n===== releasing control =====" << std::endl;
        clear_cmd_buffer();  // still need to clear the command buffer
        set_high_level_maintain_position();
      }
    }

    // logging
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); _ctrl_flag_saving) {
      if (!ctrl_flag_saving) {
        std::cout << "[key thread] Start saving key pressing data."
                  << std::endl;
        json_file_start(_ctrl_key_data_stream);
        ctrl_flag_saving = true;
      }

      _state_key_thread_saving = true;
      if (key_event >= 0) {
        // new key event!
        save_key_data_json(_ctrl_key_data_stream, _state_key_seq_id,
                           timer.toc_ms(), key_event);
        json_frame_ending(_ctrl_key_data_stream);
        _state_key_seq_id++;
      }
    } else {
      if (ctrl_flag_saving) {
        std::cout << "[key thread] Stop saving key data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_key_data_json(_ctrl_key_data_stream, _state_key_seq_id,
                           timer.toc_ms(), key_event);
        json_file_ending(_ctrl_key_data_stream);
        _ctrl_key_data_stream.close();
        ctrl_flag_saving = false;
        _state_key_thread_saving = false;
      }
    }
    // Check if the key event thread should stop
    if (std::lock_guard<std::mutex> lock(_ctrl_mtx); !_ctrl_flag_running) {
      std::cout << header
                << "_ctrl_flag_running is false. Shuting "
                   "down this thread"
                << std::endl;
      break;
    }
    timer.sleep_till_next();
  }

  // Close the input device
  close(fd);

  std::cout << header << "Joined." << std::endl;
}
