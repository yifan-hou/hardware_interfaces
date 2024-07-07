#include "robotiq_ft_modbus/robotiq_ft_modbus.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
extern "C" {
#include "rq_int.h"
#include "rq_sensor_com.h"
#include "rq_sensor_state.h"
#include "rq_thread.h"
}

using namespace RUT;

/**
 * \fn static void wait_for_other_connection()
 * \brief Function who wait for another connection
 */
static void wait_for_other_connection() {
  INT_8 ret;
  struct timespec tim;
  tim.tv_sec = 1;
  tim.tv_nsec = 0L;

  while (1) {
    nanosleep(&tim, (struct timespec*)NULL);
    ret = rq_sensor_state();
    if (ret == 0) {
      break;
    }
  }
}

/**
 * \fn void get_data(void)
 * \brief Function to retrieve the power applied to the sensor
 * \param chr_return String to return forces applied
 */
static void get_data(INT_8* chr_return) {
  INT_8 i;
  INT_8 floatData[50];
  for (i = 0; i < 6; i++) {
    sprintf(floatData, "%f", rq_state_get_received_data(i));
    if (i == 0) {
      strcpy(chr_return, "( ");
      strcat(chr_return, floatData);
    } else {
      strcat(chr_return, " , ");
      strcat(chr_return, floatData);
    }
    if (i == 5) {
      strcat(chr_return, " )");
    }
  }
}

static void get_data(RUT::Vector6d& data) {
  for (INT_8 i = 0; i < 6; i++) {
    data(i) = rq_state_get_received_data(i);
  }
}

void set_realtime_priority() {
  int ret;
  // We'll operate on the currently running thread.
  pthread_t this_thread = pthread_self();

  sched_param sch;
  int policy;
  pthread_getschedparam(this_thread, &policy, &sch);
  sch.sched_priority = 99;
  if (pthread_setschedparam(this_thread, SCHED_FIFO, &sch)) {
    std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
  }

  std::cout << "[Netft] Thread priority is " << sch.sched_priority << std::endl;
}

void* ft_Monitor(void* pParam) {
  RobotiqFTModbus* robotiq_ft_modbus = (RobotiqFTModbus*)pParam;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(robotiq_ft_modbus->_config.publish_rate);
  loop_timer.start_timed_loop();
  loop_timer.tic();

  /**
   * robotiq specific initializations
   */
  std::cout << "[RobotiqFTModbus] Initializing data thread.." << std::endl;
  //IF can't connect with the sensor wait for another connection
  INT_8 ret = rq_sensor_state();
  if (ret == -1) {
    wait_for_other_connection();
  }

  std::cout << "[RobotiqFTModbus] Connected to the sensor at t = "
            << loop_timer.toc_ms() << std::endl;
  //Read high-level informations
  ret = rq_sensor_state();
  if (ret == -1) {
    wait_for_other_connection();
  }
  std::cout << "[RobotiqFTModbus] Read high level state at t = "
            << loop_timer.toc_ms() << std::endl;

  //Initialize connection with the client
  ret = rq_sensor_state();
  if (ret == -1) {
    wait_for_other_connection();
  }
  std::cout << "[RobotiqFTModbus] Finished initializing connection at t = "
            << loop_timer.toc_ms() << std::endl;

  set_realtime_priority();
  bool status_ok = true;
  RUT::Vector6d data;
  while (status_ok) {
    ret = rq_sensor_state();
    if (ret == -1) {
      wait_for_other_connection();
    }
    if (rq_sensor_get_current_state() == RQ_STATE_RUN) {
      // read data
      get_data(data);
      robotiq_ft_modbus->_mutex.lock();
      robotiq_ft_modbus->_force[0] = data[0];
      robotiq_ft_modbus->_force[1] = data[1];
      robotiq_ft_modbus->_force[2] = data[2];
      robotiq_ft_modbus->_torque[0] = data[3];
      robotiq_ft_modbus->_torque[1] = data[4];
      robotiq_ft_modbus->_torque[2] = data[5];
      robotiq_ft_modbus->_mutex.unlock();
      // std::cout << "t = " << loop_timer.toc_ms()
      //           << ",\tft data: " << data.transpose() << std::endl;
      if (robotiq_ft_modbus->_config.print_flag) {
        robotiq_ft_modbus->_file << loop_timer.toc_ms() << "\t";
        stream_array_in(robotiq_ft_modbus->_file, robotiq_ft_modbus->_force, 3);
        stream_array_in(robotiq_ft_modbus->_file, robotiq_ft_modbus->_torque,
                        3);
        robotiq_ft_modbus->_file << std::endl;
      }
      robotiq_ft_modbus->_flag_started = true;
    }
    loop_timer.sleep_till_next();
  }
}

RobotiqFTModbus::RobotiqFTModbus() {
  _force = RUT::Vector3d::Zero();
  _force_old = RUT::Vector3d::Zero();
  _WrenchSafety = RUT::Vector6d::Zero();
  _torque = RUT::Vector3d::Zero();
  _torque_old = RUT::Vector3d::Zero();
  _stall_counts = 0;
}

bool RobotiqFTModbus::init(RUT::TimePoint time0,
                           const RobotiqFTModbusConfig& config) {
  std::cout << "[RobotiqFTModbus] scanning available ttyUSB devices.."
            << std::endl;
  _time0 = time0;
  _config = config;

  _adj_sensor_tool = SE32Adj(pose2SE3(config.PoseSensorTool));
  _flag_started = false;

  // open file
  if (_config.print_flag) {
    _file.open(config.fullpath);
    if (_file.is_open())
      std::cout << "[RobotiqFTModbus] file opened successfully." << std::endl;
    else
      std::cerr << "[RobotiqFTModbus] Failed to open file." << std::endl;
  }

  std::cout << "[RobotiqFTModbus] Creating thread for data streaming.."
            << std::endl;
  // create thread
  int rc = pthread_create(&_thread, NULL, ft_Monitor, this);
  if (rc) {
    std::cerr
        << "[RobotiqFTModbus] Robotiq FT driver initialization error: unable "
           "to create thread."
        << std::endl;
    return false;
  }

  std::cout << "[RobotiqFTModbus] Initialized successfully." << std::endl;
  return true;
}

int RobotiqFTModbus::getWrenchSensor(RUT::Vector6d& wrench) {
  _mutex.lock();
  wrench.head(3) = _force;
  wrench.tail(3) = _torque;

  double data_change = (wrench.head(3) - _force_old).norm() +
                       10 * (wrench.tail(3) - _torque_old).norm();

  _force_old = _force;
  _torque_old = _torque;
  _mutex.unlock();

  if (data_change > _config.noise_level) {
    _stall_counts = 0;
  } else {
    _stall_counts++;
    if (_stall_counts >= _config.stall_threshold) {
      std::cout << "\033[1;31m[RobotiqFTModbus] Dead Stream\033[0m\n";
      return 2;
    }
  }

  return 0;
}

int RobotiqFTModbus::getWrenchTool(RUT::Vector6d& wrench_T) {
  int flag = getWrenchSensor(_wrench_sensor_temp);
  wrench_T = _adj_sensor_tool.transpose() * _wrench_sensor_temp;
  return flag;
}

int RobotiqFTModbus::getWrenchNetTool(const RUT::Vector7d& pose,
                                      RUT::Vector6d& wrench_net_T) {
  int flag = getWrenchTool(_wrench_tool_temp);

  // compensate for the weight of object
  _R_WT = RUT::quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  _GinF = _R_WT.transpose() * _Gravity;
  _GinT = _Pcom.cross(_GinF);
  wrench_net_T.head(3) = _wrench_tool_temp.head(3) + _Foffset - _GinF;
  wrench_net_T.tail(3) = _wrench_tool_temp.tail(3) + _Toffset - _GinT;

  // safety
  for (int i = 0; i < 6; ++i) {
    if (abs(wrench_net_T[i]) > _WrenchSafety[i])
      return 3;
  }

  return flag;
}

RobotiqFTModbus::~RobotiqFTModbus() {
  if (_config.print_flag)
    _file.close();
}
