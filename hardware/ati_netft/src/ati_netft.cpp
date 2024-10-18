#include "ati_netft/ati_netft.h"

#include <sched.h>

using namespace RUT;

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

void* ATI_Monitor(void* pParam) {
  ATINetft* netft_hardware = (ATINetft*)pParam;

  netft_rdt_driver::WrenchData data;
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(netft_hardware->_config.publish_rate);
  loop_timer.start_timed_loop();
  loop_timer.tic();

  set_realtime_priority();
  bool status_ok = true;
  while (status_ok) {
    if (netft_hardware->_netft->waitForNewData()) {
      netft_hardware->_netft->getData(data);

      // read data
      netft_hardware->_force[0] = data.fx;
      netft_hardware->_force[1] = data.fy;
      netft_hardware->_force[2] = data.fz;
      netft_hardware->_torque[0] = data.tx;
      netft_hardware->_torque[1] = data.ty;
      netft_hardware->_torque[2] = data.tz;

      netft_hardware->_flag_started = true;
    } else {
      std::cout << "\033[1;31m[ATINetft] Time out\033[0m\n";
      status_ok = false;
    }

    if (netft_hardware->_config.print_flag) {
      netft_hardware->_file << loop_timer.toc_ms() << "\t";
      stream_array_in(netft_hardware->_file, netft_hardware->_force, 3);
      stream_array_in(netft_hardware->_file, netft_hardware->_torque, 3);
      netft_hardware->_file << std::endl;
    }

    loop_timer.sleep_till_next();
  }
}

ATINetft::ATINetft() {
  _force = RUT::Vector3d::Zero();
  _force_old = RUT::Vector3d::Zero();
  _torque = RUT::Vector3d::Zero();
  _torque_old = RUT::Vector3d::Zero();
  _stall_counts = 0;
}

bool ATINetft::init(RUT::TimePoint time0, const ATINetftConfig& config) {
  std::cout << "[ATINetft] initializing connection to " << config.ip_address
            << std::endl;
  _time0 = time0;
  _config = config;
  _flag_started = false;

  _adj_sensor_tool = SE32Adj(pose2SE3(config.PoseSensorTool));

  _netft = std::shared_ptr<netft_rdt_driver::NetFTRDTDriver>(
      new netft_rdt_driver::NetFTRDTDriver(config.ip_address,
                                           config.counts_per_force,
                                           config.counts_per_torque));

  // open file
  if (_config.print_flag) {
    _file.open(config.fullpath);
    if (_file.is_open())
      std::cout << "[ATINetft] file opened successfully." << std::endl;
    else
      std::cerr << "[ATINetft] Failed to open file." << std::endl;
  }

  std::cout << "[ATINetft] Creating thread for callback.." << std::endl;
  // create thread
  int rc = pthread_create(&_thread, NULL, ATI_Monitor, this);
  if (rc) {
    std::cerr << "[ATINetft] ATI Netft Hardware initialization error: unable "
                 "to create thread."
              << std::endl;
    return false;
  }

  std::cout << "[ATINetft] Initialized successfully." << std::endl;
  return true;
}

int ATINetft::getWrenchSensor(RUT::Vector6d& wrench) {
  wrench.head(3) = _force;
  wrench.tail(3) = _torque;

  double data_change = (wrench.head(3) - _force_old).norm() +
                       10 * (wrench.tail(3) - _torque_old).norm();

  _force_old = _force;
  _torque_old = _torque;

  if (data_change > _config.noise_level) {
    _stall_counts = 0;
  } else {
    _stall_counts++;
    if (_stall_counts >= _config.stall_threshold) {
      std::cout << "\033[1;31m[ATINetft] Dead Stream\033[0m\n";
      return 2;
    }
  }

  // safety
  for (int i = 0; i < 6; ++i) {
    if (abs(wrench[i]) > _config.WrenchSafety[i]) {
      std::cout << "\033[1;31m[ATINetft] Force magnitude is above the safety "
                   "threshold:\033[0m\n";
      std::cout << "  feedback:" << wrench.transpose() << std::endl;
      std::cout << "  safety limit: " << _config.WrenchSafety.transpose()
                << std::endl;
      return -1;
    }
  }

  return 0;
}

int ATINetft::getWrenchTool(RUT::Vector6d& wrench_T) {
  int flag = getWrenchSensor(_wrench_sensor_temp);
  wrench_T = _adj_sensor_tool.transpose() * _wrench_sensor_temp;
  return flag;
}

int ATINetft::getWrenchNetTool(const RUT::Vector7d& pose,
                               RUT::Vector6d& wrench_net_T) {
  int flag = getWrenchTool(_wrench_tool_temp);

  // compensate for the weight of object
  _R_WT = RUT::quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  _GinF = _R_WT.transpose() * _config.Gravity;
  _GinT = _config.Pcom.cross(_GinF);
  wrench_net_T.head(3) = _wrench_tool_temp.head(3) + _config.Foffset - _GinF;
  wrench_net_T.tail(3) = _wrench_tool_temp.tail(3) + _config.Toffset - _GinT;

  return flag;
}

ATINetft::~ATINetft() {
  _netft.reset();
  if (_config.print_flag)
    _file.close();
}
