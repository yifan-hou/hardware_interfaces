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

void *ATI_Monitor(void *pParam) {
  ATINetft *netft_hardware = (ATINetft *)pParam;

  netft_rdt_driver::WrenchData data;
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(netft_hardware->_publish_rate);
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
    } else {
      std::cout << "\033[1;31m[ATINetft] Time out\033[0m\n";
      status_ok = false;
    }

    if (netft_hardware->_print_flag) {
      netft_hardware->_file << loop_timer.toc_ms() << "\t";
      stream_array_in(netft_hardware->_file, netft_hardware->_force, 3);
      stream_array_in(netft_hardware->_file, netft_hardware->_torque, 3);
      netft_hardware->_file << std::endl;
    }

    loop_timer.sleep_till_next();
  }
}

ATINetft::ATINetft() {
  _force = new double[3];
  _force_old = new double[3];
  _WrenchSafety = new double[6];
  _torque = new double[3];
  _torque_old = new double[3];
  _stall_counts = 0;

  for (int i = 0; i < 3; ++i) {
    _force_old[i] = 0;
    _torque_old[i] = 0;
  }
}

bool ATINetft::init(RUT::TimePoint time0, const ATINetftConfig &config) {
  std::cout << "[ATINetft] initializing.." << std::endl;
  _time0 = time0;

  _print_flag = config.print_flag;
  _adj_sensor_tool = SE32Adj(pose2SE3(config.PoseSensorTool));

  _netft = std::shared_ptr<netft_rdt_driver::NetFTRDTDriver>(
      new netft_rdt_driver::NetFTRDTDriver(config.ip_address));

  // open file
  if (_print_flag) {
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

int ATINetft::getWrenchSensor(double *wrench) {
  wrench[0] = _force[0];
  wrench[1] = _force[1];
  wrench[2] = _force[2];
  wrench[3] = _torque[0];
  wrench[4] = _torque[1];
  wrench[5] = _torque[2];

  double data_change = fabs(wrench[0] - _force_old[0]);
  data_change += fabs(wrench[1] - _force_old[1]);
  data_change += fabs(wrench[2] - _force_old[2]);
  data_change += 10 * fabs(wrench[3] - _torque_old[0]);
  data_change += 10 * fabs(wrench[4] - _torque_old[1]);
  data_change += 10 * fabs(wrench[5] - _torque_old[2]);

  _force_old[0] = wrench[0];
  _force_old[1] = wrench[1];
  _force_old[2] = wrench[2];
  _torque_old[0] = wrench[3];
  _torque_old[1] = wrench[4];
  _torque_old[2] = wrench[5];
  // cout << "         data_change: " << data_change << endl;

  if (data_change > 0.01) {
    _stall_counts = 0;
  } else {
    _stall_counts++;
    if (_stall_counts >= 10) {
      std::cout << "\033[1;31m[ATINetft] Dead Stream\033[0m\n";
      return 2;
    }
  }

  return 0;
}

int ATINetft::getWrenchTool(double *wrench) {
  double wrench_sensor[6];
  int flag = getWrenchSensor(wrench_sensor);

  // transform from sensor frame to tool frame
  Eigen::Matrix<double, 6, 1> wrench_T, wrench_S;
  wrench_S[0] = wrench_sensor[0];
  wrench_S[1] = wrench_sensor[1];
  wrench_S[2] = wrench_sensor[2];
  wrench_S[3] = wrench_sensor[3];
  wrench_S[4] = wrench_sensor[4];
  wrench_S[5] = wrench_sensor[5];

  wrench_T = _adj_sensor_tool.transpose() * wrench_S;
  wrench[0] = wrench_T[0];
  wrench[1] = wrench_T[1];
  wrench[2] = wrench_T[2];
  wrench[3] = wrench_T[3];
  wrench[4] = wrench_T[4];
  wrench[5] = wrench_T[5];
  return flag;
}

int ATINetft::getWrenchNetTool(const double *pose, double *wrench_net_T) {
  double wrench_T[6];
  int flag = getWrenchTool(wrench_T);

  // compensate for the weight of object
  Quaterniond q(pose[3], pose[4], pose[5], pose[6]);
  Vector3d GinF = q.normalized().toRotationMatrix().transpose() * _Gravity;
  Vector3d GinT = _Pcom.cross(GinF);
  wrench_net_T[0] = wrench_T[0] + _Foffset[0] - GinF[0];
  wrench_net_T[1] = wrench_T[1] + _Foffset[1] - GinF[1];
  wrench_net_T[2] = wrench_T[2] + _Foffset[2] - GinF[2];

  wrench_net_T[3] = wrench_T[3] + _Toffset[0] - GinT[0];
  wrench_net_T[4] = wrench_T[4] + _Toffset[1] - GinT[1];
  wrench_net_T[5] = wrench_T[5] + _Toffset[2] - GinT[2];

  // safety
  for (int i = 0; i < 6; ++i) {
    if (abs(wrench_net_T[i]) > _WrenchSafety[i]) return 3;
  }

  return flag;
}

ATINetft::~ATINetft() {
  delete[] _force;
  delete[] _torque;
  delete[] _WrenchSafety;
  _netft.reset();
  if (_print_flag) _file.close();
}
