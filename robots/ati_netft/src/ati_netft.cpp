#include "ati_netft/ati_netft.h"

#include <sched.h>

#include <RobotUtilities/utilities.h>

typedef std::chrono::high_resolution_clock Clock;
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

  std::cout << "Thread priority is " << sch.sched_priority << std::endl;
}

void* ATI_Monitor(void* pParam) {
  ATINetft *netft_hardware = (ATINetft*)pParam;

  geometry_msgs::WrenchStamped data;
  ros::Rate pub_rate(netft_hardware->_publish_rate);
  ros::Duration diag_pub_duration(1.0);

  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time last_diag_pub_time(ros::Time::now());

  set_realtime_priority();

  while (ros::ok()) {
    if (netft_hardware->_netft->waitForNewData()) {
      netft_hardware->_netft->getData(data);

      // read data
      netft_hardware->_force[0]  = data.wrench.force.x;
      netft_hardware->_force[1]  = data.wrench.force.y;
      netft_hardware->_force[2]  = data.wrench.force.z;
      netft_hardware->_torque[0] = data.wrench.torque.x;
      netft_hardware->_torque[1] = data.wrench.torque.y;
      netft_hardware->_torque[2] = data.wrench.torque.z;
      data.header.frame_id       = netft_hardware->_frame_id;
      netft_hardware->_pub.publish(data);
    } else {
      cout << "\033[1;31m[ATINetft] Time out\033[0m\n";
    }

    Clock::time_point timenow_clock = Clock::now();
    double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - netft_hardware->_time0).count())/1e6; // milli second

    if (netft_hardware->_print_flag) {
      netft_hardware->_file << timenow << "\t";
      stream_array_in(netft_hardware->_file, netft_hardware->_force, 3);
      stream_array_in(netft_hardware->_file, netft_hardware->_torque, 3);
      netft_hardware->_file << endl;
    }

    // ros::Time current_time(ros::Time::now());
    // if ( (current_time - last_diag_pub_time) > diag_pub_duration ) {
    //   diag_array.status.clear();
    //   netft_hardware->_netft->diagnostics(diag_status);
    //   diag_array.status.push_back(diag_status);
    //   diag_array.header.stamp = ros::Time::now();
    //   netft_hardware->_diag_pub.publish(diag_array);
    //   last_diag_pub_time = current_time;
    // }
    ros::spinOnce();
    pub_rate.sleep();
  }
}

ATINetft::ATINetft() {
  _force  = new double[3];
  _force_old  = new double[3];
  _WrenchSafety = new double[6];
  _torque = new double[3];
  _torque_old = new double[3];
  _stall_counts = 0;

  for (int i = 0; i < 3; ++i) {
    _force_old[i] = 0;
    _torque_old[i] = 0;
  }
}

bool ATINetft::init(ros::NodeHandle& root_nh, Clock::time_point time0) {
  using namespace hardware_interface;
  cout << "[ATINetft] initializing..\n";
  _time0 = time0;

  // Get parameters from the server
  string ip_address, sensor_name, fullpath;
  double PoseSensorTool[7];
  root_nh.param(string("/netft/ip_address"), ip_address, string("192.168.1.1"));
  root_nh.param(string("/netft/sensor_name"), sensor_name, string("netft"));
  root_nh.param(string("/netft/frame_id"), _frame_id, string("base_link"));
  root_nh.param(string("/netft/publish_rate"), _publish_rate, 100.0);
  root_nh.param(string("/netft/print_flag"), _print_flag, false);
  root_nh.param(string("/netft/file_path"), fullpath, string(" "));
  root_nh.param(std::string("/ftsensor/offset/fx"), _Foffset[0], 0.0);
  root_nh.param(std::string("/ftsensor/offset/fy"), _Foffset[1], 0.0);
  root_nh.param(std::string("/ftsensor/offset/fz"), _Foffset[2], 0.0);
  root_nh.param(std::string("/ftsensor/offset/tx"), _Toffset[0], 0.0);
  root_nh.param(std::string("/ftsensor/offset/ty"), _Toffset[1], 0.0);
  root_nh.param(std::string("/ftsensor/offset/tz"), _Toffset[2], 0.0);
  root_nh.param(std::string("/ftsensor/gravity/x"), _Gravity[0], 0.0);
  root_nh.param(std::string("/ftsensor/gravity/y"), _Gravity[1], 0.0);
  root_nh.param(std::string("/ftsensor/gravity/z"), _Gravity[2], 0.0);
  root_nh.param(std::string("/ftsensor/COM/x"), _Pcom[0], 0.0);
  root_nh.param(std::string("/ftsensor/COM/y"), _Pcom[1], 0.0);
  root_nh.param(std::string("/ftsensor/COM/z"), _Pcom[2], 0.0);
  root_nh.param(std::string("/ftsensor/safety/fx"), _WrenchSafety[0], 0.0);
  root_nh.param(std::string("/ftsensor/safety/fy"), _WrenchSafety[1], 0.0);
  root_nh.param(std::string("/ftsensor/safety/fz"), _WrenchSafety[2], 0.0);
  root_nh.param(std::string("/ftsensor/safety/tx"), _WrenchSafety[3], 0.0);
  root_nh.param(std::string("/ftsensor/safety/ty"), _WrenchSafety[4], 0.0);
  root_nh.param(std::string("/ftsensor/safety/tz"), _WrenchSafety[5], 0.0);
  root_nh.param(std::string("/ftsensor/transform_sensor_to_tool/x"), PoseSensorTool[0], 0.0);
  root_nh.param(std::string("/ftsensor/transform_sensor_to_tool/y"), PoseSensorTool[1], 0.0);
  root_nh.param(std::string("/ftsensor/transform_sensor_to_tool/z"), PoseSensorTool[2], 0.0);
  root_nh.param(std::string("/ftsensor/transform_sensor_to_tool/qw"), PoseSensorTool[3], 1.0);
  root_nh.param(std::string("/ftsensor/transform_sensor_to_tool/qx"), PoseSensorTool[4], 0.0);
  root_nh.param(std::string("/ftsensor/transform_sensor_to_tool/qy"), PoseSensorTool[5], 0.0);
  root_nh.param(std::string("/ftsensor/transform_sensor_to_tool/qz"), PoseSensorTool[6], 0.0);

  if (!root_nh.hasParam("/netft/ip_address"))
    ROS_WARN_STREAM("Parameter [/netft/ip_address] not found");
  if (!root_nh.hasParam("/netft/sensor_name"))
    ROS_WARN_STREAM("Parameter [/netft/sensor_name] not found");
  if (!root_nh.hasParam("/netft/frame_id"))
    ROS_WARN_STREAM("Parameter [/netft/frame_id] not found");
  if (!root_nh.hasParam("/netft/publish_rate"))
    ROS_WARN_STREAM("Parameter [/netft/publish_rate] not found");
  if (!root_nh.hasParam("/netft/print_flag"))
    ROS_WARN_STREAM("Parameter [/netft/print_flag] not found");
  if (!root_nh.hasParam("/netft/file_path"))
    ROS_WARN_STREAM("Parameter [/netft/file_path] not found");
  if (!root_nh.hasParam("/ftsensor/offset"))
    ROS_WARN_STREAM("Parameter [/ftsensor/offset] not found");
  if (!root_nh.hasParam("/ftsensor/gravity"))
    ROS_WARN_STREAM("Parameter [/ftsensor/gravity] not found");
  if (!root_nh.hasParam("/ftsensor/COM"))
    ROS_WARN_STREAM("Parameter [/ftsensor/COM] not found");
  if (!root_nh.hasParam("/ftsensor/safety"))
    ROS_WARN_STREAM("Parameter [/ftsensor/safety] not found");
  if (!root_nh.hasParam("/ftsensor/transform_sensor_to_tool"))
    ROS_WARN_STREAM("Parameter [/ftsensor/transform_sensor_to_tool] not found");

  _adj_sensor_tool = SE32Adj(pose2SE3(PoseSensorTool));

  _netft = shared_ptr<netft_rdt_driver::NetFTRDTDriver>(new netft_rdt_driver::NetFTRDTDriver(ip_address));

  // Setup publishers
  cout << "[ATINetft] setting up ROS message publishing..\n";
  _pub      = root_nh.advertise<geometry_msgs::WrenchStamped>(sensor_name + "/data", 100);
  _diag_pub = root_nh.advertise<diagnostic_msgs::DiagnosticArray>(sensor_name + "/diagnostics", 2);

  // open file
  if (_print_flag)
  {
    _file.open(fullpath);
    if (_file.is_open())
      ROS_INFO_STREAM("[ATINetft] file opened successfully." << endl);
    else
      ROS_ERROR_STREAM("[ATINetft] Failed to open file." << endl);
  }

  cout << "[ATINetft] Creating thread for callback..\n";
  // create thread
  int rc = pthread_create(&_thread, NULL, ATI_Monitor, this);
  if (rc){
    ROS_ERROR_STREAM("[ATINetft] ATI Netft Hardware initialization error: unable to create thread.\n");
    return false;
  }

  ROS_INFO_STREAM("[ATINetft] Initialized successfully.\n");
  return true;
}

int ATINetft::getWrenchSensor(double *wrench)
{
  wrench[0] = _force[0];
  wrench[1] = _force[1];
  wrench[2] = _force[2];
  wrench[3] = _torque[0];
  wrench[4] = _torque[1];
  wrench[5] = _torque[2];

  double data_change =  fabs(wrench[0] - _force_old[0]);
  data_change        += fabs(wrench[1] - _force_old[1]);
  data_change        += fabs(wrench[2] - _force_old[2]);
  data_change        += 10*fabs(wrench[3] - _torque_old[0]);
  data_change        += 10*fabs(wrench[4] - _torque_old[1]);
  data_change        += 10*fabs(wrench[5] - _torque_old[2]);

  _force_old[0]  = wrench[0];
  _force_old[1]  = wrench[1];
  _force_old[2]  = wrench[2];
  _torque_old[0] = wrench[3];
  _torque_old[1] = wrench[4];
  _torque_old[2] = wrench[5];
  // cout << "         data_change: " << data_change << endl;

  if (data_change > 0.05) {
    _stall_counts = 0;
  } else {
    _stall_counts ++;
    if (_stall_counts >= 5) {
      cout << "\033[1;31m[ATINetft] Dead Stream\033[0m\n";
      return 2;
    }
  }

  return 0;
}

int ATINetft::getWrenchTool(double *wrench){
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

  wrench_T  = _adj_sensor_tool.transpose()*wrench_S;
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
  Vector3d GinF = q.normalized().toRotationMatrix().transpose()*_Gravity;
  Vector3d GinT = _Pcom.cross(GinF);
  wrench_net_T[0] = wrench_T[0] + _Foffset[0] - GinF[0];
  wrench_net_T[1] = wrench_T[1] + _Foffset[1] - GinF[1];
  wrench_net_T[2] = wrench_T[2] + _Foffset[2] - GinF[2];

  wrench_net_T[3] = wrench_T[3] + _Toffset[0] - GinT[0];
  wrench_net_T[4] = wrench_T[4] + _Toffset[1] - GinT[1];
  wrench_net_T[5] = wrench_T[5] + _Toffset[2] - GinT[2];

  // safety
  for (int i = 0; i < 6; ++i) {
    if(abs(wrench_net_T[i]) >_WrenchSafety[i])
      return 3;
  }

  return flag;
}

ATINetft::~ATINetft(){
  delete [] _force;
  delete [] _torque;
  delete [] _WrenchSafety;
  _netft.reset();
  if (_print_flag)
    _file.close();
}
