#include "ur_socket/ur_socket.h"

#include <Eigen/Dense>
#include <RobotUtilities/utilities.h>


using std::cout;
using std::endl;

using RUT::Vector3d;
using RUT::Quaterniond;

URSocket* URSocket::pinstance = 0;


typedef union {
  double d;
  unsigned char bytes[sizeof(double)];
} DOUBLE_UNION;


void ReadXBytes(int socket, unsigned int x, void* buffer) {
    int bytesRead = 0;
    int result;
    while (bytesRead < x) {
        result = read(socket, buffer + bytesRead, x - bytesRead);
        if (result < 1 ){
            // Throw your error.
        }
        bytesRead += result;
    }
}

int buffToInteger(unsigned char * buffer) {
    int a = int((unsigned char)(buffer[0]) << 24 |
                (unsigned char)(buffer[1]) << 16 |
                (unsigned char)(buffer[2]) << 8 |
                (unsigned char)(buffer[3]));
    return a;
}


/**
 * Use this to convert big endian (Network) to little endian (host). You can use
 * the following code to check whether your system is little endian or not:
 *   unsigned int i = 1;
 *   char *c = (char*)&i;
 *   if (*c)
 *      printf("Little endian");
 *   else
 *      printf("Big endian");
 * For big endian system, there is no need to call reverseDouble.
 *
 * @param[in]  data  The byte array
 *
 * @return     The reversed bytes as a double
 */
double reverseDouble(const char *data){
    double result;
    char *dest = (char *)&result;
    for(int i=0; i<sizeof(double); i++)
        dest[i] = data[sizeof(double)-i-1];
    return result;
}

double buffToDouble(uint8_t * buff){
    double value;
    memcpy(&value,buff,sizeof(double));
    return reverseDouble((char *)&value);
}

URSocket* URSocket::Instance() {
  if (pinstance == 0) {
    pinstance = new URSocket();
  }
  return pinstance;
}

URSocket::URSocket() {
  _pose            = new double[7];
  _pose_set        = new double[7];
  _joints          = new double[6];
  _safe_zone       = new double[6];
  _send_buffer     = new char[1000];
  _isInitialized   = false;
  _stop_monitoring = false;
  _safetyMode      = SAFETY_MODE_STOP;
  _operationMode   = OPERATION_MODE_CARTESIAN;
}

URSocket::~URSocket() {
  cout << "[URSocket] finishing.." << endl;
  _stop_monitoring = true;
  _thread.join();

  delete pinstance;
  delete [] _pose;
  delete [] _pose_set;
  delete [] _joints;
  delete [] _safe_zone;
  delete [] _send_buffer;
}

int URSocket::init(ros::NodeHandle& root_nh, Clock::time_point time0) {
  _time0 = time0;
  // read egm parameters from parameter server
  int ur_portnum;
  std::string ur_ip;

  double max_dist_tran, max_dist_rot;
  double safe_zone[6];
  int safety_mode_int;
  int operation_mode_int;

  root_nh.param(std::string("/ur/portnum"), ur_portnum, 30003);
  root_nh.param(std::string("/ur/ip"), ur_ip, std::string("192.168.1.98"));
  root_nh.param(std::string("/ur/t"), _move_para_t, 0.02f);
  root_nh.param(std::string("/ur/lookahead"), _move_para_lookahead, 0.2f);
  root_nh.param(std::string("/ur/gain"), _move_para_gain, 300.0f);
  root_nh.param(std::string("/robot/max_dist_tran"), max_dist_tran, 0.0);
  root_nh.param(std::string("/robot/max_dist_rot"), max_dist_rot, 0.0);
  root_nh.param(std::string("/robot/safety_mode"), safety_mode_int, 0);
  root_nh.param(std::string("/robot/operation_mode"), operation_mode_int, 0);
  root_nh.param(std::string("/robot/safe_zone/xmin"), safe_zone[0], 0.0);
  root_nh.param(std::string("/robot/safe_zone/xmax"), safe_zone[1], 0.0);
  root_nh.param(std::string("/robot/safe_zone/ymin"), safe_zone[2], 0.0);
  root_nh.param(std::string("/robot/safe_zone/ymax"), safe_zone[3], 0.0);
  root_nh.param(std::string("/robot/safe_zone/zmin"), safe_zone[4], 0.0);
  root_nh.param(std::string("/robot/safe_zone/zmax"), safe_zone[5], 0.0);

  if (!root_nh.hasParam("/ur/portnum"))
    ROS_WARN_STREAM("Parameter [/ur/portnum] not found");
  if (!root_nh.hasParam("/ur/ip"))
    ROS_WARN_STREAM("Parameter [/ur/ip] not found");
  if (!root_nh.hasParam("/robot/max_dist_tran"))
    ROS_WARN_STREAM("Parameter [/robot/max_dist_tran] not found");
  if (!root_nh.hasParam("/robot/max_dist_rot"))
    ROS_WARN_STREAM("Parameter [/robot/max_dist_rot] not found");
  if (!root_nh.hasParam("/robot/safety_mode"))
    ROS_WARN_STREAM("Parameter [/robot/safety_mode] not found");
  if (!root_nh.hasParam("/robot/operation_mode"))
    ROS_WARN_STREAM("Parameter [/robot/operation_mode] not found");
  if (!root_nh.hasParam("/robot/safe_zone"))
    ROS_WARN_STREAM("Parameter [/egm/robot/safe_zone] not found");

  switch(safety_mode_int) {
    case 0 : _safetyMode = SAFETY_MODE_NONE;
    break;
    case 1 : _safetyMode = SAFETY_MODE_TRUNCATE;
    break;
    case 2 : _safetyMode = SAFETY_MODE_STOP;
    break;
  }
  switch(operation_mode_int) {
    case 0 : _operationMode = OPERATION_MODE_CARTESIAN;
    break;
    case 1 : _operationMode = OPERATION_MODE_JOINT;
    break;
  }

  /* Establish connection with UR */
  cout << "[URSocket] Connecting to robot at " << ur_ip << ":" << ur_portnum << endl;
  _sock = 0;
  struct sockaddr_in serv_addr;
  if ((_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    cout << "\n Socket creation error \n";
    return false;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(ur_portnum);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if(inet_pton(AF_INET, ur_ip.c_str(), &serv_addr.sin_addr)<=0) {
    cout << "\nInvalid address/ Address not supported \n";
    return false;
  }

  if (connect(_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    cout << "\nConnection Failed \n";
    return false;
  }

  cout << "UR socket connection established.\n";

  _max_dist_tran = max_dist_tran;
  _max_dist_rot  = max_dist_rot;

  /* Initialize goal */
  RUT::copyArray(safe_zone, _safe_zone, 6);
  assert(safe_zone[0] < safe_zone[1]); // make sure the order is min max

  /* Create thread to listen to UR states */
  cout << "Trying to create thread.\n";
  _thread = std::thread(&URSocket::UR_STATE_MONITOR, this);

  while (!_isInitialized)
    usleep(200*1000);
  cout << "UR thread is created and running.\n";
  return true;
}

void URSocket::UR_STATE_MONITOR() {
  unsigned char buffer[1116];
  unsigned int length = 0;
  assert(sizeof(length) == 4);
  while(true) {
    /**
     * Read pose feedback
     */
    ReadXBytes(_sock, 1116, (void*)(buffer));

    // decode the message
    unsigned char *pointer = buffer;
    int length = buffToInteger(pointer);
    assert(length == 1116);
    pointer += 444;
    double x = buffToDouble(pointer) * 1000.0;
    pointer += 8;
    double y = buffToDouble(pointer) * 1000.0;
    pointer += 8;
    double z = buffToDouble(pointer) * 1000.0;
    pointer += 8;
    double Rx = buffToDouble(pointer);
    pointer += 8;
    double Ry = buffToDouble(pointer);
    pointer += 8;
    double Rz = buffToDouble(pointer);

    // printf("Length: %d, x: %.3f, y: %.3f, z: %.3f, rx: %.3f, ry: %.3f, rz: %.3f\n", length,
    //   x, y, z, Rx, Ry, Rz);

    // convert to pose
    Vector3d ax;
    ax << Rx, Ry, Rz;
    double angle = ax.norm();
    Quaterniond q(Eigen::AngleAxisd(angle, ax.normalized()));

    _mtx_pose.lock();
    _pose[0] = x;
    _pose[1] = y;
    _pose[2] = z;
    _pose[3] = q.w();
    _pose[4] = q.x();
    _pose[5] = q.y();
    _pose[6] = q.z();
    _mtx_pose.unlock();

    // check safety
    if ((x < _safe_zone[0]) || (x > _safe_zone[1]) || (y < _safe_zone[2]) ||
        (y > _safe_zone[3]) || (z < _safe_zone[4]) || (z > _safe_zone[5])) {
      ROS_ERROR_STREAM("[URSocket] Error: out of safety bound");
      exit(1);
    }

    if (!_isInitialized) {
      RUT::copyArray(_pose, _pose_set, 7);
      _isInitialized = true;
    }

    /**
     * Send pose command
     */
    // Quaternion to axis angle
    Vector3d ax_send;
    _mtx_pose_set.lock();
    angle = 2.0*acos(_pose_set[3]);
    ax_send << _pose_set[4], _pose_set[5], _pose_set[6];
    ax_send.normalize();
    ax_send *= angle;
    sprintf (_send_buffer, "servoj(get_inverse_kin(p[ %f, %f, %f, %f, %f, %f]), t = %f, lookahead_time = %f, gain = %f)\n",
        _pose_set[0]/1000.0, _pose_set[1]/1000.0, _pose_set[2]/1000.0,
        ax_send[0], ax_send[1], ax_send[2], _move_para_t, _move_para_lookahead, _move_para_gain);
    // sprintf (_send_buffer, "movel(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f, t = %f, r = %f)\n",
    //     _pose_set[0]/1000.0, _pose_set[1]/1000.0, _pose_set[2]/1000.0,
    //     ax_send[0], ax_send[1], ax_send[2], _move_para_a, _move_para_v, _move_para_t, _move_para_r);
    _mtx_pose_set.unlock();
    send(_sock, _send_buffer, strlen(_send_buffer), 0);

    // stop condition
    if (_stop_monitoring) break;
  }
}


bool URSocket::getCartesian(double *pose) {
  if (!_isInitialized) return false;

  _mtx_pose.lock();
  RUT::copyArray(_pose, pose, 7);
  _mtx_pose.unlock();

  // check safety
  if ((_pose[0] < _safe_zone[0]) || (_pose[0] > _safe_zone[1]))
    return false;
  if ((_pose[1] < _safe_zone[2]) || (_pose[1] > _safe_zone[3]))
    return false;
  if ((_pose[2] < _safe_zone[4]) || (_pose[2] > _safe_zone[5]))
    return false;

  return true;
}

bool URSocket::setCartesian(const double *pose) {
  assert(_operationMode == OPERATION_MODE_CARTESIAN);

  _mtx_pose_set.lock();
  RUT::copyArray(pose, _pose_set, 7);
  _mtx_pose_set.unlock();

  return true;
}

bool URSocket::getJoints(double *joints) {
  if (!_isInitialized)
    return false;

  RUT::copyArray(_joints, joints, 6);
  return true;
}

bool URSocket::setJoints(const double *joints) {
  assert(_operationMode == OPERATION_MODE_JOINT);
  cout << "not implemented yet" << endl;
  return true;
}