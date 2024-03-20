#include "ur_socket/ur_socket.h"

#include <thread>
#include <memory>
#include <mutex>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <yaml-cpp/yaml.h>

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
        result = recv(socket, buffer + bytesRead, x - bytesRead, 0);
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

double buffToDouble(uint8_t * buff) {
    double value;
    memcpy(&value,buff,sizeof(double));
    return reverseDouble((char *)&value);
}

struct URSocket::Implementation {
  // Indicates whether the initialization() function is called.
  bool is_initialized;
  // motion parameters
  float move_para_t;
  float move_para_lookahead;
  float move_para_gain;

  char *send_buffer;

  double *pose_xyzq;
  double *pose_xyzq_set;
  double *joints;
  std::mutex mtx_pose_xyzq;
  std::mutex mtx_pose_xyzq_set;
  std::mutex mtx_joint;

  int sock;
  std::thread thread;
  Clock::time_point time0;

  bool stop_monitoring;

  URSocket::URSocketConfig config;

  Implementation();
  ~Implementation();

  bool initialize(Clock::time_point time0, const URSocket::URSocketConfig &config);
  void ur_state_monitor_call_back();
  bool getCartesian(double *pose_xyzq);
  bool setCartesian(const double *pose_xyzq);
};

URSocket::Implementation::Implementation() {
  pose_xyzq       = new double[7];
  pose_xyzq_set   = new double[7];
  joints          = new double[6];
  send_buffer     = new char[1000];
  stop_monitoring = false;
}

URSocket::Implementation::~Implementation() {
  std::cout << "[URSocket] finishing.." << endl;
  stop_monitoring = true;
  thread.join();

  delete pinstance;
  delete [] pose_xyzq;
  delete [] pose_xyzq_set;
  delete [] joints;
  delete [] send_buffer;
}

bool URSocket::Implementation::initialize(Clock::time_point time0,
    const URSocket::URSocketConfig &ur_socket_config) {
  time0 = time0;
  config = ur_socket_config;

  /* Establish connection with UR */
  std::cout << "[URSocket] Connecting to robot at " << config.ur_ip << ":" << config.ur_portnum << endl;
  sock = 0;
  struct sockaddr_in serv_addr;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "\n[URSocket] Socket creation error \n";
    return false;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(config.ur_portnum);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if(inet_pton(AF_INET, config.ur_ip.c_str(), &serv_addr.sin_addr)<=0) {
    std::cout << "\n[URSocket] Invalid address/ Address not supported \n";
    return false;
  }

  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    std::cout << "\n[URSocket]Connection Failed \n";
    return false;
  }

  std::cout << "[URSocket] UR socket connection established.\n";

  /* Create thread to listen to UR states */
  std::cout << "[URSocket] Trying to create thread.\n";
  thread = std::thread(&URSocket::Implementation::ur_state_monitor_call_back, this);

  while (!is_initialized)
    usleep(200*1000);
  std::cout << "[URSocket] UR thread is created and running.\n";
  return true;
}

// callbacks
void URSocket::Implementation::ur_state_monitor_call_back() {
  unsigned char buffer[1116];
  unsigned int length = 0;
  assert(sizeof(length) == 4);
  while(true) {
    /**
     * Read pose_xyzq feedback
     */
    ReadXBytes(sock, 1116, (void*)(buffer));

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

    // convert to pose_xyzq
    Vector3d ax;
    ax << Rx, Ry, Rz;
    double angle = ax.norm();
    Quaterniond q(Eigen::AngleAxisd(angle, ax.normalized()));

    mtx_pose_xyzq.lock();
    pose_xyzq[0] = x;
    pose_xyzq[1] = y;
    pose_xyzq[2] = z;
    pose_xyzq[3] = q.w();
    pose_xyzq[4] = q.x();
    pose_xyzq[5] = q.y();
    pose_xyzq[6] = q.z();
    mtx_pose_xyzq.unlock();

    // check safety
    if ((x < config.safe_zone[0]) || (x > config.safe_zone[1]) || (y < config.safe_zone[2]) ||
        (y > config.safe_zone[3]) || (z < config.safe_zone[4]) || (z > config.safe_zone[5])) {
      std::cerr << "[URSocket] Error: out of safety bound" << std::endl;
      exit(1);
    }

    if (!is_initialized) {
      RUT::copyArray(pose_xyzq, pose_xyzq_set, 7);
      is_initialized = true;
    }

    /**
     * Send pose_xyzq command
     */
    // Quaternion to axis angle
    Vector3d ax_send;
    mtx_pose_xyzq_set.lock();
    angle = 2.0*acos(pose_xyzq_set[3]);
    ax_send << pose_xyzq_set[4], pose_xyzq_set[5], pose_xyzq_set[6];
    ax_send.normalize();
    ax_send *= angle;
    sprintf (send_buffer, "servoj(get_inverse_kin(p[ %f, %f, %f, %f, %f, %f]), t = %f, lookahead_time = %f, gain = %f)\n",
        pose_xyzq_set[0]/1000.0, pose_xyzq_set[1]/1000.0, pose_xyzq_set[2]/1000.0,
        ax_send[0], ax_send[1], ax_send[2], move_para_t, move_para_lookahead, move_para_gain);
    // sprintf (send_buffer, "movel(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f, t = %f, r = %f)\n",
    //     pose_xyzq_set[0]/1000.0, pose_xyzq_set[1]/1000.0, pose_xyzq_set[2]/1000.0,
    //     ax_send[0], ax_send[1], ax_send[2], _move_para_a, _move_para_v, move_para_t, _move_para_r);
    mtx_pose_xyzq_set.unlock();
    send(sock, send_buffer, strlen(send_buffer), 0);

    // stop condition
    if (stop_monitoring) break;
  }
}

bool URSocket::Implementation::getCartesian(double *pose_xyzq) {
  if (!is_initialized) return false;

  mtx_pose_xyzq.lock();
  RUT::copyArray(pose_xyzq, pose_xyzq, 7);
  mtx_pose_xyzq.unlock();

  // check safety
  if ((pose_xyzq[0] < config.robot_interface_config.safe_zone[0]) || (pose_xyzq[0] > config.robot_interface_config.safe_zone[1]))
    return false;
  if ((pose_xyzq[1] < config.robot_interface_config.safe_zone[2]) || (pose_xyzq[1] > config.robot_interface_config.safe_zone[3]))
    return false;
  if ((pose_xyzq[2] < config.robot_interface_config.safe_zone[4]) || (pose_xyzq[2] > config.robot_interface_config.safe_zone[5]))
    return false;

  return true;
}

bool URSocket::Implementation::setCartesian(const double *pose_xyzq) {
  if (!is_initialized) return false;
  assert(config.robot_interface_config.operationMode == OPERATION_MODE_CARTESIAN);

  mtx_pose_xyzq_set.lock();
  RUT::copyArray(pose_xyzq, pose_xyzq_set, 7);
  mtx_pose_xyzq_set.unlock();

  return true;
}

URSocket::URSocket()
    : m_impl{std::make_unique<Implementation>()} {}

URSocket::~URSocket(){}

URSocket* URSocket::Instance() {
  if (pinstance == 0) {
    pinstance = new URSocket();
  }
  return pinstance;
}

bool URSocket::init(Clock::time_point time0, const URSocketConfig &ur_socket_config) {
  return m_impl->initialize(time0, ur_socket_config);
}

bool URSocket::getCartesian(double *pose_xyzq) {
  return m_impl->getCartesian(pose_xyzq);
}

bool URSocket::setCartesian(const double *pose_xyzq) {
  return m_impl->setCartesian(pose_xyzq);
}

bool URSocket::getJoints(double *joints) {
  std::cerr << "[URSocket] not implemented yet" << endl;
  return false;
}

bool URSocket::setJoints(const double *joints) {
  std::cerr << "[URSocket] not implemented yet" << endl;
  return false;
}