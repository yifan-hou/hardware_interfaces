#include <cmath>
#include <cassert>
#include <iostream>
#include <mutex>
#include <Eigen/Dense>
#include <RobotUtilities/utilities.h>

#include "abb_egm/abb_egm.h"


#define RCBFLENGTH 1400

using namespace std;
using namespace Eigen;

ABBEGM* ABBEGM::pinstance = 0;
mutex mtx;

/**
 * Compute the quaternion @p qm that is @p angle away from quaternion @p qa
 * towards @p qb, along the direction of spherical interpolation. If the
 * distance between qa and qb is smaller than @p angle, return qb. Quaternions
 * are represented as [qw qx qy qz]. Throw error if qa and qb are exactly 180
 * degree away.
 * Modified from:
 *   http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
 *
 * @param[in]  qa     Starting quaternion.
 * @param[in]  qb     Final quaternion.
 * @param      qm     The interpolated quaternion.
 * @param[in]  angle  The angle.
 *
 * @return     0 if distance between qa and qb is smaller than @p angle.
 * Return 1 otherwise.
 */
int SlerpFixAngle(const Vector4d &qa, const Vector4d &qb, Vector4d &qm,
    float angle) {
  assert(abs(qa.norm()-1)<0.01);
  assert(abs(qb.norm()-1)<0.01);

  // Calculate angle between them.
  double cosHalfTheta =
      qa(0) * qb(0) + qa(1) * qb(1) + qa(2) * qb(2) + qa(3) * qb(3);
  // if qa=qb or qa=-qb then theta = 0 and we can return qa
  if (abs(cosHalfTheta) >= 1.0){
    qm(0) = qa(0); qm(1) = qa(1); qm(2) = qa(2);qm(3) = qa(3);
    return 0;
  }
  // Calculate temporary values. acos return [0, PI]
  double halfTheta = acos(cosHalfTheta);

  // qa-qb is smaller than angle. Return qb
  if (2*halfTheta < angle) {
    qm(0) = qb(0); qm(1) = qb(1); qm(2) = qb(2); qm(3) = qb(3);
    return 0;
  }

  double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // throw error
  if (fabs(sinHalfTheta) < 0.001){
    // qm(0) = qa(0); qm(1) = qa(1); qm(2) = qa(2);qm(3) = qa(3);
    cout << "[ABBEGM] [Commanded orientation is 180 deg away from current"
        << " orientation! ] Motion Stopped." << endl;
    exit(1);
  }

  angle /=2;
  double ratioA = sin(halfTheta-angle) / sinHalfTheta;
  double ratioB = sin(angle) / sinHalfTheta;

  //calculate Quaternion.
  qm(0) = (qa(0) * ratioA + qb(0) * ratioB);
  qm(1) = (qa(1) * ratioA + qb(1) * ratioB);
  qm(2) = (qa(2) * ratioA + qb(2) * ratioB);
  qm(3) = (qa(3) * ratioA + qb(3) * ratioB);
  return 1;
}

ABBEGM* ABBEGM::Instance() {
  if (pinstance == 0) {
    pinstance = new ABBEGM();
  }
  return pinstance;
}

ABBEGM::ABBEGM() {
  _pose          = new double[7];
  _joints        = new double[6];
  _set_pose      = new double[7];
  _set_joints    = new double[6];
  _safe_zone     = new double[6];
  _isInitialized = false;
  _RobotPort     = 0;
  _safetyMode    = SAFETY_MODE_STOP;
  _operationMode = OPERATION_MODE_CARTESIAN;
}

ABBEGM::~ABBEGM() {
  _thread.join();
  if (_print_flag)
    _file.close();

  delete pinstance;
  delete [] _pose;
  delete [] _joints;
  delete [] _set_pose;
  delete [] _set_joints;
  delete [] _safe_zone;
}

// todo: finish this function
void ABBEGM::EGM_JOINT_MODE_MONITOR() {
  std::cout << "EGMClss Joint Mode is running." << std::endl;
  assert(_operationMode == OPERATION_MODE_JOINT);

  double tempjoint[6];
  while(true) {
    send(_set_joints);
    listen();

    getJoints(tempjoint);
    std::cout << "[monitor] [_joints] ";
    for (int i = 0; i < 6; ++i) {
      std::cout << tempjoint[i] << "|";
    }
    std::cout << std::endl;
  }
}

void ABBEGM::EGM_CARTESIAN_MODE_MONITOR() {
  cout << "[ABBEGM] EGM_Monitor is running." << endl;
  assert(_operationMode == OPERATION_MODE_CARTESIAN);
  double pose[7], poseSet[7];

  double qvalue[4];

  Vector3d tran;
  double tranSet[3];
  Vector4d quatGoal, quatNow, quatSet;

  Clock::time_point timenow_clock;
  // begin the loop
  for (;;) {
    // read from buffer
    bool issafe = getCartesian(pose);
    // check safety zone
    if (!issafe) {
      cout << "[ABBEGM] [Out of safety zone!!!] Motion Stopped." << endl;
      send(pose);
      listen();
      exit(1);
    }

    if(_print_flag) {
      timenow_clock = Clock::now();
      double timenow = double(std::chrono::duration_cast<   // milli second
          std::chrono::nanoseconds>(timenow_clock - _time0).count())/1e6;
      _file << timenow << "\t";
      RUT::stream_array_in(_file, pose, 7);
    }

    // command is given by _set_pose
    // mtx.lock();
    for(int i=0; i<3; i++)
      tran(i) = _set_pose[i] - pose[i];
    for(int i = 0; i < 4; i++) {
      quatNow(i) = pose[i+3];
      quatGoal(i) = _set_pose[i+3];
      qvalue[i] = _set_pose[i+3];
    }
    // mtx.unlock();

    // numerical stability: reverse quatGoal if it is in the other hemisphere
    int id = 0;
    RUT::vec_max_abs(qvalue, 4, &id);
    if (quatNow(id)*quatGoal(id) < 0) {
      quatGoal(0) = -quatGoal(0);
      quatGoal(1) = -quatGoal(1);
      quatGoal(2) = -quatGoal(2);
      quatGoal(3) = -quatGoal(3);
    }

    if (_safetyMode == SAFETY_MODE_NONE) {
      quatSet(0) = quatGoal(0);
      quatSet(1) = quatGoal(1);
      quatSet(2) = quatGoal(2);
      quatSet(3) = quatGoal(3);
    }
    else {
      // check translation and rotation
      // compute the truncated motion
      bool violated = false;
      if (tran.norm() > _max_dist_tran) {
        violated = true;
        tran *= (_max_dist_tran/tran.norm());
      }

      if (SlerpFixAngle(quatNow, quatGoal, quatSet, _max_dist_rot)) {
        violated = true;
      }
      assert(abs(quatSet.norm()-1) < 0.01);

      // stop if necessary
      if ((_safetyMode == SAFETY_MODE_STOP) & violated) {
        cout << "[ABBEGM] [Command is Too Fast!!] Motion Stopped." << endl;
        send(pose);
        listen();
        exit(1);
      }
    }

    tranSet[0] = pose[0] + tran(0);
    tranSet[1] = pose[1] + tran(1);
    tranSet[2] = pose[2] + tran(2);
    RUT::copyArray(tranSet, poseSet, 3);
    poseSet[3] = quatSet(0);
    poseSet[4] = quatSet(1);
    poseSet[5] = quatSet(2);
    poseSet[6] = quatSet(3);

    if(_print_flag) {
      timenow_clock  = Clock::now();
      double timenow = double(std::chrono::duration_cast< // milli second
          std::chrono::nanoseconds>(timenow_clock - _time0).count())/1e6;
      _file << timenow << "\t";
      RUT::stream_array_in(_file, poseSet, 7);
      _file << endl;
    }

    //  sending
    send(poseSet);

    //  receiving
    listen();
  }
}

int ABBEGM::init(ros::NodeHandle& root_nh, Clock::time_point time0) {
  _time0 = time0;
  // read egm parameters from parameter server
  int portnum;
  double max_dist_tran, max_dist_rot;
  double safe_zone[6];
  bool print_flag;
  string filefullpath;
  int safety_mode_int;
  int operation_mode_int;

  root_nh.param(std::string("/egm/portnum"), portnum, 6510);
  root_nh.param(std::string("/egm/print_flag"), print_flag, false);
  root_nh.param(std::string("/egm/file_path"), filefullpath, std::string(" "));
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

  if (!root_nh.hasParam("/egm/portnum"))
    ROS_WARN_STREAM("Parameter [/egm/portnum] not found");
  if (!root_nh.hasParam("/egm/print_flag"))
    ROS_WARN_STREAM("Parameter [/egm/print_flag] not found");
  if (!root_nh.hasParam("/egm/file_path"))
    ROS_WARN_STREAM("Parameter [/egm/file_path] not found");
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

  /* Establish connection with ABB EGM */
  _EGMsock.setLocalPort(portnum);
  _print_flag    = print_flag;

  if (_print_flag)
    _file.open(filefullpath);

  cout << "EGM server is waiting for connection..\n";
  listen(); // _pose is set here
  cout << "EGM connection established.\n";

  _max_dist_tran = max_dist_tran;
  _max_dist_rot  = max_dist_rot;

  /* Initialize goal */
  RUT::copyArray(_pose, _set_pose, 7);
  RUT::copyArray(_joints, _set_joints, 6);
  RUT::copyArray(safe_zone, _safe_zone, 6);
  assert(safe_zone[0] < safe_zone[1]); // make sure the order is min max

  /* Create thread to run communication with EGM */
  cout << "EGM is trying to create thread.\n";
  if (_operationMode == OPERATION_MODE_JOINT)
    _thread = std::thread(&ABBEGM::EGM_JOINT_MODE_MONITOR, this);
  else
    _thread = std::thread(&ABBEGM::EGM_CARTESIAN_MODE_MONITOR, this);

  _isInitialized = true;
  cout << "EGM thread created.\n";
  return true;
}

bool ABBEGM::getCartesian(double *pose)
{
  if (!_isInitialized) return false;

  RUT::copyArray(_pose, pose, 7);

  // check safety
  if ((_pose[0] < _safe_zone[0]) || (_pose[0] > _safe_zone[1]))
    return false;
  if ((_pose[1] < _safe_zone[2]) || (_pose[1] > _safe_zone[3]))
    return false;
  if ((_pose[2] < _safe_zone[4]) || (_pose[2] > _safe_zone[5]))
    return false;

  return true;
}

bool ABBEGM::setCartesian(const double *pose)
{
  assert(_operationMode == OPERATION_MODE_CARTESIAN);
    // mtx.lock();
  RUT::copyArray(pose, _set_pose, 7);
    // mtx.unlock();
  return true;
}

bool ABBEGM::getJoints(double *joints) {
  if (!_isInitialized)
    return false;

  RUT::copyArray(_joints, joints, 6);
  return true;
}

bool ABBEGM::setJoints(const double *joints) {
  assert(_operationMode == OPERATION_MODE_JOINT);
    // mtx.lock();
  RUT::copyArray(joints, _set_joints, 6);
    // mtx.unlock();
  return true;
}

int ABBEGM::listen()
{
  if (_isInitialized == false)
  {
    cout << "EGM error: listen() is called before initialization().\n";
    return false;
  }

    // cout << "If can not receive message, run 'sudo ufw allow 6510'\n";
  char recvBuffer[RCBFLENGTH];
  int n = _EGMsock.recvFrom(recvBuffer, RCBFLENGTH, _RobotAddress, _RobotPort);
  if (n < 0)
  {
    cout << "EGM error: Error receiving message.\n";
    exit(1);
  }
    // cout << "[Egm server node] message received, connection established!\n";
    // cout << "Address: " << _RobotAddress.c_str() << ", port:" << _RobotPort << endl;

  _pRecvMessage = new EgmRobot();
  _pRecvMessage->ParseFromArray(recvBuffer, n);
  ReadRobotMessage(_pRecvMessage);
    // printf("x: %lf\ny: %lf\nz: %lf\n", x, y, z);
  delete _pRecvMessage;

  return true;
}

void ABBEGM::send(double *set_cmd_double)
{
  if (_RobotPort == 0)
  {
    cout << "EGM error: send() is called before listen().\n";
    return;
  }
  // ---------------------------------------------------------------------------
  //      create and send a sensor message
  // ---------------------------------------------------------------------------
  float set_cmd[7];
  RUT::double2float(set_cmd_double, set_cmd, 7);
  _pSendingMessage = new EgmSensor();
  if (_operationMode == OPERATION_MODE_CARTESIAN) {
    assert(CreateCartesianTargetSensorMessage(set_cmd, _pSendingMessage));
  } else if (_operationMode == OPERATION_MODE_JOINT) {
    assert(CreateJointTargetSensorMessage(set_cmd, _pSendingMessage));
  }

  _pSendingMessage->SerializeToString(&_sendBuffer);
  delete _pSendingMessage;

  _EGMsock.sendTo(_sendBuffer.c_str(), _sendBuffer.length(),
    _RobotAddress, _RobotPort);
}

int ABBEGM::CreateCartesianTargetSensorMessage(const float* setpose,
  EgmSensor* pSensorMessage)
{
  assert(_operationMode == OPERATION_MODE_CARTESIAN);
  static unsigned int sequenceNumber = 0;
  EgmHeader* header = new EgmHeader();
  header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
  header->set_seqno(sequenceNumber++);

  Clock::time_point timenow_clock = Clock::now();
    double time = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - _time0).count())/1e6; // milli second
    header->set_tm(time);

    pSensorMessage->set_allocated_header(header);
    EgmCartesian *pc = new EgmCartesian();

    // in mm
    pc->set_x(setpose[0]);
    pc->set_y(setpose[1]);
    pc->set_z(setpose[2]);

    EgmQuaternion *pq = new EgmQuaternion();
    pq->set_u0(setpose[3]);
    pq->set_u1(setpose[4]);
    pq->set_u2(setpose[5]);
    pq->set_u3(setpose[6]);

    EgmPose *pcartesian = new EgmPose();
    pcartesian->set_allocated_orient(pq);
    pcartesian->set_allocated_pos(pc);

    EgmPlanned *planned = new EgmPlanned();
    planned->set_allocated_cartesian(pcartesian);

    pSensorMessage->set_allocated_planned(planned);

    return true;
  }

/// Create a protocol buffer message based on commanded joint.
  int ABBEGM::CreateJointTargetSensorMessage(const float* joints,
    EgmSensor* pSensorMessage)
  {
    assert(_operationMode == OPERATION_MODE_JOINT);
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);

    Clock::time_point timenow_clock = Clock::now();
    double time = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - _time0).count())/1e6; // milli second
    header->set_tm(time);

    pSensorMessage->set_allocated_header(header);

    EgmJoints * pb_joints = new EgmJoints();
    // std::cout << "Joints size " << pb_joints->joints_size() << std::endl;
    constexpr int num_dofs = 6;
    for (int i = 0; i < num_dofs; ++i) {
      float val = joints[i] * 180 / M_PI;
      pb_joints->add_joints(val);
    }
    std::cout << "[CreateJointMessage] [pb_joints] ";
    for (int i = 0; i < num_dofs; ++i)
    {
      std::cout << pb_joints->joints(i) << "|";
    }
    std::cout<< std::endl;

    EgmPlanned * pb_plan = new EgmPlanned();
    pb_plan->set_allocated_joints(pb_joints);

    pSensorMessage->set_allocated_planned(pb_plan);

    return true;
  }


// Create a simple robot message
  void ABBEGM::CreateSensorMessageEmpty(EgmSensor* pSensorMessage)
  {
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(0);
    // header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

  }

// ************************
  void ABBEGM::ReadRobotMessage(EgmRobot *pRobotMessage)
  {
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype()  )
    {
        //printf("SeqNo=%d Tm=%u Type=%d\n", pRobotMessage->header().seqno(), pRobotMessage->header().tm(), pRobotMessage->header().mtype());
      _pose[0] = (double)(pRobotMessage->feedback().cartesian().pos().x());
      _pose[1] = (double)(pRobotMessage->feedback().cartesian().pos().y());
      _pose[2] = (double)(pRobotMessage->feedback().cartesian().pos().z());

      _pose[3] = (double)(pRobotMessage->feedback().cartesian().orient().u0());
      _pose[4] = (double)(pRobotMessage->feedback().cartesian().orient().u1());
      _pose[5] = (double)(pRobotMessage->feedback().cartesian().orient().u2());
      _pose[6] = (double)(pRobotMessage->feedback().cartesian().orient().u3());
        // std::cout << "read egm joint size " << pRobotMessage->feedback().joints().joints_size() << std::endl;
        // Get joints value.
      for (int i = 0; i < 6; ++i) {
        _joints[i] = (double)(pRobotMessage->feedback().joints().joints(i));
      }
    }
    else
    {
      cout << "No header\n";
    }
  }

  void ABBEGM::DisplayRobotMessage(EgmRobot *pRobotMessage)
  {
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype())
    {
      cout << "SeqNo=" << pRobotMessage->header().seqno() << " Tm=" << pRobotMessage->header().tm();
      cout << " Type=" << pRobotMessage->header().mtype() << endl;
    }
    else
    {
      cout << "No header\n";
    }
  }
