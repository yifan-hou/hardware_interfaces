#include <iostream>
#include <cmath>
#include <cassert>
#include <Eigen/Dense>

#include "egm/EGMClass.h"
#include "egm/utilities.h" 


#define RCBFLENGTH 1400
// #define PI 3.1416  // already defined in matVec

using namespace std;
using namespace Eigen;

EGMClass* EGMClass::pinstance = 0;

typedef std::chrono::high_resolution_clock Clock;

// Slerp with angle limit.
// qa, qb: starting and ending quaternions.
// angle: maximum angle limit
// 
// return
//  0: qb-qa is within angle limit.
//  1: qb-qa is out of angle limit.
// 
// Modified from:
//  http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
int SlerpFixAngle(const Vector4d &qa, const Vector4d &qb, Vector4d &qm, float angle)
{
    assert(abs(qa.norm()-1)<0.01);
    assert(abs(qb.norm()-1)<0.01);

    // Calculate angle between them.
    double cosHalfTheta = qa(0) * qb(0) + qa(1) * qb(1) + qa(2) * qb(2) + qa(3) * qb(3);
    // if qa=qb or qa=-qb then theta = 0 and we can return qa
    if (abs(cosHalfTheta) >= 1.0){
        qm(0) = qa(0); qm(1) = qa(1); qm(2) = qa(2);qm(3) = qa(3);
        return 0;
    }
    // Calculate temporary values. acos return [0, PI]
    double halfTheta = acos(cosHalfTheta);

    // qa-qb is smaller than angle. Return qb
    if (2*halfTheta < angle)
    {
        qm(0) = qb(0); qm(1) = qb(1); qm(2) = qb(2); qm(3) = qb(3);
        return 0;
    }

    double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
    // if theta = 180 degrees then result is not fully defined
    // throw error
    if (fabs(sinHalfTheta) < 0.001){ 
        // qm(0) = qa(0); qm(1) = qa(1); qm(2) = qa(2);qm(3) = qa(3);
        cout << "[EGMClass] [Commanded orientation is 180 deg away from current orientation! ] Motion Stopped." << endl;
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

EGMClass* EGMClass::Instance()
{
  if (pinstance == 0) // first time call
  {
    pinstance = new EGMClass;
  }
  return pinstance;
}

EGMClass::EGMClass()
{
  // do some initialization
    _pose          = new float[7];
    _joints        = new float[6];
    _set_pose      = new float[7];
    _set_joints    = new float[6];
    _safe_zone     = new float[6];
    _isInitialized = false;
    _RobotPort     = 0;
    _safetyMode    = SAFETY_MODE_STOP;
    _operationMode = OPERATION_MODE_CARTESIAN;
}

EGMClass & EGMClass::operator=(const EGMClass & olc)
{
  // shall never get called
  EGMClass* lc = new EGMClass();
  return *lc;
}

EGMClass::~EGMClass()
{
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
void* EGM_JOINT_MODE_MONITOR(void *pParam) {

    std::cout << "EGMClss Joint Mode is running." << std::endl;
    EGMClass *egm = (EGMClass*)pParam;
    assert(egm->_operationMode == OPERATION_MODE_JOINT);

    float tempjoint[6];
    while(true) {
        //UT::copyArray(egm->_set_joints, set_joints);
        std::cout << "[monitor] [_set_joints] ";
        for (int i = 0; i < 6; ++i) 
        {
            std::cout << egm->_set_joints[i] << "|";
        }
        std::cout << std::endl;

        egm->send(egm->_set_joints);
        egm->listen();

        egm->GetJoints(tempjoint);
        std::cout << "[monitor] [_joints] ";
        for (int i = 0; i < 6; ++i) 
        {
            std::cout << tempjoint[i] << "|";
        }
        std::cout << std::endl;
        
        //UT::copyArray(egm->_set_joints, set_joints);
    }
}

//
//  The thread Function.
//
void* EGM_CARTESIAN_MODE_MONITOR(void* pParam)
{
    cout << "[EGMClass] EGM_Monitor is running." << endl;
    EGMClass *egm = (EGMClass*)pParam;
    assert(egm->_operationMode == OPERATION_MODE_CARTESIAN);
    float pose[7], poseSet[7]; 

    Vector3d tran;
    float tranSet[3];
    Vector4d quatGoal, quatNow, quatSet;
    
    Clock::time_point timenow_clock;
    // begin the loop
    for (;;) {
      // read from buffer
      bool issafe = egm->GetCartesian(pose);
      // check safety zone
      if (!issafe)
      {
          cout << "[EGMClass] [Out of safety zone!!!] Motion Stopped." << endl;
          egm->send(pose);
          egm->listen();
          exit(1);
      }

      if(egm->_print_flag)
      {
          timenow_clock = Clock::now();
          double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - egm->_time0).count())/1e6; // milli second
          egm->_file << timenow << "\t";
          UT::stream_array_in(egm->_file, pose, 7);
      }

      // command is given by _set_pose
      for(int i=0; i<3; i++) 
          tran(i) = egm->_set_pose[i] - pose[i];
      for(int i = 0; i < 4; i++)
      {
          quatNow(i) = pose[i+3];
          quatGoal(i) = egm->_set_pose[i+3];
      }

      if (egm->_safetyMode == SAFETY_MODE_NONE)
      {
          quatSet(0) = quatGoal(0);
          quatSet(1) = quatGoal(1);
          quatSet(2) = quatGoal(2);
          quatSet(3) = quatGoal(3);
      }
      else 
      {
          // check translation and rotation
          // compute the truncated motion
          bool violated = false;
          if (tran.norm() > egm->_max_dist_tran) 
          {
              violated = true;
              tran *= (egm->_max_dist_tran/tran.norm());
          }
          
          if (SlerpFixAngle(quatNow, quatGoal, quatSet, egm->_max_dist_rot))
          {
              violated = true;
          }
          assert(abs(quatSet.norm()-1) < 0.01);

          // stop if necessary
          if ((egm->_safetyMode == SAFETY_MODE_STOP) & violated)
          {
              cout << "[EGMClass] [Command is Too Fast!!] Motion Stopped." << endl;
              egm->send(pose);
              egm->listen();
              exit(1);
          }
      }

        tranSet[0] = pose[0] + tran(0);
        tranSet[1] = pose[1] + tran(1);
        tranSet[2] = pose[2] + tran(2);
        UT::copyArray(tranSet, poseSet, 3);
        poseSet[3] = quatSet(0);
        poseSet[4] = quatSet(1);
        poseSet[5] = quatSet(2);
        poseSet[6] = quatSet(3);

        if(egm->_print_flag)
        {
            timenow_clock  = Clock::now();
            double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(timenow_clock - egm->_time0).count())/1e6; // milli second
            egm->_file << timenow << "\t";
            UT::stream_array_in(egm->_file, poseSet, 7);
            egm->_file << endl;
        }
        
        //  sending
        egm->send(poseSet);

        //  receiving
        egm->listen();
    }
}

int EGMClass::init(std::chrono::high_resolution_clock::time_point time0, unsigned short portnum, float max_dist_tran, float max_dist_rot, float *safe_zone, EGMSafetyMode sf_mode, EGMOperationMode op_mode, bool printflag, string filefullpath) {
  /* Establish connection with ABB EGM */ 
  _EGMsock.setLocalPort(portnum);
  _print_flag    = printflag;
  _isInitialized = true;

  if (printflag)
      _file.open(filefullpath);

  cout << "EGM server is waiting for connection..\n";
  listen(); // _pose is set here
  cout << "EGM connection established.\n";
  
  _time0         = time0;
  _safetyMode    = sf_mode;
  _operationMode = op_mode;
  _max_dist_tran = max_dist_tran;
  _max_dist_rot  = max_dist_rot;

  /* Initialize goal */
  UT::copyArray(_pose, _set_pose, 7);
  UT::copyArray(_joints, _set_joints, 6);
  UT::copyArray(safe_zone, _safe_zone, 6);
  assert(safe_zone[0] < safe_zone[1]); // make sure the order is min max

  /* Create thread to run communication with EGM */
  cout << "EGM is trying to create thread.\n";
  int rc = 0;
  if (_operationMode == OPERATION_MODE_JOINT)
    rc = pthread_create(&_thread, NULL, EGM_JOINT_MODE_MONITOR, this);
  else
   rc = pthread_create(&_thread, NULL, EGM_CARTESIAN_MODE_MONITOR, this);

  if (rc){
      cout <<"EGM error:unable to create thread.\n";
      return false;
  }
  cout << "EGM thread created.\n";
  
  return true;
}

bool EGMClass::GetCartesian(float *pose)
{
  if (!_isInitialized) return false;

  UT::copyArray(_pose, pose, 7);

  // check safety
  if ((_pose[0] < _safe_zone[0]) || (_pose[0] > _safe_zone[1]))
      return false;
  if ((_pose[1] < _safe_zone[2]) || (_pose[1] > _safe_zone[3]))
      return false;
  if ((_pose[2] < _safe_zone[4]) || (_pose[2] > _safe_zone[5]))
      return false;

  return true;
}

bool EGMClass::SetCartesian(const float *pose)
{
    assert(_operationMode == OPERATION_MODE_CARTESIAN);
    UT::copyArray(pose, _set_pose, 7);
    return true;
}

bool EGMClass::GetJoints(float *joints) {
    if (!_isInitialized) 
        return false;

    UT::copyArray(_joints, joints, 6);   
    // Todo(Jiaji:) Check joint limits here.
    return true;
}

bool EGMClass::SetJoints(const float *joints) {
    assert(_operationMode == OPERATION_MODE_JOINT);
    UT::copyArray(joints, _set_joints, 6);
    return true;
}

int EGMClass::listen()
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

void EGMClass::send(float *set_cmd)
{
    if (_RobotPort == 0)
    {
        cout << "EGM error: send() is called before listen().\n";
        return;
    }
  // -----------------------------------------------------------------------------
  //      create and send a sensor message
  // -----------------------------------------------------------------------------
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

int EGMClass::CreateCartesianTargetSensorMessage(const float* setpose, 
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
int EGMClass::CreateJointTargetSensorMessage(const float* joints, 
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
void EGMClass::CreateSensorMessageEmpty(EgmSensor* pSensorMessage)
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
void EGMClass::ReadRobotMessage(EgmRobot *pRobotMessage)
{
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype()  )
    {
        //printf("SeqNo=%d Tm=%u Type=%d\n", pRobotMessage->header().seqno(), pRobotMessage->header().tm(), pRobotMessage->header().mtype());
        _pose[0] =  pRobotMessage->feedback().cartesian().pos().x();
        _pose[1] =  pRobotMessage->feedback().cartesian().pos().y();
        _pose[2] =  pRobotMessage->feedback().cartesian().pos().z();

        _pose[3] =  pRobotMessage->feedback().cartesian().orient().u0();
        _pose[4] =  pRobotMessage->feedback().cartesian().orient().u1();
        _pose[5] =  pRobotMessage->feedback().cartesian().orient().u2();
        _pose[6] =  pRobotMessage->feedback().cartesian().orient().u3();
        // std::cout << "read egm joint size " << pRobotMessage->feedback().joints().joints_size() << std::endl;
        // Get joints value.
        for (int i = 0; i < 6; ++i) {
          _joints[i] = pRobotMessage->feedback().joints().joints(i);
        }
    }
    else
    {
        cout << "No header\n";
    }
}

void EGMClass::DisplayRobotMessage(EgmRobot *pRobotMessage)
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
