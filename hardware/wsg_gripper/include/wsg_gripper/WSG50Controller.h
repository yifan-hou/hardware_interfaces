#pragma once
/**
 * @brief The WSG50Controller class
 *
 * this class is an Ethernet-Controller for the Schunk WSG-50-Gripper
 * In a first version it will offer following functions
 *
 * - read:
 *  - width
 *  - speed
 *  - forcelimit
 *  - acceleration
 *
 * - command:
 *  - width
 *  - speed
 *  - forcelimit
 *  - acceleration
 *  - move
 *  - stop
 *  - homing (reset gripper to home position)
 */

#include "WSG50Communicator.h"
#include "WSG50Observer.h"
#include "WSG50Subject.h"

#include <mutex>
#include <queue>
#include <string>

/*!
 * Manufacturer's default network config
 */
#define DEFAULTIP "192.168.1.20"
#define DEFAULTPORT "1000"

class WSG50Controller : public WSG50Observer {

 public:
  // Connection Manager
  //
  WSG50Controller(void);
  WSG50Controller(std::string ip, std::string port);
  virtual ~WSG50Controller(void);

  /**
     *  ####################################
     *  ###### MOTION CONTROL       ########
     *  ####################################
     *
     */
  bool ready(void);  // ready for next command
  /*
     *homing:
     *@params: unsigned int direction
     *      0: default
     *      1: positive
     *      2: negative
     */
  void homing(void);  // using default value 0
  void homing(unsigned int direction);
  void prePositionFingers(
      bool stopOnBlock, float width,
      float speed = 400);  // position finger prior to a grap movement
  void stop(void);         // stop
  void fastStop(
      void);  // stop immediately and prevent any further motion-related commands
  void ackFastStop(
      void);  // if fast-stop was issued, this will bring the Gripper back to normal mode
  void grasp(
      float width,  // grasp a part
      float
          speed);  // width: nominal width of the part; speed: grasping speed in mm/s
  void setVelResolvedControl(float pos, float force, float stiffness,
                             float damping);
  void setPDControl(float pos, float kp, float kd, float force_limit);
  void release(float openWidth,  // the opening width
               float speed);     // the opening-speed in mm/s

  /**
     *  ####################################
     *  ###### MOTION CONFIGURATION ########
     *  ####################################
     *
     */
  void setAcceleration(float acceleration);  // set the acceleration
  void setForceLimit(float forcelimit);      // set the grasping-force-limit
  void setSoftLimits(
      float minusLimit,  // set the soft-limits
      float
          plusLimit);  // minus: negative motion direction; plus: positive motion direction
  void clearSoftLimits(void);  // clear all prior set soft limits
  bool areSoftLimitsSet(void);
  void tareForceSensor(void);  // zeroes the connected force sensor

  /**
     *  ####################################
     *  ###### GETTER METHODS       ########
     *  ####################################
     *
     */

  float getMaxWidth();
  float getMinWidth();
  float getMaxSpeed();
  float getMinSpeed();
  float getMaxAcceleration();
  float getMinAcceleration();
  float getMaxForceLimit();
  float getMinForceLimit();

  // Gripper Stats
  //
  float getWidth(void);
  float getSpeed(void);
  float getForce(void);
  int getTemperature(void);

  float getAcceleration(void);
  float getAccelerationFromCache(void);
  float getForceLimit(void);
  float getForceLimitFromCache(void);
  void getSoftLimits(
      float* softLimits);  // expects a float array with at least the size of 2
  void loop();
  bool isCommunicationOk(void);
  std::string getSystemInformation(void);
  std::string getSystemTag(void);

  /**
     *  ####################################
     *  ###### SYSTEM STATE COMMANDS #######
     *  ####################################
     */
  SSTATE getSystemState(bool updateOnChangeOnly, bool enableAutoUpdate,
                        short updatePeriodInMillisec);
  int getGraspingState(void);
  void getGraspingStateUpdates(bool updateOnChangeOnly, bool enableAutoUpdate,
                               short updatePeriodInMillisec);
  void getOpeningWidthUpdates(
      bool updateOnChangeOnly,       // 1 = yes; 0 = update always
      bool automaticUpdatesEnabled,  // 1 = yes; 0 = no
      short updatePeriodInMs);
  void getSpeedUpdates(bool updateOnChangeOnly,  // 1 = yes; 0 = update always
                       bool automaticUpdatesEnabled,  // 1 = yes; 0 = no
                       short updatePeriodInMs);
  void getForceUpdates(bool updateOnChangeOnly,  // 1 = yes; 0 = update always
                       bool automaticUpdatesEnabled,  // 1 = yes; 0 = no
                       short updatePeriodInMs);

  /**
     *  #####################################
     *  ###### OVERLOADING OBSERVER METHODS #
     *  #####################################
     */
  using WSG50Observer::update;
  void update(TRESPONSE* resp);

 private:
  std::string _IP;
  std::string _PORT;

  // variables
  //
  int _currentGraspingState;

  float _MaxWidth,  // mm
      _MinWidth, _MaxSpeed, _MinSpeed, _MaxForceLimit, _MinForceLimit,
      _MaxAcceleration, _MinAcceleration,
      _acceleration,    // mm/s²
      _speed,           // mm/s
      _softLimitMinus,  //
      _softLimitPlus,   //
      _forceLimit,      // N

      // current system data
      _currentOpeningWidth, _currentSpeed, _currentForce, _currentForceLimit,
      _currentTemperature;

  TMESSAGE _msg;
  TRESPONSE _resp;
  std::queue<TRESPONSE> _responseQueue;
  SSTATE _systemState;

  WSG50Communicator* _wsgComm;
  WSG50Observer* _observer;

  bool _checkingCommunication, _communicationOK, _ready,
      _systStatesReadyForCommand, _widthAutoUpdate, _speedAutoUpdate,
      _forceAutoUpdate, _graspingStateAutoUpdates, _softLimitsSet;
  unsigned char* _LoopTestData;
  unsigned char* _dat;
  int _LoopTestDataLength;

  // threading
  //
  std::mutex _msgMutex, _currentForceMutex, _currentSpeedMutex,
      _currentWidthMutex, _currentGraspingStateMutex, _responseMutex;

  // run State-Machine
  //
  void setupConnection();

  // Update handler
  //
  void updateHandler();
};