#include "delta_dji/delta_dji.h"

#define PI 3.1415926

using std::cout, std::endl;

DeltaDJI::DeltaDJI(ros::NodeHandle& root_nh) {
  _robot.init("can0");
  cout<<"Robomaster_communicator initialized\n";

  /**
   * Geometry of the delta robot
   */
  double BaseCenter2Edge, PlatformCenter2Edge, UpperLegLength, LowerLegLength;
  double default_joints, jointUpperLimit[3], jointLowerLimit[3];
  bool ModeUp;
  if (!root_nh.hasParam("/delta/base_center2edge"))
    ROS_WARN_STREAM("Parameter [/delta/base_center2edge] not found");
  if (!root_nh.hasParam("/delta/platform_center2edge"))
    ROS_WARN_STREAM("Parameter [/delta/platform_center2edge] not found");
  if (!root_nh.hasParam("/delta/upper_leg_length"))
    ROS_WARN_STREAM("Parameter [/delta/upper_leg_length] not found");
  if (!root_nh.hasParam("/delta/lower_leg_length"))
    ROS_WARN_STREAM("Parameter [/delta/lower_leg_length] not found");
  if (!root_nh.hasParam("/delta/modeUp"))
    ROS_WARN_STREAM("Parameter [/delta/modeUp] not found");
  if (!root_nh.hasParam("/delta/default_joints"))
    ROS_WARN_STREAM("Parameter [/delta/default_joints] not found");
  if (!root_nh.hasParam("/delta/jointUpperLimit"))
    ROS_WARN_STREAM("Parameter [/delta/jointUpperLimit] not found");
  if (!root_nh.hasParam("/delta/jointLowerLimit"))
    ROS_WARN_STREAM("Parameter [/delta/jointLowerLimit] not found");

  root_nh.param(std::string("/delta/base_center2edge"), BaseCenter2Edge, 0.1);
  root_nh.param(std::string("/delta/platform_center2edge"), PlatformCenter2Edge, 0.1);
  root_nh.param(std::string("/delta/upper_leg_length"), UpperLegLength, 0.1);
  root_nh.param(std::string("/delta/lower_leg_length"), LowerLegLength, 0.1);
  root_nh.param(std::string("/delta/modeUp"), ModeUp, true);
  root_nh.param(std::string("/delta/default_joints"), default_joints, 0.0);
  root_nh.param(std::string("/delta/jointUpperLimit/j1"), jointUpperLimit[0], 0.0);
  root_nh.param(std::string("/delta/jointUpperLimit/j2"), jointUpperLimit[1], 0.0);
  root_nh.param(std::string("/delta/jointUpperLimit/j3"), jointUpperLimit[2], 0.0);
  root_nh.param(std::string("/delta/jointLowerLimit/j1"), jointLowerLimit[0], 0.0);
  root_nh.param(std::string("/delta/jointLowerLimit/j2"), jointLowerLimit[1], 0.0);
  root_nh.param(std::string("/delta/jointLowerLimit/j3"), jointLowerLimit[2], 0.0);

  // initialize delta kinematics
  init(BaseCenter2Edge, PlatformCenter2Edge, UpperLegLength, LowerLegLength,
      ModeUp, jointUpperLimit, jointLowerLimit);

  /**
   * Parameters for DJI delta robot
   */
  if (!root_nh.hasParam("/delta/motor1"))
    ROS_WARN_STREAM("Parameter [/delta/motor1] not found");
  if (!root_nh.hasParam("/delta/motor2"))
    ROS_WARN_STREAM("Parameter [/delta/motor2] not found");
  if (!root_nh.hasParam("/delta/PGain"))
    ROS_WARN_STREAM("Parameter [/delta/PGain] not found");
  if (!root_nh.hasParam("/delta/DGain"))
    ROS_WARN_STREAM("Parameter [/delta/DGain] not found");

  double Kp, Kd;
  int rate;
  _motor_offsets = new double[3];
  _motor_directions = new double[3];

  double offset[3], direction[3];
  root_nh.param(std::string("/delta/motor1/offset"), _motor_offsets[0], 0.0);
  root_nh.param(std::string("/delta/motor2/offset"), _motor_offsets[1], 0.0);
  root_nh.param(std::string("/delta/motor3/offset"), _motor_offsets[2], 0.0);
  root_nh.param(std::string("/delta/motor1/direction"), _motor_directions[0], 1.0);
  root_nh.param(std::string("/delta/motor2/direction"), _motor_directions[1], 1.0);
  root_nh.param(std::string("/delta/motor3/direction"), _motor_directions[2], 1.0);
  root_nh.param(std::string("/delta/control_rate"), rate, 100);
  root_nh.param(std::string("/delta/PGain"), Kp, 10.0);
  root_nh.param(std::string("/delta/DGain"), Kd, 1.0);

  for (int i = 0; i < 3; ++i)
    _motor_offsets[i] *= PI/180.0;

  default_joints *= PI/180.0;
  double default_joints_array[3] = {default_joints, default_joints, default_joints};
  setJoints(default_joints_array);
  _robot.set_gains(Kp, 0, Kd);
  _robot.spin(rate);
}

DeltaDJI::~DeltaDJI() {
  delete [] _motor_offsets;
  delete [] _motor_directions;
}

int DeltaDJI::getPos(double *pos) {
  double joints[3];
  if(!getJoints(joints))
    return -1;
  if(!fk(joints, pos))
    return -2;
  return 0;
}

int DeltaDJI::setPos(const double *pos) {
  double joints[3];
  if(!ik(pos, joints))
    return -2;
  if(!setJoints(joints))
    return -1;
  return 0;
}

bool DeltaDJI::getJoints(double *joints) {
  // robomaster_communicator motor counts from 1
  double motor[3];
  motor[0] = _robot.get_position(1);
  motor[1] = _robot.get_position(2);
  motor[2] = _robot.get_position(3);
  motor2joint(motor, joints);
  return true;
}

bool DeltaDJI::setJoints(const double *joints) {
  double motor[3];
  joint2motor(joints, motor);
  _robot.set_command_position(1, motor[0]);
  _robot.set_command_position(2, motor[1]);
  _robot.set_command_position(3, motor[2]);
  return true;
}

void DeltaDJI::joint2motor(const double *joints, double *motors) {
  motors[0] = joints[0]/_motor_directions[0] + _motor_offsets[0];
  motors[1] = joints[1]/_motor_directions[1] + _motor_offsets[1];
  motors[2] = joints[2]/_motor_directions[2] + _motor_offsets[2];
}

void DeltaDJI::motor2joint(const double *motors, double *joints) {
  for (int i = 0; i < 3; ++i) {
    // DJI motor: 0 ~ 2pi
    joints[i] = _motor_directions[i]*(motors[i] - _motor_offsets[i]);
  }
}
