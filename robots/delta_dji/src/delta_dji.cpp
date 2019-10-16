#include "delta_dji/delta_dji.h"

DeltaDJI::DeltaDJI(ros::NodeHandle& root_nh) {
  _robot.init("can0");
  cout<<"Robomaster_communicator initialized\n";

  double BaseCenter2Edge, PlatformCenter2Edge, UpperLegLength, LowerLegLength;
  double jointUpperLimit[3], jointLowerLimit[3];
  bool ModeUp;
  root_nh.param(std::string("/delta/base_center2edge"), BaseCenter2Edge, 0.1);
  root_nh.param(std::string("/delta/platform_center2edge"), PlatformCenter2Edge, 0.1);
  root_nh.param(std::string("/delta/upper_leg_length"), UpperLegLength, 0.1);
  root_nh.param(std::string("/delta/lower_leg_length"), LowerLegLength, 0.1);
  root_nh.param(std::string("/delta/modeUp"), ModeUp, true);
  root_nh.param(std::string("/delta/jointUpperLimit/j1"), jointUpperLimit[0], 0);
  root_nh.param(std::string("/delta/jointUpperLimit/j2"), jointUpperLimit[1], 0);
  root_nh.param(std::string("/delta/jointUpperLimit/j3"), jointUpperLimit[2], 0);
  root_nh.param(std::string("/delta/jointLowerLimit/j1"), jointLowerLimit[0], 0);
  root_nh.param(std::string("/delta/jointLowerLimit/j2"), jointLowerLimit[1], 0);
  root_nh.param(std::string("/delta/jointLowerLimit/j3"), jointLowerLimit[2], 0);

  init(BaseCenter2Edge, PlatformCenter2Edge, UpperLegLength, LowerLegLength,
      ModeUp, jointUpperLimit, jointLowerLimit);


  root_nh.param(std::string("/delta/motor1/offset"), jointLowerLimit[2], 0);

  delta:
  motor1:
    offset: 30 # degree: motor angle at joint = 0 pose
    direction: 1 # is the motor positive direction the same with the joint

  // (TODO)YIfan: get default position
  // double setpoint = M_PI/2;
  // double Kp = 50;
  // double Kd = 0.1;
  // _robot.set_command_position(1, setpoint);
  // _robot.set_command_position(2, setpoint);
  // _robot.set_command_position(3, setpoint);
  // cout<<"Set command position\n";
  // _robot.set_gains(Kp, 0, Kd);
  // cout<<"Set gains position\n";

  _robot.spin();
}


int DeltaDJI::getPos(double *pos) {
  double joints[3];
  int error_code = 0;
  if(!getJoints(joints))
    return -1;
  if(!fk(joints, pos))
    return -2;
  return true;
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

  return motor2joint(motor, joints);
}

bool DeltaDJI::setJoints(const double *joints) {
  double motor[3];
  if(joint2motor(joints, motor)) {
    _robot.set_command_position(1, motor[0]);
    _robot.set_command_position(2, motor[1]);
    _robot.set_command_position(3, motor[2]);
    return true;
  } else {
    return false;
  }
}

bool DeltaDJI::joint2motor(const double *joints, double *motors) {

}

bool DeltaDJI::motor2joint(const double *motors, double *joints) {
}
