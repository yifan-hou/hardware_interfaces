#ifndef _DELTA_DJI_CLASS_HEADER_
#define _DELTA_DJI_CLASS_HEADER_

#include "hardware_interfaces/delta_interfaces.h"
#include "Robomaster/robomaster_communicator.h"

#include <ros/ros.h>


class DeltaDJI: public DeltaInterfaces {
public:
  DeltaDJI(ros::NodeHandle& root_nh);
  ~DeltaDJI(){}

  // ----------------------------------------
  //  user interfaces
  // ----------------------------------------

  /**
   * Gets the position of the platform measured in the base frame.
   *
   * @param      pos   The [x y z] position in mm.
   *
   * @return     0 if success. -1 if motor joints are not available. -2 if FK has
   *             no solution.
   */
  int getPos(double *pos) override;
  /**
   * Sets the position of the platform about the base.
   *
   * @param[in]  pos  The [x y z] position in mm.
   *
   * @return     0 if success, -1 if set motor fails, -2 if fk has no solution.
   */
  int setPos(const double *pos) override;
  /**
   * Gets the joint angles in rad.
   *
   * @param      joints  The joints [j1, j2, j3].
   *
   * @return     True if success.
   */
  bool getJoints(double *joints) override;
  /**
   * Sets the joint angles in rad.
   *
   * @param[in]  joints  The joints [j1, j2, j3].
   *
   * @return     True if success.
   */
  bool setJoints(const double *joints) override;


private:
  DeltaDJI(){};
  /**
   * Convert joint angles (delta robot convention) to motor commands. joint =
   * direction*(motor - motor0)
   *
   * @param[in]  joints  The joint angles in rad.
   * @param      motors  The motor commands
   */
  void joint2motor(const double *joints, double *motors);
  void motor2joint(const double *motors, double *joints);

  RobomasterCommunicator _robot;
  double *_motor_directions;
  double *_motor_offsets;
};

#endif
