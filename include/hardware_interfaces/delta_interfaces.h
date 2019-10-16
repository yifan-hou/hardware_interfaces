/*
    DeltaInterfaces: virtual class with common interfaces with any delta robot.
    Provides basic read/write interfaces for robot poses and joints.

    Reference frames, positive directions follow the convention in

    TODO(Yifan):
      Jacobian interface

    Author:
        Yifan Hou <yifanh@cmu.edu>
*/

#ifndef _DELTA_INTERFACE_CLASS_HEADER_
#define _DELTA_INTERFACE_CLASS_HEADER_

class DeltaInterfaces {
public:
  DeltaInterfaces();
  ~DeltaInterfaces();
  /**
   * Constructs a Delta robot. Input arguments specify the dimensions.
   *
   * @param[in]  baseCenter2Edge      The distance from base center to base
   *                                  joint axis.
   * @param[in]  platformCenter2Edge  The distance from platform center to
   *                                  platform joint axis
   * @param[in]  upperLegLength       The base side leg length
   * @param[in]  lowerLegLength       The platform side leg length
   * @param[in]  modeUp               True if Z of the platform is higher than Z
   *                                  of base. Used for resolving ambiguities in
   *                                  forward kinematics
   * @param[in]  jointUpperLimit      The joint angle upper limits. Used for
   *                                  resolving ambiguities in inverse
   *                                  kinematics.
   * @param[in]  jointLowerLimit      The joint angle lower limits
   */
  void init(double baseCenter2Edge, double platformCenter2Edge,
    double upperLegLength, double lowerLegLength, bool modeUp,
    double *jointUpperLimit, double *jointLowerLimit);

  // ----------------------------------------
  //  user interfaces
  // ----------------------------------------

  /**
   * Gets the position of the platform measured in the base frame.
   *
   * @param      pos   The [x y z] position in mm.
   *
   * @return     0 if success. -1 if motor joints are not available. -2 if FK
   *             has no solution..
   */
  virtual int getPos(double *pos) = 0;
  /**
   * Sets the position of the platform about the base.
   *
   * @param[in]  pos   The [x y z] position in mm.
   *
   * @return     0 if success, -1 if set motor fails, -2 if fk has no solution.
   */
  virtual int setPos(const double *pos) = 0;
  /**
   * Gets the joint angles in rad.
   *
   * @param      joints  The joints [j1, j2, j3].
   *
   * @return     True if success.
   */
  virtual bool getJoints(double *joints) = 0;
  /**
   * Sets the joint angles in rad.
   *
   * @param[in]  joints  The joints [j1, j2, j3].
   *
   * @return     True if success.
   */
  virtual bool setJoints(const double *joints) = 0;

  // ----------------------------------------
  //  helper functions (implemented)
  // ----------------------------------------


  /**
   * Forward kinematics. Compute the platform location given joint angles. When
   * there are two solutions, the choice is made based on _kModeUp.
   * If _kModeUp = true, select the solution with higher z value, vice versa.
   *
   * @param[in]  theta  Array of size three, [j1, j2, j3] in rad.
   * @param[in]  p      Array of size three, [x, y, z] in mm.
   *
   * @return     True if a solution is found.
   */
  bool fk(const double *theta, double *p);
  /**
   * Inverse kinematics. Compute the three joint angles given the platform
   * location. There are two solutions for each joint angle, the function will
   * select one based on the joint angle limits.
   *
   * @param[in]  p      The [x, y, z] position of the platform.
   * @param      theta  Array of joint angles [j1, j2, j3]
   *
   * @return     True if a solution is found.
   */
  bool ik(const double *p, double *theta);


  // ----------------------------------------
  //  public state and parameters
  // ----------------------------------------

  /**
   * Indicates whether the initialization() function is called.
   */
  bool _isInitialized;

  /**
   * Dimensions of the Delta platform
   */
  double _kBaseCenter2Edge; // base, distance from center to motor axis
  double _kPlatformCenter2Edge; // platform, distance from center to axis
  double _kUpperLegLength; // upper legs length
  double _kLowerLegLength; // lower legs length

protected:
  /**
   * Internal parameters
   */
  double _k_uP;
  double _k_sP;
  double _a;
  double _b;
  double _c;

  bool _ModeUp;
  double *_JointUpperLimit;
  double *_JointLowerLimit;
};

#endif
