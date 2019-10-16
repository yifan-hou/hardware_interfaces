#include "hardware_interfaces/delta_interfaces.h"

#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

double sqrt3 = sqrt(3);
double sqrt3_2 = sqrt3/2.0;
double sqrt3_3 = sqrt3/3.0;
double sqrt3_6 = sqrt3/6.0;

/**
 * Solve the 3D trilateration problem. Find the intersection of three spheres of
 * the same radius. c1, c2, c3 must not be colinear. Modified from
 * https://math.stackexchange.com/questions/786489/calculating-intersection-of-three-spheres-step-by-step
 * https://en.wikipedia.org/wiki/True_range_multilateration#Three_Cartesian_dimensions%2C_three_measured_slant_ranges
 *
 * @param[in]  c1    The center of sphere 1
 * @param[in]  c2    The center of sphere 2
 * @param[in]  c3    The center of sphere 3
 * @param[in]  r     radius of all the spheres
 * @param      p1    Intersection point 1
 * @param      p2    Intersection point 2
 *
 * @return     True if there is a solution.
 */
bool threeSphereIntersection(const Vector3d &c1, const Vector3d &c2,
    const Vector3d &c3, double r, Vector3d *p1, Vector3d *p2) {
  // Build a frame centered at c1
  Vector3d p0, ex, ey, ez;
  p0 = c1;
  ex = (c2 - c1).normalized();
  double i = (ex.transpose() * ( c3 - c1 ))(0, 0);
  ey = (c3 - c1 - i * ex).normalized();
  ez = ex.cross(ey);
  Matrix3d T;
  T << ex, ey, ez;

  // transform to this frame
  double d = (T.transpose()*(c2 - p0))(0);
  double j = (T.transpose()*(c3 - p0))(1);

  // solve
  double x = d/2.0;
  double y = (i*i + j*j)/(2.0*j) - i/j*x;
  double z = sqrt(r*r - x*x - y*y);

  // transform back
  Vector3d p1_t, p2_t;
  p1_t << x, y, z;
  p2_t << x, y, -z;
  *p1 = T * p1_t + p0;
  *p2 = T * p2_t + p0;

  return true;
}

DeltaInterfaces::DeltaInterfaces() {
  _JointUpperLimit = new double[3];
  _JointLowerLimit = new double[3];
}

void DeltaInterfaces::init(double baseCenter2Edge,
    double platformCenter2Edge,
    double upperLegLength, double lowerLegLength, bool modeUp,
    double *jointUpperLimit, double *jointLowerLimit) {
  _kBaseCenter2Edge = baseCenter2Edge;
  _kPlatformCenter2Edge = platformCenter2Edge;
  _kUpperLegLength = upperLegLength;
  _kLowerLegLength = lowerLegLength;

  _k_uP = 2.0*_kPlatformCenter2Edge;
  _k_sP = sqrt3*_k_uP;
  _a = _kBaseCenter2Edge - _k_uP;
  _b = _k_sP*0.5 - sqrt3_2*_kBaseCenter2Edge;
  _c = _kPlatformCenter2Edge - 0.5*_kBaseCenter2Edge;

  _ModeUp = modeUp;
  for (int i = 0; i < 3; ++i) {
    _JointUpperLimit[i] = jointUpperLimit[i];
    _JointLowerLimit[i] = jointLowerLimit[i];
  }

}

DeltaInterfaces::~DeltaInterfaces() {
  delete [] _JointUpperLimit;
  delete [] _JointLowerLimit;
}

bool DeltaInterfaces::fk(const double *theta, double *p) {
  // Virtual sphere centers
  Vector3d p_BA_1v, p_BA_2v, p_BA_3v;
  p_BA_1v << 0,
      -_kBaseCenter2Edge-_kUpperLegLength*cos(theta[0])+_k_uP,
      -_kUpperLegLength*sin(theta[0]);
  p_BA_2v << sqrt3_2*(_kBaseCenter2Edge+_kUpperLegLength*cos(theta[1])) - _k_sP/2.0,
      0.5*(_kBaseCenter2Edge + _kUpperLegLength*cos(theta[1])) - _kPlatformCenter2Edge,
      -_kUpperLegLength*sin(theta[1]);
  p_BA_3v << -sqrt3_2*(_kBaseCenter2Edge + _kUpperLegLength*cos(theta[2])) + _k_sP/2.0,
      0.5*(_kBaseCenter2Edge + _kUpperLegLength*cos(theta[2])) - _kPlatformCenter2Edge,
      - _kUpperLegLength*sin(theta[2]);

  // solve the three-spheres problem
  Vector3d p1, p2;
  if(!threeSphereIntersection(p_BA_1v, p_BA_2v, p_BA_3v, _kLowerLegLength,
      &p1, &p2))
    return false;

  p[0] = p1(0);
  p[1] = p1(1);
  p[2] = p1(2);
  if ( (_ModeUp && p1(2)<p2(2)) || (!_ModeUp && p1(2) > p2(2)) ) {
    p[0] = p2(0);
    p[1] = p2(1);
    p[2] = p2(2);
  }
  return true;
}

bool DeltaInterfaces::ik(const double *p, double *theta) {
  double x = p[0];
  double y = p[1];
  double z = p[2];

  double E1 = 2.0*_kUpperLegLength*(y + _a);
  double F1 = 2.0*z*_kUpperLegLength;
  double G1 = x*x + y*y + z*z + _a*_a + _kUpperLegLength*_kUpperLegLength + 2.0*y*_a - _kLowerLegLength*_kLowerLegLength;
  double Delta1 = sqrt(E1*E1 + F1*F1 - G1*G1);

  double E2 = -_kUpperLegLength*(sqrt3*(x+_b) + y + _c);
  double F2 = F1;
  double G2 = x*x + y*y + z*z + _b*_b + _c*_c +
      _kUpperLegLength*_kUpperLegLength + 2.0*(x*_b + y*_c) -
      _kLowerLegLength*_kLowerLegLength;
  double Delta2 = sqrt(E2*E2 + F2*F2 - G2*G2);

  double E3 = _kUpperLegLength*(sqrt3*(x-_b) - y - _c);
  double F3 = F1;
  double G3 = x*x + y*y + z*z + _b*_b + _c*_c + _kUpperLegLength*_kUpperLegLength + 2.0*(-x*_b + y*_c) - _kLowerLegLength*_kLowerLegLength;
  double Delta3 = sqrt(E3*E3 + F3*F3 - G3*G3);

  theta[0] = 2.0*atan((-F1 + Delta1)/(G1 - E1));
  theta[1] = 2.0*atan((-F2 + Delta2)/(G2 - E2));
  theta[2] = 2.0*atan((-F3 + Delta3)/(G3 - E3));
  if ((theta[0] > _JointUpperLimit[0]) || (theta[0] < _JointLowerLimit[0])) {
    theta[0] = 2.0*atan((-F1 - Delta1)/(G1 - E1));
  }
  if ((theta[1] > _JointUpperLimit[1]) || (theta[1] < _JointLowerLimit[1])) {
    theta[1] = 2.0*atan((-F2 - Delta2)/(G2 - E2));
  }
  if ((theta[2] > _JointUpperLimit[2]) || (theta[2] < _JointLowerLimit[2])) {
    theta[2] = 2.0*atan((-F3 - Delta3)/(G3 - E3));
  }
  return true;
}
