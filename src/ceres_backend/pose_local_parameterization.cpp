// pimax_slam.pi.dll -- src/ceres_backend/pose_local_parameterization.cpp  (draft c05)
//
// Object 0x18008C980..0x18008D960.  Port of
// svo_ceres_backend/src/pose_local_parameterization.cpp, edited to match the binary:
//   * minus():        q_diff is normalised before the log map            (pimax change)
//   * plusJacobian(): q is normalised before quaternionOplusMatrix()     (pimax change)
//   * liftJacobian(): q_inv is normalised before quaternionOplusMatrix() (pimax change)
//   * plus():         upstream structure, but the exp map is the 1e-12-threshold helper
//                     quaternionExp() (common/transformation.h; small-angle branch 0.5 + theta^2/48,
//                     no norm CHECK) and the product is normalised with Eigen's normalize()
//                     (`if (n > 0) q /= sqrt(n)`, checked in 0x18008D270).
//   * no DEBUG_CHECK_NEAR, VerifyJacobianNumDiff not emitted.
// quaternionOplusMatrix = 0x1800453F0, Quaterniond::normalized() = 0x1800426F0 (both chunk c04).
#include "ceres_backend/pose_local_parameterization.hpp"

#include <cmath>

#include <Eigen/Geometry>

#include "common/transformation.h"
#include "ceres_backend/matrix_operations.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x18008CC40  (folded with PoseParameterBlock::plus)
// Generalization of the addition operation,
//        x_plus_delta = Plus(x, delta)
//        with the condition that Plus(x, 0) = x.
bool PoseLocalParameterization::Plus(const double* x, const double* delta,
                                     double* x_plus_delta) const
{
  return plus(x, delta, x_plus_delta);
}

// 0x18008CC20  (folded with PoseParameterBlock::minus)
// Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
bool PoseLocalParameterization::Minus(const double* x,
                                      const double* x_plus_delta,
                                      double* delta) const
{
  return minus(x, x_plus_delta, delta);
}

// 0x18008C990
// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization::ComputeLiftJacobian(const double* x,
                                                    double* jacobian) const
{
  return liftJacobian(x, jacobian);
}

// 0x18008D270  upstream-modified (quaternionExp + explicit normalize, see file comment).
bool PoseLocalParameterization::plus(const double* x, const double* delta,
                                     double* x_plus_delta)
{

  Eigen::Map<const Eigen::Matrix<double, 6, 1> > delta_(delta);

  Quaternion q(x[6], x[3], x[4], x[5]);
  q = quaternionExp(delta_.tail<3>()) * q;
  q.normalize();

  // copy back
  x_plus_delta[0] = x[0] + delta[0];
  x_plus_delta[1] = x[1] + delta[1];
  x_plus_delta[2] = x[2] + delta[2];
  x_plus_delta[3] = q.x();
  x_plus_delta[4] = q.y();
  x_plus_delta[5] = q.z();
  x_plus_delta[6] = q.w();

  return true;
}

// 0x18008CEE0  upstream-modified: q_diff.normalized().
// Computes the minimal difference between a variable x and a perturbed variable x_plus_delta.
bool PoseLocalParameterization::minus(const double* x,
                                      const double* x_plus_delta,
                                      double* delta)
{
  delta[0] = x_plus_delta[0] - x[0];
  delta[1] = x_plus_delta[1] - x[1];
  delta[2] = x_plus_delta[2] - x[2];
  Eigen::Map<const Eigen::Quaterniond> q_plus_delta_(&x_plus_delta[3]);
  Eigen::Map<const Eigen::Quaterniond> q_(&x[3]);
  Eigen::Map<Eigen::Vector3d> omega(&delta[3]);

  Eigen::Quaterniond q_diff = (q_plus_delta_ * q_.inverse()).normalized();

  // Quaternion implementation part copied from GTSAM.
  // define these compile time constants to avoid std::abs:
  static const double twoPi = 2.0 * M_PI, NearlyOne = 1.0 - 1e-10,
  NearlyNegativeOne = -1.0 + 1e-10;

  const double qw = q_diff.w();
  // See Quaternion-Logmap.nb in doc for Taylor expansions
  if (qw > NearlyOne)
  {
    // Taylor expansion of (angle / s) at 1
    // (2 + 2 * (1-qw) / 3) * q.vec();
    omega = ( 8. / 3. - 2. / 3. * qw) * q_diff.vec();
  }
  else if (qw < NearlyNegativeOne)
  {
    // Taylor expansion of (angle / s) at -1
    // (-2 - 2 * (1 + qw) / 3) * q.vec();
    omega = (-8. / 3. - 2. / 3. * qw) * q_diff.vec();
  }
  else
  {
    // Normal, away from zero case
    double angle = 2 * std::acos(qw), s = std::sqrt(1 - qw * qw);
    // Important:  convert to [-pi,pi] to keep error continuous
    if (angle > M_PI)
    {
      angle -= twoPi;
    }
    else if (angle < -M_PI)
    {
      angle += twoPi;
    }
    omega = (angle / s) * q_diff.vec();
  }
  return true;
}

// 0x18008D570  upstream-modified: q normalised.
// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization::plusJacobian(const double* x,
                                             double* jacobian)
{
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > Jp(jacobian);
  Jp.setZero();

  // Translational part:
  Jp.topLeftCorner<3, 3>().setIdentity();

  // Rotation:
  // exp(dalpha) x q \approx [dalpha/2; 1] x q
  // \approx ([0 0 0 1]^T + 0.5 * I_3x4 * dalpha) x q
  // \approx oplus(q) * ([0 0 0 1]^T + 0.5 * I_4x3 * dalpha)
  // => derivative wrt dalpha = oplus(q) * 0.5 * I_4x3
  const Eigen::Quaterniond q = Eigen::Map<const Eigen::Quaterniond>(&x[3]).normalized();
  Jp.bottomRightCorner<4, 3>() = 0.5 * quaternionOplusMatrix(q) *
                                 Eigen::Matrix<double, 4, 3>::Identity();
  return true;
}

// 0x18008CC60  upstream-modified: q_inv normalised.
// Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
bool PoseLocalParameterization::liftJacobian(const double* x,
                                             double* jacobian)
{
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > J_lift(jacobian);
  // Translational part.
  J_lift.setZero();
  J_lift.topLeftCorner<3, 3>().setIdentity();

  const Eigen::Quaterniond q_inv(x[6], -x[3], -x[4], -x[5]);
  Eigen::Matrix4d Qplus = quaternionOplusMatrix(q_inv.normalized());
  J_lift.bottomRightCorner<3, 4>() = 2.0 * Qplus.topLeftCorner<3, 4>();
  return true;
}

// 0x18008C980
// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
bool PoseLocalParameterization::ComputeJacobian(const double* x,
                                                double* jacobian) const
{
  return plusJacobian(x, jacobian);
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
