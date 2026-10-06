// pimax_slam.pi.dll -- src/frontend/utility.h  (drafts c08 + c13 + c15 merged)
//
// VINS-Mono "Utility" helpers (utility/utility.h), Pimax variant.  Header-only; instantiations:
//   Utility::deltaQ<Product<Matrix3d, Vector3d - Vector3d>>   0x1800CBD00 (frame_processor_base.obj)
//   Utility::skewSymmetric<Vector3d>                          0x180042410 (comma initializer)
//   Utility::skewSymmetric<Block<const Vector4d/Quaternion coeffs,3,1>>  0x1800DDA80 (inside Qleft/Qright)
//   Utility::Qleft<Quaterniond>                               0x1800B8A50 (w*I + skew)
//   Utility::Qright<Quaterniond>                              0x1800B8C00 (w*I - skew)
//   Utility::R2ypr(R, bool radians)                           0x1800F3260 (frame_processor_base.obj;
//                                                             inlined into g2R / loop closing)
//   Utility::ypr2R<Vector3d>(ypr)                             0x18014DB90 (visual_imu_alignment.obj)
//   Utility::ypr2R<Vector3d>(ypr, bool is_radian)             0x18017EE10 (loop_closing.obj, c15)
//   Utility::g2R                                              0x1801572E0 (visual_imu_alignment.obj)
//   TangentBasis(const Vector3d&) 0x18001ACE0 is in ceres_backend/gravity_local_parameterization.hpp
//   (namespace pimax::totem).
// NOTE: 0x1800DDCF0 (frame_processor_base.obj, called by initializeImu "diff yaw") has the same
// signature as ypr2R(ypr, bool) but computes Rz(e0)*Rx(e1)*Ry(e2): it is a different function
// (c07 name `eulerToRotation`, drafted in frame_processor_base_part_templates.cpp) and must NOT be
// spelled Utility::ypr2R (c09's draft does) -- otherwise the two COMDATs would be merged.
// TODO(verify): the original header name/location (no __FILE__ string).
#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace pimax {
namespace totem {

class Utility
{
 public:
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived>& theta)
  {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived>& q)
  {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
  }

  // 0x1800F3260 (this chunk) -- VINS-Mono Utility::R2ypr with a Pimax-added flag: when
  // `radians` is true the angles are returned in radians, otherwise in degrees
  // (ypr / M_PI * 180.0, constants 3.141592653589793 and 180.0 as {x,x} vectors).
  // Callers: 0x180110490 (VINS-like IMU init) and 0x18018D770. Call order in the binary:
  // atan2, sin(y), cos(y), atan2, atan2.
  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R, bool radians = false)
  {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    if (radians)
      return ypr;
    return ypr / M_PI * 180.0;
  }

  // 0x18014DB90
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived>& ypr)
  {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
          0., 1., 0.,
          -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
          0., cos(r), -sin(r),
          0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  // 0x18017ee10 (instantiated for Eigen::Vector3d; caller ReLocalize 0x18018d770)
  // project template.  R = Rz(yaw) * Ry(pitch) * Rx(roll).
  // Constants: 180.0 (0x1803b6d38), M_PI (0x1803b1390).  TODO(verify) class/namespace.
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(
      const Eigen::MatrixBase<Derived>& ypr, bool is_radian)
  {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0);
    Scalar_t p = ypr(1);
    Scalar_t r = ypr(2);
    if (!is_radian)
    {
      y = y / 180.0 * M_PI;
      p = p / 180.0 * M_PI;
      r = r / 180.0 * M_PI;
    }

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
          0., 1., 0.,
          -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
          0., cos(r), -sin(r),
          0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  // 0x1801572E0
  static Eigen::Matrix3d g2R(const Eigen::Vector3d& g)
  {
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();   // 0x1800DD6E0 + 0x180029DF0
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
  }
  template <typename QType>
  static Eigen::Quaternion<typename QType::Scalar> positify(const Eigen::QuaternionBase<QType>& q)
  {
    return q;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived>& q)
  {
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec(),
                                ans.template block<3, 3>(1, 1) =
                                    qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
                                    skewSymmetric(qq.vec());
    return ans;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived>& p)
  {
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec(),
                                ans.template block<3, 3>(1, 1) =
                                    pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
                                    skewSymmetric(pp.vec());
    return ans;
  }
};

}  // namespace totem
}  // namespace pimax
