// pimax_slam.pi.dll -- src/common/sophus/so3ex_base.h
//
// Original path: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\common\sophus\so3ex_base.h
// (two __FILE__ spellings exist: "src\common\sophus\so3ex_base.h" 0x1803AF640 from the
// imu_error TU, "src\common/sophus/so3ex_base.h" 0x1803B3D90 from the frame_processor_base TU;
// the text depends on how the including TU spells the include, nothing to reproduce here).
//
// Pimax-modified Sophus SO3: SO3<Scalar,Options> derives from SO3exBase<SO3<...>> (__FUNCTION__
// strings "Sophus::SO3exBase<class Sophus::SO3<double,0> >::log" / "::normalize" and
// "Sophus::SO3<double,0>::exp").  Only the members visible in the binary are reconstructed.
// Reconciled from the c03 draft (ctor/normalize, operator*, exp(omega, eps), log, hat, matrix,
// alternatingSeries) and the c09 draft (exp(omega) with the "SO3::exp failed!" ensure).
//
// glog/ensure line numbers (they are printed by the ensure handler):
//   log -> 109, normalize -> 125, exp -> 303.  Forced with #line below.
#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/sophus/common.hpp"

namespace Sophus {

// Pimax helper used by SO3::exp(omega, eps) (inlined) and by common/so3_gamma.h.
// Out-of-line copy at 0x1800409E0 (imu_error object).  Name and location are guesses.
// Returns  sum_{j>=0} (-1)^j * x^(p+2j) / (n+2j)!   (x2 = x*x unless given), stopping at the first
// term whose magnitude is <= tol (that term is not added).  Default x2 = NaN (0x1803AF8E8),
// tol = DBL_EPSILON.
// TODO(verify): real name / header.
inline double alternatingSeries(const double& x, int n, int p,
                                double x2 = std::numeric_limits<double>::quiet_NaN(),
                                const double& tol = std::numeric_limits<double>::epsilon())
{
  double sum = 0.0;
  double term = std::pow(x, p) / std::tgamma(n + 1);
  if (std::isnan(x2)) x2 = x * x;
  while (std::fabs(term) > tol) {
    ++n;
    sum += term;
    term = -term * x2 / static_cast<double>(n * (n + 1));
    ++n;
  }
  return sum;
}

template <class Scalar_, int Options = 0>
class SO3;

template <class T>
struct so3_traits;

template <class Scalar_, int Options>
struct so3_traits<SO3<Scalar_, Options>> {
  using Scalar = Scalar_;
};

template <class Derived>
class SO3exBase
{
public:
  using Scalar = typename so3_traits<Derived>::Scalar;
  using Tangent = Eigen::Matrix<Scalar, 3, 1>;
  using Transformation = Eigen::Matrix<Scalar, 3, 3>;
  using QuaternionType = Eigen::Quaternion<Scalar>;

  const QuaternionType& unit_quaternion() const
  {
    return static_cast<const Derived*>(this)->unit_quaternion();
  }

  // 0x180042530  newer-Sophus logAndTheta (atan2 wrap); ensure line 109.
  Tangent log() const
  {
    const Scalar squared_n = unit_quaternion().vec().squaredNorm();
    const Scalar w = unit_quaternion().w();
    Scalar two_atan_nbyw_by_n;
    if (squared_n < Constants<Scalar>::epsilon() * Constants<Scalar>::epsilon())
    {
#line 109
      SOPHUS_ENSURE(std::abs(w) >= Constants<Scalar>::epsilon(), "Quaternion ({}) should be normalized!", unit_quaternion().coeffs().transpose());
      two_atan_nbyw_by_n = Scalar(2) / w - Scalar(2.0 / 3.0) * (squared_n) / (w * w * w);
    }
    else
    {
      const Scalar n = std::sqrt(squared_n);
      const Scalar atan_nbyw = (w < Scalar(0)) ? Scalar(std::atan2(-n, -w)) : Scalar(std::atan2(n, w));
      two_atan_nbyw_by_n = Scalar(2) * atan_nbyw / n;
    }
    return two_atan_nbyw_by_n * unit_quaternion().vec();
  }

  // 0x1800423F0  (-> toRotationMatrix 0x180029DF0; callers 0x18009B240, 0x1800A0AB0)
  Transformation matrix() const { return unit_quaternion().toRotationMatrix(); }

  // 0x18002E710  quaternion product, then the normalising SO3(q) ctor.
  template <class OtherDerived>
  SO3<Scalar> operator*(const SO3exBase<OtherDerived>& other) const;

protected:
  QuaternionType& unit_quaternion_nonconst()
  {
    return static_cast<Derived*>(this)->unit_quaternion_nonconst();
  }

  // inlined into 0x18002E0E0 / 0x18002E710 / 0x180042060; ensure line 125.
  void normalize()
  {
    const Scalar length = unit_quaternion_nonconst().norm();
#line 125
    SOPHUS_ENSURE(length >= Constants<Scalar>::epsilon(), "Quaternion ({}) should not be close to zero!", unit_quaternion_nonconst().coeffs().transpose());
    unit_quaternion_nonconst().coeffs() /= length;
  }
};

template <class Scalar_, int Options>
class SO3 : public SO3exBase<SO3<Scalar_, Options>>
{
  using Base = SO3exBase<SO3<Scalar_, Options>>;
  friend Base;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Scalar = Scalar_;
  using Tangent = typename Base::Tangent;
  using Transformation = typename Base::Transformation;
  using QuaternionMember = Eigen::Quaternion<Scalar, Options>;

  SO3() : unit_quaternion_(Scalar(1), Scalar(0), Scalar(0), Scalar(0)) {}

  // 0x18002E0E0  (copy, then SO3exBase::normalize with the line-125 ensure)
  explicit SO3(const QuaternionMember& quat) : unit_quaternion_(quat) { Base::normalize(); }

  const QuaternionMember& unit_quaternion() const { return unit_quaternion_; }

  // 0x180042410  (comma initializer, CommaInitializer<Matrix3d>::operator, = 0x18003A120)
  // (ICF-identical to vk::skew)
  static Transformation hat(const Tangent& omega)
  {
    Transformation Omega;
    Omega << Scalar(0), -omega(2), omega(1),
             omega(2), Scalar(0), -omega(0),
             -omega(1), omega(0), Scalar(0);
    return Omega;
  }

  // 0x180042060  Pimax: exp with explicit small-angle tolerance; the small-angle branch uses the
  // alternating series (inlined) instead of upstream's 4th-order Taylor polynomial.
  // Callers (imu_error 0x1800427F0, 0x180045750) pass eps = 1e-5 (0x1803AF4F8).
  // TODO(verify): real name (exp / expTol / ...).
  static SO3 exp(const Tangent& omega, const Scalar& eps)
  {
    const Scalar theta = omega.norm();
    Scalar imag_factor;
    Scalar real_factor;
    if (theta < eps)
    {
      imag_factor = alternatingSeries(theta, 1, 0, theta * theta * 0.25) * 0.5;  // sin(theta/2)/theta
      const Scalar half_theta = theta * 0.5;
      real_factor = alternatingSeries(half_theta, 0, 0);                          // cos(theta/2)
    }
    else
    {
      const Scalar half_theta = theta * 0.5;
      imag_factor = std::sin(half_theta) / theta;
      real_factor = std::cos(half_theta);
    }
    return SO3(QuaternionMember(real_factor, imag_factor * omega.x(), imag_factor * omega.y(),
                                imag_factor * omega.z()));
  }

  // 0x18010A400  (only caller in the binary: IntegrationBase::midPointIntegration)
  // Sophus SO3::expAndTheta without the theta output.  The binary calls cos() before sin()
  // in the large-angle branch (upstream Sophus computes sin first) -- kept.  Ensure line 303.
  static SO3 exp(const Tangent& omega)
  {
    using std::abs;
    using std::cos;
    using std::sin;
    using std::sqrt;
    const Scalar theta_sq = omega.squaredNorm();

    Scalar imag_factor;
    Scalar real_factor;
    if (theta_sq < Constants<Scalar>::epsilon() * Constants<Scalar>::epsilon())
    {
      const Scalar theta_po4 = theta_sq * theta_sq;
      imag_factor = Scalar(0.5) - Scalar(1.0 / 48.0) * theta_sq +
                    Scalar(1.0 / 3840.0) * theta_po4;
      real_factor = Scalar(1) - Scalar(1.0 / 8.0) * theta_sq +
                    Scalar(1.0 / 384.0) * theta_po4;
    }
    else
    {
      const Scalar theta = sqrt(theta_sq);
      const Scalar half_theta = Scalar(0.5) * theta;
      real_factor = cos(half_theta);
      const Scalar sin_half_theta = sin(half_theta);
      imag_factor = sin_half_theta / theta;
    }

    SO3 q;
    q.unit_quaternion_nonconst() =
        QuaternionMember(real_factor, imag_factor * omega.x(), imag_factor * omega.y(),
                         imag_factor * omega.z());
#line 303
    SOPHUS_ENSURE(abs(q.unit_quaternion().squaredNorm() - Scalar(1)) < Constants<Scalar>::epsilon(), "SO3::exp failed! omega: {}, real: {}, img: {}", omega.transpose(), real_factor, imag_factor);
    return q;
  }

protected:
  QuaternionMember& unit_quaternion_nonconst() { return unit_quaternion_; }

  QuaternionMember unit_quaternion_;
};

// 0x18002E710
template <class Derived>
template <class OtherDerived>
SO3<typename SO3exBase<Derived>::Scalar> SO3exBase<Derived>::operator*(
    const SO3exBase<OtherDerived>& other) const
{
  const QuaternionType& a = unit_quaternion();
  const QuaternionType& b = other.unit_quaternion();
  return SO3<Scalar>(QuaternionType(a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
                                    a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
                                    a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
                                    a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x()));
}

using SO3d = SO3<double>;
using SO3f = SO3<float>;

static_assert(sizeof(SO3d) == 32, "Sophus::SO3d holds one Quaterniond");

}  // namespace Sophus
