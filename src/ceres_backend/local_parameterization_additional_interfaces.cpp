// pimax_slam.pi.dll -- src/ceres_backend/local_parameterization_additional_interfaces.cpp  (draft c04)
//   pimax::totem::ceres_backend::LocalParamizationAdditionalInterfaces::verify
// Reconstructed by chunk c04_imu_error.  Object-file contribution 0x180050EF0 .. 0x180053DA8
// (verify at 0x180052D90 is the only non-template function; the Eigen kernels it instantiated are
// emitted in front of it: 0x180050EF0 .. 0x1800528F0).
//
// Upstream: rpg_svo_pro_open/svo_ceres_backend/src/local_parameterization_additional_interfaces.cpp
// Status: upstream-identical (constants 1e-12, 1e-9 (-> /2e-9), 1e-6 verified; same RowMajor matrices).
// verify() is slot 3 of the LocalParamizationAdditionalInterfaces sub-vtable of every Pimax local
// parameterization (HomogeneousPoint 0x1803ADF30, Pose 0x1803ADF90, Gravity 0x1803ADFF0).
// Object placement (B1 check): 0x180050EF0..0x180053DA8 lies between the last imu_error.obj function
// (ImuError::typeInfo 0x180050EE0) and the first marginalization_error.obj code (0x180053DB0), i.e.
// exactly where the alphabetical link order (imu_error < local_parameterization_additional_interfaces
// < marginalization_error) puts this object; verify() is a non-inline member (one definition only)
// and the Eigen kernels in front of it are only reachable from it.  No __FILE__ string in the range.
#include "ceres_backend/local_parameterization_additional_interfaces.hpp"

#include <cstring>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x180052D90
// Verifies the correctness of a inplementation.
bool LocalParamizationAdditionalInterfaces::verify(
    const double* x_raw, double purturbation_magnitude) const
{
  const ceres::LocalParameterization* casted =
      dynamic_cast<const ceres::LocalParameterization*>(this);
  if (!casted)
  {
    return false;
  }
  // verify plus/minus
  Eigen::VectorXd x(casted->GlobalSize());
  memcpy(x.data(), x_raw, sizeof(double) * casted->GlobalSize());
  Eigen::VectorXd delta_x(casted->LocalSize());
  Eigen::VectorXd x_plus_delta(casted->GlobalSize());
  Eigen::VectorXd delta_x2(casted->LocalSize());
  delta_x.setRandom();                       // rand()-based: (2.0*r)/32767.0 - 1.0
  delta_x *= purturbation_magnitude;
  casted->Plus(x.data(), delta_x.data(), x_plus_delta.data());
  this->Minus(x.data(), x_plus_delta.data(), delta_x2.data());
  if ((delta_x2 - delta_x).norm() > 1.0e-12)
  {
    return false;
  }

  // plusJacobian numDiff
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus_num_diff(
      casted->GlobalSize(), casted->LocalSize());
  const double dx = 1.0e-9;
  for (int i = 0; i < casted->LocalSize(); ++i)
  {
    Eigen::VectorXd delta_p(casted->LocalSize());
    delta_p.setZero();
    delta_p[i] = dx;
    Eigen::VectorXd delta_m(casted->LocalSize());
    delta_m.setZero();
    delta_m[i] = -dx;

    // reset
    Eigen::VectorXd x_p(casted->GlobalSize());
    Eigen::VectorXd x_m(casted->GlobalSize());
    memcpy(x_p.data(), x_raw, sizeof(double) * casted->GlobalSize());
    memcpy(x_m.data(), x_raw, sizeof(double) * casted->GlobalSize());
    casted->Plus(x.data(), delta_p.data(), x_p.data());
    casted->Plus(x.data(), delta_m.data(), x_m.data());
    J_plus_num_diff.col(i) = (x_p - x_m) / (2 * dx);
  }

  // verify lift
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus(casted->GlobalSize(),
                                                        casted->LocalSize());
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_lift(casted->LocalSize(),
                                                        casted->GlobalSize());
  casted->ComputeJacobian(x_raw, J_plus.data());
  ComputeLiftJacobian(x_raw, J_lift.data());
  Eigen::MatrixXd identity(casted->LocalSize(), casted->LocalSize());
  identity.setIdentity();
  if (((J_lift * J_plus) - identity).norm() > 1.0e-6)
  {
    return false;
  }

  // verify numDiff jacobian
  if ((J_plus - J_plus_num_diff).norm() > 1.0e-6)
  {
    return false;
  }

  // everything fine...
  return true;
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
