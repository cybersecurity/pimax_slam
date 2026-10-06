// pimax_slam.pi.dll -- src/ceres_backend/pose_error.cpp  (draft c05)
//
// Object 0x18008AD40..0x18008C980:
//   0x18008AD40..0x18008B930  lib: Eigen::LLT<Matrix<double,6,6>> compute / blocked / unblocked,
//                             TriangularView helpers, Map<Matrix<6,1>> ctor check
//   0x18008B9E0  lib: ceres::SizedCostFunction<6,7>::SizedCostFunction()
//   0x18008BAC0  PoseError::PoseError(const Transformation&, const Matrix6d&)
//   0x18008BBD0  thunk -> 0x18008BBE0 PoseError scalar deleting dtor
//   0x18008BC30  PoseError::EvaluateWithMinimalJacobians
//   0x18008C600/0x18008C620/0x18008C640  lib: triangular (upper) assignment kernel of L^T
//   0x18008C820  PoseError::setInformation
//   0x18008C970  PoseError::typeInfo
#include "ceres_backend/pose_error.hpp"

#include <Eigen/Cholesky>

#include "ceres_backend/estimator_types.hpp"
#include "ceres_backend/pose_local_parameterization.hpp"
#include "ceres_backend/matrix_operations.hpp"   // quaternionPlusMatrix (comma-initialiser version)

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x18008BAC0  upstream-identical.
// Construct with measurement and information matrix.
PoseError::PoseError(const Transformation& measurement,
                     const Eigen::Matrix<double, 6, 6>& information)
{
  setMeasurement(measurement);
  setInformation(information);
}

// 0x18008C820  upstream-modified: information_ / covariance_ (inverse()) are gone.
// Set the information.
void PoseError::setInformation(const information_t& information)
{
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information);
  square_root_information_ = lltOfInformation.matrixL().transpose();
}

// folded into 0x18003AE00 (identical code for ImuError / PoseError / SpeedAndBiasError).
// upstream-modified: dispatch on ErrorInterface::use_minimal_jacobians_.
// This evaluates the error term and additionally computes the Jacobians.
bool PoseError::Evaluate(double const* const * parameters, double* residuals,
                         double** jacobians) const
{
  if (use_minimal_jacobians_)
  {
    return EvaluateWithMinimalJacobians(parameters, residuals, nullptr, jacobians);
  }
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// 0x18008BC30  upstream-modified:
//   * the input quaternion is normalised (Eigen normalized()) before building T_WS;
//   * the residual is NOT sqrt_info * error: only
//       r[0..2] = (p_meas - p) * sqrt_info(0,0),   r[3] = r[4] = 0,
//       r[5]    = dtheta.z     * sqrt_info(5,5)            (position + yaw prior)
//   * J0_minimal = [ -sqrt_info(0,0) * I3 , 0 ; 0 , 0 ; 0 , -sqrt_info(5,5) * Q(2,0..2) ]
//     with Q = quaternionPlusMatrix(dp)  (only row 5 of the rotation block);
//   * computed when (jacobians || jacobians_minimal); the minimal one is written first and
//     independently of jacobians[0].
// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool PoseError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                             double* residuals,
                                             double** jacobians,
                                             double** jacobians_minimal) const
{
  // compute error
  Transformation T_WS(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4],
                         parameters[0][5]).normalized());
  // delta pose
  Transformation dp = measurement_ * T_WS.inverse();
  // get the error
  const Eigen::Vector3d dtheta = 2 * dp.getRotation().imaginary();

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 6, 1> > weighted_error(residuals);
  weighted_error.setZero();
  weighted_error.head<3>() =
      (measurement_.getPosition() - T_WS.getPosition()) * square_root_information_(0, 0);
  weighted_error[5] = dtheta[2] * square_root_information_(5, 5);

  // compute Jacobian...
  if (jacobians != nullptr || jacobians_minimal != nullptr)
  {
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J0_minimal;
    J0_minimal.setZero();
    J0_minimal.topLeftCorner<3, 3>() =
        square_root_information_(0, 0) * -Eigen::Matrix3d::Identity();
    const Eigen::Matrix4d Q = quaternionPlusMatrix(dp.getEigenQuaternion());
    J0_minimal.block<1, 3>(5, 3) =
        -square_root_information_(5, 5) * Q.topLeftCorner<3, 3>().row(2);

    if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr)
    {
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >
          J0_minimal_mapped(jacobians_minimal[0]);
      J0_minimal_mapped = J0_minimal;
    }

    if (jacobians != nullptr && jacobians[0] != nullptr)
    {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> >
          J0(jacobians[0]);

      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

      // hallucinate Jacobian w.r.t. state
      J0 = J0_minimal * J_lift;
    }
  }

  return true;
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
