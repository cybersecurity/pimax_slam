// pimax_slam.pi.dll -- src/ceres_backend/speed_and_bias_error.cpp  (draft c05)
//
// Object 0x18008DB80..0x18008FE60:
//   0x18008DB80..0x18008E760  lib: Eigen::LLT<Matrix<double,9,9>> (compute/blocked/unblocked,
//                             l1 norm, gemv, Map/Block ctor checks)
//   0x18008E810  lib: ceres::SizedCostFunction<9,9>::SizedCostFunction()
//   0x18008E8F0  SpeedAndBiasError(const SpeedAndBias&, const information_t&)
//   0x18008E980  SpeedAndBiasError(const SpeedAndBias&, double, double, double)
//   0x18008EC08/0x18008EC20  dtor thunk / scalar deleting dtor
//   0x18008EC70  EvaluateWithMinimalJacobians
//   0x18008F450..0x18008F8F0  lib: triangular (upper) assignment of L^T (9x9)
//   0x18008FB10  setInformation
//   0x18008F490  residualDim (9)     0x18008FE50 typeInfo (2)
#include "ceres_backend/speed_and_bias_error.hpp"

#include <Eigen/Cholesky>

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x18008E8F0  upstream-identical
// Construct with measurement and information matrix
SpeedAndBiasError::SpeedAndBiasError(const SpeedAndBias& measurement,
                                     const information_t& information)
{
  setMeasurement(measurement);
  setInformation(information);
}

// 0x18008E980  upstream-identical (minus the DEBUG_CHECK_NEs)
// Construct with measurement and variance.
SpeedAndBiasError::SpeedAndBiasError(const SpeedAndBias& measurement,
                                     double speed_variance,
                                     double gyr_bias_variance,
                                     double acc_bias_variance)
{
  setMeasurement(measurement);

  information_t information;
  information.setZero();
  information.topLeftCorner<3, 3>() =
      Eigen::Matrix3d::Identity() * 1.0 / speed_variance;
  information.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * 1.0 / gyr_bias_variance;
  information.bottomRightCorner<3, 3>() =
      Eigen::Matrix3d::Identity() * 1.0 / acc_bias_variance;
    setInformation(information);
}

// 0x18008FB10  upstream-identical
// Set the information.
void SpeedAndBiasError::setInformation(const information_t& information)
{
  information_ = information;
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();
}

// folded into 0x18003AE00, see PoseError::Evaluate.
bool SpeedAndBiasError::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  if (use_minimal_jacobians_)
  {
    return EvaluateWithMinimalJacobians(parameters, residuals, nullptr, jacobians);
  }
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// 0x18008EC70  upstream-modified:
//   * residual weighting uses only the three diagonal entries sqrt_info(0,0), (3,3), (6,6)
//     (one scalar per 3-block) instead of the full 9x9 product;
//   * J0 = J0min = -square_root_information_ (upstream: -sqrt_info * Identity, a real product).
bool SpeedAndBiasError::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  // compute error
  Eigen::Map<const SpeedAndBias> estimate(parameters[0]);
  SpeedAndBias error = measurement_ - estimate;

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 9, 1> > weighted_error(residuals);
  weighted_error.head<3>() = square_root_information_(0, 0) * error.head<3>();
  weighted_error.segment<3>(3) = square_root_information_(3, 3) * error.segment<3>(3);
  weighted_error.tail(3) = square_root_information_(6, 6) * error.tail(3);

  // compute Jacobian - this is rather trivial in this case...
  if (jacobians != nullptr && jacobians[0] != nullptr)
  {
    Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> > J0(jacobians[0]);
    J0 = -square_root_information_;
  }
  if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr)
  {
    Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> >
        J0min(jacobians_minimal[0]);
    J0min = -square_root_information_;
  }

  return true;
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
