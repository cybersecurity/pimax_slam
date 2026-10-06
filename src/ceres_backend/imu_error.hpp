// pimax_slam.pi.dll -- src/ceres_backend/imu_error.hpp
//
// Pimax fork of svo_ceres_backend/include/svo/ceres_backend/imu_error.hpp (OKVIS ImuError).
// Reconciled from drafts c03 (imu_error_c03.hpp: ctor 0x1800398A0, Evaluate 0x18003AE00,
// EvaluateWithMinimalJacobians 0x18003AE40, deltaQ 0x180041ED0) and c04 (imu_error.h:
// propagation 0x1800427F0, redoPreintegration 0x180045750).  c04's corrections are applied:
//   * propagation() is a NON-static member (call site 0x180026743: rcx = ImuError, 10 args): it
//     reads timestamps from its argument but gyro/acc from this->imu_measurements_ (quirk), takes
//     the world gravity g_W explicitly and no longer subtracts imu_params.delay_imu_cam.
//   * redoPreintegration(speed_and_biases, const Eigen::Vector3d* g_S0): the binary null-checks the
//     2nd argument (`if (g_S0)` twice) -> pointer (c03 had a reference).
// Other Pimax changes: 5 parameter blocks SizedCostFunction<15,7,9,7,9,3> (5th = gravity block);
// extra speed&bias reference argument in the ctor; closed-form (Gamma-function) SO(3) integration;
// rotation residual = SO3 log; information_ via LLT solveInPlace; no mutex locking at all.
//
// Layout (sizeof 0x18B0 = 6320; make_shared allocates 0x18C0; `_Ref_count_obj2<ImuError>` vtable
// 0x1803AEFB0; deleting dtor 0x18003AD60 uses free()):
//   +0    ceres::SizedCostFunction<15,7,9,7,9,3> (vtable 0x1803AF4B0: [dtor 0x18003AD60,
//         Evaluate 0x18003AE00]); +8 parameter_block_sizes_ {7,9,7,9,3}; +32 num_residuals_ 15
//   +40   ErrorInterface (vtable 0x1803AF4C8: [thunk 0x18003AD54, residualDim 0x18004E420,
//         parameterBlocks 0x180014980, parameterBlockDim 0x180014950,
//         EvaluateWithMinimalJacobians 0x18003AE40, typeInfo 0x180050EE0]); +48 bool flag
//   +56.. members, see the static_asserts.
#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/sized_cost_function.h>

#include "common/imu_calibration.h"           // ImuMeasurement(s) (32-byte float samples)
#include "common/transformation.h"
#include "ceres_backend/error_interface.hpp"
#include "ceres_backend/estimator_types.hpp"  // ImuParameters, SpeedAndBias

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x180041ED0 is the out-of-line copy of deltaQ (sinc inlined) -- upstream-identical header code.
__inline__ double sinc(double x)
{
  if (fabs(x) > 1e-6)
  {
    return sin(x) / x;
  }
  else
  {
    static const double c_2 = 1.0 / 6.0;
    static const double c_4 = 1.0 / 120.0;
    static const double c_6 = 1.0 / 5040.0;
    const double x_2 = x * x;
    const double x_4 = x_2 * x_2;
    const double x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

// 0x180041ED0
__inline__ Eigen::Quaterniond deltaQ(const Eigen::Vector3d& dAlpha)
{
  Eigen::Vector4d dq;
  double halfnorm = 0.5 * dAlpha.template tail<3>().norm();
  dq.template head<3>() = sinc(halfnorm) * 0.5 * dAlpha.template tail<3>();
  dq[3] = cos(halfnorm);
  return Eigen::Quaterniond(dq);
}

/// \brief Implements a nonlinear IMU factor.  Pimax: 5th parameter block = gravity vector in W
/// (GravityParameterBlock, 3 dof, minimal 2 dof) replaces upstream's constant (0,0,g).
class ImuError :
    public ceres::SizedCostFunction<15, 7, 9, 7, 9, 3>,
    public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ceres::SizedCostFunction<15, 7, 9, 7, 9, 3> base_t;
  static const int kNumResiduals = 15;
  typedef Eigen::Matrix<double, 15, 15> covariance_t;
  typedef covariance_t information_t;
  typedef Eigen::Matrix<double, 15, 15> jacobian_t;

  ImuError() = default;
  virtual ~ImuError() = default;   // 0x18003AD60 / thunk 0x18003AD54

  /// 0x1800398A0.  Pimax-new 5th argument (copied into speed_and_biases_ref_).
  ImuError(const ImuMeasurements& imu_measurements, const ImuParameters& imu_parameters,
           const double& t_0, const double& t_1, const SpeedAndBias& speed_and_biases_ref);

  /// 0x1800427F0 -- non-static (see header comment); called by Estimator::addStates with
  /// covariance = jacobian = nullptr.
  int propagation(const ImuMeasurements& imu_measurements, const ImuParameters& imu_params,
                  Transformation& T_WS, SpeedAndBias& speed_and_biases, const double& t_start,
                  const double& t_end, const Eigen::Vector3d& g_W,
                  covariance_t* covariance = nullptr, jacobian_t* jacobian = nullptr);

  /// 0x180045750 -- called from EvaluateWithMinimalJacobians with &(C_S0_W * (-g_W)).
  int redoPreintegration(const SpeedAndBias& speed_and_biases, const Eigen::Vector3d* g_S0) const;

  void setRedo(const bool redo = true) const { redo_ = redo; }

  void setImuParameters(const ImuParameters& imuParameters) { imu_parameters_ = imuParameters; }
  void setImuMeasurements(const ImuMeasurements& imu_measurements)
  {
    imu_measurements_ = imu_measurements;   // deque assign 0x180036F50
  }
  void setT0(const double& t_0) { t0_ = t_0; }
  void setT1(const double& t_1) { t1_ = t_1; }

  const ImuParameters& imuParameters() const { return imu_parameters_; }
  const ImuMeasurements& imuMeasurements() const { return imu_measurements_; }
  double t0() const { return t0_; }
  double t1() const { return t1_; }

  /// 0x18003AE00 (ICF-shared with PoseError / SpeedAndBiasError)
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;

  /// 0x18003AE40
  virtual bool EvaluateWithMinimalJacobians(double const* const* parameters, double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const;

  /// 0x18004E420
  size_t residualDim() const { return kNumResiduals; }
  /// 0x180014980 (shared)
  size_t parameterBlocks() const { return parameter_block_sizes().size(); }
  /// 0x180014950 (shared)
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }
  /// 0x180050EE0 (returns 5)
  virtual ErrorType typeInfo() const { return ErrorType::kIMUError; }

 protected:
  ImuParameters imu_parameters_;            ///< +56  (128 bytes)
  ImuMeasurements imu_measurements_;        ///< +184 (MSVC deque, 40 bytes)
  double t0_;                               ///< +224 (no delay subtraction)
  double t1_;                               ///< +232
  mutable std::mutex preintegration_mutex_; ///< +240 (80 bytes; _Mtx_init_in_situ(.., 2); never locked)

  // increments (Delta_q_ = (x,y,z,w) = (0,0,0,1) in the ctor)
  mutable Eigen::Quaterniond Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);  ///< +320
  mutable Eigen::Matrix3d C_integral_ = Eigen::Matrix3d::Zero();         ///< +352 (= -J(6,12) after redo)
  mutable Eigen::Matrix3d C_doubleintegral_ = Eigen::Matrix3d::Zero();   ///< +424 (= -J(0,12))
  mutable Eigen::Vector3d acc_integral_ = Eigen::Vector3d::Zero();       ///< +496 (delta v)
  mutable Eigen::Vector3d acc_doubleintegral_ = Eigen::Vector3d::Zero(); ///< +520 (delta p)
  mutable Eigen::Matrix3d cross_ = Eigen::Matrix3d::Zero();              ///< +544 (never written by redo)
  mutable Eigen::Matrix3d dalpha_db_g_ = Eigen::Matrix3d::Zero();        ///< +616 (= -C*J(3,9))
  mutable Eigen::Matrix3d dv_db_g_ = Eigen::Matrix3d::Zero();            ///< +688 (= J(6,9))
  mutable Eigen::Matrix3d dp_db_g_ = Eigen::Matrix3d::Zero();            ///< +760 (= J(0,9))
  mutable Eigen::Matrix<double, 15, 15> P_delta_ =
      Eigen::Matrix<double, 15, 15>::Zero();                             ///< +832
  mutable SpeedAndBias speed_and_biases_ref_ = SpeedAndBias::Zero();     ///< +2632
  mutable bool redo_ = true;                                             ///< +2704
  mutable int redoCounter_ = 0;                                          ///< +2708
  mutable information_t information_;                                    ///< +2712 (8-aligned)
  mutable information_t square_root_information_;                        ///< +4512

 private:
  friend struct ImuErrorLayoutCheck;
};

struct ImuErrorLayoutCheck
{
  static_assert(sizeof(ImuError) == 0x18B0, "");
  static_assert(offsetof(ImuError, imu_parameters_) == 56, "");
  static_assert(offsetof(ImuError, imu_measurements_) == 184, "");
  static_assert(offsetof(ImuError, t0_) == 224, "");
  static_assert(offsetof(ImuError, t1_) == 232, "");
  static_assert(offsetof(ImuError, preintegration_mutex_) == 240, "");
  static_assert(offsetof(ImuError, Delta_q_) == 320, "");
  static_assert(offsetof(ImuError, C_integral_) == 352, "");
  static_assert(offsetof(ImuError, C_doubleintegral_) == 424, "");
  static_assert(offsetof(ImuError, acc_integral_) == 496, "");
  static_assert(offsetof(ImuError, acc_doubleintegral_) == 520, "");
  static_assert(offsetof(ImuError, cross_) == 544, "");
  static_assert(offsetof(ImuError, dalpha_db_g_) == 616, "");
  static_assert(offsetof(ImuError, dv_db_g_) == 688, "");
  static_assert(offsetof(ImuError, dp_db_g_) == 760, "");
  static_assert(offsetof(ImuError, P_delta_) == 832, "");
  static_assert(offsetof(ImuError, speed_and_biases_ref_) == 2632, "");
  static_assert(offsetof(ImuError, redo_) == 2704, "");
  static_assert(offsetof(ImuError, redoCounter_) == 2708, "");
  static_assert(offsetof(ImuError, information_) == 2712, "");
  static_assert(offsetof(ImuError, square_root_information_) == 4512, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
