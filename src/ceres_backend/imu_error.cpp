// pimax_slam.pi.dll -- src/ceres_backend/imu_error.cpp
// __FILE__ = "E:\code_codex\pimax_slam\beta111_5a7902_dll\src\ceres_backend\imu_error.cpp"
//
// Pimax fork of svo_ceres_backend/src/imu_error.cpp (OKVIS ImuError), namespace
// pimax::totem::ceres_backend.  Merged (phase B1) from
//   draft c03 imu_error_c03.cpp : ctor 0x1800398A0, Evaluate 0x18003AE00, EvaluateWithMinimalJacobians 0x18003AE40
//   draft c04 imu_error.cpp     : propagation 0x1800427F0, redoPreintegration 0x180045750
// Object imu_error.obj: ~0x18002E0A0 (template/helper COMDATs first, see c03 notes) .. 0x180050EE3.
// Functions are kept in binary order (MSVC emits them in source order).
// Header-inline members/helpers emitted in this object: deltaQ 0x180041ED0 (imu_error.hpp),
// residualDim 0x18004E420 / typeInfo 0x180050EE0 (imu_error.hpp), quaternionOplusMatrix 0x1800453F0 /
// quaternionPlusMatrix 0x1800455A0 (matrix_operations.hpp), the SO(3)/Gamma helpers
// (common/so3_gamma.h, common/sophus/so3ex_base.h).
//
// glog __LINE__s forced with #line: LOG(WARNING) 488 / 495 in propagation (checked in 0x1800427F0).
#include "ceres_backend/imu_error.hpp"

#include <cmath>

#include <glog/logging.h>

#include "common/logger.h"                                 // LOGE 0x18000C2C0
#include "common/so3_gamma.h"                              // gamma1/2/3, gamma1Right, dGammaTV2/3/4, dGammaV1/2/3
#include "common/sophus/so3ex_base.h"                      // Sophus::SO3d (log 0x180042530, exp 0x180042060, hat)
#include "ceres_backend/gravity_local_parameterization.hpp" // TangentBasis 0x18001ACE0
#include "ceres_backend/matrix_operations.hpp"             // skewSymmetric 0x180016490, quaternionPlus/OplusMatrix
#include "ceres_backend/pose_local_parameterization.hpp"   // liftJacobian 0x18008CC60

namespace pimax {
namespace totem {
namespace ceres_backend {

namespace {
// Small-angle thresholds; the binary passes the ADDRESSES of these .rdata doubles (const double&).
const double kImuEps1e5 = 1e-5;   // 0x1803AF4F8
const double kImuEps1e3 = 1e-3;   // 0x1803AF500
const double kImuEps2e2 = 0.02;   // 0x1803AF508
const double kImuEps6e2 = 0.06;   // 0x1803AF510
const double kImuEps1e1 = 0.1;    // 0x1803AF518
}  // namespace

// 0x1800398A0
// upstream-modified: SizedCostFunction<15,7,9,7,9,3> (extra gravity block), extra argument
// speed_and_biases_ref (stored in speed_and_biases_ref_), no lock_guard, no delay_imu_cam subtraction,
// no DEBUG_CHECKs.
ImuError::ImuError(const ImuMeasurements& imu_measurements,
                   const ImuParameters& imu_parameters,
                   const double& t_0, const double& t_1,
                   const SpeedAndBias& speed_and_biases_ref)
{
  setImuMeasurements(imu_measurements);      // deque operator= (0x180036F50 = assign(first,last))
  setImuParameters(imu_parameters);
  setT0(t_0);
  setT1(t_1);
  speed_and_biases_ref_ = speed_and_biases_ref;
}

// 0x18003AD60 (scalar deleting dtor; 0x18003AD54 = thunk from the ErrorInterface sub-object)
// ImuError::~ImuError() = default;  -> ~mutex (_Mtx_destroy_in_situ +240), ~deque (+184),
//                                      ~CostFunction (0x1801b77d0), aligned free (size 0x18B0).

// 0x18003AE00 (identical code folded with PoseError::Evaluate and SpeedAndBiasError::Evaluate)
// upstream-modified: honours ErrorInterface::use_minimal_jacobians_ (+48): when set, the Jacobians
// ceres asks for are filled with the *minimal* Jacobians.
bool ImuError::Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const
{
  if (use_minimal_jacobians_)
  {
    return EvaluateWithMinimalJacobians(parameters, residuals, nullptr, jacobians);
  }
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// 0x18003AE40
// upstream-modified:
//  * gravity g_W comes from parameters[4] instead of (0,0,imu_parameters_.g);
//  * input quaternions are normalized before building T_WS_0 / T_WS_1;
//  * NO preintegration_mutex_ locking at all;
//  * redoPreintegration(speed_and_biases_0, C_S0_W * (-g_W)) and Delta_b is recomputed (not zeroed);
//  * Dq is normalized;
//  * rotation residual = SO3(Dq * q_WS_1^-1 * q_WS_0).log() instead of 2*vec();
//  * all Jacobians are computed up front; minimal Jacobians are written first, then the full ones;
//  * Jacobians for the gravity block (15x3 full, 15x2 minimal = J4 * tangentBasis(g_W)).
bool ImuError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const
{
  // get poses
  const Transformation T_WS_0(
        Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
        Eigen::Quaterniond(parameters[0][6], parameters[0][3],
                           parameters[0][4], parameters[0][5]).normalized());

  const Transformation T_WS_1(
        Eigen::Vector3d(parameters[2][0], parameters[2][1], parameters[2][2]),
        Eigen::Quaterniond(parameters[2][6], parameters[2][3],
                           parameters[2][4], parameters[2][5]).normalized());

  // get speed and bias
  SpeedAndBias speed_and_biases_0;
  SpeedAndBias speed_and_biases_1;
  for (size_t i = 0; i < 9; ++i)
  {
    speed_and_biases_0[i] = parameters[1][i];
    speed_and_biases_1[i] = parameters[3][i];
  }
  // Pimax: gravity parameter block
  const Eigen::Vector3d g_W(parameters[4][0], parameters[4][1], parameters[4][2]);

  // this will NOT be changed:
  const Eigen::Matrix3d C_WS_0 = T_WS_0.getRotationMatrix();
  const Eigen::Matrix3d C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double delta_t = t1_ - t0_;
  Eigen::Matrix<double, 6, 1> Delta_b;
  Delta_b = speed_and_biases_0.tail<6>() - speed_and_biases_ref_.tail<6>();
  redo_ = redo_ || (Delta_b.head<3>().norm() * delta_t > 0.0001);
  if (redo_)
  {
    // c04 correction: the 2nd argument is a pointer (null-checked inside); the binary passes the
    // address of the evaluated temporary C_S0_W * (-g_W) (0x18003AE40: call 0x180045750 with &tmp).
    const Eigen::Vector3d g_S0 = C_S0_W * (-g_W);
    redoPreintegration(speed_and_biases_0, &g_S0);
    redoCounter_++;
    Delta_b = speed_and_biases_0.tail<6>() - speed_and_biases_ref_.tail<6>();
    redo_ = false;
  }

  // actual propagation output:
  {
    // assign Jacobian w.r.t. x0
    Eigen::Matrix<double, 15, 15> F0 =
        Eigen::Matrix<double, 15, 15>::Identity(); // holds for d/db_g, d/db_a
    const Eigen::Vector3d delta_p_est_W =
        T_WS_0.getPosition() - T_WS_1.getPosition()
        + speed_and_biases_0.head<3>() * delta_t - 0.5 * delta_t * delta_t * g_W;
    const Eigen::Vector3d delta_v_est_W = speed_and_biases_0.head<3>()
        - speed_and_biases_1.head<3>() - g_W * delta_t;
    const Eigen::Quaterniond Dq =
        (deltaQ(-dalpha_db_g_ * Delta_b.head<3>()) * Delta_q_).normalized();
    F0.block<3, 3>(0, 0) = C_S0_W;
    F0.block<3, 3>(0, 3) = C_S0_W * skewSymmetric(delta_p_est_W);
    F0.block<3, 3>(0, 6) = C_S0_W * delta_t;   // upstream: C_S0_W * Identity * delta_t
    F0.block<3, 3>(0, 9) = dp_db_g_;
    F0.block<3, 3>(0, 12) = -C_doubleintegral_;
    F0.block<3, 3>(3, 3) =
        (quaternionPlusMatrix(Dq * T_WS_1.getEigenQuaternion().inverse()) *
         quaternionOplusMatrix(T_WS_0.getEigenQuaternion())).topLeftCorner<3, 3>();
    F0.block<3, 3>(3, 9) =
        (quaternionOplusMatrix(T_WS_1.getEigenQuaternion().inverse() *
                               T_WS_0.getEigenQuaternion()) *
         quaternionOplusMatrix(Dq)).topLeftCorner<3, 3>() * (-dalpha_db_g_);
    F0.block<3, 3>(6, 3) = C_S0_W * skewSymmetric(delta_v_est_W);
    F0.block<3, 3>(6, 6) = C_S0_W;
    F0.block<3, 3>(6, 9) = dv_db_g_;
    F0.block<3, 3>(6, 12) = -C_integral_;

    // assign Jacobian w.r.t. x1
    Eigen::Matrix<double, 15, 15> F1 =
        -Eigen::Matrix<double, 15, 15>::Identity(); // holds for the biases
    F1.block<3, 3>(0, 0) = -C_S0_W;
    F1.block<3, 3>(3, 3) =
        -(quaternionPlusMatrix(Dq) *
          quaternionOplusMatrix(T_WS_0.getEigenQuaternion()) *
          quaternionPlusMatrix(T_WS_1.getEigenQuaternion().inverse()))
        .topLeftCorner<3, 3>();
    F1.block<3, 3>(6, 6) = -C_S0_W;

    // the overall error vector
    Eigen::Matrix<double, 15, 1> error;
    error.segment<3>(0) =
        C_S0_W * delta_p_est_W + acc_doubleintegral_ + F0.block<3, 6>(0, 9) * Delta_b;
    // Pimax: SO(3) log instead of 2*vec(); Sophus ensure "Quaternion ({}) should not be close to
    // zero!" in the ctor (0x18002E0E0), "should be normalized!" in log (0x180042530).
    error.segment<3>(3) =
        Sophus::SO3d(Dq * T_WS_1.getEigenQuaternion().inverse() *
                     T_WS_0.getEigenQuaternion()).log();
    error.segment<3>(6) =
        C_S0_W * delta_v_est_W + acc_integral_ + F0.block<3, 6>(6, 9) * Delta_b;
    error.tail<6>() = speed_and_biases_0.tail<6>() - speed_and_biases_1.tail<6>();

    // error weighting
    Eigen::Map<Eigen::Matrix<double, 15, 1>> weighted_error(residuals);
    weighted_error = square_root_information_ * error;

    // get the Jacobians
    if (jacobians != nullptr || jacobians_minimal != nullptr)
    {
      Eigen::Matrix<double, 15, 6> J0_minimal;
      Eigen::Matrix<double, 15, 6> J2_minimal;
      // TODO(verify): the binary re-tests the pointers in the opposite order here (redundant).
      if (jacobians_minimal != nullptr || jacobians != nullptr)
      {
        Eigen::Matrix<double, 15, 9> J1;
        Eigen::Matrix<double, 15, 9> J3;
        J0_minimal = square_root_information_ * F0.block<15, 6>(0, 0);
        J1 = square_root_information_ * F0.block<15, 9>(0, 6);
        J2_minimal = square_root_information_ * F1.block<15, 6>(0, 0);
        J3 = square_root_information_ * F1.block<15, 9>(0, 6);

        // Pimax: Jacobian w.r.t. gravity g_W (3-dim parameter block)
        Eigen::Matrix<double, 15, 3> J4 = Eigen::Matrix<double, 15, 3>::Zero();
        J4.block<3, 3>(0, 0) = -0.5 * delta_t * delta_t * C_S0_W;
        J4.block<3, 3>(6, 0) = -C_S0_W * delta_t;
        J4 = square_root_information_ * J4;

        // minimal Jacobians first
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[0] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>>
                J0_minimal_mapped(jacobians_minimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
          if (jacobians_minimal[1] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
                J1_minimal_mapped(jacobians_minimal[1]);
            J1_minimal_mapped = J1;
          }
          if (jacobians_minimal[2] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>>
                J2_minimal_mapped(jacobians_minimal[2]);
            J2_minimal_mapped = J2_minimal;
          }
          if (jacobians_minimal[3] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
                J3_minimal_mapped(jacobians_minimal[3]);
            J3_minimal_mapped = J3;
          }
          if (jacobians_minimal[4] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 2, Eigen::RowMajor>>
                J4_minimal_mapped(jacobians_minimal[4]);
            // MatrixXd(3,2) then move-assign (free(0)+alloc(6)+swap pattern, cf. 0x18001A620)
            Eigen::MatrixXd basis(3, 2);
            basis = TangentBasis(g_W);  // 0x18001ACE0 (gravity_local_parameterization.hpp)
            J4_minimal_mapped = J4 * basis;   // product type Matrix<double,15,Dynamic>
          }
        }

        if (jacobians != nullptr)
        {
          if (jacobians[0] != nullptr)
          {
            // pseudo inverse of the local parametrization Jacobian:
            Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
            PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

            // hallucinate Jacobian w.r.t. state
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J0(jacobians[0]);
            J0 = J0_minimal * J_lift;
          }
          if (jacobians[1] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J1_mapped(jacobians[1]);
            J1_mapped = J1;
          }
          if (jacobians[2] != nullptr)
          {
            Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
            PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> J2(jacobians[2]);
            J2 = J2_minimal * J_lift;
          }
          if (jacobians[3] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> J3_mapped(jacobians[3]);
            J3_mapped = J3;
          }
          if (jacobians[4] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J4_mapped(jacobians[4]);
            J4_mapped = J4;
          }
        }
      }
    }
  }
  return true;
}

// 0x1800427F0
// Pimax: non-static; timestamps come from the argument, the samples from this->imu_measurements_.
// The covariance pointer only gates the saturation warnings (no covariance / jacobian is produced any
// more); the jacobian pointer is unused.  The caller (Estimator, 0x180026100) passes the ImuError it
// just constructed from the same measurements, covariance = jacobian = nullptr.
int ImuError::propagation(const ImuMeasurements& imu_measurements, const ImuParameters& imu_params,
                          Transformation& T_WS, SpeedAndBias& speed_and_biases,
                          const double& t_start, const double& t_end, const Eigen::Vector3d& g,
                          covariance_t* covariance, jacobian_t* /*jacobian*/)
{
  const double t_start_adjusted = t_start;   // Pimax: no "- imu_params.delay_imu_cam"
  const double t_end_adjusted = t_end;
  if (imu_measurements.front().timestamp_ < t_end_adjusted)
  {
    LOGE("replace assert, imu_measurements.front().timestamp_ < t_end_adjusted\n");
    return -1;  // nothing to do...
  }

  // initial condition
  Eigen::Vector3d r_0 = T_WS.getPosition();
  Eigen::Quaterniond q_WS_0 = T_WS.getEigenQuaternion();
  Eigen::Matrix3d C_WS_0 = T_WS.getRotationMatrix();

  // increments (initialise with identity)
  Eigen::Quaterniond Delta_q(1, 0, 0, 0);
  Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();

  double Delta_t = 0;
  bool has_started = false;
  int num_propagated = 0;

  // gravity expressed in the start frame S0
  const Eigen::Vector3d g_W = g;
  const Eigen::Vector3d g_S = C_WS_0.transpose() * (-g_W);

  double time = t_start_adjusted;
  for (size_t i = imu_measurements.size() - 1; i != 0u; --i)
  {
    // bias-corrected samples (float -> double), taken from the MEMBER deque (quirk)
    Eigen::Vector3d omega_S_0 = imu_measurements_[i].angular_velocity_.cast<double>()
        - speed_and_biases.segment<3>(3);
    Eigen::Vector3d acc_S_0 = imu_measurements_[i].linear_acceleration_.cast<double>()
        - speed_and_biases.segment<3>(6);
    Eigen::Vector3d omega_S_1 = imu_measurements_[i - 1].angular_velocity_.cast<double>()
        - speed_and_biases.segment<3>(3);
    Eigen::Vector3d acc_S_1 = imu_measurements_[i - 1].linear_acceleration_.cast<double>()
        - speed_and_biases.segment<3>(6);
    double nexttime = imu_measurements[i - 1].timestamp_;

    // time delta
    double dt = nexttime - time;

    if (t_end_adjusted < nexttime)
    {
      double interval = nexttime - imu_measurements[i].timestamp_;
      nexttime = t_end_adjusted;
      dt = nexttime - time;
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0)
    {
      continue;
    }
    Delta_t += dt;

    if (!has_started)
    {
      has_started = true;
      const double r = dt / (nexttime - imu_measurements[i].timestamp_);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity (Pimax: only the warnings survive, the sigmas are not inflated any more;
    // note the bias-corrected samples are tested)
    if (covariance)
    {
      if (std::abs(omega_S_0[0]) > imu_params.g_max
          || std::abs(omega_S_0[1]) > imu_params.g_max
          || std::abs(omega_S_0[2]) > imu_params.g_max
          || std::abs(omega_S_1[0]) > imu_params.g_max
          || std::abs(omega_S_1[1]) > imu_params.g_max
          || std::abs(omega_S_1[2]) > imu_params.g_max)
      {
#line 488
        LOG(WARNING) << "gyr saturation";   // imu_error.cpp:488
      }

      if (std::abs(acc_S_0[0]) > imu_params.a_max
          || std::abs(acc_S_0[1]) > imu_params.a_max
          || std::abs(acc_S_0[2]) > imu_params.a_max
          || std::abs(acc_S_1[0]) > imu_params.a_max
          || std::abs(acc_S_1[1]) > imu_params.a_max
          || std::abs(acc_S_1[2]) > imu_params.a_max)
      {
#line 495
        LOG(WARNING) << "acc saturation";   // imu_error.cpp:495
      }
    }

    // actual propagation
    // orientation:
    const Eigen::Vector3d omega_S_true = 0.5 * (omega_S_0 + omega_S_1);
    const Eigen::Vector3d phi = omega_S_true * dt;
    const Sophus::SO3d R_k(Delta_q);                    // 0x18002E0E0
    const Sophus::SO3d dR = Sophus::SO3d::exp(phi, kImuEps1e5);   // 0x180042060
    const Eigen::Quaterniond Delta_q_1 = (R_k * dR).unit_quaternion();
    Eigen::Matrix3d dR_T;
    dR_T = dR.matrix().transpose();                     // binary runs the transpose-aliasing check
    const Eigen::Matrix3d C = R_k.matrix();

    // gravity-compensated accelerations in the frames of sample k / k+1
    Eigen::Vector3d a_0 = acc_S_0;
    Eigen::Vector3d a_1 = acc_S_1;
    const Eigen::Vector3d g_k = C.transpose() * g_S;
    a_0 += g_k;
    a_1 += dR_T * g_k;

    const Eigen::Vector3d d_omega = -(dt * 0.5) * (omega_S_1 - omega_S_0);
    const Eigen::Vector3d d_a = a_1 - a_0;

    const Eigen::Matrix3d Gamma_1 = gamma1(phi, kImuEps1e5);
    const Eigen::Matrix3d Gamma_2 = gamma2(phi, kImuEps1e3);
    const Eigen::Matrix3d Gamma_3 = gamma3(phi, kImuEps2e2);
    const Eigen::Matrix3d DGamma_1 = dGammaTV2(phi, d_omega, kImuEps2e2);
    const Eigen::Matrix3d DGamma_2 = dGammaTV3(phi, d_omega, kImuEps6e2);
    const Eigen::Matrix3d DGamma_3 = dGammaTV4(phi, d_omega, kImuEps1e1);

    const Eigen::Matrix3d M_v = Gamma_1 + 0.5 * DGamma_1;
    const Eigen::Matrix3d M_p = Gamma_2 + 2.0 / 3.0 * DGamma_2;
    const Eigen::Matrix3d M_dv = 0.5 * Gamma_2 + DGamma_2 / 3.0;
    const Eigen::Matrix3d M_dp = 2.0 / 3.0 * Gamma_3 + 0.5 * DGamma_3;

    Eigen::Vector3d dv_k = M_v * a_1 - M_dv * d_a;
    Eigen::Vector3d dp_k = M_p * a_1 - M_dp * d_a;

    acc_doubleintegral += acc_integral * dt + C * (dp_k * (dt * dt * 0.5));
    Eigen::Vector3d acc_integral_1 = acc_integral + C * (dv_k * dt);
    acc_doubleintegral -= (dt * dt * 0.5) * g_S;
    acc_integral_1 -= dt * g_S;

    // memory shift
    Delta_q = Delta_q_1;
    acc_integral = acc_integral_1;
    time = nexttime;

    ++num_propagated;

    if (nexttime == t_end_adjusted)
      break;
  }

  // actual propagation output:
  T_WS = Transformation(
      (q_WS_0 * Delta_q).normalized(),
      r_0 + speed_and_biases.head<3>() * Delta_t + C_WS_0 * acc_doubleintegral
      - 0.5 * Delta_t * Delta_t * g_W);
  speed_and_biases.head<3>() += C_WS_0 * acc_integral - g_W * Delta_t;

  return num_propagated;
}

// 0x180045750
// Called from EvaluateWithMinimalJacobians (0x18003AE40) with g_S0 = C_WS_0^T * (-g_W) where g_W is
// parameters[4]; the caller then does ++redoCounter_.  No lock is taken here (upstream locks).
int ImuError::redoPreintegration(const SpeedAndBias& speed_and_biases,
                                 const Eigen::Vector3d* g_S0) const
{
  // now the propagation
  double time = t0_;

  // sanity check (upstream: assert on back() + "if (!(front >= t1_)) return -1")
  if (imu_measurements_.front().timestamp_ < t1_)
  {
    LOGE("imu_measurements_.front().timestamp_ < t1_\n");
    return -1;  // nothing to do...
  }

  // increments (initialise with identity)
  Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  acc_integral_ = Eigen::Vector3d::Zero();
  acc_doubleintegral_ = Eigen::Vector3d::Zero();
  // NOTE: C_integral_, C_doubleintegral_, cross_, d*_db_g_ are NOT reset here (they are overwritten
  // from J after the loop).

  // the Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  bool has_started = false;
  bool last_iteration = false;
  int n_integrated = 0;

  // Jacobian of the preintegrated state w.r.t. the start state (incl. biases)
  Eigen::Matrix<double, 15, 15> J = Eigen::Matrix<double, 15, 15>::Identity();

  for (size_t i = imu_measurements_.size() - 1; i != 0u; --i)
  {
    Eigen::Vector3d omega_S_0 = imu_measurements_[i].angular_velocity_.cast<double>()
        - speed_and_biases.segment<3>(3);
    Eigen::Vector3d acc_S_0 = imu_measurements_[i].linear_acceleration_.cast<double>()
        - speed_and_biases.segment<3>(6);
    Eigen::Vector3d omega_S_1 = imu_measurements_[i - 1].angular_velocity_.cast<double>()
        - speed_and_biases.segment<3>(3);
    Eigen::Vector3d acc_S_1 = imu_measurements_[i - 1].linear_acceleration_.cast<double>()
        - speed_and_biases.segment<3>(6);
    double nexttime = imu_measurements_[i - 1].timestamp_;

    // time delta
    double dt = nexttime - time;

    if (t1_ < nexttime)
    {
      double interval = nexttime - imu_measurements_[i].timestamp_;
      nexttime = t1_;
      last_iteration = true;
      dt = nexttime - time;
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0)
    {
      continue;
    }

    if (!has_started)
    {
      has_started = true;
      const double r = dt / (nexttime - imu_measurements_[i].timestamp_);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity (Pimax: saturation checks removed here)
    double sigma_g_c = imu_parameters_.sigma_g_c;
    double sigma_a_c = imu_parameters_.sigma_a_c;

    // actual propagation
    // orientation:
    Eigen::Quaterniond dq;   // unused (only its alignment assert survives, 0x180008630)
    const Eigen::Vector3d omega_S_true = 0.5 * (omega_S_0 + omega_S_1);
    const double half_dt2 = dt * dt * 0.5;
    const Eigen::Vector3d phi = omega_S_true * dt;
    const Sophus::SO3d R_k(Delta_q_);
    const Sophus::SO3d dR = Sophus::SO3d::exp(phi, kImuEps1e5);
    const Eigen::Quaterniond Delta_q_1 = (R_k * dR).unit_quaternion();
    const Eigen::Matrix3d dR_T = dR.matrix().transpose();
    const Eigen::Matrix3d C = R_k.matrix();

    Eigen::Vector3d a_0 = acc_S_0;
    Eigen::Vector3d a_1 = acc_S_1;
    Eigen::Vector3d g_k;
    if (g_S0)
    {
      g_k = C.transpose() * (*g_S0);
      a_0 += g_k;
      a_1 += dR_T * g_k;
    }

    const Eigen::Vector3d d_omega = -(dt * 0.5) * (omega_S_1 - omega_S_0);
    const Eigen::Vector3d d_a = a_1 - a_0;

    const Eigen::Matrix3d Gamma_1 = gamma1(phi, kImuEps1e5);
    const Eigen::Matrix3d Gamma_2 = gamma2(phi, kImuEps1e3);
    const Eigen::Matrix3d Gamma_3 = gamma3(phi, kImuEps2e2);
    const Eigen::Matrix3d DGamma_1 = dGammaTV2(phi, d_omega, kImuEps2e2);
    const Eigen::Matrix3d DGamma_2 = dGammaTV3(phi, d_omega, kImuEps6e2);
    const Eigen::Matrix3d DGamma_3 = dGammaTV4(phi, d_omega, kImuEps1e1);

    const Eigen::Matrix3d M_v = Gamma_1 + 0.5 * DGamma_1;
    const Eigen::Matrix3d M_p = Gamma_2 + 2.0 / 3.0 * DGamma_2;
    const Eigen::Matrix3d M_dv = 0.5 * Gamma_2 + DGamma_2 / 3.0;
    const Eigen::Matrix3d M_dp = 2.0 / 3.0 * Gamma_3 + 0.5 * DGamma_3;

    Eigen::Vector3d dv_k = M_v * a_1 - M_dv * d_a;
    Eigen::Vector3d dp_k = M_p * a_1 - M_dp * d_a;

    acc_doubleintegral_ += acc_integral_ * dt + C * (dp_k * half_dt2);
    Eigen::Vector3d acc_integral_1 = acc_integral_ + C * (dv_k * dt);

    // covariance propagation
    Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();

    Eigen::Matrix3d dv_x = Sophus::SO3d::hat(dv_k);
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d dp_x = Sophus::SO3d::hat(dp_k);

    // Jacobians of the local increments w.r.t. the gyro bias
    Eigen::Matrix3d dv_db_g = (dGammaV1(phi, a_1, kImuEps1e3)
                               - dGammaV2(phi, 0.5 * d_a, kImuEps2e2)) * (-dt);
    Eigen::Matrix3d dp_db_g = (dGammaV2(phi, a_1, kImuEps2e2)
                               - dGammaV3(phi, 2.0 / 3.0 * d_a, kImuEps6e2)) * (-dt);

    if (g_S0)
    {
      const Eigen::Matrix3d g_x = Sophus::SO3d::hat(g_k);
      const Eigen::Matrix3d dR_T_minus_I = dR_T - I3;
      const Eigen::Matrix3d A_v = M_v * dR_T - M_dv * dR_T_minus_I;
      const Eigen::Matrix3d A_p = M_p * dR_T - M_dp * dR_T_minus_I;
      dv_x -= A_v * g_x;
      dp_x -= A_p * g_x;
      dv_db_g += (M_v - M_dv) * dR_T * g_x * Gamma_1 * (-dt);
      dp_db_g += (M_p - M_dp) * dR_T * g_x * Gamma_1 * (-dt);
    }

    const Eigen::Matrix3d C_Gamma_1 = C * Gamma_1;
    const Eigen::Matrix3d C_Gamma_2 = C * Gamma_2;

    F_delta.block<3, 3>(3, 3) = dR_T;
    F_delta.block<3, 3>(6, 3) = C * dv_x * (-dt);
    F_delta.block<3, 3>(0, 3) = C * dp_x * (-half_dt2);
    F_delta.block<3, 3>(0, 6) = I3 * dt;
    F_delta.block<3, 3>(3, 9) = gamma1Right(phi, kImuEps1e5) * (-dt);
    F_delta.block<3, 3>(6, 9) = C * dv_db_g * dt;
    F_delta.block<3, 3>(0, 9) = C * dp_db_g * half_dt2;
    F_delta.block<3, 3>(6, 12) = C_Gamma_1 * (-dt);
    F_delta.block<3, 3>(0, 12) = C_Gamma_2 * (-half_dt2);

    // continuous noise -> discrete, entering like the biases
    const Eigen::Matrix3d Q_g = sigma_g_c * sigma_g_c / dt * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Q_a = sigma_a_c * sigma_a_c / dt * Eigen::Matrix3d::Identity();
    // TODO(verify): the acc term is evaluated before F P F^T in the binary (named 9x9 temporary,
    // stored transposed from a row-major product temp), the gyro term inside the final "+=".
    const Eigen::Matrix<double, 9, 9> GQG_a =
        F_delta.block<9, 3>(0, 12) * Q_a * F_delta.block<9, 3>(0, 12).transpose();
    P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    P_delta_.block<9, 9>(0, 0) +=
        F_delta.block<9, 3>(0, 9) * Q_g * F_delta.block<9, 3>(0, 9).transpose() + GQG_a;
    // bias random walks
    P_delta_.block<3, 3>(9, 9).diagonal() +=
        Eigen::Vector3d::Constant(imu_parameters_.sigma_gw_c * imu_parameters_.sigma_gw_c * dt);
    P_delta_.block<3, 3>(12, 12).diagonal() +=
        Eigen::Vector3d::Constant(imu_parameters_.sigma_aw_c * imu_parameters_.sigma_aw_c * dt);

    // J = F_delta * J, exploiting the block structure of F_delta (rows 9..14 of F are identity)
    const Eigen::Matrix<double, 3, 15> J_p = J.block<3, 15>(0, 0);
    const Eigen::Matrix<double, 3, 15> J_q = J.block<3, 15>(3, 0);
    const Eigen::Matrix<double, 3, 15> J_v = J.block<3, 15>(6, 0);
    J.block<3, 15>(0, 0) = J_p + F_delta.block<3, 3>(0, 3) * J_q + dt * J_v
        + F_delta.block<3, 3>(0, 9) * J.block<3, 15>(9, 0)
        + F_delta.block<3, 3>(0, 12) * J.block<3, 15>(12, 0);
    J.block<3, 15>(3, 0) = F_delta.block<3, 3>(3, 3) * J_q
        + F_delta.block<3, 3>(3, 9) * J.block<3, 15>(9, 0);
    J.block<3, 15>(6, 0) = J_v + F_delta.block<3, 3>(6, 3) * J_q
        + F_delta.block<3, 3>(6, 9) * J.block<3, 15>(9, 0)
        + F_delta.block<3, 3>(6, 12) * J.block<3, 15>(12, 0);

    if (g_S0)
    {
      acc_doubleintegral_ -= half_dt2 * (*g_S0);
      acc_integral_1 -= dt * (*g_S0);
    }

    // memory shift
    ++n_integrated;
    time = nexttime;
    Delta_q_ = Delta_q_1;
    acc_integral_ = acc_integral_1;

    if (last_iteration)
      break;
  }

  // export the bias Jacobians in the upstream member names
  const Eigen::Matrix3d C_end = Sophus::SO3d(Delta_q_).matrix();
  dalpha_db_g_ = -C_end * J.block<3, 3>(3, 9);
  dv_db_g_ = J.block<3, 3>(6, 9);
  dp_db_g_ = J.block<3, 3>(0, 9);
  C_integral_ = -J.block<3, 3>(6, 12);
  C_doubleintegral_ = -J.block<3, 3>(0, 12);

  // store the reference (linearisation) point
  speed_and_biases_ref_ = speed_and_biases;

  // get the weighting:
  // enforce symmetric
  P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();

  // calculate inverse (Pimax: LLT solve instead of .inverse())
  information_.setIdentity();
  P_delta_.llt().solveInPlace(information_);
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();

  return n_integrated;
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
