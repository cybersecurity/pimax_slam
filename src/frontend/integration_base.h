// pimax_slam.pi.dll -- src/frontend/integration_base.h  (drafts c07 + c08 + c09 + c10 merged)
//
// pimax::totem::IntegrationBase -- IMU pre-integration, VINS-Mono integration_base.h derived,
// header-only: all members are inline COMDATs emitted in frame_processor_base.obj:
//   ctor 0x1800E20B0 (c07), midPointIntegration 0x180104140 / push_back 0x180108AE0 /
//   evaluate 0x180109180 (c09), rightJacobian 0x1800F1400 / skewSymmetric 0x180110290 (c08),
//   propagate 0x18011A810 / repropagate 0x18011AFE0 (c10).
// make_shared<IntegrationBase> allocates 0x2960 -> sizeof 0x2950 = 10576; _Destroy 0x1800F86C0.
//
// Reconciliation:
//   * +0..+23: c07 (ctor) proves a Vector3d (0, 0, 9.80667) = VINS `G`, c09 called the three
//     doubles reserved0_/reserved1_/G_NORM (initializeImu writes 0, 0, gravity_magnitude into
//     them) -- same bytes; the c07 spelling `Eigen::Vector3d G` is used.  Noise parameters are
//     per-instance members ACC_N/ACC_W/GYR_N/GYR_W (+24..+48; VINS globals).
//   * 0x180110290 is named skewSymmetric (c08, owner of its only caller rightJacobian); c09's
//     `hat` is the same function.
//   * step_V is 16-aligned at +5664 (c09; c08's +5656 ignores the alignment).
//   * Quirks: evaluate discards corrected_delta_q.normalized(); push_back rejects dt < 1e-4 with
//     LOGW; repropagate skips buffered dt < 1e-4; initializeImu overwrites the noise members
//     after construction (noise matrix keeps the defaults).
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/logger.h"              // LOGW
#include "common/sophus/so3ex_base.h"   // Sophus::SO3d::exp (0x18010A400)
#include "frontend/utility.h"           // Utility::skewSymmetric (0x180042410), Utility::deltaQ

namespace pimax {
namespace totem {

enum StateOrder
{
  O_P = 0,
  O_R = 3,
  O_V = 6,
  O_BA = 9,
  O_BG = 12
};

class IntegrationBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IntegrationBase() = delete;

  // 0x1800E20B0  (created with std::make_shared<IntegrationBase>(acc_0, gyr_0, ba, bg),
  // _Ref_count_obj2<IntegrationBase>, from 0x180110490)
  // upstream-modified (VINS-Mono): noise parameters and gravity are members with default
  // initialisers instead of globals ACC_N/GYR_N/ACC_W/GYR_W/G; noise is a fixed 18x18.
  IntegrationBase(const Eigen::Vector3d& _acc_0, const Eigen::Vector3d& _gyr_0,
                  const Eigen::Vector3d& _linearized_ba, const Eigen::Vector3d& _linearized_bg)
      : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
        linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
        jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
        sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}
  {
    noise = Eigen::Matrix<double, 18, 18>::Zero();
    noise.block<3, 3>(0, 0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
  }


  //----------------------------------------------------------------------------
  // 0x180108AE0
  // upstream (VINS): push_back. Pimax: rejects dt < 1e-4 with a warning.
  void push_back(double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr)
  {
    if (dt < 0.0001)
    {
      LOGW("imu_initial dt error, dt %f\n", dt);
      return;
    }
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
  }


  // 0x18011AFE0  (callers: 0x180110490 imu init "diff yaw", 0x180158050 "INFO=ImuInitial")
  // VINS repropagate() + Pimax: samples with dt < 1e-4 are skipped.
  void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
  {
      sum_dt = 0.0;
      acc_0 = linearized_acc;
      gyr_0 = linearized_gyr;
      delta_p.setZero();
      delta_q.setIdentity();
      delta_v.setZero();
      linearized_ba = _linearized_ba;
      linearized_bg = _linearized_bg;
      jacobian.setIdentity();
      covariance.setZero();
      for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
      {
          if (dt_buf[i] < 0.0001)     // 0x1803ADDA0; NaN dt is propagated
              continue;
          propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
      }
  }

  //----------------------------------------------------------------------------
  // 0x180104140
  // upstream (VINS midPointIntegration) heavily modified:
  //  * un_gyr = 0.5*((gyr_0 - bg) + (gyr_1 - bg)); rotation increment via Sophus SO3::exp
  //    instead of the first-order quaternion, result normalised;
  //  * accelerations rotated with rotation matrices;
  //  * Jacobian F rebuilt with right Jacobian Jr and exp(-w dt) (simplified mid-point model,
  //    -0.5 factors instead of -0.25, no gyro-bias terms for p and v);
  //  * jacobian updated block-wise (rows 0..8), noise term built block-wise (no V matrix);
  //  * step_jacobian / step_V are not written.
  void midPointIntegration(double _dt,
                           const Eigen::Vector3d& _acc_0, const Eigen::Vector3d& _gyr_0,
                           const Eigen::Vector3d& _acc_1, const Eigen::Vector3d& _gyr_1,
                           const Eigen::Vector3d& delta_p, const Eigen::Quaterniond& delta_q,
                           const Eigen::Vector3d& delta_v,
                           const Eigen::Vector3d& linearized_ba, const Eigen::Vector3d& linearized_bg,
                           Eigen::Vector3d& result_delta_p, Eigen::Quaterniond& result_delta_q,
                           Eigen::Vector3d& result_delta_v,
                           Eigen::Vector3d& result_linearized_ba, Eigen::Vector3d& result_linearized_bg,
                           bool update_jacobian)
  {
    const Eigen::Vector3d w_0 = _gyr_0 - linearized_bg;
    const Eigen::Vector3d w_1 = _gyr_1 - linearized_bg;
    const Eigen::Vector3d un_gyr = 0.5 * (w_0 + w_1);
    const Eigen::Vector3d theta = un_gyr * _dt;
    result_delta_q = (delta_q * Sophus::SO3d::exp(theta).unit_quaternion()).normalized();

    const Eigen::Vector3d a_0 = _acc_0 - linearized_ba;
    const Eigen::Vector3d a_1 = _acc_1 - linearized_ba;
    const Eigen::Matrix3d R_0 = delta_q.toRotationMatrix();
    const Eigen::Matrix3d R_1 = result_delta_q.toRotationMatrix();
    const Eigen::Vector3d un_acc_0 = R_0 * a_0;
    const Eigen::Vector3d un_acc_1 = R_1 * a_1;
    const Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * (_dt * _dt);
    result_delta_v = delta_v + un_acc * _dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian)
    {
      const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
      const Eigen::Matrix3d Jr = rightJacobian(theta, 1e-8);
      const Eigen::Matrix3d R_a_0_x = R_0 * Utility::skewSymmetric(a_0);
      const Eigen::Matrix3d R_sum = R_0 + R_1;

      const Eigen::Matrix3d F_p_theta = -0.5 * R_a_0_x * (_dt * _dt);
      const Eigen::Matrix3d F_p_ba = -0.5 * R_sum * (_dt * _dt);
      const Eigen::Matrix3d F_theta_theta = Sophus::SO3d::exp(-theta).matrix();
      const Eigen::Matrix3d F_theta_bg = -Jr * _dt;
      const Eigen::Matrix3d F_v_theta = -R_a_0_x * _dt;
      const Eigen::Matrix3d F_v_ba = -0.5 * R_sum * _dt;

      Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
      F.block<3, 3>(O_P, O_R) = F_p_theta;
      F.block<3, 3>(O_P, O_V) = I3 * _dt;
      F.block<3, 3>(O_P, O_BA) = F_p_ba;
      F.block<3, 3>(O_R, O_R) = F_theta_theta;
      F.block<3, 3>(O_R, O_BG) = F_theta_bg;
      F.block<3, 3>(O_V, O_R) = F_v_theta;
      F.block<3, 3>(O_V, O_BA) = F_v_ba;

      // jacobian = F * jacobian, exploiting the sparsity of F (rows 9..14 unchanged).
      // Each statement has the form "other + product": Eigen assigns `other` and then
      // accumulates the product without temporary (assignment_from_xpr_op_product).
      const Eigen::Matrix<double, 3, 15> J_p = jacobian.block<3, 15>(O_P, 0);
      const Eigen::Matrix<double, 3, 15> J_theta = jacobian.block<3, 15>(O_R, 0);
      const Eigen::Matrix<double, 3, 15> J_v = jacobian.block<3, 15>(O_V, 0);
      jacobian.block<3, 15>(O_P, 0) = J_p + F_p_theta * J_theta + _dt * J_v
                                    + F_p_ba * jacobian.block<3, 15>(O_BA, 0);
      jacobian.block<3, 15>(O_R, 0) = F_theta_theta * J_theta
                                    + F_theta_bg * jacobian.block<3, 15>(O_BG, 0);
      jacobian.block<3, 15>(O_V, 0) = J_v + F_v_theta * J_theta
                                    + F_v_ba * jacobian.block<3, 15>(O_BA, 0);

      // noise propagation  Q = V * noise * V^T, block-wise
      const Eigen::Matrix3d G_p_acc = 0.5 * R_0 * (_dt * _dt);
      const Eigen::Matrix3d G_theta_gyr = Jr * _dt;
      const Eigen::Matrix3d G_v_acc = R_0 * _dt;
      const Eigen::Matrix3d G_ba = I3 * _dt;
      const Eigen::Matrix3d G_bg = I3 * _dt;
      const Eigen::Matrix3d N_acc = noise.block<3, 3>(0, 0);
      const Eigen::Matrix3d N_gyr = noise.block<3, 3>(3, 3);
      const Eigen::Matrix3d N_ba = noise.block<3, 3>(12, 12);
      const Eigen::Matrix3d N_bg = noise.block<3, 3>(15, 15);

      Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
      Q.block<3, 3>(O_P, O_P) = G_p_acc * N_acc * G_p_acc.transpose();
      Q.block<3, 3>(O_P, O_V) = G_p_acc * N_acc * G_v_acc.transpose();
      Q.block<3, 3>(O_V, O_P) = Q.block<3, 3>(O_P, O_V).transpose();
      Q.block<3, 3>(O_V, O_V) = G_v_acc * N_acc * G_v_acc.transpose();
      Q.block<3, 3>(O_R, O_R) = G_theta_gyr * N_gyr * G_theta_gyr.transpose();
      Q.block<3, 3>(O_BA, O_BA) = G_ba * N_ba * G_ba.transpose();
      Q.block<3, 3>(O_BG, O_BG) = G_bg * N_bg * G_bg.transpose();

      covariance = F * covariance * F.transpose();
      covariance += Q;
    }
  }


  // 0x18011A810  (callers: 0x180108AE0 push_back, 0x18011AFE0 repropagate)
  // VINS-identical.
  void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
  {
      dt = _dt;
      acc_1 = _acc_1;
      gyr_1 = _gyr_1;
      Eigen::Vector3d result_delta_p;
      Eigen::Quaterniond result_delta_q;
      Eigen::Vector3d result_delta_v;
      Eigen::Vector3d result_linearized_ba;
      Eigen::Vector3d result_linearized_bg;

      midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                          linearized_ba, linearized_bg,
                          result_delta_p, result_delta_q, result_delta_v,
                          result_linearized_ba, result_linearized_bg, 1);

      delta_p = result_delta_p;
      delta_q = result_delta_q;
      delta_v = result_delta_v;
      linearized_ba = result_linearized_ba;
      linearized_bg = result_linearized_bg;
      delta_q.normalize();
      sum_dt += dt;
      acc_0 = acc_1;
      gyr_0 = gyr_1;
  }

  //----------------------------------------------------------------------------
  // 0x180109180   (only caller: 0x1800ECFD0, "numerical unstable in preintegration")
  // upstream (VINS evaluate) modified: gravity vector passed as last argument instead of the
  // global G; `corrected_delta_q.normalized();` result is discarded (bug kept: the
  // un-normalised quaternion is used for the rotation residual).
  Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d& Pi, const Eigen::Quaterniond& Qi,
                                        const Eigen::Vector3d& Vi, const Eigen::Vector3d& Bai,
                                        const Eigen::Vector3d& Bgi,
                                        const Eigen::Vector3d& Pj, const Eigen::Quaterniond& Qj,
                                        const Eigen::Vector3d& Vj, const Eigen::Vector3d& Baj,
                                        const Eigen::Vector3d& Bgj, const Eigen::Vector3d& G)
  {
    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;

    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
    corrected_delta_q.normalized();   // sic: result unused
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) =
        Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
  }


  // 0x180110290 -- non-static member (gets `this` in rcx, never uses it). Comma initializer,
  // negations are unary minus (xorpd with the sign mask).
  Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& q) const
  {
    Eigen::Matrix3d ans;
    ans << 0.0, -q(2), q(1),
           q(2), 0.0, -q(0),
           -q(1), q(0), 0.0;
    return ans;
  }

  // 0x1800F1400 -- SO(3) right Jacobian with an explicit small-angle threshold.
  // Only caller: midPointIntegration 0x180104140 with eps = 1e-8 (0.00000001).
  //   theta = |phi| (sqrt of squaredNorm, sqrtpd)
  //   theta <  eps : I - 0.5*K + (1/6)*K^2,  K = hat(phi)   (hat inlined here)
  //   theta >= eps : I - (1-cos t)/t * K + (1 - sin t / t) * K^2,  K = hat(phi.normalized())
  //                  (hat via the out-of-line skewSymmetric 0x180110290)
  // In both branches K*K is evaluated into a temporary first (coefficient loop with the
  // Block<> row-index assert) and only then scaled; in the large branch K*K is computed *before*
  // the sin/cos calls, and sin is called before cos (right-to-left operand evaluation of
  // operator+), hence the explicit K2 temporaries below. 1/6 is the literal
  // 0.16666666666666666 (0x3FC5555555555555, i.e. 1.0 / 6.0).
  Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& phi, double eps) const
  {
    const double theta = phi.norm();
    if (theta < eps)
    {
      Eigen::Matrix3d K = skewSymmetric(phi);
      Eigen::Matrix3d K2 = K * K;
      return Eigen::Matrix3d::Identity() - 0.5 * K + 1.0 / 6.0 * K2;
    }
    Eigen::Vector3d axis = phi.normalized();
    Eigen::Matrix3d K = skewSymmetric(axis);
    Eigen::Matrix3d K2 = K * K;
    return Eigen::Matrix3d::Identity() - (1.0 - std::cos(theta)) / theta * K +
           (1.0 - std::sin(theta) / theta) * K2;
  }

  // ---- data ------------------------------------------------------------------------------
  Eigen::Vector3d G{0.0, 0.0, 9.80667};   // +0   (0x40239D03D9A95422)  TODO(verify) name
  double ACC_N = 0.04;                    // +24  (0x3FA47AE147AE147B)  noise.block(0,0)/(6,6)
  double ACC_W = 0.004;                   // +32  (0x3F70624DD2F1A9FC)  noise.block(12,12)
  double GYR_N = 0.008;                   // +40  (0x3F80624DD2F1A9FC)  noise.block(3,3)/(9,9)
  double GYR_W = 0.0008;                  // +48  (0x3F4A36E2EB1C432D)  noise.block(15,15)

  double dt;                                   // +56
  Eigen::Vector3d acc_0, gyr_0;                // +64, +88
  Eigen::Vector3d acc_1, gyr_1;                // +112, +136

  const Eigen::Vector3d linearized_acc, linearized_gyr;  // +160, +184
  Eigen::Vector3d linearized_ba, linearized_bg;          // +208, +232

  Eigen::Matrix<double, 15, 15> jacobian, covariance;    // +256, +2056
  Eigen::Matrix<double, 15, 15> step_jacobian;           // +3856
  Eigen::Matrix<double, 15, 18> step_V;                  // +5664 (16-aligned)
  Eigen::Matrix<double, 18, 18> noise;                   // +7824

  double sum_dt;                 // +10416
  Eigen::Vector3d delta_p;       // +10424
  Eigen::Quaterniond delta_q;    // +10448
  Eigen::Vector3d delta_v;       // +10480

  std::vector<double> dt_buf;              // +10504
  std::vector<Eigen::Vector3d> acc_buf;    // +10528
  std::vector<Eigen::Vector3d> gyr_buf;    // +10552

  static void layout_check();
};

inline void IntegrationBase::layout_check()
{
  static_assert(sizeof(IntegrationBase) == 10576, "sizeof(IntegrationBase)");
  static_assert(offsetof(IntegrationBase, ACC_N) == 24, "");
  static_assert(offsetof(IntegrationBase, GYR_W) == 48, "");
  static_assert(offsetof(IntegrationBase, dt) == 56, "");
  static_assert(offsetof(IntegrationBase, acc_0) == 64, "");
  static_assert(offsetof(IntegrationBase, gyr_1) == 136, "");
  static_assert(offsetof(IntegrationBase, linearized_acc) == 160, "");
  static_assert(offsetof(IntegrationBase, linearized_bg) == 232, "");
  static_assert(offsetof(IntegrationBase, jacobian) == 256, "");
  static_assert(offsetof(IntegrationBase, covariance) == 2056, "");
  static_assert(offsetof(IntegrationBase, step_jacobian) == 3856, "");
  static_assert(offsetof(IntegrationBase, step_V) == 5664, "");
  static_assert(offsetof(IntegrationBase, noise) == 7824, "");
  static_assert(offsetof(IntegrationBase, sum_dt) == 10416, "");
  static_assert(offsetof(IntegrationBase, delta_p) == 10424, "");
  static_assert(offsetof(IntegrationBase, delta_q) == 10448, "");
  static_assert(offsetof(IntegrationBase, delta_v) == 10480, "");
  static_assert(offsetof(IntegrationBase, dt_buf) == 10504, "");
  static_assert(offsetof(IntegrationBase, acc_buf) == 10528, "");
  static_assert(offsetof(IntegrationBase, gyr_buf) == 10552, "");
}

}  // namespace totem
}  // namespace pimax
