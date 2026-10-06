// pimax_slam.pi.dll -- src/frontend/imu_factor.h  (from draft c08; ctor from c07)
//
// pimax::totem::IMUFactor (VINS-Mono imu_factor.h, modified).
//
// Header-only like upstream: Evaluate (0x1800ECFD0, 17322 bytes) and the scalar deleting
// destructor (0x1800EC000) are COMDATs emitted into frame_processor_base.obj. The ctor
// (0x1800E1950, c07) is called from the VINS-like initialisation 0x180110490 (c09+) as
//   new IMUFactor(pre_integration /*std::shared_ptr<IntegrationBase>*/, <Vector3d>)   // 0x50 bytes
// via Eigen's aligned operator new (0x180011350) -> EIGEN_MAKE_ALIGNED_OPERATOR_NEW.
//
// vtable 0x1803B2DF8: [0] 0x1800EC000 scalar deleting dtor, [1] 0x1800ECFD0 Evaluate.
// Base: ceres::SizedCostFunction<15, 7, 3, 3, 3, 7, 3, 3, 3, 3> (ctor writes that vtable,
// num_residuals 15 at +32, parameter_block_sizes {7,3,3,3,7,3,3,3,3} from 0x1803B6DA0 + 3).
//
// Layout (sizeof 0x50 = 80):
//   +0   vptr
//   +8   std::vector<int32_t> parameter_block_sizes_   (ceres::CostFunction)
//   +32  int num_residuals_                             (ceres::CostFunction)
//   +40  std::shared_ptr<IntegrationBase> pre_integration   (+40 ptr, +48 ctrl; dtor releases +48)
//   +56  Eigen::Vector3d (ctor 3rd arg, copied by value; never read by Evaluate)  TODO(verify) name
//
// Differences to VINS-Mono IMUFactor (SizedCostFunction<15, 7, 9, 7, 9>):
//  * 9 parameter blocks: pose_i(7) Vi(3) Bai(3) Bgi(3) pose_j(7) Vj(3) Baj(3) Bgj(3) G(3) --
//    speed and the two biases are separate blocks and the gravity vector is a parameter
//    (InitGravityLocalParameter, see pose_local_parameterization.h); the jacobian blocks
//    [1..3], [5..7] are the column slices of VINS' 15x9 speed-bias jacobians, [8] is new.
//  * pre_integration is a std::shared_ptr; IntegrationBase::evaluate takes G explicitly.
//  * Evaluate writes the gravity parameter into the global `G` (0x18047ED98, Vector3d) every
//    call (also read by 0x180110490).
//  * Qi.normalized(), Qj.normalized() and corrected_delta_q.normalized() are called and their
//    results discarded (the un-normalized quaternions are used everywhere) -- the
//    normalisation is visible in the binary (Qi inline, Qj / corrected_delta_q via 0x1800426F0)
//    but the results are dead. Preserve.
//  * ROS_WARN -> LOGW (0x18000F6A0) with the same text, no trailing newline.
//  * The jacobian min/max check after jacobian_pose_i is kept (upstream).
#pragma once

#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "common/logger.h"              // LOGW
#include "frontend/integration_base.h"
#include "frontend/utility.h"

namespace pimax {
namespace totem {

// 0x18047ED98 -- global gravity vector (VINS-Mono parameters.cpp `Eigen::Vector3d G`).
// Written by every IMUFactor::Evaluate, read by FrameProcessorBase::initializeImu (0x180110490)
// and passed by pointer to CeresBackendInterface::loadMapFromBundleAdjustment.  Zero-initialised
// .bss (no dynamic initializer found).  Definition: frame_processor_base.cpp (phase B).
// TODO(verify) definition site.
extern Eigen::Vector3d G;

class IMUFactor : public ceres::SizedCostFunction<15, 7, 3, 3, 3, 7, 3, 3, 3, 3>
{
 public:
  IMUFactor() = delete;
  // 0x1800E1950 (c07)
  IMUFactor(std::shared_ptr<IntegrationBase> _pre_integration, const Eigen::Vector3d& _unknown)
      : pre_integration(_pre_integration), unknown_56(_unknown)
  {
  }

  // 0x1800ECFD0
  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
  {
    // Pi: 3 scalar copies. Qi: x,y,z,w loaded from [3..6]; the normalized() result
    // (two dead copies on the stack) is never used.
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Qi.normalized();  // quirk: result discarded (inlined in the binary)

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Vector3d Bgi(parameters[3][0], parameters[3][1], parameters[3][2]);

    Eigen::Vector3d Pj(parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Quaterniond Qj(parameters[4][6], parameters[4][3], parameters[4][4], parameters[4][5]);
    Qj.normalized();  // quirk: result discarded (call 0x1800426F0, result unused)

    Eigen::Vector3d Vj(parameters[5][0], parameters[5][1], parameters[5][2]);
    Eigen::Vector3d Baj(parameters[6][0], parameters[6][1], parameters[6][2]);
    Eigen::Vector3d Bgj(parameters[7][0], parameters[7][1], parameters[7][2]);

    // The local copy is stored to the global G (0x18047ED98/0x18047EDA8) and then the *local*
    // is used below (stack copy passed to evaluate and read by the jacobian code).
    Eigen::Vector3d Gw(parameters[8][0], parameters[8][1], parameters[8][2]);
    G = Gw;  // TODO(verify) exact spelling; behaviour: global gravity overwritten every Evaluate

    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                         Pj, Qj, Vj, Baj, Bgj, Gw);   // 0x180109180

    // LLT ctor from covariance.inverse() (0x1800CB6D0), matrixL().transpose() (0x18004F9A0)
    Eigen::Matrix<double, 15, 15> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
    // GEMV 0x18001F0F0 into a zero-initialised temporary, then copied back
    residual = sqrt_info * residual;

    if (jacobians)
    {
      double sum_dt = pre_integration->sum_dt;
      Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
      Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);

      Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

      Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
      Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

      // maxCoeff 0x1800D90F0 / minCoeff 0x1800D9370 on the 15x15; constants 1e8 (0x1803AF2D8)
      // and -1e8 (0x1803B6D58).
      if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
      {
        LOGW("numerical unstable in preintegration");
      }

      if (jacobians[0])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();

        jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
        jacobian_pose_i.block<3, 3>(O_P, O_R) =
            Utility::skewSymmetric(Qi.inverse() * (0.5 * Gw * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

        Eigen::Quaterniond corrected_delta_q =
            pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
        corrected_delta_q.normalized();  // quirk: result discarded (0x1800426F0)
        // NOTE: the binary computes Qj.inverse() * Qi before Qright(corrected_delta_q) and
        // Qleft(...) after it; pure computations, order has no numerical effect.
        jacobian_pose_i.block<3, 3>(O_R, O_R) =
            -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();

        jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (Gw * sum_dt + Vj - Vi));

        jacobian_pose_i = sqrt_info * jacobian_pose_i;   // lazy product 0x1800CF940 into a temp

        // maxCoeff 0x180117F50 / minCoeff 0x180118300 on the row-major Map
        if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
        {
          LOGW("numerical unstable in preintegration");
        }
      }
      if (jacobians[1])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speed_i(jacobians[1]);
        jacobian_speed_i.setZero();
        jacobian_speed_i.block<3, 3>(O_P, 0) = -Qi.inverse().toRotationMatrix() * sum_dt;
        jacobian_speed_i.block<3, 3>(O_V, 0) = -Qi.inverse().toRotationMatrix();
        jacobian_speed_i = sqrt_info * jacobian_speed_i;   // 0x1800CF830
      }
      if (jacobians[2])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_ba_i(jacobians[2]);
        jacobian_ba_i.setZero();
        jacobian_ba_i.block<3, 3>(O_P, 0) = -dp_dba;
        jacobian_ba_i.block<3, 3>(O_V, 0) = -dv_dba;
        jacobian_ba_i.block<3, 3>(O_BA, 0) = -Eigen::Matrix3d::Identity();
        jacobian_ba_i = sqrt_info * jacobian_ba_i;
      }
      if (jacobians[3])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_bg_i(jacobians[3]);
        jacobian_bg_i.setZero();
        jacobian_bg_i.block<3, 3>(O_P, 0) = -dp_dbg;
        jacobian_bg_i.block<3, 3>(O_R, 0) =
            -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
        jacobian_bg_i.block<3, 3>(O_V, 0) = -dv_dbg;
        jacobian_bg_i.block<3, 3>(O_BG, 0) = -Eigen::Matrix3d::Identity();
        jacobian_bg_i = sqrt_info * jacobian_bg_i;
      }
      if (jacobians[4])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[4]);
        jacobian_pose_j.setZero();

        jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

        Eigen::Quaterniond corrected_delta_q =
            pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
        corrected_delta_q.normalized();  // quirk: result discarded (0x1800426F0)
        jacobian_pose_j.block<3, 3>(O_R, O_R) =
            Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();

        jacobian_pose_j = sqrt_info * jacobian_pose_j;   // 0x1800CF940
      }
      if (jacobians[5])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speed_j(jacobians[5]);
        jacobian_speed_j.setZero();
        jacobian_speed_j.block<3, 3>(O_V, 0) = Qi.inverse().toRotationMatrix();
        jacobian_speed_j = sqrt_info * jacobian_speed_j;
      }
      if (jacobians[6])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_ba_j(jacobians[6]);
        jacobian_ba_j.setZero();
        jacobian_ba_j.block<3, 3>(O_BA, 0) = Eigen::Matrix3d::Identity();
        jacobian_ba_j = sqrt_info * jacobian_ba_j;
      }
      if (jacobians[7])
      {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_bg_j(jacobians[7]);
        jacobian_bg_j.setZero();
        jacobian_bg_j.block<3, 3>(O_BG, 0) = Eigen::Matrix3d::Identity();
        jacobian_bg_j = sqrt_info * jacobian_bg_j;
      }
      if (jacobians[8])
      {
        // d r_p / d G = 0.5 * Ri^T * dt^2,  d r_v / d G = Ri^T * dt
        // binary: ((0.5 * R) * sum_dt) * sum_dt with three separate Constant(3,3,.) operands.
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_g(jacobians[8]);
        jacobian_g.setZero();
        jacobian_g.block<3, 3>(O_P, 0) = 0.5 * Qi.inverse().toRotationMatrix() * sum_dt * sum_dt;
        jacobian_g.block<3, 3>(O_V, 0) = Qi.inverse().toRotationMatrix() * sum_dt;
        jacobian_g = sqrt_info * jacobian_g;
      }
    }

    return true;
  }

  std::shared_ptr<IntegrationBase> pre_integration;   // +40
  Eigen::Vector3d unknown_56;                          // +56  TODO(verify) name/meaning

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace totem
}  // namespace pimax
