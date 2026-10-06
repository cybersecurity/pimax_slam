// pimax_slam.pi.dll -- src/frontend/visual_imu_alignment.cpp  (draft c13, phase B3)
//
// VINS-Mono initial_alignment.cpp, Pimax fork, as members of ImuInitializer.  Object TU37
// (file/class name ours, TODO(verify): the object sits between frontend/stereo_triangulation.obj
// and interface/ceres_backend_factory.obj), code 0x18014DB90..0x180158F30 incl. the Eigen
// LDLT/GEMM/GEMV instantiations and Utility::ypr2R<Vector3d> 0x18014DB90 / Utility::g2R
// 0x1801572E0 (header-inline, frontend/utility.h).  No glog lines (Pimax logger only).
//
//   0x180157240  ImuInitializer::VisualIMUAlignment          modified (ba arg, extra world-state step)
//   0x180158050  ImuInitializer::solveGyroscopeBias          modified (log-map residual, ba, all frames)
//   0x18014DFE0  ImuInitializer::LinearAlignment             modified (see below)
//   0x1801518F0  ImuInitializer::RefineGravity               modified (up to 10 iterations, early exit)
//   0x1801569A0  ImuInitializer::TangentBasis                identical
//   0x180156C10  ImuInitializer::computeStatesInGravityFrame new ("INFO=ImuInitialGw0")
//   0x180158C90  ImuInitializer::computeStd                  new
//
// Literals (tools/ida_dump.py rd): 0.01 @0x1803B79F0 (all "/ 100.0" of VINS became "* 0.01" -- the
// Constant() nodes hold 0.01, so this is in the source, not a /fp:fast artefact), 1000.0
// (0x408F400000000000), 0.5 (Constant(3,3,0.5) in RefineGravity), 0.001 @0x1803ADDA8, 0.1 @0x1803B2C00,
// 0.4 @0x1803B6CD8, 0.75 @0x1803B79B8, 1.25 @0x1803B79C0, 2.0 @0x1803B6DC0.
#include "frontend/visual_imu_alignment.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>

#include <Eigen/Cholesky>

#include "common/logger.h"
#include "common/transformation.h"
#include "frontend/utility.h"

namespace pimax {
namespace totem {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// 0x180158050
// Pimax changes vs VINS-Mono:
//   * the rotation residual is the full quaternion log (2*atan2 form) instead of 2*vec()
//   * a dead `q_ij.normalized()` temporary (stored, never read)
//   * log line instead of ROS_WARN_STREAM; Bgs updated for all_image_frame.size() entries
//     (upstream: WINDOW_SIZE+1)
//   * repropagate with the given accelerometer bias instead of Vector3d::Zero()
void ImuInitializer::solveGyroscopeBias(std::map<double, ImageFrame>& all_image_frame,
                                        GyroBiasVector& Bgs, Vector3d ba)
{
  Matrix3d A;
  Vector3d b;
  Vector3d delta_bg;
  A.setZero();
  b.setZero();
  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
  {
    frame_j = next(frame_i);
    Matrix3d tmp_A;
    Vector3d tmp_b;
    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);   // 0x18014B200
    Eigen::Quaterniond q_ij_n = q_ij.normalized();   // computed and stored but never used
    (void)q_ij_n;
    tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
    Eigen::Quaterniond q_err = frame_j->second.pre_integration->delta_q.inverse() * q_ij;
    const double theta = 2.0 * atan2(q_err.vec().norm(), q_err.w());
    if (theta > 0.001)   // comisd/jbe: NaN takes the small-angle branch
      tmp_b = theta / sin(theta * 0.5) * q_err.vec();
    else
      tmp_b = 2.0 * q_err.vec();
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;
  }
  delta_bg = A.ldlt().solve(b);
  LOGI("INFO=ImuInitial||frame_num=%d||gyroscope_bias=%f,%f,%f\n", all_image_frame.size(),
       delta_bg(0), delta_bg(1), delta_bg(2));

  for (size_t i = 0; i < all_image_frame.size(); i++)
    Bgs[i] += delta_bg;

  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
  {
    frame_j = next(frame_i);
    frame_j->second.pre_integration->repropagate(ba, Bgs[0]);
  }
}

// 0x1801569A0 -- upstream-identical (`this` unused)
MatrixXd ImuInitializer::TangentBasis(Vector3d& g0)
{
  Vector3d b, c;
  Vector3d a = g0.normalized();
  Vector3d tmp(0, 0, 1);
  if (a == tmp)
    tmp << 1, 0, 0;
  b = (tmp - a * (a.transpose() * tmp)).normalized();
  c = a.cross(b);
  MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}

// 0x1801518F0
// Pimax changes vs VINS-Mono:
//   * no `* Matrix3d::Identity()` factors; "/ 2" -> "* 0.5"; "/ 100.0" -> "* 0.01"
//   * tmp_b(0..2) is evaluated as (delta_p - TIC) + R_i^T R_j TIC - R_i^T dt^2/2 g0
//   * no cov_inv; tmp_A.transpose() is materialised once and used for r_A and r_b
//   * up to 10 iterations (upstream 4) with an early exit when |dg| < 0.001
//   * as upstream, A and b are NOT reset between iterations (they keep accumulating and are
//     multiplied by 1000 every iteration) -- quirk preserved
void ImuInitializer::RefineGravity(std::map<double, ImageFrame>& all_image_frame, Vector3d& g,
                                   VectorXd& x)
{
  Vector3d g0 = g.normalized() * G.norm();
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 2 + 1;

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
  for (int k = 0; k < 10; k++)
  {
    MatrixXd lxly(3, 2);
    lxly = TangentBasis(g0);
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
      frame_j = next(frame_i);

      MatrixXd tmp_A(6, 9);
      tmp_A.setZero();
      VectorXd tmp_b(6);
      tmp_b.setZero();

      double dt = frame_j->second.pre_integration->sum_dt;

      tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt * 0.5 * lxly;
      tmp_A.block<3, 1>(0, 8) =
          frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) * 0.01;
      tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - TIC[0] +
                                frame_i->second.R.transpose() * frame_j->second.R * TIC[0] -
                                frame_i->second.R.transpose() * dt * dt * 0.5 * g0;

      tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
      tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
      tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * lxly;
      tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v -
                                frame_i->second.R.transpose() * dt * g0;

      MatrixXd tmp_A_t = tmp_A.transpose();   // TODO(verify) name
      MatrixXd r_A = tmp_A_t * tmp_A;
      VectorXd r_b = tmp_A_t * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
      b.tail<3>() += r_b.tail<3>();

      A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    VectorXd dg = x.segment<2>(n_state - 3);
    g0 = (g0 + lxly * dg).normalized() * G.norm();
    if (dg.norm() < 0.001)
      break;
  }
  g = g0;
}

// 0x18014DFE0
// Pimax changes vs VINS-Mono:
//   * no `* Matrix3d::Identity()` factors, "/ 2" -> "* 0.5", "/ 100.0" -> "* 0.01"
//   * tmp_b(0..2) = (delta_p - TIC[0]) + R_i^T R_j TIC[0]   (operand order changed)
//   * no cov_inv
//   * the camera positions frame_j.T are collected per axis; if all three std-devs are < 1 mm the
//     sequence is "static" and only |g| is checked (tolerance 0.1), otherwise |g| (0.4) and the
//     scale (0.75 .. 1.25) are checked (upstream: |g| 1.0 and s >= 0)
//   * after RefineGravity the `s < 0` rejection of upstream is gone; two INFO log lines
bool ImuInitializer::LinearAlignment(std::map<double, ImageFrame>& all_image_frame, Vector3d& g,
                                     VectorXd& x)
{
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 3 + 1;

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();

  std::vector<double> pos_x;   // TODO(verify) names
  std::vector<double> pos_y;
  std::vector<double> pos_z;
  pos_x.reserve(all_frame_count);
  pos_y.reserve(all_frame_count);
  pos_z.reserve(all_frame_count);

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
  int i = 0;
  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
  {
    frame_j = next(frame_i);

    MatrixXd tmp_A(6, 10);
    tmp_A.setZero();
    VectorXd tmp_b(6);
    tmp_b.setZero();

    double dt = frame_j->second.pre_integration->sum_dt;

    tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt * 0.5;
    tmp_A.block<3, 1>(0, 9) =
        frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) * 0.01;
    tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - TIC[0] +
                              frame_i->second.R.transpose() * frame_j->second.R * TIC[0];
    tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
    tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
    tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt;
    tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;

    MatrixXd r_A = tmp_A.transpose() * tmp_A;
    VectorXd r_b = tmp_A.transpose() * tmp_b;

    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
    b.tail<4>() += r_b.tail<4>();

    A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
    A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();

    pos_x.push_back(frame_j->second.T(0));
    pos_y.push_back(frame_j->second.T(1));
    pos_z.push_back(frame_j->second.T(2));
  }
  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);
  double s = x(n_state - 1) * 0.01;
  g = x.segment<3>(n_state - 4);
  Vector3d g_imu = RIC[0] * g;
  LOGI("INFO=ImuInitial||estimated_scale=%f||g_norm=%f||g_cam0=%f,%f,%f||g_imu=%f,%f,%f\n", s,
       g.norm(), g(0), g(1), g(2), g_imu(0), g_imu(1), g_imu(2));

  const double std_x = computeStd(pos_x);
  const double std_y = computeStd(pos_y);
  const double std_z = computeStd(pos_z);
  const bool is_static = std_x < 0.001 && std_y < 0.001 && std_z < 0.001;
  const double g_err = fabs(g.norm() - G.norm());
  // Reject forms on purpose: NaN g_err / s pass (comisd + ja in the binary).
  if (is_static)
  {
    if (g_err > 0.1)
      return false;
  }
  else if (g_err > 0.4 || s < 0.75 || s > 1.25)
  {
    return false;
  }

  RefineGravity(all_image_frame, g, x);
  s = (x.tail<1>())(0) * 0.01;
  (x.tail<1>())(0) = s;
  g_imu = RIC[0] * g;
  LOGI("INFO=ImuInitialRefine||estimated_scale=%f||g_norm=%f||g_cam0=%f,%f,%f||g_imu=%f,%f,%f\n", s,
       g.norm(), g(0), g(1), g(2), g_imu(0), g_imu(1), g_imu(2));
  return true;
}

// 0x180156C10 -- Pimax-new. Rotates the c0-frame solution into a gravity-aligned world frame w0
// (VINS-Mono does the equivalent in Estimator::visualInitialAlign) and writes the per-frame world
// pose, velocity and gyro bias into the ImageFrames.
void ImuInitializer::computeStatesInGravityFrame(std::map<double, ImageFrame>& all_image_frame,
                                                 GyroBiasVector& Bgs, Vector3d& g,
                                                 VectorXd& x)
{
  Vector3d g_imu = RIC[0] * g;
  Matrix3d R0 = Utility::g2R(g_imu);
  Vector3d g_w0 = R0 * g_imu;
  LOGI("INFO=ImuInitialGw0||g_norm=%f||g_w0=%f,%f,%f\n", g_w0.norm(), g_w0(0), g_w0(1), g_w0(2));
  // Quaterniond(Matrix3d) 0x180020AF0, then the CHECKing kindr RotationQuaternion ctor 0x1800089C0
  // (see integration_A1.md: Quaternion is Eigen, the kindr ctor is spelled explicitly).
  T_w0_ = Transformation(kindr::minimal::RotationQuaternionTemplate<double>(Eigen::Quaterniond(R0)),
                         Vector3d::Zero());

  int kv = -1;
  std::map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
  {
    kv++;
    frame_i->second.T_w_body = T_w0_ * frame_i->second.T_c0_body;
    frame_i->second.V_w = T_w0_ * (RIC[0] * frame_i->second.R * x.segment<3>(kv * 3));
    frame_i->second.Bg = Bgs[kv];
  }
}

// 0x180157240
bool ImuInitializer::VisualIMUAlignment(std::map<double, ImageFrame>& all_image_frame,
                                        GyroBiasVector& Bgs, Vector3d& g, VectorXd& x,
                                        Vector3d ba)
{
  solveGyroscopeBias(all_image_frame, Bgs, ba);
  if (LinearAlignment(all_image_frame, g, x))
  {
    computeStatesInGravityFrame(all_image_frame, Bgs, g, x);
    return true;
  }
  else
    return false;
}

// 0x180158C90 -- Pimax-new (`this` unused)
double ImuInitializer::computeStd(const std::vector<double>& values)
{
  double sum = std::accumulate(values.begin(), values.end(), 0.0);
  double mean = sum / values.size();
  std::vector<double> diff(values.size());
  std::transform(values.begin(), values.end(), diff.begin(),
                 [mean](double v) { return v - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / values.size());
  return stdev;
}

}  // namespace totem
}  // namespace pimax
