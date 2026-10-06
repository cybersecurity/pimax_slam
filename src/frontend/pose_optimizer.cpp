// pimax_slam.pi.dll -- src/frontend/pose_optimizer.cpp  (drafts c11 + c12 merged, phase B3)
//
// pimax::totem::PoseOptimizer -- fork of svo/src/pose_optimizer.cpp.
// Original file: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\frontend\pose_optimizer.cpp
// Object TU34, code 0x18012EFE0..0x18013FA30.  The functions of the object are laid out in
// decorated-name order (ctor, applyPrior, pose_optimizer_utils::calculate*, evaluateErrorImpl,
// getDefaultSolverOptions, Frame::jacobian_*_imu COMDATs, MiniLeastSquaresSolver<6,...>::optimize*,
// removeOutliers, run, setRotationPrior, update), so the source order below is free; the only
// glog line of this file (VLOG(5) in run, line 62) is forced with #line.
// The vikit solver template bodies (GN 0x18013C500, LM 0x18013CD60, with their own glog lines)
// come from third_party/vikit/.../implementation/mini_least_squares_solver.hpp (A1).
//
// Global Pimax changes:
//  * landmark positions, bearing vectors, keypoints and gradients are float; every residual helper
//    converts them to double with .cast<double>() before the (double) math;
//  * evaluateErrorImpl only uses features with track_id_vec_(i) > -1 and a landmark (seed branch
//    removed) and drops CHECK_GE(unwhitened_error, 0.0);
//  * calculateEdgeletResidualBearingVectorDiff: f_est normalised in float, new guard for
//    |px_diff| <= 1e-8;
//  * getDefaultSolverOptions: max_iter 10 -> 6;
//  * run(): extra remove_outliers flag, CHECKs -> LOGE; removeOutliers: CHECK_NOTNULL -> LOGE,
//    thresholds non-static, track-id based landmark test, keyframes drop the observation.
#include "frontend/pose_optimizer.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <glog/logging.h>
#include <vikit/math_utils.h>

#include "common/camera.h"
#include "common/frame.h"
#include "common/logger.h"
#include "common/point.h"

namespace pimax {
namespace totem {

namespace {

// vk::project2 for any 3-vector expression (also float). The binary reads f.head<2>() through a
// 2x1 Map and divides by f(2) in the vector's own scalar type.
// TODO(verify): Pimax vikit math_utils spelling.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 2, 1> project2(const Eigen::MatrixBase<Derived>& v) {
  return v.template head<2>() / v(2);
}

}  // namespace

// 0x180132D90
// upstream-identical (base copies the 56-byte options, then the member initialisers listed in the
// header; ofstream ctor 0x180097B60).
PoseOptimizer::PoseOptimizer(SolverOptions solver_options)
    : vk::solver::MiniLeastSquaresSolver<6, Transformation, PoseOptimizer>(solver_options) {}

// 0x18013B2B0
// upstream-modified: max_iter 10 -> 6. (mu_init keeps the upstream float literal 0.01f.)
PoseOptimizer::SolverOptions PoseOptimizer::getDefaultSolverOptions() {
  SolverOptions options;
  options.strategy = vk::solver::Strategy::GaussNewton;
  options.max_iter = 6;
  options.eps = 0.000001;
  return options;
}

// 0x1801331C0 (virtual, vtbl[1]; the vcall thunk 0x1801331A0 is used by the base-class
// "&MiniLeastSquaresSolver::applyPrior != &Implementation::applyPrior" test)
// upstream-identical.
void PoseOptimizer::applyPrior(const State& T_cur_from_world) {
  if (iter_ == 0) {
    I_prior_ = Matrix6d::Zero();
    I_prior_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

    double H_max_diag = 0;
    // double tau = 1e-4;
    for (size_t j = 3; j < 6; ++j)
      H_max_diag = std::max(H_max_diag, std::fabs(H_(j, j)));
    I_prior_ *= H_max_diag * prior_lambda_;
    if (solver_options_.verbose) {
      std::cout << "applying rotation prior, I = " << H_max_diag * prior_lambda_ << std::endl;
    }
  }

  H_.noalias() += I_prior_;
  g_.noalias() -= I_prior_ * Transformation::log(T_cur_from_world * prior_.inverse());   // log: 0x18013C150
}

// Upstream inline wrapper (no out-of-line copy; inlined into optimizeGaussNewton /
// optimizeLevenbergMarquardt).
double PoseOptimizer::evaluateError(const Transformation& T_imu_world, HessianMatrix* H,
                                    GradientVector* g)
{
  return evaluateErrorImpl(T_imu_world, H, g, nullptr);
}

// 0x18013A840
// upstream-modified: see file header. Feature loop body:
//   track_id_vec_(i) > -1  &&  landmark_vec_[i] != nullptr  -> residual; everything else skipped.
double PoseOptimizer::evaluateErrorImpl(const Transformation& T_imu_world, HessianMatrix* H,
                                        GradientVector* g, std::vector<float>* unwhitened_errors) {
  double chi2_error_sum = 0.0;

  // compute the weights on the first iteration
  if (unwhitened_errors)
    unwhitened_errors->reserve(frame_bundle_->numFeatures());

  for (const FramePtr& frame : frame_bundle_->frames_) {
    const Transformation T_cam_imu = frame->T_cam_imu();

    // compute residual and update normal equation
    for (size_t i = 0; i < frame->num_features_; ++i) {
      if (frame->track_id_vec_(i) > -1) {
        const PointPtr& point = frame->landmark_vec_[i];
        if (point) {
          const Position xyz_world = point->pos_;

          const int scale = (1 << frame->level_vec_(i));
          double unwhitened_error, chi2_error;
          double measurement_sigma = measurement_sigma_ * scale;
          if (isEdgelet(frame->type_vec_[i])) {
            // Edgelets should have less weight than corners.
            constexpr double kEdgeletSigmaExtraFactor = 2.0;
            measurement_sigma *= kEdgeletSigmaExtraFactor;
            if (err_type_ == ErrorType::kUnitPlane)
              pose_optimizer_utils::calculateEdgeletResidualUnitPlane(
                  frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i), T_imu_world, T_cam_imu,
                  measurement_sigma, robust_weight_, &unwhitened_error, &chi2_error, H, g);
            else if (err_type_ == ErrorType::kImagePlane)
              pose_optimizer_utils::calculateEdgeletResidualImagePlane(
                  frame->px_vec_.col(i), xyz_world, frame->grad_vec_.col(i), T_imu_world, T_cam_imu,
                  *frame->cam(), measurement_sigma, robust_weight_, &unwhitened_error, &chi2_error,
                  H, g);
            else if (err_type_ == ErrorType::kBearingVectorDiff)
              pose_optimizer_utils::calculateEdgeletResidualBearingVectorDiff(
                  frame->px_vec_.col(i), frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                  T_imu_world, T_cam_imu, *frame->cam(), measurement_sigma, robust_weight_,
                  &unwhitened_error, &chi2_error, H, g);
          } else {
            if (err_type_ == ErrorType::kUnitPlane)
              pose_optimizer_utils::calculateFeatureResidualUnitPlane(
                  frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu, measurement_sigma,
                  robust_weight_, &unwhitened_error, &chi2_error, H, g);
            else if (err_type_ == ErrorType::kImagePlane) {
              pose_optimizer_utils::calculateFeatureResidualImagePlane(
                  frame->px_vec_.col(i), xyz_world, T_imu_world, T_cam_imu, *frame->cam(),
                  measurement_sigma, robust_weight_, &unwhitened_error, &chi2_error, H, g);
            } else if (err_type_ == ErrorType::kBearingVectorDiff) {
              pose_optimizer_utils::calculateFeatureResidualBearingVectorDiff(
                  frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu, measurement_sigma,
                  robust_weight_, &unwhitened_error, &chi2_error, H, g);
            }
          }
          if (unwhitened_errors) {
            // emplace_back(double) (vector<float>::_Emplace_reallocate<double&> 0x18012F6E0)
            unwhitened_errors->emplace_back(unwhitened_error / scale);
          }
          chi2_error_sum += chi2_error;
          ++n_meas_;
        }
      }
    }  // for each feature
  }    // for each frame

  return chi2_error_sum;
}

namespace pose_optimizer_utils {

// 0x180139AE0
// upstream-modified (float inputs).
void calculateFeatureResidualUnitPlane(const Eigen::Ref<const BearingVector>& f,
                                       const Position& xyz_in_world,
                                       const Transformation& T_imu_world,
                                       const Transformation& T_cam_imu, double measurement_sigma,
                                       const PoseOptimizer::RobustWeightFunction& robust_weight,
                                       double* unwhitened_error, double* chi2_error,
                                       PoseOptimizer::HessianMatrix* H,
                                       PoseOptimizer::GradientVector* g) {
  const Eigen::Vector3d xyz_in_imu(T_imu_world * xyz_in_world.cast<double>());
  const Eigen::Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

  // Prediction error. project2(f) is evaluated in float.
  Eigen::Vector2d e = project2(f).cast<double>() - project2(xyz_in_cam);
  if (unwhitened_error)
    *unwhitened_error = e.norm();

  // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
  double R = 1.0 / measurement_sigma;
  e *= R;

  // M-estimator weighting
  double weight = robust_weight.weight(e.norm());

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  *chi2_error = 0.5 * e.squaredNorm() * weight;

  if (H && g) {
    // compute jacobian
    PoseOptimizer::Matrix26d J_proj;
    Frame::jacobian_xyz2uv_imu(T_cam_imu, xyz_in_imu, J_proj);
    J_proj *= R;
    H->noalias() += J_proj.transpose() * J_proj * weight;
    g->noalias() -= J_proj.transpose() * e * weight;
  }
}

// 0x180138430
// upstream-modified (float inputs).
void calculateFeatureResidualImagePlane(const Eigen::Ref<const Keypoint>& px,
                                        const Position& xyz_in_world,
                                        const Transformation& T_imu_world,
                                        const Transformation& T_cam_imu, const Camera& cam,
                                        double measurement_sigma,
                                        const PoseOptimizer::RobustWeightFunction& robust_weight,
                                        double* unwhitened_error, double* chi2_error,
                                        PoseOptimizer::HessianMatrix* H,
                                        PoseOptimizer::GradientVector* g) {
  const Eigen::Vector3d xyz_in_imu(T_imu_world * xyz_in_world.cast<double>());
  const Eigen::Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

  // Prediction error.
  Eigen::Matrix<double, 2, 3> J_cam;
  Eigen::Vector2d px_est;
  cam.project3(xyz_in_cam, &px_est, &J_cam);
  Eigen::Vector2d e = px.cast<double>() - px_est;
  if (unwhitened_error)
    *unwhitened_error = e.norm();

  // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
  double R = 1.0 / measurement_sigma;
  e *= R;

  // M-estimator weighting
  double weight = robust_weight.weight(e.norm());

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  *chi2_error = 0.5 * e.squaredNorm() * weight;

  if (H && g) {
    // compute jacobian
    PoseOptimizer::Matrix26d J_proj;
    Frame::jacobian_xyz2img_imu(T_cam_imu, xyz_in_imu, J_cam, J_proj);
    J_proj = (-1.0) * J_proj;
    J_proj *= R;
    H->noalias() += J_proj.transpose() * J_proj * weight;
    g->noalias() -= J_proj.transpose() * e * weight;
  }
}

// 0x180136EA0
// upstream-modified (float inputs).
void calculateFeatureResidualBearingVectorDiff(
    const Eigen::Ref<const BearingVector>& f, const Position& xyz_in_world,
    const Transformation& T_imu_world, const Transformation& T_cam_imu, double measurement_sigma,
    const PoseOptimizer::RobustWeightFunction& robust_weight, double* unwhitened_error,
    double* chi2_error, PoseOptimizer::HessianMatrix* H, PoseOptimizer::GradientVector* g) {
  const Eigen::Vector3d xyz_in_imu(T_imu_world * xyz_in_world.cast<double>());
  const Eigen::Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

  // Prediction error (normalisation in double here, unlike the edgelet variant).
  Eigen::Vector3d e = f.cast<double>() - xyz_in_cam.normalized();
  if (unwhitened_error)
    *unwhitened_error = e.norm();

  // Whitened error.
  double R = 1.0 / measurement_sigma;
  e *= R;

  // M-estimator.
  double weight = robust_weight.weight(e.norm());

  // Chi2 error (likelihood).
  *chi2_error = 0.5 * e.squaredNorm() * weight;

  if (H && g) {
    // compute jacobian
    PoseOptimizer::Matrix36d J_bearing;
    Frame::jacobian_xyz2f_imu(T_cam_imu, xyz_in_imu, J_bearing);
    PoseOptimizer::Matrix36d J_proj = (-1.0) * J_bearing;
    J_proj *= R;
    H->noalias() += J_proj.transpose() * J_proj * weight;
    g->noalias() -= J_proj.transpose() * e * weight;
  }
}

// 0x1801364D0
// upstream-modified (float inputs).
void calculateEdgeletResidualUnitPlane(const Eigen::Ref<const BearingVector>& f,
                                       const Position& xyz_in_world,
                                       const Eigen::Ref<const GradientVector>& grad,
                                       const Transformation& T_imu_world,
                                       const Transformation& T_cam_imu, double measurement_sigma,
                                       const PoseOptimizer::RobustWeightFunction& robust_weight,
                                       double* unwhitened_error, double* chi2_error,
                                       PoseOptimizer::HessianMatrix* H,
                                       PoseOptimizer::GradientVector* g) {
  const Eigen::Vector3d xyz_in_imu(T_imu_world * xyz_in_world.cast<double>());
  const Eigen::Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

  // Compute error.
  double e = grad.cast<double>().dot(project2(f).cast<double>() - project2(xyz_in_cam));
  if (unwhitened_error)
    *unwhitened_error = std::abs(e);

  // Whiten error.
  double R = 1.0 / measurement_sigma;
  e *= R;

  // Robustification.
  double weight = robust_weight.weight(e);

  // Chi2 error (likelihood).
  *chi2_error = 0.5 * e * e * weight;

  if (H && g) {
    // Compute Jacobian.
    PoseOptimizer::Matrix26d J_proj;
    Frame::jacobian_xyz2uv_imu(T_cam_imu, xyz_in_imu, J_proj);
    PoseOptimizer::Vector6d J = grad.cast<double>().transpose() * J_proj;
    J *= R;
    H->noalias() += J * J.transpose() * weight;
    g->noalias() -= J * e * weight;
  }
}

// 0x180135AF0
// upstream-modified (float inputs).
void calculateEdgeletResidualImagePlane(const Eigen::Ref<const Keypoint>& px,
                                        const Position& xyz_in_world,
                                        const Eigen::Ref<const GradientVector>& grad,
                                        const Transformation& T_imu_world,
                                        const Transformation& T_cam_imu, const Camera& cam,
                                        double measurement_sigma,
                                        const PoseOptimizer::RobustWeightFunction& robust_weight,
                                        double* unwhitened_error, double* chi2_error,
                                        PoseOptimizer::HessianMatrix* H,
                                        PoseOptimizer::GradientVector* g) {
  const Eigen::Vector3d xyz_in_imu(T_imu_world * xyz_in_world.cast<double>());
  const Eigen::Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

  // Prediction error.
  Eigen::Matrix<double, 2, 3> J_cam;
  Eigen::Vector2d px_est;
  cam.project3(xyz_in_cam, &px_est, &J_cam);
  double e = grad.cast<double>().dot(px.cast<double>() - px_est);
  if (unwhitened_error)
    *unwhitened_error = std::abs(e);

  // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
  double R = 1.0 / measurement_sigma;
  e *= R;

  // M-estimator weighting.
  double weight = robust_weight.weight(e);

  // Chi2 error, i.e. log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  *chi2_error = 0.5 * e * e * weight;

  if (H && g) {
    // Compute Jacobian.
    PoseOptimizer::Matrix26d J_proj;
    Frame::jacobian_xyz2img_imu(T_cam_imu, xyz_in_imu, J_cam, J_proj);
    PoseOptimizer::Vector6d J = grad.cast<double>().transpose() * (-1.0) * J_proj;
    J *= R;
    H->noalias() += J * J.transpose() * weight;
    g->noalias() -= J * e * weight;
  }
}

// 0x1801338F0
// upstream-modified: float inputs; f_est = xyz_in_cam.cast<float>().normalized() (float
// normalisation, then promoted again); scale_ratio guarded against |px_diff| <= 1e-8
// (0x1803B6C58): falls back to 1 / (|xyz_in_cam| + 1e-8).
// NOTE: current implementation basically scales the residual on
//       the image plane to the unit sphere.
void calculateEdgeletResidualBearingVectorDiff(
    const Eigen::Ref<const Keypoint>& px, const Eigen::Ref<const BearingVector>& f,
    const Position& xyz_in_world, const Eigen::Ref<const GradientVector>& grad,
    const Transformation& T_imu_world, const Transformation& T_cam_imu, const Camera& cam,
    double measurement_sigma, const PoseOptimizer::RobustWeightFunction& robust_weight,
    double* unwhitened_error, double* chi2_error, PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g) {
  const Eigen::Vector3d xyz_in_imu(T_imu_world * xyz_in_world.cast<double>());
  const Eigen::Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

  // Prediction error.
  Eigen::Matrix<double, 2, 3> J_cam;
  Eigen::Vector2d px_est;
  cam.project3(xyz_in_cam, &px_est, &J_cam);
  Eigen::Vector2d px_diff = px.cast<double>() - px_est;
  double px_diff_norm2 = px_diff.squaredNorm();
  BearingVector f_est = xyz_in_cam.cast<float>().normalized();
  Eigen::Vector3d f_diff = f.cast<double>() - f_est.cast<double>();
  double f_diff_norm2 = f_diff.squaredNorm();
  double e_img = grad.cast<double>().dot(px_diff);
  double scale_ratio;
  if (px_diff.norm() > 1e-8)
    scale_ratio = f_diff.norm() / px_diff.norm();
  else
    scale_ratio = 1.0 / (xyz_in_cam.norm() + 1e-8);
  double e = e_img * scale_ratio;
  if (unwhitened_error)
    *unwhitened_error = std::abs(e);

  // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
  double R = 1.0 / measurement_sigma;
  e *= R;

  // M-estimator weighting
  double weight = robust_weight.weight(e);

  // Chi2 error, i.e. log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  *chi2_error = 0.5 * e * e * weight;

  if (H && g) {
    // Compute Jacobian.
    PoseOptimizer::Matrix26d J_proj;
    Frame::jacobian_xyz2img_imu(T_cam_imu, xyz_in_imu, J_cam, J_proj);
    PoseOptimizer::Matrix36d J_bearing;
    Frame::jacobian_xyz2f_imu(T_cam_imu, xyz_in_imu, J_bearing);

    PoseOptimizer::Vector6d J_img = grad.cast<double>().transpose() * (-1.0) * J_proj;

    PoseOptimizer::Vector6d J_ftf = 2 * f_diff.transpose() * (-1.0) * J_bearing;
    PoseOptimizer::Vector6d J_ptp = 2 * px_diff.transpose() * (-1.0) * J_proj;
    // binary folds the scalar as 1.0/(p2*p2) * (0.5/scale_ratio) (same value)
    PoseOptimizer::Vector6d J_ratio = (0.5) * (1.0 / (scale_ratio)) *
                                      (1 / (px_diff_norm2 * px_diff_norm2)) *
                                      (J_ftf * px_diff_norm2 - J_ptp * f_diff_norm2);

    PoseOptimizer::Vector6d J = e_img * J_ratio + scale_ratio * J_img;

    J *= R;
    H->noalias() += J * J.transpose() * weight;
    g->noalias() -= J * e * weight;
  }
}

}  // namespace pose_optimizer_utils


// ---- part 2 (c12) -----------------------------------------------------------------------

// 0x18013F580  upstream-identical
void PoseOptimizer::setRotationPrior(const Quaternion& R_frame_world, double lambda)
{
  Transformation T_cur_world_prior(R_frame_world, Eigen::Vector3d::Zero());
  Matrix6d Information = Matrix6d::Zero();
  Information.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity();
  prior_lambda_ = lambda;
  setPrior(T_cur_world_prior, Information);
}

// 0x18013ED70  upstream-modified (see notes)
size_t PoseOptimizer::run(const FrameBundle::Ptr& frame_bundle, double reproj_thresh_px,
                          bool remove_outliers)
{
  // Pimax: CHECKs replaced by a log line (no early return for the "no features" case);
  // at(0) still throws std::out_of_range on an empty bundle.
  if(frame_bundle->numFeatures() == 0)
    LOGE("PoseOptimizer: No features in frames\n");
  focal_length_ = frame_bundle->at(0)->getErrorMultiplier();
  frame_bundle_ = frame_bundle;
  Transformation T_imu_world = frame_bundle->at(0)->T_imu_world();

  // Check the scale of the errors.
  std::vector<float> start_errors;
  evaluateErrorImpl(T_imu_world, nullptr, nullptr, &start_errors);
  measurement_sigma_ = scale_estimator_.compute(start_errors);

  // Run Gauss Newton optimization.
  optimize(T_imu_world);

  for(const FramePtr& frame : frame_bundle->frames_)
  {
    frame->T_f_w_ = frame->T_cam_imu()*T_imu_world;
  }

  // Remove Measurements with too large reprojection error
  size_t n_deleted_edges = 0, n_deleted_corners = 0;
  if(remove_outliers)
  {
    std::vector<double> final_errors;
    for(const FramePtr& f : frame_bundle->frames_)
    {
      removeOutliers(reproj_thresh_px, f.get(),
                     &final_errors, &n_deleted_edges, &n_deleted_corners);
    }
#line 62
    VLOG(5) <<"PoseOptimzer: drop " << n_deleted_corners << " corner outliers and "
            << n_deleted_edges << " edgelet outliers out of "
            << n_meas_ << " measurements.";

    // save statistics
    double error_scale = (err_type_ == ErrorType::kUnitPlane) ? focal_length_ : 1.0;
    stats_.reproj_error_before = vk::getMedian<float>(start_errors)*error_scale;
    stats_.reproj_error_after = vk::getMedian<double>(final_errors)*error_scale;

    // trace to file
    if(ofs_reproj_errors_.is_open())
    {
      for(auto i : start_errors)
        ofs_reproj_errors_ << i*error_scale << ", ";
      ofs_reproj_errors_ << std::endl;
      for(auto i : final_errors)
        ofs_reproj_errors_ << i*error_scale << ", ";
      ofs_reproj_errors_ << std::endl;
    }
  }

  return n_meas_-n_deleted_corners-n_deleted_edges;
}

// 0x18013D8B0  upstream-modified (see notes)
void PoseOptimizer::removeOutliers(
    const double reproj_err_threshold,
    Frame* frame,
    std::vector<double>* reproj_errors,
    std::size_t* n_deleted_edges,
    std::size_t* n_deleted_corners)
{
  // Pimax: CHECK_NOTNULL x4 replaced by one log line.
  if(frame == nullptr || reproj_errors == nullptr
     || n_deleted_edges == nullptr || n_deleted_corners == nullptr)
  {
    LOGE("removeOutliers has NULL\n");
    return;
  }

  // Pimax: thresholds recomputed on every call (upstream: function-local statics).
  double threshold_uplane = reproj_err_threshold / focal_length_;
  double threshold_bearing_diff = std::fabs(2 * std::sin(0.5*frame->getAngleError(reproj_err_threshold)));

  double outlier_threshold = reproj_err_threshold; // image error
  if(err_type_ == ErrorType::kUnitPlane)
    outlier_threshold = threshold_uplane;
  else if(err_type_ == ErrorType::kBearingVectorDiff)
    outlier_threshold = threshold_bearing_diff;

  reproj_errors->reserve(frame->num_features_);
  for(size_t i = 0; i < frame->num_features_; ++i)
  {
    Position xyz_world;
    if(frame->track_id_vec_(i) > -1)          // Pimax: track id instead of landmark_vec_[i] != nullptr
    {
      xyz_world = frame->landmark_vec_[i]->pos_;
    }
    else if(isCornerEdgeletSeed(frame->type_vec_[i]))
    {
      const SeedRef& ref = frame->seed_ref_vec_[i];
      // Pimax: float position; T_world_cam() is cast<float>() (renormalised quaternion).
      xyz_world = ref.keyframe->T_world_cam().cast<FloatType>()
          * ref.keyframe->getSeedPosInFrame(ref.seed_id);
    }
    else
      continue;

    // calculate residual according to different feature type and residual
    Transformation T_imu_world = frame->T_imu_world();
    Transformation T_cam_imu = frame->T_cam_imu();
    double unwhitened_error = 0.0, chi2_error = 0.0;   // Pimax: initialised
    if(isEdgelet(frame->type_vec_[i]))
    {
      if(err_type_ == ErrorType::kUnitPlane)
      {
         pose_optimizer_utils::calculateEdgeletResidualUnitPlane(
              frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
              T_imu_world, T_cam_imu,
              0.0, robust_weight_, &unwhitened_error, &chi2_error, nullptr, nullptr);
      }
      else if(err_type_ == ErrorType::kBearingVectorDiff)
      {
        pose_optimizer_utils::calculateEdgeletResidualBearingVectorDiff(
              frame->px_vec_.col(i), frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
              T_imu_world, T_cam_imu, *frame->cam(),
              0.0, robust_weight_, &unwhitened_error, &chi2_error, nullptr, nullptr);
      }
      else if(err_type_ == ErrorType::kImagePlane)
      {
        pose_optimizer_utils::calculateEdgeletResidualImagePlane(
              frame->px_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
              T_imu_world, T_cam_imu, *frame->cam(),
              0.0, robust_weight_, &unwhitened_error, &chi2_error, nullptr, nullptr);
      }
    }
    else
    {
      if(err_type_ == ErrorType::kUnitPlane)
      {
        pose_optimizer_utils::calculateFeatureResidualUnitPlane(
              frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
              0.0, robust_weight_, &unwhitened_error, &chi2_error, nullptr, nullptr);
      }
      else if(err_type_ == ErrorType::kBearingVectorDiff)
      {
        pose_optimizer_utils::calculateFeatureResidualBearingVectorDiff(
              frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
              0.0, robust_weight_, &unwhitened_error, &chi2_error, nullptr, nullptr);
      }
      else if(err_type_ == ErrorType::kImagePlane)
      {
        pose_optimizer_utils::calculateFeatureResidualImagePlane(
              frame->px_vec_.col(i), xyz_world, T_imu_world, T_cam_imu, *frame->cam(),
              0.0, robust_weight_, &unwhitened_error, &chi2_error, nullptr, nullptr);
      }
    }
    unwhitened_error *= 1.0 / (1 << frame->level_vec_(i));
    reproj_errors->push_back(unwhitened_error);
    if(std::fabs(unwhitened_error) > outlier_threshold)
    {
      if(isEdgelet(frame->type_vec_[i]))
        ++(*n_deleted_edges);
      else
        ++(*n_deleted_corners);

      frame->type_vec_[i] = FeatureType::kOutlier;
      frame->seed_ref_vec_[i].keyframe.reset();
      // Pimax: keyframes also drop the observation from the point.
      if(frame->is_keyframe_)
      {
        PointPtr point = frame->landmark_vec_[i];
        if(point)
          point->removeObservation(frame->id_);
      }
      frame->landmark_vec_[i] = nullptr; // delete landmark observation
      frame->track_id_vec_(i) = -1;      // Pimax
    }
  }
}

// 0x18013F830  upstream-identical
void PoseOptimizer::update(
    const State& T_imuold_world,
    const UpdateVector& dx,
    State& T_imunew_world)
{
  T_imunew_world = Transformation::exp(dx)*T_imuold_world;

  // we need to normalize from time to time, otherwise rounding errors sum up
  T_imunew_world.getRotation().toImplementation().normalize();
}

}  // namespace totem
}  // namespace pimax
