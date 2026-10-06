// pimax_slam.pi.dll -- src/frontend/pose_optimizer.h  (drafts c11 + c12 merged)
//
// pimax::totem::PoseOptimizer (fork of svo/include/svo/pose_optimizer.h).
// c11: getDefaultSolverOptions, ctor/dtor, applyPrior, evaluateErrorImpl and the six residual
// helpers; c12: run (Pimax extra flag), removeOutliers, update, evaluateError, setRotationPrior and
// the MiniLeastSquaresSolver<6,...> optimize* instantiations (vendored vikit, A1).
#pragma once

#include <fstream>
#include <iostream>

#include <vikit/solver/mini_least_squares_solver.h>
#include <vikit/solver/robust_cost.h>

#include <memory>
#include <string>
#include <vector>

#include "common/camera.h"
#include "common/frame.h"
#include "common/transformation.h"
#include "common/types.h"

namespace pimax {
namespace totem {

/// sizeof 0x4E0 (allocated with Eigen aligned new at 0x1800DFDF0).
/// Base vk::solver::MiniLeastSquaresSolver<6, Transformation, PoseOptimizer> (upstream layout):
///   +0 vfptr (padded to 16) | +16 solver_options_ (56) | +80 H_ | +368 g_ | +416 dx_ |
///   +464 have_prior_ | +480 prior_ | +544 I_prior_ | +832 chi2_ | +840 rho_ | +848 mu_ (0.01) |
///   +856 nu_ (2.0) | +864 n_meas_ | +872 stop_ | +880 iter_ | +888 trials_
class PoseOptimizer
    : public vk::solver::MiniLeastSquaresSolver<6, Transformation, PoseOptimizer> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using ScaleEstimator = vk::solver::MADScaleEstimator;
  using RobustWeightFunction = vk::solver::TukeyWeightFunction;
  using SolverOptions = vk::solver::MiniLeastSquaresSolverOptions;
  typedef std::shared_ptr<PoseOptimizer> Ptr;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 2, 6> Matrix26d;
  typedef Eigen::Matrix<double, 3, 6> Matrix36d;
  typedef Eigen::Matrix<double, 1, 6> Matrix16d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  enum class ErrorType { kUnitPlane, kBearingVectorDiff, kImagePlane };

  struct Statistics {
    double reproj_error_after;
    double reproj_error_before;
    Statistics() : reproj_error_after(0.0), reproj_error_before(0.0) {}
  } stats_;                                          // +896

  PoseOptimizer(SolverOptions solver_options);       // 0x180132D90
  virtual ~PoseOptimizer() = default;                // vtbl[0] 0x1801330F0

  static SolverOptions getDefaultSolverOptions();    // 0x18013B2B0

  /// 0x18013ED70.  Pimax: extra flag; outlier removal + statistics only if remove_outliers
  /// (FrameProcessorBase::optimizePose passes a bool, c10/c12).
  size_t run(const FrameBundlePtr& frame, double reproj_thresh, bool remove_outliers);

  /// 0x18013F580
  void setRotationPrior(const Quaternion& R_frame_world, double lambda);

  inline size_t iterCount() const { return iter_; }
  inline void setErrorType(ErrorType type) { err_type_ = type; }

  inline void initTracing(const std::string& trace_dir) {
    ofs_reproj_errors_.open((trace_dir + "/reproj_errors.txt").c_str());
  }

  FrameBundlePtr frame_bundle_;                      // +912
  double prior_lambda_;                              // +928
  ScaleEstimator scale_estimator_;                   // +936 (vtable 0x1803B7300)
  RobustWeightFunction robust_weight_;               // +944 (ctor 0x1801B5C30, default b)
  double measurement_sigma_ = 1.0;                   // +960
  ErrorType err_type_ = ErrorType::kUnitPlane;       // +968
  double focal_length_ = 1.0;                        // +976
  std::ofstream ofs_reproj_errors_;                  // +984

  double evaluateError(const Transformation& T_imu_world, HessianMatrix* H, GradientVector* g);

  double evaluateErrorImpl(const Transformation& T_imu_world, HessianMatrix* H, GradientVector* g,
                           std::vector<float>* unwhitened_errors);   // 0x18013A840

  void removeOutliers(const double reproj_err_threshold, Frame* frame,
                      std::vector<double>* reproj_errors, size_t* n_deleted_edges,
                      size_t* n_deleted_corners);                    // 0x18013D8B0 (c12)

  void update(const State& T_frameold_from_world, const UpdateVector& dx,
              State& T_framenew_from_world);                         // 0x18013F830 (c12)

  virtual void applyPrior(const State& current_model);               // vtbl[1] 0x1801331C0

 private:
  friend struct PoseOptimizerLayoutCheck;
};

struct PoseOptimizerLayoutCheck
{
  static_assert(sizeof(PoseOptimizer) == 0x4E0, "sizeof(PoseOptimizer)");
  static_assert(offsetof(PoseOptimizer, stats_) == 0x380, "");
  static_assert(offsetof(PoseOptimizer, frame_bundle_) == 0x390, "");
  static_assert(offsetof(PoseOptimizer, prior_lambda_) == 0x3A0, "");
  static_assert(offsetof(PoseOptimizer, scale_estimator_) == 0x3A8, "");
  static_assert(offsetof(PoseOptimizer, robust_weight_) == 0x3B0, "");
  static_assert(offsetof(PoseOptimizer, measurement_sigma_) == 0x3C0, "");
  static_assert(offsetof(PoseOptimizer, err_type_) == 0x3C8, "");
  static_assert(offsetof(PoseOptimizer, focal_length_) == 0x3D0, "");
  static_assert(offsetof(PoseOptimizer, ofs_reproj_errors_) == 0x3D8, "");
};

namespace pose_optimizer_utils {

// Pimax: f / px / grad / xyz are float vectors (Ref<const Vector3f/Vector2f>, Vector3f).
void calculateFeatureResidualUnitPlane(
    const Eigen::Ref<const BearingVector>& f, const Position& xyz_in_world,
    const Transformation& T_imu_world, const Transformation& T_cam_imu, double measurement_sigma,
    const PoseOptimizer::RobustWeightFunction& robust_weight, double* unwhitened_error,
    double* chi2_error, PoseOptimizer::HessianMatrix* H, PoseOptimizer::GradientVector* g);   // 0x180139AE0

void calculateFeatureResidualImagePlane(
    const Eigen::Ref<const Keypoint>& px, const Position& xyz_in_world,
    const Transformation& T_imu_world, const Transformation& T_cam_imu, const Camera& cam,
    double measurement_sigma, const PoseOptimizer::RobustWeightFunction& robust_weight,
    double* unwhitened_error, double* chi2_error, PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g);                                                          // 0x180138430

void calculateFeatureResidualBearingVectorDiff(
    const Eigen::Ref<const BearingVector>& f, const Position& xyz_in_world,
    const Transformation& T_imu_world, const Transformation& T_cam_imu, double measurement_sigma,
    const PoseOptimizer::RobustWeightFunction& robust_weight, double* unwhitened_error,
    double* chi2_error, PoseOptimizer::HessianMatrix* H, PoseOptimizer::GradientVector* g);   // 0x180136EA0

void calculateEdgeletResidualUnitPlane(
    const Eigen::Ref<const BearingVector>& f, const Position& xyz_in_world,
    const Eigen::Ref<const GradientVector>& grad, const Transformation& T_imu_world,
    const Transformation& T_cam_imu, double measurement_sigma,
    const PoseOptimizer::RobustWeightFunction& robust_weight, double* unwhitened_error,
    double* chi2_error, PoseOptimizer::HessianMatrix* H, PoseOptimizer::GradientVector* g);   // 0x1801364D0

void calculateEdgeletResidualImagePlane(
    const Eigen::Ref<const Keypoint>& px, const Position& xyz_in_world,
    const Eigen::Ref<const GradientVector>& grad, const Transformation& T_imu_world,
    const Transformation& T_cam_imu, const Camera& cam, double measurement_sigma,
    const PoseOptimizer::RobustWeightFunction& robust_weight, double* unwhitened_error,
    double* chi2_error, PoseOptimizer::HessianMatrix* H, PoseOptimizer::GradientVector* g);   // 0x180135AF0

void calculateEdgeletResidualBearingVectorDiff(
    const Eigen::Ref<const Keypoint>& px, const Eigen::Ref<const BearingVector>& f,
    const Position& xyz_in_world, const Eigen::Ref<const GradientVector>& grad,
    const Transformation& T_imu_world, const Transformation& T_cam_imu, const Camera& cam,
    double measurement_sigma, const PoseOptimizer::RobustWeightFunction& robust_weight,
    double* unwhitened_error, double* chi2_error, PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g);                                                          // 0x1801338F0

}  // namespace pose_optimizer_utils

}  // namespace totem
}  // namespace pimax
