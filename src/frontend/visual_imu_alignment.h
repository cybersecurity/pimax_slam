// pimax_slam.pi.dll -- src/frontend/visual_imu_alignment.h  (drafts c13 + c07 + c09 merged)
//
// VINS-Mono style visual-inertial initialisation (initial_alignment.h) as a small Pimax class,
// plus the VINS ImageFrame (value of FrameProcessorBase::all_image_frame_).  Class/file names are
// ours: TODO(verify).  The object sits between frontend/stereo_triangulation.obj and
// interface/ceres_backend_factory.obj.
//
// c09 had modelled the ImuInitializer bytes as {double, double, double g_norm, ...} with free
// functions; c13's member-function view (the alignment functions receive `this`) and c07's
// ctor 0x1800E1FD0 (G = (0,0,9.80667), identity Transformation) are used.
#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "common/transformation.h"
#include "frontend/integration_base.h"

namespace pimax {
namespace totem {

/// VINS-Mono ImageFrame, Pimax-extended.  sizeof == 448 (std::map<double, ImageFrame> node 0x1F0).
/// Default ctor 0x1800E1CC0 (user-provided and empty: only the map, the shared_ptr and the four
/// Transformations are initialised), implicit copy ctor 0x1800E1AA0, dtor 0x1800E4E10.
/// Names of the Pimax members: c13 where the alignment code (0x180156C10) reads/writes them,
/// c09 otherwise.  TODO(verify) all Pimax names.
class ImageFrame
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImageFrame() {}

  std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> points;  // +0  (VINS)
  double t;                                          // +16
  Eigen::Matrix3d R;                                 // +24
  Eigen::Vector3d T;                                 // +96
  std::shared_ptr<IntegrationBase> pre_integration;  // +120 (VINS: raw pointer)
  bool is_key_frame;                                 // +136
  Transformation T_c0_body;     // +144 (c13 T_c0_body_, read by 0x180156C10; c09 T_imu_aligned)
  Transformation T_world_cam;   // +208 (c09)
  Transformation T_i;           // +272 (c09: pose relative to the first frame)
  Transformation T_w_body;      // +336 (c13 T_w_body_, written by 0x180156C10; c09 T_world_imu)
  Eigen::Vector3d V_w;          // +400 (c13 V_w_; c09 velocity)
  Eigen::Vector3d Bg;           // +424 (c13 Bg_; c09 vec_424)

 private:
  friend struct ImageFrameLayoutCheck;
};

struct ImageFrameLayoutCheck
{
  static_assert(sizeof(ImageFrame) == 448, "sizeof(ImageFrame)");
  static_assert(offsetof(ImageFrame, t) == 16, "");
  static_assert(offsetof(ImageFrame, R) == 24, "");
  static_assert(offsetof(ImageFrame, T) == 96, "");
  static_assert(offsetof(ImageFrame, pre_integration) == 120, "");
  static_assert(offsetof(ImageFrame, is_key_frame) == 136, "");
  static_assert(offsetof(ImageFrame, T_c0_body) == 144, "");
  static_assert(offsetof(ImageFrame, T_world_cam) == 208, "");
  static_assert(offsetof(ImageFrame, T_i) == 272, "");
  static_assert(offsetof(ImageFrame, T_w_body) == 336, "");
  static_assert(offsetof(ImageFrame, V_w) == 400, "");
  static_assert(offsetof(ImageFrame, Bg) == 424, "");
};

/// Gyroscope-bias vector of initializeImu (B2 request: built with vector(n, Zero()) 0x1800DFCA0
/// whose allocator 0x1800FD8E0 is Eigen's aligned malloc, freed with free() -> aligned_allocator).
using GyroBiasVector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

/// sizeof 136; a stack local of FrameProcessorBase::initializeImu 0x180110490.  ctor 0x1800E1FD0
/// (G = (0, 0, 9.80667), T_w0_ = identity; the caller then overwrites G.z() with the configured
/// gravity magnitude and fills RIC/TIC), dtor 0x1800E4E90 (two plain free()s: aligned vectors).
class ImuInitializer
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuInitializer() = default;    // 0x1800E1FD0
  ~ImuInitializer() = default;   // 0x1800E4E90

  /// 0x180157240 ("INFO=ImuInitial||frame_num=...")
  bool VisualIMUAlignment(std::map<double, ImageFrame>& all_image_frame,
                          GyroBiasVector& Bgs, Eigen::Vector3d& g,
                          Eigen::VectorXd& x, Eigen::Vector3d ba);

  /// 0x180158050
  void solveGyroscopeBias(std::map<double, ImageFrame>& all_image_frame,
                          GyroBiasVector& Bgs, Eigen::Vector3d ba);
  /// 0x1801569A0 (a member copy, distinct from pimax::totem::TangentBasis 0x18001ACE0)
  Eigen::MatrixXd TangentBasis(Eigen::Vector3d& g0);
  /// 0x1801518F0
  void RefineGravity(std::map<double, ImageFrame>& all_image_frame, Eigen::Vector3d& g,
                     Eigen::VectorXd& x);
  /// 0x18014DFE0
  bool LinearAlignment(std::map<double, ImageFrame>& all_image_frame, Eigen::Vector3d& g,
                       Eigen::VectorXd& x);
  /// 0x180156C10 [pimax-new] ("INFO=ImuInitialGw0").  TODO(verify) name.
  void computeStatesInGravityFrame(std::map<double, ImageFrame>& all_image_frame,
                                   GyroBiasVector& Bgs, Eigen::Vector3d& g,
                                   Eigen::VectorXd& x);
  /// 0x180158C90 [pimax-new] (`this` unused).  Population standard deviation.  TODO(verify) name.
  double computeStd(const std::vector<double>& values);

  Eigen::Vector3d G{0.0, 0.0, 9.80667};                                          // +0
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> RIC;   // +24
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> TIC;   // +48
  Transformation T_w0_;                                                          // +80
};

static_assert(sizeof(ImuInitializer) == 144, "136 bytes of data, 16-aligned");
static_assert(offsetof(ImuInitializer, RIC) == 24, "");
static_assert(offsetof(ImuInitializer, TIC) == 48, "");
static_assert(offsetof(ImuInitializer, T_w0_) == 80, "");

}  // namespace totem
}  // namespace pimax
