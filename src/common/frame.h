// pimax_slam.pi.dll -- src/common/frame.h
//
// Pimax fork of svo_common/include/svo/common/frame.h, namespace pimax::totem.
// Single reconciled declaration of Frame and FrameBundle.  Sources: c05 (frame.cpp object
// 0x180090180..0x180096E90: ctors, dtor, all out-of-line members), c06 (+0x14, +0x180, +0x188,
// +0x1A8/+0x1AC, +0x1C8, +0x2F8), c07 (+0xEA, FrameBundle +0xE5), c09 (inline copies imuPos
// 0x180110400, get_T_W_B 0x18010ADA0, numLandmarks 0x180118580; FrameBundle layout), c10
// (set_T_w_imu 0x1801283F0, setIMUState 0x180127700), c11 (Jacobians 0x18013BC70 / 0x18013B8F0 /
// 0x18013B300), c12 (getSeedDepth 0x1801420C0, +0x1B0), c13 (+0x250 is 3xN float), c01/c02/c03
// (FrameBundle +0xD8, +0xE4, Frame +0x300).
//
// ---------------------------------------------------------------------------------------------
// Frame: sizeof == 0x340 (make_shared<Frame> allocates 0x350), vtable 0x1803B1588 (only the
// deleting dtor 0x1800935E0).  MSVC pads the vfptr slot to 16 bytes because the class is
// 16-aligned (Eigen members), hence nothing lives at +0x08.
//   +0x010 int id_                   +0x014 int cam_index_ [pimax]
//   +0x018 int exposure_time_ [pimax] +0x01C int gain_ [pimax]
//   +0x020 BundleId bundle_id_        +0x024 int nframe_index_ (-1)
//   +0x028 CameraPtr cam_             +0x040 Transformation T_f_w_
//   +0x080 ImgPyr img_pyr_            +0x098 key_pts_ (5 x {int, Vector3f})
//   +0x0B0 bool is_keyframe_          +0x0B1 bool flag_b1_ [pimax?]
//   +0x0B4 int last_published_ts_     +0x0C0 Eigen::Quaterniond R_imu_world_ (uninitialised)
//   +0x0E0 double mean_intensity_     +0x0E8/+0x0E9 bool is_too_dark_/is_too_bright_
//   +0x0EA bool is_redundant_kf_      +0x0F0 int64_t timestamp_
//   +0x100 Transformation T_body_cam_ +0x140 Transformation T_cam_body_
//   +0x180 uint16_t grid_cell_size_   +0x188 std::vector<bool> grid_occupancy_
//   +0x1A8 float min_depth_           +0x1AC float median_depth_
//   +0x1B0 std::vector<double> level_reproj_thresh_ {1.5,3,6,12}
//   +0x1C8 cv::Mat feature_mask_
//   +0x228 size_t num_features_ and the feature storage up to +0x2F8, float seed_mu_range_ +0x2F8
//   +0x300 Transformation accumulated_w_T_correction_
// No in_ba_graph_vec_, is_stable_, imu_vel_w_/imu_*_bias_ (the IMU state lives in FrameBundle).
//
// FrameBundle: sizeof == 0x100 (malloc(0x100) in 0x180127550, make_shared 0x110 in 0x1800FB650).
// ---------------------------------------------------------------------------------------------
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdDeque>
#include <opencv2/core/core.hpp>

#include <vikit/math_utils.h>          // vk::skew (out-of-line 0x180042410)

#include "common/types.h"              // FeatureType, Keypoints, Bearings, Scores, Levels, ...
#include "common/transformation.h"     // Transformation, Quaternion, Position
#include "common/camera_fwd.h"         // Camera, CameraPtr (vk::cameras::CameraGeometryBase)
#include "common/feature_wrapper.h"    // FeatureWrapper, SeedRef
#include "common/imu_calibration.h"    // ImuMeasurements
#include "common/point.h"              // Point, PointPtr
#include "common/seed.h"               // seed::getDepth

namespace pimax {
namespace totem {

//------------------------------------------------------------------------------
/// A frame saves the image, the associated features and the estimated pose.
class Frame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Landmarks = std::vector<PointPtr>;
  using SeedRefs  = std::vector<SeedRef>;
  using KeyPoints = std::vector<std::pair<int, Position>,
                                Eigen::aligned_allocator<std::pair<int, Position> > >;

  static int frame_counter_;          ///< dword_18047DB38

  int id_;                            ///< +0x010 Unique id of the frame.
  int cam_index_;                     ///< +0x014 [pimax] index of the camera in the rig (ctor arg).
                                      ///<        Used as frames_ index by initializeSeeds and by
                                      ///<        Map::getClosestNKeyframesWithOverlap's comparator.
  int exposure_time_;                 ///< +0x018 [pimax] ctor arg (per-camera exposure) TODO(verify)
  int gain_;                          ///< +0x01C [pimax] ctor arg (per-camera gain) TODO(verify)
  BundleId bundle_id_;                ///< +0x020 set by the FrameBundle ctor lambda
  int nframe_index_ = -1;             ///< +0x024 Storage index in NFrame.
  CameraPtr cam_;                     ///< +0x028 Camera model.
  Transformation T_f_w_;              ///< +0x040 Transform (f)rame from (w)orld.
  ImgPyr img_pyr_;                    ///< +0x080 Image Pyramid.
  KeyPoints key_pts_;                 ///< +0x098 Five features and associated 3D points.
  bool is_keyframe_ = false;          ///< +0x0B0
  bool flag_b1_ = false;              ///< +0x0B1 [pimax?] only written (=false) by the ctors TODO(verify)
  int last_published_ts_;             ///< +0x0B4 (not initialised)
  Quaternion R_imu_world_;            ///< +0x0C0 Eigen quaternion, not initialised
  double mean_intensity_;             ///< +0x0E0 [pimax] mean of the 3x3-blurred level-0 image
  bool is_too_dark_ = false;          ///< +0x0E8 [pimax] mean_intensity_ < 25
  bool is_too_bright_ = false;        ///< +0x0E9 [pimax] mean_intensity_ > 220
  bool is_redundant_kf_ = false;      ///< +0x0EA [pimax] set by FrameProcessor::makeKeyframe
                                      ///<        (c01: "near_map_kf_") TODO(verify) name
  int64_t timestamp_ = -1;            ///< +0x0F0 Timestamp of when the image was recorded [ns].
  Transformation T_body_cam_;         ///< +0x100
  Transformation T_cam_body_;         ///< +0x140
  uint16_t grid_cell_size_;           ///< +0x180 [pimax] ctor arg (FrameProcessorBase options grid size)
  std::vector<bool> grid_occupancy_;  ///< +0x188 [pimax] ctor arg copy (FrameProcessorBase grid)
  float min_depth_ = 0.5f;            ///< +0x1A8 [pimax] written by frame_utils::getSceneDepth
  float median_depth_ = 2.0f;         ///< +0x1AC [pimax] written by frame_utils::getSceneDepth
  std::vector<double> level_reproj_thresh_ = {1.5, 3.0, 6.0, 12.0};  ///< +0x1B0 [pimax] TODO(verify) name
  cv::Mat feature_mask_;              ///< +0x1C8 [pimax] mask clone used by initializeSeeds TODO(verify) name

  // Features
  // All vectors must have the same length/cols at all time!
  // {
  size_t num_features_ = 0u;          ///< +0x228
  Keypoints px_vec_;                  ///< +0x230 Pixel Coordinates (float 2xN)
  Bearings f_vec_;                    ///< +0x240 Bearing Vector (float 3xN, normalised)
  Bearings f_vec_raw_;                ///< +0x250 [pimax] un-normalised back-projection TODO(verify) name
  Scores score_vec_;                  ///< +0x260 Keypoint detection scores
  Levels  level_vec_;                 ///< +0x270 Level of the feature
  Gradients grad_vec_;                ///< +0x280 Gradient direction of edgelet normal
  FeatureTypes type_vec_;             ///< +0x290 Is the feature a corner or an edgelet?
  Landmarks landmark_vec_;            ///< +0x2A8 Reference to 3D point. Can contain nullpointers!
  TrackIds track_id_vec_;             ///< +0x2C0 ID of every observed 3d point. -1 if no point assigned.
  SeedRefs seed_ref_vec_;             ///< +0x2D0 Only for seeds during reprojection
  SeedStates invmu_sigma2_a_b_vec_;   ///< +0x2E8 Vector containing all necessary information for seed update.
  // }

  FloatType seed_mu_range_;           ///< +0x2F8

  // loop correction
  Transformation accumulated_w_T_correction_;   ///< +0x300

  /// 0x180091DA0  upstream-modified: five extra arguments stored in the frame, initFrame()
  /// inlined without its CHECKs and without the colour-image branch.
  /// Called from make_shared<Frame>(...) in FrameProcessorBase::addFrameBundle 0x1800FB650 with
  /// (cams_->getCameraShared(i), images[i], timestamp, n_pyr_levels, i, exposure[i], gain[i],
  ///  grid occupancy (FrameProcessorBase +0xC08), grid size (FrameProcessorBase +0x110)).
  Frame(
      const CameraPtr& cam,
      const cv::Mat& img,
      const int64_t timestamp_ns,
      const size_t n_pyr_levels,
      const int cam_index,
      const int exposure_time,
      const int gain,
      const std::vector<bool>& grid_occupancy,
      const uint16_t grid_cell_size);

  /// 0x180092420  upstream-identical (constructor without image).
  Frame(
      const int id,
      const int64_t timestamp_ns,
      const CameraPtr& cam,
      const Transformation& T_world_cam);

  /// Empty constructor. Just for testing!
  Frame() {}

  /// 0x180092D60 (body) / 0x1800935E0 (deleting)
  virtual ~Frame();

  // no copy
  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  /// Initialize new frame and create image pyramid (inlined into the main ctor).
  void initFrame(const cv::Mat& img, size_t n_pyr_levels);

  void setKeyframe();                                       ///< 0x180096E80
  void deleteLandmark(const size_t& feature_index);         ///< 0x180094460 (c12: "removeLandmark")
  void resizeFeatureStorage(size_t num);                    ///< 0x180095730
  FeatureWrapper getFeatureWrapper(size_t idx);             ///< 0x180094580
  FeatureWrapper getEmptyFeatureWrapper();                  ///< 0x180094550

  /// Get depth at seed.  Out-of-line copy 0x1801420C0 (reprojector.obj).
  inline FloatType getSeedDepth(size_t idx) const {
    return seed::getDepth(invmu_sigma2_a_b_vec_.col(idx));
  }

  /// Get coordinates of seed in frame coordinates.
  inline Position getSeedPosInFrame(size_t idx) const {
    return f_vec_.col(idx) * getSeedDepth(idx);
  }

  /// Number of features. Not necessarily succesfully tracked ones.
  inline size_t numFeatures() const {
    return num_features_;
  }

  /// Check if i-th keypoint has a reference to a landmark.
  inline bool isValidLandmark(size_t i) const {
    return (landmark_vec_.at(i) != nullptr);
  }

  /// [pimax] number of features with a track id.  Upstream counted non-null landmark_vec_
  /// entries.  Out-of-line copy 0x180118580 (frame_processor_base.obj).
  inline size_t numLandmarks() const {
    size_t count = 0;
    for(size_t i = 0; i < num_features_; ++i)
    {
      if(track_id_vec_(i) > -1)
        ++count;
    }
    return count;
  }

  /// Number of landmark references in BA (upstream; inlined into FrameBundle 0x180095240).
  inline size_t numLandmarksInBA() const {
    return static_cast<size_t>(
          std::count_if(landmark_vec_.begin(), landmark_vec_.end(),
                        [](const PointPtr& p)
    { return p != nullptr && p->in_ba_graph_; }));
  }

  /// [pimax] predicate of FrameBundle::numTrackedEdgelets 0x1800952D0 (see types.h).
  inline size_t numTrackedEdgelets() const {
    size_t count = 0;
    for(size_t i = 0; i < num_features_; ++i)
    {
      if(landmark_vec_[i] != nullptr && isTrackedEdgeletType(type_vec_[i]))
        ++count;
    }
    return count;
  }

  /// upstream-modified (inlined into FrameBundle 0x180095360): landmark || corner/edgelet seed
  /// (the fixed-landmark / map-point exclusions are gone).
  inline size_t numTrackedFeatures() const {
    size_t count = 0;
    for(size_t i = 0; i < num_features_; ++i)
    {
      if(landmark_vec_[i] != nullptr || isCornerEdgeletSeed(type_vec_[i]))
        ++count;
    }
    return count;
  }

  /// upstream-modified (inlined into FrameBundle 0x1800953F0): any non-null landmark among the
  /// first num_features_ slots (the fixed-landmark / map-point exclusions are gone).
  inline size_t numTrackedLandmarks() const {
    size_t count = 0;
    for(size_t i = 0; i < num_features_; ++i)
    {
      if(landmark_vec_[i] != nullptr)
        ++count;
    }
    return count;
  }

  /// The KeyPoints are those five features which are closest to the 4 image corners
  /// and to the center and which have a 3D point assigned.
  void setKeyPoints();                                      ///< 0x180096730
  inline void resetKeyPoints()
  {
    key_pts_.resize(5, std::make_pair(-1, Position()));
  }

  /// Check if a point in (w)orld coordinate frame is visible in the image.
  /// [pimax] z >= 0 and hard-coded 640x480 bounds with a 16 px border.
  bool isVisible(const Eigen::Vector3d& xyz_w,
                 Eigen::Vector2d* px = nullptr) const;      ///< 0x180094FB0

  /// [pimax] true if more than 80% of the 5x5 window around px (level 0) is brighter than 220.
  /// TODO(verify) name (c06: "isSaturated").
  bool isSaturatedPatch(const Eigen::Vector2i& px) const;  ///< 0x180094EC0

  /// [pimax] project a world point: cam_->project3(T_f_w_ * xyz_w).  Callers: backend
  /// (0x180010440, 0x180010C20, 0x180011700), outlier counting 0x18008A7C0.
  /// TODO(verify) name: the in-object order (sorted by decorated name) puts it between
  /// "num*" and "resizeFeatureStorage" (e.g. "project...").
  Eigen::Vector2d w2c(const Eigen::Vector3d& xyz_w, Eigen::Vector2d* px = nullptr) const;   ///< 0x180095450

  /// [pimax] project a point given in camera coordinates.  TODO(verify) name (see w2c).
  Eigen::Vector2d f2c(const Eigen::Vector3d& xyz_f, Eigen::Vector2d* px = nullptr) const;   ///< 0x1800955C0

  /// Masks is same size as the image. Convention: 0 == masked, nonzero == valid.
  const cv::Mat& getMask() const;                           ///< 0x180094880

  /// Full resolution image stored in the frame.
  inline const cv::Mat& img() const { return img_pyr_[0]; }

  /// Id of frame.
  inline int id() const { return id_; }

  /// Id of frame bundle
  inline BundleId bundleId() const { return bundle_id_; }

  /// Get storage index of frame in parent NFrame.  (Pimax: no CHECK_GE)
  inline int getNFrameIndex() const { return nframe_index_; }

  /// Set storage index of frame in parent NFrame.
  inline void setNFrameIndex(size_t nframe_index) { nframe_index_ = nframe_index; }

  /// Get camera pose in imu frame.
  inline const Transformation& T_imu_cam() const { return T_body_cam_; }

  /// Get imu pose in camera frame.
  inline const Transformation& T_cam_imu() const { return T_cam_body_; }

  /// Get pose of world origin in frame coordinates.
  inline const Transformation& T_cam_world() const { return T_f_w_; }

  /// Get pose of the cam in world coordinates.
  inline Transformation T_world_cam() const { return T_f_w_.inverse(); }

  /// Get pose of imu in world coordinates.
  inline Transformation T_world_imu() const { return (T_imu_cam()*T_f_w_).inverse(); }

  /// Get pose of world-origin in IMU coordinates.
  inline Transformation T_imu_world() const { return T_imu_cam()*T_f_w_; }

  /// [pimax] both directions are passed (NCamera::get_T_C_B / get_T_B_C, the latter evaluated
  /// first by MSVC); upstream computed T_body_cam_ = T_cam_imu.inverse().
  inline void set_T_cam_imu(const Transformation& T_cam_imu, const Transformation& T_imu_cam)
  {
    T_cam_body_ = T_cam_imu;
    T_body_cam_ = T_imu_cam;
  }

  /// set new pose.  Out-of-line copy 0x1801283F0.  [pimax] explicit renormalisation.
  inline void set_T_w_imu(const Transformation& T_w_imu)
  {
    T_f_w_ = (T_w_imu * T_body_cam_).inverse();
    T_f_w_.getRotation().normalize();
  }

  /// Camera model.
  inline const CameraPtr& cam() const { return cam_; }

  /// Timestamp of frame in nanoseconds.
  inline int64_t getTimestampNSec() const { return timestamp_; }

  /// Timestamp of frame in seconds (division, as upstream; FrameBundle multiplies by 1e-9).
  inline double getTimestampSec() const { return static_cast<double>(timestamp_)/1e9; }

  /// Was this frames selected as keyframe?
  inline bool isKeyframe() const { return is_keyframe_; }

  /// [pimax] position of the frame in the world frame, as float.
  inline Position pos() const { return T_world_cam().getPosition().cast<FloatType>(); }

  /// [pimax] position of the imu in the world frame, as float.  Out-of-line copy 0x180110400.
  inline Eigen::Vector3f imuPos() const { return T_world_imu().getPosition().cast<float>(); }

  double getErrorMultiplier() const;                        ///< 0x180094570 (camera vslot 5)

  double getAngleError(double img_err) const;               ///< 0x180094540 (camera vslot 8)

  /// Frame jacobian for projection of 3D point in (f)rame coordinate to
  /// unit plane coordinates uv (focal length = 1).  (upstream)
  inline static void jacobian_xyz2uv(
      const Eigen::Vector3d& xyz_in_f,
      Eigen::Matrix<double,2,6>& J)
  {
    const double x = xyz_in_f[0];
    const double y = xyz_in_f[1];
    const double z_inv = 1./xyz_in_f[2];
    const double z_inv_2 = z_inv*z_inv;

    J(0,0) = -z_inv;              // -1/z
    J(0,1) = 0.0;                 // 0
    J(0,2) = x*z_inv_2;           // x/z^2
    J(0,3) = y*J(0,2);            // x*y/z^2
    J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
    J(0,5) = y*z_inv;             // y/z

    J(1,0) = 0.0;                 // 0
    J(1,1) = -z_inv;              // -1/z
    J(1,2) = y*z_inv_2;           // y/z^2
    J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
    J(1,4) = -J(0,3);             // -x*y/z^2
    J(1,5) = -x*z_inv;            // -x/z
  }

  /// Jacobian of reprojection error (on unit plane) w.r.t. IMU pose.
  /// Out-of-line copy 0x18013BC70 (pose_optimizer.obj).  upstream-identical.
  inline static void jacobian_xyz2uv_imu(
      const Transformation& T_cam_imu,
      const Eigen::Vector3d& p_in_imu,
      Eigen::Matrix<double,2,6>& J)
  {
    Eigen::Matrix<double,3,6> G_x; // Generators times pose
    G_x.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    G_x.block<3,3>(0,3) = -vk::skew(p_in_imu);
    const Eigen::Vector3d p_in_cam = T_cam_imu * p_in_imu;

    Eigen::Matrix<double,2,3> J_proj; // projection derivative
    J_proj << 1, 0, -p_in_cam[0]/p_in_cam[2],
        0, 1, -p_in_cam[1]/p_in_cam[2];

    J = - 1.0/p_in_cam[2] * J_proj * T_cam_imu.getRotation().getRotationMatrix() * G_x;
  }

  /// Jacobian of reprojection error (on image plane) w.r.t. IMU pose.
  /// Out-of-line copy 0x18013B8F0.  upstream-identical.
  inline static void jacobian_xyz2img_imu(
      const Transformation& T_cam_imu,
      const Eigen::Vector3d& p_in_imu,
      const Eigen::Matrix<double, 2, 3>& J_cam,
      Eigen::Matrix<double,2,6>& J)
  {
    Eigen::Matrix<double,3,6> G_x; // Generators times pose
    G_x.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    G_x.block<3,3>(0,3) = -vk::skew(p_in_imu);

    J =  J_cam * T_cam_imu.getRotation().getRotationMatrix() * G_x;
  }

  /// Jacobian of using unit bearing vector for map point w.r.t IMU pose.
  /// Out-of-line copy 0x18013B300.  upstream-modified: 1/pow(n2, 1.5) is computed as
  /// 1/(sqrt(n2)*n2) (binary: sqrt with the /fp:precise errno path, mulsd, divsd; no pow call --
  /// whereas Point::jacobian_xyz2f 0x18009ADE0 does call pow).
  inline static void jacobian_xyz2f_imu(
      const Transformation& T_cam_imu,
      const Eigen::Vector3d& p_in_imu,
      Eigen::Matrix<double, 3, 6>& J)
  {
    Eigen::Matrix<double,3,6> G_x;
    G_x.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    G_x.block<3,3>(0,3) = -vk::skew(p_in_imu);
    const Eigen::Vector3d p_in_cam = T_cam_imu * p_in_imu;

    Eigen::Matrix<double, 3, 3> J_normalize;
    double x2 = p_in_cam[0]*p_in_cam[0];
    double y2 = p_in_cam[1]*p_in_cam[1];
    double z2 = p_in_cam[2]*p_in_cam[2];
    double xy = p_in_cam[0]*p_in_cam[1];
    double yz = p_in_cam[1]*p_in_cam[2];
    double zx = p_in_cam[2]*p_in_cam[0];
    J_normalize << y2+z2, -xy, -zx,
        -xy, x2+z2, -yz,
        -zx, -yz, x2+y2;
    const double n2 = x2 + y2 + z2;
    J_normalize *= 1 / (std::sqrt(n2) * n2);

    J = J_normalize * T_cam_imu.getRotationMatrix() * G_x;
  }

  static void layout_check();
};

//------------------------------------------------------------------------------
/// A collection of frames observing the scene at the same time.
class FrameBundle
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<FrameBundle> Ptr;
  typedef std::vector<FramePtr> FrameList;

  /// 0x1800928D0  upstream-modified (parallel image statistics, see frame.cpp)
  FrameBundle(const std::vector<FramePtr>& frames);
  ~FrameBundle() = default;

  FrameBundle(const FrameBundle&) = delete;
  FrameBundle& operator=(const FrameBundle&) = delete;

  /// out-of-line copy 0x1800116D0 (std::vector::at)
  inline const FramePtr& at(size_t i) const {
    return frames_.at(i);
  }

  inline size_t size() const {
    return frames_.size();
  }

  inline bool empty() const {
    return frames_.empty();
  }

  inline BundleId getBundleId() const {
    return bundle_id_;
  }

  /// Get timestamp of camera rig.  (Pimax: no CHECK)
  inline int64_t getMinTimestampNanoseconds() const {
    return frames_[0]->getTimestampNSec();
  }

  /// Get timestamp of camera rig (multiplication by 1e-9, as upstream).
  inline double getMinTimestampSeconds() const {
    const double cur_t_sec =
        1e-9 * static_cast<double>(getMinTimestampNanoseconds());   // kNanoSecondsToSeconds
    return cur_t_sec;
  }

  /// Get pose of camera rig.  Out-of-line copy 0x18010ADA0 (Pimax: no CHECK).
  inline Transformation get_T_W_B() const {
    return frames_[0]->T_world_imu();
  }

  /// Set pose of camera rig.  Out-of-line copy 0x1800162B0 (Frame::set_T_w_imu inlined, which
  /// renormalises).
  inline void set_T_W_B(const Transformation& T_W_B) {
    for(const FramePtr& frame : frames_)
      frame->set_T_w_imu(T_W_B);
  }

  /// [pimax] the IMU state is stored in the bundle, not in every frame.  Out-of-line 0x180127700.
  inline void setIMUState(const Eigen::Vector3d& imu_vel_w,
                          const Eigen::Vector3d& gyr_bias,
                          const Eigen::Vector3d& acc_bias)
  {
    imu_vel_w_ = imu_vel_w;
    imu_gyr_bias_ = gyr_bias;
    imu_acc_bias_ = acc_bias;
  }

  inline void setKeyframe() {
    is_keyframe_ = true;
  }

  inline bool isKeyframe() const {
    return is_keyframe_;
  }

  // The counting functions.  Names follow the in-object order of frame.obj, whose COMDATs are
  // sorted by decorated name: numFeatures < numLandmarks < numLandmarksInBA <
  // numTrackedEdgelets < numTrackedFeatures < numTrackedLandmarks.  (c05/c09/c10 drafts used
  // other names for some of them; map them by address.)
  size_t numFeatures() const;          ///< 0x180095150  sum of num_features_
  size_t numLandmarks() const;         ///< 0x180095180  [pimax] track ids > -1 (c10: "numTrackedIds", c05: "numTrackIds")
  size_t numLandmarksInBA() const;     ///< 0x180095240
  size_t numTrackedEdgelets() const;   ///< 0x1800952D0  [pimax] TODO(verify) name (c05/c10: "numTrackedLandmarks")
  size_t numTrackedFeatures() const;   ///< 0x180095360
  size_t numTrackedLandmarks() const;  ///< 0x1800953F0  (c10: "numLandmarks")

  FrameList frames_;                              ///< +0x00
  ImuMeasurements imu_measurements_;              ///< +0x18 [pimax] IMU samples since the last bundle
  bool is_relocalized_ = false;                   ///< +0x40 [pimax] set by reLocalize TODO(verify) name
  Transformation T_W_B_init_;                     ///< +0x50 [pimax] VI-initialisation pose (identity) TODO(verify) name
  Eigen::Vector3d imu_vel_w_ = Eigen::Vector3d::Zero();      ///< +0x90
  Eigen::Vector3d imu_gyr_bias_ = Eigen::Vector3d::Zero();   ///< +0xA8
  Eigen::Vector3d imu_acc_bias_ = Eigen::Vector3d::Zero();   ///< +0xC0
  Eigen::Vector3f gravity_ = Eigen::Vector3f::Zero();        ///< +0xD8 [pimax] gravity written back by Estimator::addStates TODO(verify) name
  bool is_static_ = false;                        ///< +0xE4 [pimax] IMU stationary (addFrameBundle)
  bool low_feature_kf_ = false;                   ///< +0xE5 [pimax] (c10: "force_stereo_triangulation_") TODO(verify) name
  size_t num_tracked_ = 0;                        ///< +0xE8 [pimax] written by FrameProcessor TODO(verify) name
  double last_timestamp_sec_ = 0.0;               ///< +0xF0 [pimax] previous bundle's timestamp
  bool is_keyframe_ = false;                      ///< +0xF8
  BundleId bundle_id_;                            ///< +0xFC (counter dword_18047DB48)

  // Make class iterable:
  typedef FrameList::value_type value_type;
  typedef FrameList::iterator iterator;
  typedef FrameList::const_iterator const_iterator;
  FrameList::iterator begin() { return frames_.begin(); }
  FrameList::iterator end() { return frames_.end(); }
  FrameList::const_iterator begin() const { return frames_.begin(); }
  FrameList::const_iterator end() const { return frames_.end(); }
  FrameList::const_iterator cbegin() const { return frames_.cbegin(); }
  FrameList::const_iterator cend() const { return frames_.cend(); }

  static void layout_check();
};

/// Some helper functions for the frame object.
namespace frame_utils {

/// 0x1800942B0  upstream-modified (cv::pyrDown instead of vk::halfSample, no CHECKs)
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

/// 0x180094890  upstream-modified (rewritten; results stored in the frame: min_depth_,
/// median_depth_).  c07 called it "computeSceneDepth".
bool getSceneDepth(const FramePtr& frame);

/// 0x180093BB0  upstream-modified (float bearings + additional un-normalised output)
void computeNormalizedBearingVectors(
    const Keypoints& px_vec,
    const Camera& cam,
    Bearings* f_vec,
    Bearings* f_vec_raw);

} // namespace frame_utils

//------------------------------------------------------------------------------
inline void Frame::layout_check()
{
  static_assert(sizeof(Frame) == 0x340, "sizeof(Frame)");
  static_assert(offsetof(Frame, id_) == 0x010, "Frame::id_");
  static_assert(offsetof(Frame, cam_index_) == 0x014, "Frame::cam_index_");
  static_assert(offsetof(Frame, exposure_time_) == 0x018, "Frame::exposure_time_");
  static_assert(offsetof(Frame, gain_) == 0x01C, "Frame::gain_");
  static_assert(offsetof(Frame, bundle_id_) == 0x020, "Frame::bundle_id_");
  static_assert(offsetof(Frame, nframe_index_) == 0x024, "Frame::nframe_index_");
  static_assert(offsetof(Frame, cam_) == 0x028, "Frame::cam_");
  static_assert(offsetof(Frame, T_f_w_) == 0x040, "Frame::T_f_w_");
  static_assert(offsetof(Frame, img_pyr_) == 0x080, "Frame::img_pyr_");
  static_assert(offsetof(Frame, key_pts_) == 0x098, "Frame::key_pts_");
  static_assert(offsetof(Frame, is_keyframe_) == 0x0B0, "Frame::is_keyframe_");
  static_assert(offsetof(Frame, flag_b1_) == 0x0B1, "Frame::flag_b1_");
  static_assert(offsetof(Frame, last_published_ts_) == 0x0B4, "Frame::last_published_ts_");
  static_assert(offsetof(Frame, R_imu_world_) == 0x0C0, "Frame::R_imu_world_");
  static_assert(offsetof(Frame, mean_intensity_) == 0x0E0, "Frame::mean_intensity_");
  static_assert(offsetof(Frame, is_too_dark_) == 0x0E8, "Frame::is_too_dark_");
  static_assert(offsetof(Frame, is_too_bright_) == 0x0E9, "Frame::is_too_bright_");
  static_assert(offsetof(Frame, is_redundant_kf_) == 0x0EA, "Frame::is_redundant_kf_");
  static_assert(offsetof(Frame, timestamp_) == 0x0F0, "Frame::timestamp_");
  static_assert(offsetof(Frame, T_body_cam_) == 0x100, "Frame::T_body_cam_");
  static_assert(offsetof(Frame, T_cam_body_) == 0x140, "Frame::T_cam_body_");
  static_assert(offsetof(Frame, grid_cell_size_) == 0x180, "Frame::grid_cell_size_");
  static_assert(offsetof(Frame, grid_occupancy_) == 0x188, "Frame::grid_occupancy_");
  static_assert(offsetof(Frame, min_depth_) == 0x1A8, "Frame::min_depth_");
  static_assert(offsetof(Frame, median_depth_) == 0x1AC, "Frame::median_depth_");
  static_assert(offsetof(Frame, level_reproj_thresh_) == 0x1B0, "Frame::level_reproj_thresh_");
  static_assert(offsetof(Frame, feature_mask_) == 0x1C8, "Frame::feature_mask_");
  static_assert(offsetof(Frame, num_features_) == 0x228, "Frame::num_features_");
  static_assert(offsetof(Frame, px_vec_) == 0x230, "Frame::px_vec_");
  static_assert(offsetof(Frame, f_vec_) == 0x240, "Frame::f_vec_");
  static_assert(offsetof(Frame, f_vec_raw_) == 0x250, "Frame::f_vec_raw_");
  static_assert(offsetof(Frame, score_vec_) == 0x260, "Frame::score_vec_");
  static_assert(offsetof(Frame, level_vec_) == 0x270, "Frame::level_vec_");
  static_assert(offsetof(Frame, grad_vec_) == 0x280, "Frame::grad_vec_");
  static_assert(offsetof(Frame, type_vec_) == 0x290, "Frame::type_vec_");
  static_assert(offsetof(Frame, landmark_vec_) == 0x2A8, "Frame::landmark_vec_");
  static_assert(offsetof(Frame, track_id_vec_) == 0x2C0, "Frame::track_id_vec_");
  static_assert(offsetof(Frame, seed_ref_vec_) == 0x2D0, "Frame::seed_ref_vec_");
  static_assert(offsetof(Frame, invmu_sigma2_a_b_vec_) == 0x2E8, "Frame::invmu_sigma2_a_b_vec_");
  static_assert(offsetof(Frame, seed_mu_range_) == 0x2F8, "Frame::seed_mu_range_");
  static_assert(offsetof(Frame, accumulated_w_T_correction_) == 0x300, "Frame::accumulated_w_T_correction_");
}

inline void FrameBundle::layout_check()
{
  static_assert(sizeof(FrameBundle) == 0x100, "sizeof(FrameBundle)");
  static_assert(offsetof(FrameBundle, frames_) == 0x00, "FrameBundle::frames_");
  static_assert(offsetof(FrameBundle, imu_measurements_) == 0x18, "FrameBundle::imu_measurements_");
  static_assert(offsetof(FrameBundle, is_relocalized_) == 0x40, "FrameBundle::is_relocalized_");
  static_assert(offsetof(FrameBundle, T_W_B_init_) == 0x50, "FrameBundle::T_W_B_init_");
  static_assert(offsetof(FrameBundle, imu_vel_w_) == 0x90, "FrameBundle::imu_vel_w_");
  static_assert(offsetof(FrameBundle, imu_gyr_bias_) == 0xA8, "FrameBundle::imu_gyr_bias_");
  static_assert(offsetof(FrameBundle, imu_acc_bias_) == 0xC0, "FrameBundle::imu_acc_bias_");
  static_assert(offsetof(FrameBundle, gravity_) == 0xD8, "FrameBundle::gravity_");
  static_assert(offsetof(FrameBundle, is_static_) == 0xE4, "FrameBundle::is_static_");
  static_assert(offsetof(FrameBundle, low_feature_kf_) == 0xE5, "FrameBundle::low_feature_kf_");
  static_assert(offsetof(FrameBundle, num_tracked_) == 0xE8, "FrameBundle::num_tracked_");
  static_assert(offsetof(FrameBundle, last_timestamp_sec_) == 0xF0, "FrameBundle::last_timestamp_sec_");
  static_assert(offsetof(FrameBundle, is_keyframe_) == 0xF8, "FrameBundle::is_keyframe_");
  static_assert(offsetof(FrameBundle, bundle_id_) == 0xFC, "FrameBundle::bundle_id_");
}

} // namespace totem
} // namespace pimax
