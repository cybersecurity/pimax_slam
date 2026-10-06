// pimax_slam.pi.dll -- src/common/point.h
//
// Pimax fork of svo_common/include/svo/common/point.h, namespace pimax::totem.
// Reconciled from c06 (point.h/point.cpp), c05 (ctor 0x180099F80, KeypointIdentifier ctor
// 0x180099F40), c03 (BA bookkeeping fields used by the estimator), c09/c10 (tracking counters),
// c12 (reprojection counters).
//
// Point: sizeof == 0x88 (make_shared allocates 0x98), no vtable.
//   +0x00 int id_                    (PointIdProvider::last_id_ 0x18047DB60, lock xadd)
//   +0x04 Position pos_              (Vector3f -- FloatType == float)
//   +0x10 obs_                       std::unordered_map<int frame_id, KeypointIdentifier> (64 B;
//                                    node 0x38: key +16, value +24).  Upstream: std::vector.
//   +0x50 int n_failed_reproj_       (c03/c12)
//   +0x54 int n_succeeded_reproj_    (c03/c12)
//   +0x58 int last_structure_optim_  (=0; upstream member order) TODO(verify)
//   +0x60 std::set<int> ba_bundle_ids_  (c03: bundle ids of the BA observations)
//   +0x70 int ba_inlier_count_       (c03)
//   +0x74 int ba_total_count_        (c03)
//   +0x78 int n_consecutive_obs_     (c09; c03 "ba_obs_frames_", c10 "n_consecutive_tracked_",
//                                     c01 "Point+120 < 2" gate)
//   +0x7C int last_obs_bundle_id_    (=-1; c09)
//   +0x80 bool in_ba_graph_          (=false)
// Upstream members normal_, normal_information_, normal_set_, last_published_ts_,
// last_projected_kf_id_, type_, last_ba_update_ do not exist.
#pragma once

#include <atomic>
#include <cmath>
#include <cstddef>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "common/types.h"

namespace pimax {
namespace totem {

/// Thread-safe point-ID provider.  last_id_ is the global at 0x18047DB60.
class PointIdProvider
{
public:
  PointIdProvider() = delete;
  static int getNewPointId() { return last_id_.fetch_add(1); }
private:
  static std::atomic<int> last_id_;
};

/// 32 bytes (node of Point::obs_ is 0x38 = 16 list links + int key (+16) + pad + this (+24)).
struct KeypointIdentifier
{
  FrameWeakPtr frame;        // +0
  int frame_id;              // +16  frame->id_          (Frame +16)
  int bundle_id;             // +20  [pimax] frame->bundle_id_ (Frame +32) TODO(verify name)
  size_t keypoint_index_;    // +24

  /// 0x180099F40 (point.cpp); also inlined into Point::addObservation (0x18009A530).
  KeypointIdentifier(const FramePtr& _frame, const size_t _feature_index);
};
// Pimax: obs_ is an unordered_map keyed by frame id instead of a vector (FNV-1a hash of
// the int key, load factor 1.0, 8 initial buckets -> std::unordered_map<int, ...> default).
using KeypointIdentifierList = std::unordered_map<int, KeypointIdentifier>;
using Matrix23d = Eigen::Matrix<double, 2, 3>;

/// A 3D point on the surface of the scene.
class Point
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum PointType {
    TYPE_EDGELET_SEED,
    TYPE_CORNER_SEED,
    TYPE_EDGELET,
    TYPE_CORNER
  };

  int           id_;                        //!< +0x00 Unique ID of the point.
  Position      pos_;                       //!< +0x04 3d pos of the point in the world frame (float).
  KeypointIdentifierList obs_;              //!< +0x10 observations, keyed by frame id
  int           n_failed_reproj_ = 0;       //!< +0x50
  int           n_succeeded_reproj_ = 0;    //!< +0x54
  int           last_structure_optim_ = 0;  //!< +0x58 TODO(verify) (only written by the ctor)
  std::set<int> ba_bundle_ids_;             //!< +0x60 [pimax] bundle ids seen by the backend
  int           ba_inlier_count_ = 0;       //!< +0x70 [pimax]
  int           ba_total_count_ = 0;        //!< +0x74 [pimax]
  int           n_consecutive_obs_ = 0;     //!< +0x78 [pimax] TODO(verify) name
  int           last_obs_bundle_id_ = -1;   //!< +0x7C [pimax] TODO(verify) name
  bool          in_ba_graph_ = false;       //!< +0x80 Was this point already added to the BA graph?

  /// 0x180099F80  upstream-modified (Pimax layout, float position):
  /// Point(pos) : Point(PointIdProvider::getNewPointId(), pos), delegating ctor inlined.
  Point(const Position& pos);

  Point(const int id, const Position& pos);

  /// (member-wise; set/unordered_map dtors 0x18009A160 / 0x18009A070)
  ~Point() = default;

  // no copy
  Point(const Point&) = delete;
  Point& operator=(const Point&) = delete;

  void addObservation(const FramePtr& frame, const size_t feature_index);   // 0x18009A530
  void removeObservation(int frame_id);                                     // 0x18009BBE0
  bool getCloseViewObs(const Eigen::Vector3d& framepos, FramePtr& ref_frame,
                       size_t& ref_feature_index) const;                    // 0x18009A780
  double getTriangulationParallax() const;                                  // 0x18009AA50

  void updateHessianGradientUnitPlane(                                      // 0x18009BE10
      const Eigen::Ref<BearingVector>& f,
      const Eigen::Vector3d& p_in_f,
      const Eigen::Matrix3d& R_f_w,
      Eigen::Matrix3d& A,
      Eigen::Vector3d& b,
      double& new_chi2);

  void updateHessianGradientUnitSphere(                                     // 0x18009C130
      const Eigen::Ref<BearingVector>& f,
      const Eigen::Vector3d& p_in_f,
      const Eigen::Matrix3d& R_f_w,
      Eigen::Matrix3d& A,
      Eigen::Vector3d& b,
      double& new_chi2);

  void optimize(const size_t n_iter, bool using_bearing_vector = false);   // 0x18009B240

  inline size_t nRefs() const { return obs_.size(); }
  inline int id() const { return id_; }
  inline const Position& pos() const { return pos_; }

  /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
  /// Emitted out of line as COMDAT 0x18009B040 (upstream-identical).
  inline static void jacobian_xyz2uv(
      const Eigen::Vector3d& p_in_f,
      const Eigen::Matrix3d& R_f_w,
      Matrix23d& point_jac)
  {
    const double z_inv = 1.0/p_in_f[2];
    const double z_inv_sq = z_inv*z_inv;
    point_jac(0, 0) = z_inv;
    point_jac(0, 1) = 0.0;
    point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
    point_jac(1, 0) = 0.0;
    point_jac(1, 1) = z_inv;
    point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
    point_jac = - point_jac * R_f_w;
  }

  /// Jacobian of point to unit bearing vector.
  /// Emitted out of line as COMDAT 0x18009ADE0 (upstream-identical).
  inline static void jacobian_xyz2f(
      const Eigen::Vector3d& p_in_f,
      const Eigen::Matrix3d& R_f_w,
      Eigen::Matrix3d& point_jac)
  {
    Eigen::Matrix3d J_normalize;
    double x2 = p_in_f[0]*p_in_f[0];
    double y2 = p_in_f[1]*p_in_f[1];
    double z2 = p_in_f[2]*p_in_f[2];
    double xy = p_in_f[0]*p_in_f[1];
    double yz = p_in_f[1]*p_in_f[2];
    double zx = p_in_f[2]*p_in_f[0];
    J_normalize << y2+z2, -xy, -zx,
        -xy, x2+z2, -yz,
        -zx, -yz, x2+y2;
    J_normalize *= 1.0 / std::pow(x2+y2+z2, 1.5);
    point_jac = (-1.0) * J_normalize * R_f_w;
  }

  static void layout_check();
};

// Pimax: shared_ptr everywhere (as upstream).
using PointPtr = std::shared_ptr<Point>;

inline void Point::layout_check()
{
  static_assert(sizeof(Point) == 0x88, "sizeof(Point)");
  static_assert(offsetof(Point, pos_) == 0x04, "Point::pos_");
  static_assert(offsetof(Point, obs_) == 0x10, "Point::obs_");
  static_assert(offsetof(Point, n_failed_reproj_) == 0x50, "Point::n_failed_reproj_");
  static_assert(offsetof(Point, n_succeeded_reproj_) == 0x54, "Point::n_succeeded_reproj_");
  static_assert(offsetof(Point, last_structure_optim_) == 0x58, "Point::last_structure_optim_");
  static_assert(offsetof(Point, ba_bundle_ids_) == 0x60, "Point::ba_bundle_ids_");
  static_assert(offsetof(Point, ba_inlier_count_) == 0x70, "Point::ba_inlier_count_");
  static_assert(offsetof(Point, ba_total_count_) == 0x74, "Point::ba_total_count_");
  static_assert(offsetof(Point, n_consecutive_obs_) == 0x78, "Point::n_consecutive_obs_");
  static_assert(offsetof(Point, last_obs_bundle_id_) == 0x7C, "Point::last_obs_bundle_id_");
  static_assert(offsetof(Point, in_ba_graph_) == 0x80, "Point::in_ba_graph_");
  static_assert(sizeof(KeypointIdentifier) == 32, "sizeof(KeypointIdentifier)");
  static_assert(offsetof(KeypointIdentifier, frame_id) == 16, "KeypointIdentifier::frame_id");
  static_assert(offsetof(KeypointIdentifier, bundle_id) == 20, "KeypointIdentifier::bundle_id");
  static_assert(offsetof(KeypointIdentifier, keypoint_index_) == 24, "KeypointIdentifier::keypoint_index_");
}

} // namespace totem
} // namespace pimax
