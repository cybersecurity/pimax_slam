// pimax_slam.pi.dll -- src/ceres_backend/estimator_types.hpp
//
// Pimax fork of svo_ceres_backend/include/svo/ceres_backend/estimator_types.hpp
// (namespace svo -> pimax::totem).  File name proven by the binary: __FILE__ of the two
// LOG(ERROR)s in MapPoint::getTriangulationParallax is
// "E:\code_codex\pimax_slam\beta111_5a7902_dll\src\ceres_backend/estimator_types.hpp" (lines
// 187 / 203), i.e. the header is included as "ceres_backend/estimator_types.hpp".
//
// Differences from upstream (notes c01, c02, c03, c04):
//   * ImuParameters: 128 bytes; defaults a_max 150, g_max 35, g 9.80667 (sic), rate 1000; the
//     sigma_* have NO default initializer; a Pimax Vector3d at +0x60; no delay_imu_cam.
//   * createNFrameId(): no CHECK_GE(bundle_id, 0) (every inlined use is a bare movsxd + shl 16).
//   * MapPoint (128 bytes): observations is an unordered_map<residual id, KeypointIdentifier>
//     (upstream: std::map<KeypointIdentifier, uint64_t>); new `parallax` double (10.0 in the
//     ctor); new inline getTriangulationParallax (0x18002A160).  Point::pos_ is float.
//   * isFinite(Vector3d) helper 0x180027A20 (out-of-line inline, estimator.obj).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <ostream>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "common/transformation.h"
#include "common/types.h"
#include "common/frame.h"   // Frame::pos() (MapPoint::getTriangulationParallax)
#include "common/point.h"   // Point, KeypointIdentifier

namespace pimax {
namespace totem {

//------------------------------------------------------------------------------
/// \brief Struct to define the behavior of the camera extrinsics.  Upstream-identical; only the
/// (never filled) vector Estimator::extrinsics_estimation_parameters_ (+0x1A0) remains.
struct ExtrinsicsEstimationParameters
{
  // set to 0 in order to turn off
  /// \brief Default Constructor -- fixed camera extrinsics.
  ExtrinsicsEstimationParameters()
      : sigma_absolute_translation(0.0),
        sigma_absolute_orientation(0.0),
        sigma_c_relative_translation(0.0),
        sigma_c_relative_orientation(0.0)
  {
  }

  ExtrinsicsEstimationParameters(double sigma_absolute_translation,
                                 double sigma_absolute_orientation,
                                 double sigma_c_relative_translation,
                                 double sigma_c_relative_orientation)
      : sigma_absolute_translation(sigma_absolute_translation),
        sigma_absolute_orientation(sigma_absolute_orientation),
        sigma_c_relative_translation(sigma_c_relative_translation),
        sigma_c_relative_orientation(sigma_c_relative_orientation)
  {
  }

  inline bool isExtrinsicsFixed() const
  {
    return absoluteTranslationVar() < 1.0e-16 &&
        absoluteRotationVar() < 1.0e-16;
  }

  inline double absoluteTranslationVar() const
  {
    return sigma_absolute_translation * sigma_absolute_translation;
  }

  inline double absoluteRotationVar() const
  {
    return sigma_absolute_orientation * sigma_absolute_orientation;
  }

  // absolute (prior) w.r.t frame S
  double sigma_absolute_translation; ///< Absolute translation stdev. [m]
  double sigma_absolute_orientation; ///< Absolute orientation stdev. [rad]

  // relative (temporal)
  double sigma_c_relative_translation;
  ///< Relative translation noise density. [m/sqrt(Hz)]
  double sigma_c_relative_orientation;
  ///< Relative orientation noise density. [rad/sqrt(Hz)]
};

typedef std::vector<ExtrinsicsEstimationParameters,
                    Eigen::aligned_allocator<ExtrinsicsEstimationParameters> >
        ExtrinsicsEstimationParametersVec;

// -----------------------------------------------------------------------------
/// IMU parameters (Pimax, 128 bytes; Estimator+0x1B8, ImuError+56; copied verbatim by
/// Estimator::addImu 0x180025BE0).  Defaults = the stores of the Estimator ctor 0x180022EF0 and
/// of the ImuError ctor 0x1800398A0.
struct ImuParameters
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double a_max = 150;        ///< +0x00 Accelerometer saturation. [m/s^2] (0x4062C00000000000)
  double g_max = 35;         ///< +0x08 Gyroscope saturation. [rad/s] (upstream 7.8)
  double sigma_g_c;          ///< +0x10 Gyroscope noise density (no default in Pimax)
  double sigma_bg;           ///< +0x18 Initial gyroscope bias uncertainty (used by addStates)
  double sigma_a_c;          ///< +0x20 Accelerometer noise density
  double sigma_ba;           ///< +0x28 Initial accelerometer bias uncertainty (used by addStates)
  double sigma_gw_c;         ///< +0x30 Gyroscope drift noise density
  double sigma_aw_c;         ///< +0x38 Accelerometer drift noise density
  double g = 9.80667;        ///< +0x40 Earth acceleration (0x40239D03D9A95422, sic)
  Eigen::Vector3d a0 = Eigen::Vector3d::Zero();          ///< +0x48 Mean of the prior acc bias
  Eigen::Vector3d unknown_60 = Eigen::Vector3d::Zero();  ///< +0x60 [pimax-new] TODO(verify) name
  double rate = 1000;        ///< +0x78 IMU rate [Hz] (upstream 200; delay_imu_cam removed)
};
static_assert(sizeof(ImuParameters) == 0x80, "ImuParameters size");
static_assert(offsetof(ImuParameters, g) == 0x40, "");
static_assert(offsetof(ImuParameters, a0) == 0x48, "");
static_assert(offsetof(ImuParameters, unknown_60) == 0x60, "");
static_assert(offsetof(ImuParameters, rate) == 0x78, "");

// -----------------------------------------------------------------------------
// IDs
enum class IdType : uint8_t
{
  NFrame = 0,
  Landmark = 1,
  ImuStates = 2,
  Extrinsics = 3
};

//! The Backend ID for multiple types (upstream layout, see svo estimator_types.hpp).
class BackendId
{
 public:
  BackendId() = default;
  explicit BackendId(uint64_t id) : id_(id) {}

  uint64_t asInteger() const
  {
    return id_;
  }

  IdType type() const
  {
    // The first byte represents the type.
    return static_cast<IdType>(id_ >> 56);
  }

  int32_t bundleId() const
  {
    // DEBUG_CHECK(type() != IdType::Landmark) compiled out
    // The bundle ID is byte 2 -> 6 in id.
    return static_cast<int32_t>((id_ >> 16) & 0xFFFFFFFF);
  }

  uint32_t trackId() const
  {
    // In case of a landmark, the last 4 bytes are the track ID.
    return static_cast<uint32_t>(id_ & 0xFFFFFFFF);
  }

  uint16_t nFrameHandle() const
  {
    // In case of an NFrame, the last 2 bytes are the handle.
    return static_cast<uint16_t>(id_ & 0xFFFF);
  }

  uint8_t cameraIndex() const
  {
    // The second byte is the camara index.
    return static_cast<uint8_t>((id_ >> 48) & 0x00000FF);
  }

  bool valid() const
  {
    return id_ != 0;
  }

 private:
  uint64_t id_{0};
};
static_assert(sizeof(BackendId) == 8, "");

// Factories
inline BackendId createLandmarkId(int track_id)
{
  return BackendId(static_cast<uint64_t>(track_id) |
                   (static_cast<uint64_t>(IdType::Landmark) << 56));
}

/// Pimax: the upstream CHECK_GE(bundle_id, 0) is gone.
inline BackendId createNFrameId(int32_t bundle_id)
{
  return BackendId((static_cast<uint64_t>(bundle_id) << 16) |
                   (static_cast<uint64_t>(IdType::NFrame) << 56));
}

inline BackendId createExtrinsicsId(uint8_t camera_index,
                                    int32_t bundle_id)
{
  return BackendId((static_cast<uint64_t>(
                      static_cast<uint32_t>(bundle_id)) << 16) |
                   (static_cast<uint64_t>(camera_index) << 48) |
                   (static_cast<uint64_t>(IdType::Extrinsics) << 56));
}

inline BackendId createImuStateId(int32_t bundle_id)
{
  return BackendId((static_cast<uint64_t>(
                      static_cast<uint32_t>(bundle_id)) << 16) |
                   (static_cast<uint64_t>(IdType::ImuStates) << 56));
}

inline BackendId changeIdType(BackendId id, IdType type, size_t cam_index = 0)
{
  // DEBUG_CHECKs compiled out.  Last 6 bytes remain the same.
  return BackendId((id.asInteger() & 0xFFFFFFFFFFFF) |
                   (static_cast<uint64_t>(cam_index) << 48) |
                   (static_cast<uint64_t>(type) << 56));
}

// Comparison operator for use in maps.
inline bool operator<(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() < rhs.asInteger();
}

inline bool operator==(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() == rhs.asInteger();
}

inline bool operator!=(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() != rhs.asInteger();
}

inline bool operator>=(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() >= rhs.asInteger();
}

/// std::hex @0x18001CEA0 / std::dec @0x18001CBD0 inline manipulator instances.
inline std::ostream& operator<<(std::ostream& out, const BackendId& id)
{
  out << std::hex << id.asInteger() << std::dec;
  return out;
}

//------------------------------------------------------------------------------
/// [pimax-new] 0x180027A20 (out-of-line inline; used by GroundPlaneError 0x18002D110,
/// Estimator::setGroundPlaneConstraint 0x18002C530 and FrameProcessorBase::estimateGroundPlane
/// 0x1800F5170).  MSVC std::isfinite == (_dclass(x) <= 0).  TODO(verify) name / header.
inline bool isFinite(const Eigen::Vector3d& v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

//------------------------------------------------------------------------------
/**
 * @brief A type to store information about a point in the world map.
 * Pimax: 128 bytes (PointMap node 0xB0 (malloc'ed: aligned_allocator), MapPoint at node+0x30).
 * Ctor inlined into Estimator::addLandmark 0x180025C80; move ctor 0x1800234D0 and dtor
 * 0x180024430 compiler generated.
 */
struct MapPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor. Point is nullptr.
  MapPoint()
      : point(nullptr), fixed_position(false)
  {}

  /// \brief Constructor.  Pimax: Point::pos_ is a float vector.
  MapPoint(const PointPtr& point)
    : point(point), fixed_position(false)
  {
    hom_coordinates << point->pos().cast<double>(), 1;
  }

  /// [pimax-new] 0x18002A160 (out-of-line inline).  Largest angle between the viewing ray of the
  /// first observation (reference frame) and the rays of all other observations; float math.
  double getTriangulationParallax() const;

  Eigen::Vector4d hom_coordinates; ///< +0x00 Continuosly updates position of point
  //! +0x20 Pointer to the point. The position is not updated inside backend
  //! because of possible multithreading conflicts.
  PointPtr point;
  //! +0x30 Observations of this point.  Pimax: key = casted ceres::ResidualBlockId of the
  //! reprojection error residual block; value = the observing keypoint (FNV-1a hash of the
  //! 8-byte key; node: key +0x10, KeypointIdentifier +0x18).
  std::unordered_map<uint64_t, KeypointIdentifier> observations;
  //! +0x70 Pimax uses it as "set constant for this optimize() call".
  bool fixed_position;
  //! +0x78 [pimax-new] last triangulation parallax [rad]; written by Estimator::optimize.
  //! TODO(verify) name.
  double parallax = 10.0;
};
static_assert(sizeof(MapPoint) == 0x80, "MapPoint size");
static_assert(offsetof(MapPoint, point) == 0x20, "");
static_assert(offsetof(MapPoint, observations) == 0x30, "");
static_assert(offsetof(MapPoint, fixed_position) == 0x70, "");
static_assert(offsetof(MapPoint, parallax) == 0x78, "");

typedef std::vector<MapPoint, Eigen::aligned_allocator<MapPoint> > MapPointVector;
typedef std::map<BackendId, MapPoint, std::less<BackendId>,
  Eigen::aligned_allocator<std::pair<const BackendId, MapPoint>> > PointMap;

// 0x18002A160 -- pimax-new.  The reference frame's pos() is inlined; the others call the
// out-of-line Frame::pos() copy 0x18002B960 (Vector3f, T_f_w_.inverse().getPosition()).
// LOG(ERROR) lines 187 / 203 of this header are reproduced with #line.
inline double MapPoint::getTriangulationParallax() const
{
  if (observations.size() < 2)
  {
    return 0.0;
  }

  FramePtr ref_frame = observations.begin()->second.frame.lock();
  if (!ref_frame)
  {
#line 187
    LOG(ERROR) << "MapPoint::getTriangualtionParallax(): Could not lock ref_frame";
    return 0.0;
  }

  const Eigen::Vector3f pos = hom_coordinates.head<3>().cast<float>();
  const Eigen::Vector3f ref_dir = (ref_frame->pos() - pos).normalized();

  double max_parallax = 0.0;
  for (auto it = std::next(observations.begin()); it != observations.end(); ++it)
  {
    FramePtr frame = it->second.frame.lock();
    if (!frame)
    {
#line 203
      LOG(ERROR) << "MapPoint::getTriangualtionParallax(): Could not lock frame";
      continue;
    }
    const Eigen::Vector3f dir = (frame->pos() - pos).normalized();
    const double parallax = std::acos(std::fabs(ref_dir.dot(dir)));   // acosf(float), widened
    max_parallax = std::max(parallax, max_parallax);   // asm keeps `parallax` unless it is smaller
  }
  return max_parallax;
}

//------------------------------------------------------------------------------
// [velocity, gyro biases, accel biases]
typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;

}  // namespace totem
}  // namespace pimax
