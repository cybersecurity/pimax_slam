// pimax_slam.pi.dll -- src/common/types.h
//
// Pimax fork of svo_common/include/svo/common/types.h, namespace pimax::totem.
//
// Differences to upstream (all chunks agree, c05/c06/c11/c12/c13 notes):
//  * FloatType is FLOAT: keypoints, bearing vectors, gradients, scores, seed states and the
//    landmark position (Point::pos_) are stored in single precision (4-byte strides,
//    cvtps2pd/cvtpd2ps everywhere).  Transformations stay double.
//  * FeatureType keeps the upstream values (proved by the compiled predicates: isEdgelet ==
//    `bt 0x49` = {0,3,6}, isCornerEdgeletSeed == `t != 2 && t < 5`, isConvergedCornerEdgeletSeed
//    == {3,4}, isUnconvergedCornerEdgeletSeed == `t <= 1`, resize default kCorner == 7,
//    Mesher/ground-plane `type == 7` for corners).  FrameBundle::numTrackedEdgelets
//    (0x1800952D0) uses a different predicate, `(t & 0xF9) == 0 && t != 2` == {0,4,6}
//    (isTrackedEdgeletType below).
//
// For convenience (the drafts of every chunk rely on it) this header also provides
// Transformation/Quaternion (common/transformation.h) and the camera forward declarations
// (common/camera_fwd.h).
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "common/camera_fwd.h"
#include "common/transformation.h"

namespace pimax {
namespace totem {

//------------------------------------------------------------------------------
// Scalars and fp precision.
using size_t = std::size_t;
using uint8_t = std::uint8_t;
using uint64_t = std::uint64_t;
using FloatType = float;

//------------------------------------------------------------------------------
// Feature containers.
using Keypoint = Eigen::Matrix<FloatType, 2, 1>;
using BearingVector = Eigen::Matrix<FloatType, 3, 1>;
using Position = Eigen::Matrix<FloatType, 3, 1>;
using GradientVector = Eigen::Matrix<FloatType, 2, 1>;
using SeedState = Eigen::Matrix<FloatType, 4, 1>;
using Level = int;
using Score = FloatType;
using Keypoints = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;
using Bearings = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Positions = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Gradients = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;
using Scores = Eigen::Matrix<FloatType, Eigen::Dynamic, 1, Eigen::ColMajor>;
using Levels = Eigen::Matrix<Level, Eigen::Dynamic, 1, Eigen::ColMajor>;
using InlierMask = Eigen::Matrix<bool, Eigen::Dynamic, 1, Eigen::ColMajor>;
using SeedStates = Eigen::Matrix<FloatType, 4, Eigen::Dynamic, Eigen::ColMajor>;
using TrackIds = Eigen::VectorXi;

//------------------------------------------------------------------------------
// Forward declarations and common types for simplicity.
struct Feature;
using FeaturePtr = std::shared_ptr<Feature>;
class Frame;
using FramePtr = std::shared_ptr<Frame>;
using FrameWeakPtr = std::weak_ptr<Frame>;
class FrameBundle;
using FrameBundlePtr = std::shared_ptr<FrameBundle>;
class Point;
using PointPtr = std::shared_ptr<Point>;
using BundleId = int;
using ImgPyr = std::vector<cv::Mat>;

//------------------------------------------------------------------------------
// Point matches
struct MatchedPointsInfo
{
  int lc_kf_id_ = -1;
  int cur_kf_id_ = -1;
  std::map<int, int> pt_id_matches_ {};
};

//------------------------------------------------------------------------------
// Feature Type (upstream values, see header comment).
enum class FeatureType : uint8_t
{
  kEdgeletSeed = 0,
  kCornerSeed = 1,
  kMapPointSeed = 2,
  kEdgeletSeedConverged = 3,
  kCornerSeedConverged = 4,
  kMapPointSeedConverged = 5,
  kEdgelet = 6,
  kCorner = 7,
  kMapPoint = 8,
  kFixedLandmark = 9,
  kOutlier = 10
};

using FeatureTypes = std::vector<FeatureType>;

inline bool isSeed(const FeatureType& t)
{
  return static_cast<uint8_t>(t) < 6;
}

inline bool isCornerEdgeletSeed(const FeatureType& t)
{
  return (t == FeatureType::kEdgeletSeedConverged
          || t == FeatureType::kCornerSeedConverged
          || t == FeatureType::kEdgeletSeed
          || t == FeatureType::kCornerSeed);
}

inline bool isConvergedCornerEdgeletSeed(const FeatureType& t)
{
  return (t == FeatureType::kEdgeletSeedConverged
          || t == FeatureType::kCornerSeedConverged);
}

inline bool isConvergedMapPointSeed(const FeatureType& t)
{
  return (t == FeatureType::kMapPointSeedConverged);
}

inline bool isUnconvergedCornerEdgeletSeed(const FeatureType& t)
{
  return (t == FeatureType::kEdgeletSeed
          || t == FeatureType::kCornerSeed);
}

inline bool isUnconvergedMapPointSeed(const FeatureType& t)
{
  return (t == FeatureType::kMapPointSeed);
}

inline bool isEdgelet(const FeatureType& t)
{
  return (t == FeatureType::kEdgelet
          || t == FeatureType::kEdgeletSeed
          || t == FeatureType::kEdgeletSeedConverged);
}

inline bool isCorner(const FeatureType& t)
{
  return (t == FeatureType::kCorner
          || t == FeatureType::kCornerSeed
          || t == FeatureType::kCornerSeedConverged);
}

inline bool isMapPoint(const FeatureType& t)
{
  return (t == FeatureType::kMapPoint
          || t == FeatureType::kMapPointSeed
          || t == FeatureType::kMapPointSeedConverged);
}

inline bool isMapPointSeed(const FeatureType& t)
{
  return (t == FeatureType::kMapPointSeed
          || t == FeatureType::kMapPointSeedConverged);
}

inline bool isUnconvergedSeed(const FeatureType& t)
{
  return (t == FeatureType::kCornerSeed
          || t == FeatureType::kMapPointSeed
          || t == FeatureType::kEdgeletSeed);
}

inline bool isFixedLandmark(const FeatureType& t)
{
  return t == FeatureType::kFixedLandmark;
}

/// [pimax] predicate of FrameBundle::numTrackedEdgelets (0x1800952D0), compiled as
/// `(t & 0xF9) == 0 && t != 2`, i.e. t in {kEdgeletSeed, kCornerSeedConverged, kEdgelet}.
/// Looks like a hand-written isEdgelet() with kCornerSeedConverged instead of
/// kEdgeletSeedConverged.  TODO(verify) the source spelling.
inline bool isTrackedEdgeletType(const FeatureType& t)
{
  return (t == FeatureType::kEdgeletSeed
          || t == FeatureType::kCornerSeedConverged
          || t == FeatureType::kEdgelet);
}

inline std::string str(const FeatureType& type)
{
  if (type == FeatureType::kEdgeletSeed)
  {
    return std::string("EdgeletSeed");
  }
  else if (type == FeatureType::kCornerSeed)
  {
    return std::string("CornerSeed");
  }
  else if (type == FeatureType::kMapPointSeed)
  {
    return std::string("MapPointSeed");
  }
  else if (type == FeatureType::kEdgeletSeedConverged)
  {
    return std::string("EdgeletSeedConverged");
  }
  else if (type == FeatureType::kCornerSeedConverged)
  {
    return std::string("CornerSeedConverged");
  }
  else if (type == FeatureType::kMapPointSeedConverged)
  {
    return std::string("MapPointSeedConverged");
  }
  else if (type == FeatureType::kEdgelet)
  {
    return std::string("Edgelet");
  }
  else if (type == FeatureType::kCorner)
  {
    return std::string("Corner");
  }
  else if (type == FeatureType::kMapPoint)
  {
    return std::string("MapPoint");
  }
  else if (type == FeatureType::kFixedLandmark)
  {
    return std::string("FixedLandmark");
  }
  else if (type == FeatureType::kOutlier)
  {
    return std::string("Outlier");
  }
  else
  {
    return std::string("Unknown");
  }
}

struct EnumClassHash
{
  template <typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

} // namespace totem
} // namespace pimax
