// pimax_slam.pi.dll -- src/common/feature_wrapper.h
//
// Pimax fork of svo_common/include/svo/common/feature_wrapper.h.
//
// SeedRef (24 bytes): FramePtr keyframe +0, int seed_id +16 (=-1).  The 4 bytes at +20 are
// padding: they read as 0 after std::vector::resize() because value-initialisation of the
// defaulted ctor zero-fills the object first (c05 took them for a member); the 2-argument ctor
// (out-of-line copy 0x180181040) does not touch them.
//
// FeatureWrapper (144 bytes, built by Frame::getFeatureWrapper 0x180094580):
//   +0x00 FeatureType& type
//   +0x08 Eigen::Ref<Keypoint> px            (24 bytes per Ref)
//   +0x20 Eigen::Ref<BearingVector> f
//   +0x38 Eigen::Ref<BearingVector> f_raw    [pimax] column of Frame::f_vec_raw_ (12-byte stride,
//                                            c05/c13; c12 took it for a Vector2f "uv")
//   +0x50 Eigen::Ref<GradientVector> grad
//   +0x68 Score& score
//   +0x70 Level& level
//   +0x78 PointPtr& landmark
//   +0x80 SeedRef& seed_ref
//   +0x88 int& track_id
// Constructor arguments are evaluated right-to-left by MSVC (track_id first, type last), which
// is the order seen in the binary.
#pragma once

#include <cstddef>
#include <memory>

#include <Eigen/Core>

#include "common/types.h"

namespace pimax {
namespace totem {

struct SeedRef
{
  FramePtr keyframe;     // +0
  int seed_id = -1;      // +16
  // 0x180181040 (out-of-line copy, loop closing)
  SeedRef(const FramePtr& _keyframe, const int _seed_id)
    : keyframe(_keyframe)
    , seed_id(_seed_id)
  { ; }
  SeedRef() = default;
  ~SeedRef() = default;
};

struct FeatureWrapper
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureType& type;                  //!< Type can be corner or edgelet.
  Eigen::Ref<Keypoint> px;            //!< Coordinates in pixels on pyramid level 0.
  Eigen::Ref<BearingVector> f;        //!< Unit-bearing vector of the feature.
  Eigen::Ref<BearingVector> f_raw;    //!< [pimax] un-normalised back-projection. TODO(verify) name
  Eigen::Ref<GradientVector> grad;    //!< Dominant gradient direction for edglets, normalized.
  Score& score;
  Level& level;                       //!< Image pyramid level where feature was extracted.
  PointPtr& landmark;
  SeedRef& seed_ref;
  int& track_id;

  FeatureWrapper(
      FeatureType& _type,
      Eigen::Ref<Keypoint> _px,
      Eigen::Ref<BearingVector> _f,
      Eigen::Ref<BearingVector> _f_raw,
      Eigen::Ref<GradientVector> _grad,
      Score& _score,
      Level& _pyramid_level,
      PointPtr& _landmark,
      SeedRef& _seed_ref,
      int& _track_id)
    : type(_type)
    , px(_px)
    , f(_f)
    , f_raw(_f_raw)
    , grad(_grad)
    , score(_score)
    , level(_pyramid_level)
    , landmark(_landmark)
    , seed_ref(_seed_ref)
    , track_id(_track_id)
  { ; }

  FeatureWrapper() = delete;
  ~FeatureWrapper() = default;

  FeatureWrapper(const FeatureWrapper& other) = default;
  FeatureWrapper& operator=(const FeatureWrapper& other) = default;
};

static_assert(sizeof(SeedRef) == 24, "sizeof(SeedRef)");
static_assert(offsetof(SeedRef, seed_id) == 16, "SeedRef::seed_id");
static_assert(sizeof(FeatureWrapper) == 144, "sizeof(FeatureWrapper)");
static_assert(offsetof(FeatureWrapper, px) == 0x08, "FeatureWrapper::px");
static_assert(offsetof(FeatureWrapper, f) == 0x20, "FeatureWrapper::f");
static_assert(offsetof(FeatureWrapper, f_raw) == 0x38, "FeatureWrapper::f_raw");
static_assert(offsetof(FeatureWrapper, grad) == 0x50, "FeatureWrapper::grad");
static_assert(offsetof(FeatureWrapper, score) == 0x68, "FeatureWrapper::score");
static_assert(offsetof(FeatureWrapper, level) == 0x70, "FeatureWrapper::level");
static_assert(offsetof(FeatureWrapper, landmark) == 0x78, "FeatureWrapper::landmark");
static_assert(offsetof(FeatureWrapper, seed_ref) == 0x80, "FeatureWrapper::seed_ref");
static_assert(offsetof(FeatureWrapper, track_id) == 0x88, "FeatureWrapper::track_id");

}  // namespace totem
}  // namespace pimax
