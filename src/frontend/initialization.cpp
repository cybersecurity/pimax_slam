// pimax_slam.pi.dll -- src/frontend/initialization.cpp  (draft c11, phase B3)
//
// pimax::totem::AbstractInitialization / StereoInit -- fork of svo/src/initialization.cpp.
// Original file: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\frontend\initialization.cpp
// Object TU32, code 0x18012B320..0x18012C0B0.
// glog line numbers seen in the binary: VLOG(20) 113, LOG(FATAL) 270 (forced with #line).
// Everything except AbstractInitialization + StereoInit + makeInitializer was removed
// (no Homography/TwoPoint/FivePoint/OneShot/Array initialisers, no triangulation utils in the image).
#include "frontend/initialization.h"

#include <iostream>

#include <glog/logging.h>
#include <vikit/cameras/ncamera.h>

#include "common/frame.h"
#include "common/logger.h"
#include "direct/feature_detection.h"
#include "direct/feature_detection_utils.h"
#include "frontend/stereo_triangulation.h"
#include "tracker/feature_tracker.h"

namespace pimax {
namespace totem {

// ============================================================================================
// 0x18012B320
// upstream-identical (FeatureTracker allocated through Eigen aligned new: malloc(0x98)).
AbstractInitialization::AbstractInitialization(const InitializationOptions& init_options,
                                               const FeatureTrackerOptions& tracker_options,
                                               const DetectorOptions& detector_options,
                                               const CameraBundlePtr& cams)
    : options_(init_options)
{
  tracker_.reset(new FeatureTracker(tracker_options, detector_options, cams));
}

// ============================================================================================
// 0x18012B860 (complete-object dtor; 0x18012B8E0 = scalar deleting dtor, Eigen aligned delete = free)
// upstream-identical. Member dtors: frames_ref_ (+88) then tracker_ (+80; FeatureTracker deleting
// dtor 0x18012B930 is compiler-generated).
AbstractInitialization::~AbstractInitialization()
{}

// ============================================================================================
// 0x18012C030
// upstream-modified: have_depth_prior_ no longer exists.
void AbstractInitialization::reset()
{
  frames_ref_.reset();
  have_rotation_prior_ = false;
  have_translation_prior_ = false;
  tracker_->reset();   // 0x1801A8030
}

// ============================================================================================
// 0x18012B4E0
// upstream-identical. StereoTriangulationOptions defaults {120, 1/3, 1, 1/50} (0x1803B71B0).
// (scalar deleting dtor 0x18012BAB0 is compiler-generated: ~detector_, ~stereo_ (0x1800B2880),
// base dtor.)
StereoInit::StereoInit(const InitializationOptions& init_options,
                       const FeatureTrackerOptions& tracker_options,
                       const DetectorOptions& detector_options, const CameraBundlePtr& cams)
    : AbstractInitialization(init_options, tracker_options, detector_options, cams)
{
  StereoTriangulationOptions stereo_options;
  detector_ = feature_detection_utils::makeDetector(detector_options, cams->getCameraShared(0));
  stereo_.reset(new StereoTriangulation(stereo_options, detector_));   // operator new(0x90)
}

// ============================================================================================
// 0x18012BB60
// upstream-modified:
//  * LOGI "StereoInit: Add frame bundle." at entry;
//  * CHECK_EQ(frames->size(), 2u) removed;
//  * triangulation only for pairs whose frames both have mean image intensity > 15.0
//    (Frame+224), pairs (0,1) and, for a 4-camera bundle, (0,2), (1,3), (2,3);
//    StereoTriangulation::compute got a 3rd argument (always true here);
//  * the VLOG prints at(0)->T_world_cam() and at(1)->T_cam_world() (upstream: both T_world_cam);
//  * success test: at(0)->numLandmarks() > min OR at(2)->numLandmarks() > min (strictly greater;
//    upstream failed on '<'). QUIRK: at(2) is evaluated (vector::at -> throws std::out_of_range)
//    whenever camera 0 fails, even for a 2-frame bundle;
//  * LOGI messages for success / failure (size_t passed to %d).
InitResult StereoInit::addFrameBundle(const FrameBundlePtr& frames)
{
  LOGI("StereoInit: Add frame bundle.\n");
  reset();   // virtual call (vtbl[2])
  frames_ref_ = frames;

#line 113
  VLOG(20) << "FRAME 1" << std::endl << frames->at(0)->T_world_cam() << std::endl   // initialization.cpp:113
           << "FRAME 2" << std::endl << frames->at(1)->T_cam_world();

  if (frames->at(0)->mean_intensity_ > 15.0 && frames->at(1)->mean_intensity_ > 15.0)
    stereo_->compute(frames->at(0), frames->at(1), true);

  if (frames->size() == 4)
  {
    if (frames->at(0)->mean_intensity_ > 15.0 && frames->at(2)->mean_intensity_ > 15.0)
      stereo_->compute(frames->at(0), frames->at(2), true);
    if (frames->at(1)->mean_intensity_ > 15.0 && frames->at(3)->mean_intensity_ > 15.0)
      stereo_->compute(frames->at(1), frames->at(3), true);
    if (frames->at(2)->mean_intensity_ > 15.0 && frames->at(3)->mean_intensity_ > 15.0)
      stereo_->compute(frames->at(2), frames->at(3), true);
  }

  if (frames->at(0)->numLandmarks() <= options_.init_min_features &&
      frames->at(2)->numLandmarks() <= options_.init_min_features)
  {
    LOGI("StereoInit: Init is failed , num is %d less than %d\n", frames->at(0)->numLandmarks(),
         options_.init_min_features);
    return InitResult::kFailure;
  }
  LOGI("StereoInit: Init is successful, num is %d\n", frames->at(0)->numLandmarks());
  return InitResult::kSuccess;
}

namespace initialization_utils {

// ============================================================================================
// 0x18012BF30
// upstream-modified: only the kStereo (== 0) case remains; StereoInit allocated with Eigen
// aligned new (malloc(0x140)).
AbstractInitialization::UniquePtr makeInitializer(const InitializationOptions& init_options,
                                                  const FeatureTrackerOptions& tracker_options,
                                                  const DetectorOptions& detector_options,
                                                  const CameraBundlePtr& camera_array)
{
  AbstractInitialization::UniquePtr initializer;
  switch (init_options.init_type)
  {
    case InitializerType::kStereo:
      initializer.reset(new StereoInit(init_options, tracker_options, detector_options, camera_array));
      break;
    default:
#line 270
      LOG(FATAL) << "Initializer type not known.";   // initialization.cpp:270
  }
  return initializer;
}

}  // namespace initialization_utils

}  // namespace totem
}  // namespace pimax
