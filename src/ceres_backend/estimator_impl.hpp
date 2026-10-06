// pimax_slam.pi.dll -- src/ceres_backend/estimator_impl.hpp  (from draft c01)
//
// Inline Estimator members (upstream estimator_impl.hpp) whose single out-of-line copy was
// emitted inside ceres_backend_interface.obj (c01 range).  Included at the end of estimator.hpp.
#pragma once

#include "ceres_backend/estimator.hpp"
#include "ceres_backend/reprojection_error.hpp"
#include "common/frame.h"
#include "common/point.h"

namespace pimax {
namespace totem {

// estimator.hpp, inlined everywhere except one copy at 0x180013420
// (landmarks_map_ = std::map<uint64_t/BackendId, MapPoint> at Estimator+264)
inline bool Estimator::isPointInEstimator(const int id) const
{
  return landmarks_map_.find(createLandmarkId(id)) != landmarks_map_.end();
}

// estimator.hpp (always inlined; used by bundleAdjustment). states_ at Estimator+320:
// ids (vector<BackendId>) +320, is_keyframe (vector<bool>) +344.
inline void Estimator::setKeyframe(BackendId nframe_id, bool is_keyframe)
{
  auto slot = states_.findSlot(nframe_id);
  if (slot.second)
  {
    states_.is_keyframe[slot.first] = is_keyframe;
  }
}

// Add an observation to a landmark.
// 0x180010C20  (upstream estimator_impl.hpp; pimax-modified)
//  - new 3rd parameter nframe_id (precomputed by the caller)
//  - no states_.findSlot() check, no isLandmarkFixed()/setPointConstant, no temporal extrinsics
//  - the measurement is the projection of the CURRENT landmark estimate into the frame
//    (frame->w2c(landmark->pos_)), not frame->px_vec_.col(kp)  <-- quirk, preserve
//  - the camera index is stored into the ReprojectionError (+56)
//  - the loss function is huber_loss_function_ptr_ (this[73] = +0x248), not the Cauchy loss
//  - observations is an unordered_map<uint64_t residual_id, KeypointIdentifier>
inline ceres::ResidualBlockId Estimator::addObservation(const FramePtr& frame,
                                                        const size_t keypoint_idx,
                                                        const BackendId& nframe_id)
{
  const int cam_idx = frame->getNFrameIndex();  // Frame +36
  // get Landmark ID.
  const BackendId landmark_backend_id = createLandmarkId(
        frame->track_id_vec_[keypoint_idx]);

  KeypointIdentifier kid(frame, keypoint_idx);  // 0x180099F40

  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  information *= 1.0 / static_cast<double>(1 << frame->level_vec_(keypoint_idx));

  const Eigen::Vector2d px =
      frame->w2c(frame->landmark_vec_[keypoint_idx]->pos_.cast<double>());

  // create error term
  std::shared_ptr<ceres_backend::ReprojectionError> reprojection_error =
      std::make_shared<ceres_backend::ReprojectionError>(
        std::static_pointer_cast<const Camera>(
          camera_rig_->getCameraShared(cam_idx)),
        px, information);
  reprojection_error->cam_index_ = cam_idx;  // +56 (int64)  TODO(verify) member name

  ceres::ResidualBlockId ret_val = map_ptr_->addResidualBlock(
        reprojection_error,
        huber_loss_function_ptr_.get(),   // +0x248 (HuberLoss(0.5)); upstream: cauchy (+0x238)
        map_ptr_->parameterBlockPtr(nframe_id.asInteger()),
        map_ptr_->parameterBlockPtr(landmark_backend_id.asInteger()),
        map_ptr_->parameterBlockPtr(constant_extrinsics_ids_[cam_idx].asInteger()));

  // remember
  landmarks_map_.at(landmark_backend_id).observations.insert(
        std::pair<uint64_t, KeypointIdentifier>(
          reinterpret_cast<uint64_t>(ret_val), kid));

  return ret_val;
}

}  // namespace totem
}  // namespace pimax
