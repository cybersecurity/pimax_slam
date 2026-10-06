// pimax_slam.pi.dll -- src/direct/matcher.cpp
// Pimax fork of svo_direct/src/matcher.cpp.  Original: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\direct\matcher.cpp
//
// Merged from the two chunk drafts (c06 head: draft/c06_common_depthfilter/direct/matcher_c06.cpp,
// c07 tail: draft/c07_direct/direct/matcher.cpp).  Object layout (COMDATs sorted by decorated
// name; starts at ~0x1800ADD10, ends at 0x1800B0450):
//   0x1800ADD10 std::_Integral_to_string<char,int> (lib COMDAT)
//   0x1800ADDF0 / 0x1800ADE80 / 0x1800ADEF0  Eigen::Ref<const Vector2d/3d> from float vectors (lib)
//   0x1800ADF30 patch_score::ZMSSD<4>::ZMSSD (header template, upstream-identical)
//   0x1800AE070 Eigen evaluator<Inverse<Matrix2d>> (lib)
//   0x1800AE150 lambda in depthFromTriangulation (the upstream solver)
//   0x1800AE5A0 patch_utils::createPatchFromPatchWithBorder (header inline, upstream-identical)
//   0x1800AE610 matcher_utils::depthFromTriangulation
//   0x1800AE820 Matcher::findEpipolarMatchDirect (with T_cur_ref)
//   0x1800AF4B0 Matcher::findLocalMatch
//   0x1800AF5E0 Matcher::findMatchDirect
//   0x1800AFD70 Matcher::scanEpipolarUnitPlane
//   0x1800B0240 std::vector<cv::Mat>::size (lib, not inlined)
//   0x1800B0270 Matcher::updateZMSSD
// Header: src/direct/matcher.h (c07 layout + c06/c13 corrections: float depth out-parameters,
// extra Vector3f f_cur_unnormalized_ at +332, sizeof 352, Pimax option defaults).

#include "direct/matcher.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <vikit/math_utils.h>          // vk::project2, vk::unproject2d

#include "direct/patch_warp.h"
#include "direct/patch_score.h"
#include "direct/patch_utils.h"
#include "direct/feature_alignment.h"
#include "common/frame.h"
#include "common/point.h"
#include "common/feature_wrapper.h"
#include "common/camera.h"

namespace pimax {
namespace totem {

// 0x1800AE820
// upstream-modified (see notes): float transforms/bearings, extra estimate point C checked for
// visibility, all three projections checked, two extra epipolar lengths, float epi_dir, the
// unit-sphere/scanEpipolarLine dispatch replaced by scanEpipolarUnitPlane, backProject3 result
// checked (kFailAngle), unnormalized f_cur kept in a second member.
Matcher::MatchResult Matcher::findEpipolarMatchDirect(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const Transformation& T_cur_ref,
    const FeatureWrapper& ref_ftr,
    const double d_estimate_inv,
    const double d_min_inv,
    const double d_max_inv,
    FloatType& depth)
{
  int zmssd_best = PatchScore::threshold();     // 128000

  // Compute start and end of epipolar line in old_kf for match search, on image plane
  const BearingVector A = T_cur_ref.getRotation().cast<float>().rotate(ref_ftr.f)
      + T_cur_ref.cast<float>().getPosition()*d_min_inv;
  const BearingVector B = T_cur_ref.cast<float>().getRotation().rotate(ref_ftr.f)
      + T_cur_ref.cast<float>().getPosition()*d_max_inv;
  // Pimax: the point at the current depth estimate is computed up-front and checked too.
  const BearingVector C = T_cur_ref.getRotation().cast<float>().rotate(ref_ftr.f)
      + T_cur_ref.cast<float>().getPosition()*d_estimate_inv;
  if(A(2) < 0.0 || C(2) < 0.0 || B(2) < 0.0)
    return MatchResult::kFailVisibility;

  Eigen::Vector2d px_A, px_B, px_C;
  if(!cur_frame.cam()->project3(A.cast<double>(), &px_A, nullptr).isKeypointVisible())
    return MatchResult::kFailVisibility;
  if(!cur_frame.cam()->project3(C.cast<double>(), &px_C, nullptr).isKeypointVisible())
    return MatchResult::kFailVisibility;
  if(!cur_frame.cam()->project3(B.cast<double>(), &px_B, nullptr).isKeypointVisible())
    return MatchResult::kFailVisibility;
  epi_image_ = px_A - px_B;
  const Eigen::Vector2d epi_image_ca = px_A - px_C;   // Pimax-new
  const Eigen::Vector2d epi_image_cb = px_C - px_B;   // Pimax-new

  // Compute affine warp matrix
  warp::getWarpMatrixAffine(
      ref_frame.cam_, cur_frame.cam_, ref_ftr.px, ref_ftr.f,
      1.0/std::max(0.000001, d_estimate_inv), T_cur_ref, ref_ftr.level, &A_cur_ref_);

  // feature pre-selection
  reject_ = false;
  if(isEdgelet(ref_ftr.type) && options_.epi_search_edgelet_filtering)
  {
    const Eigen::Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad.cast<double>()).normalized();
    const double cosangle = fabs(grad_cur.dot(epi_image_.normalized()));
    if(cosangle < options_.epi_search_edgelet_max_angle)
    {
      reject_ = true;
      return MatchResult::kFailAngle;
    }
  }

  // prepare for match
  //    - find best search level
  //    - warp the reference patch
  search_level_ = warp::getBestSearchLevel(A_cur_ref_, ref_frame.img_pyr_.size()-1);
  // length and direction on SEARCH LEVEL
  epi_length_pyramid_ = epi_image_.norm() / (1<<search_level_);
  epi_length_pyramid_ca_ = epi_image_ca.norm() / (1<<search_level_);
  epi_length_pyramid_cb_ = epi_image_cb.norm() / (1<<search_level_);
  GradientVector epi_dir_image = epi_image_.cast<float>().normalized();
  if(!warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                       ref_ftr.level, search_level_, kHalfPatchSize+1, patch_with_border_))
    return MatchResult::kFailWarp;
  patch_utils::createPatchFromPatchWithBorder(
        patch_with_border_, kPatchSize, patch_);

  if(epi_length_pyramid_ < 2.0)
  {
    // Case 1: direct search locally if the epipolar line is too short
    px_cur_ = (px_A.cast<float>() + px_B.cast<float>()) * 0.5f;
    MatchResult res = findLocalMatch(cur_frame, epi_dir_image, search_level_, px_cur_);
    if(res != MatchResult::kSuccess)
      return res;
  }
  else
  {
    // Case 2: search along the epipolar line for the best match
    PatchScore patch_score(patch_); // precompute for reference patch
    const BearingVector C2 = T_cur_ref.cast<float>().getRotation().rotate(ref_ftr.f)
        + T_cur_ref.cast<float>().getPosition()*d_estimate_inv;
    scanEpipolarUnitPlane(cur_frame, A.cast<double>(), B.cast<double>(), C2.cast<double>(),
                          patch_score, search_level_, &px_cur_, &zmssd_best);

    // check if the best match is good enough
    if(zmssd_best >= PatchScore::threshold())
      return MatchResult::kFailScore;
    if(options_.subpix_refinement)
    {
      MatchResult res = findLocalMatch(cur_frame, epi_dir_image, search_level_, px_cur_);
      if(res != MatchResult::kSuccess)
        return res;
    }
  }

  Eigen::Vector3d f_cur;
  if(!cur_frame.cam()->backProject3(px_cur_.cast<double>(), &f_cur))
    return MatchResult::kFailAngle;              // Pimax: result checked
  f_cur_ = f_cur.cast<float>();
  f_cur_unnormalized_ = f_cur_;                  // Pimax-new member (+332) TODO(verify) name
  f_cur_.normalize();
  return matcher_utils::depthFromTriangulation(
        T_cur_ref, ref_ftr.f.cast<double>(), f_cur_.cast<double>(), &depth);
}

// 0x1800AF4B0 (upstream-identical apart from float keypoints)
Matcher::MatchResult Matcher::findLocalMatch(
    const Frame& frame,
    const Eigen::Ref<GradientVector>& direction,
    const int patch_level,
    Keypoint& px_cur)
{
  Keypoint px_scaled(px_cur/(1<<patch_level));
  bool res;
  if(options_.align_1d)
    res = feature_alignment::align1D(
          frame.img_pyr_[patch_level], direction, patch_with_border_, patch_,
          options_.align_max_iter, options_.affine_est_offset_, options_.affine_est_gain_,
          &px_scaled, &h_inv_);
  else
    res = feature_alignment::align2D(
          frame.img_pyr_[patch_level], patch_with_border_, patch_,
          options_.align_max_iter, options_.affine_est_offset_, options_.affine_est_gain_,
          px_scaled);
  if(!res)
    return MatchResult::kFailAlignment;
  px_cur = px_scaled*(1<<patch_level);
  return MatchResult::kSuccess;
}

namespace matcher_utils {

// 0x1800AE610 (+ lambda 0x1800AE150)
// upstream-modified: closed-form two-ray solution; the upstream least-squares solve is kept as a
// fallback lambda for a non-finite or (exactly) borderline denominator.  depth is FloatType.
Matcher::MatchResult depthFromTriangulation(
    const Transformation& T_search_ref,
    const Eigen::Vector3d& f_ref,
    const Eigen::Vector3d& f_cur,
    FloatType* depth)
{
  auto solve_least_squares = [&]() -> Matcher::MatchResult   // 0x1800AE150
  {
    Eigen::Matrix<double,3,2> A; A << T_search_ref.getRotation().rotate(f_ref), f_cur;
    const Eigen::Matrix2d AtA = A.transpose()*A;
    if(AtA.determinant() < 0.000001)
      return Matcher::MatchResult::kFailTriangulation;
    const Eigen::Vector2d depth2 = - AtA.inverse()*A.transpose()*T_search_ref.getPosition();
    (*depth) = std::fabs(depth2[0]);
    return Matcher::MatchResult::kSuccess;
  };

  const Eigen::Vector3d Rf = T_search_ref.getRotation().rotate(f_ref);
  const Eigen::Vector3d& t = T_search_ref.getPosition();
  const double a = Rf(0)*f_cur(0) + Rf(1)*f_cur(1) + Rf(2)*f_cur(2);
  const double b = Rf(0)*Rf(0) + Rf(1)*Rf(1) + Rf(2)*Rf(2);
  const double c = f_cur(0)*f_cur(0) + f_cur(1)*f_cur(1) + f_cur(2)*f_cur(2);
  const double d = f_cur(0)*t(0) + f_cur(1)*t(1) + f_cur(2)*t(2);
  const double e = Rf(0)*t(0) + Rf(1)*t(1) + Rf(2)*t(2);
  const double denom = b * c - a * a;
  if(!std::isfinite(denom) || std::fabs(denom - 0.000001) <= 1e-12)
    return solve_least_squares();
  if(denom < 0.000001)
    return Matcher::MatchResult::kFailTriangulation;
  *depth = std::fabs(-(c * e - d * a) / denom);
  return Matcher::MatchResult::kSuccess;
}

} // namespace matcher_utils


// 0x1800AF5E0
// upstream-modified:
//  * Keypoint/BearingVector/GradientVector are float; ref_depth is float and widened to double for
//    getWarpMatrixAffine.
//  * always warp::warpAffine (options_.use_affine_warp_ / warpPixelwise branch removed).
//  * corner (non-edgelet) path: the "kFailTooFar" check was removed; only the edgelet path keeps it.
//  * backProject3() result is now checked: if it fails the function returns kFailAlignment.
//  * A_cur_ref_ (double) is cast to float to rotate the (float) gradient.
Matcher::MatchResult Matcher::findMatchDirect(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const FeatureWrapper& ref_ftr,
    const FloatType& ref_depth,
    Keypoint& px_cur)
{
  Eigen::Vector2i pxi = ref_ftr.px.cast<int>() / (1 << ref_ftr.level);
  int boundary = kHalfPatchSize + 2;
  if (pxi[0] < boundary
      || pxi[1] < boundary
      || pxi[0] >= static_cast<int>(ref_frame.cam()->imageWidth() / (1 << ref_ftr.level)) - boundary
      || pxi[1] >= static_cast<int>(ref_frame.cam()->imageHeight() / (1 << ref_ftr.level)) - boundary)
    return MatchResult::kFailVisibility;

  // warp affine
  warp::getWarpMatrixAffine(
      ref_frame.cam_, cur_frame.cam_, ref_ftr.px, ref_ftr.f, ref_depth,
      cur_frame.T_cam_world() * ref_frame.T_world_cam(), ref_ftr.level, &A_cur_ref_);
  search_level_ = warp::getBestSearchLevel(A_cur_ref_, ref_frame.img_pyr_.size() - 1);

  if (!warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                        ref_ftr.level, search_level_, kHalfPatchSize + 1, patch_with_border_))
    return MatchResult::kFailWarp;
  patch_utils::createPatchFromPatchWithBorder(
      patch_with_border_, kPatchSize, patch_);

  // px_cur should be set
  Keypoint px_scaled(px_cur / (1 << search_level_));
  Keypoint px_scaled_start(px_scaled);

  if (isEdgelet(ref_ftr.type))   // kEdgeletSeed(0) / kEdgeletSeedConverged(3) / kEdgelet(6): bitmask 0x49
  {
    GradientVector dir_cur(A_cur_ref_.cast<float>() * ref_ftr.grad);
    dir_cur.normalize();
    if (feature_alignment::align1D(
            cur_frame.img_pyr_[search_level_], dir_cur, patch_with_border_,
            patch_, options_.align_max_iter,
            options_.affine_est_offset_, options_.affine_est_gain_,
            &px_scaled, &h_inv_))
    {
      if ((px_scaled - px_scaled_start).norm() >
          options_.max_patch_diff_ratio * kPatchSize)
      {
        return MatchResult::kFailTooFar;
      }
      px_cur = px_scaled * (1 << search_level_);
      // set member variables with results (used in reprojector)
      px_cur_ = px_cur;
      Eigen::Vector3d f_cur;
      if (cur_frame.cam()->backProject3(px_cur_.cast<double>(), &f_cur))
      {
        f_cur_ = f_cur.cast<float>();
        f_cur_.normalize();
        return MatchResult::kSuccess;
      }
    }
  }
  else
  {
    std::vector<Eigen::Vector2f>* last_fail_steps = nullptr;
    bool res = feature_alignment::align2D(
        cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
        options_.align_max_iter,
        options_.affine_est_offset_, options_.affine_est_gain_,
        px_scaled, false, last_fail_steps);
    if (res)
    {
      px_cur = px_scaled * (1 << search_level_);
      // set member variables with results (used in reprojector)
      px_cur_ = px_cur;
      Eigen::Vector3d f_cur;
      if (cur_frame.cam()->backProject3(px_cur_.cast<double>(), &f_cur))
      {
        f_cur_ = f_cur.cast<float>();
        f_cur_.normalize();
        return MatchResult::kSuccess;
      }
    }
    else
    {
      VLOG(300) << "NOT CONVERGED: search level " << search_level_;   // matcher.cpp:221
    }
  }
  return MatchResult::kFailAlignment;
}

// 0x1800AFD70
// upstream-modified (heavily): no clamping to options_.max_epi_search_steps, no direction
// reversal. The search starts n_steps_b steps from C towards B and walks n_steps_a + n_steps_b
// steps towards A (each side capped at 15 steps; step length = |AB|/n_steps,
// n_steps = epi_length_pyramid_/0.7). Pixels whose projection is not
// KEYPOINT_VISIBLE, or whose patch is not fully inside the image, are skipped (the upstream
// version stopped instead).
void Matcher::scanEpipolarUnitPlane(
    const Frame& frame,
    const Eigen::Vector3d& A,
    const Eigen::Vector3d& B,
    const Eigen::Vector3d& C,
    const PatchScore& patch_score,
    const int patch_level,
    Keypoint* image_best,
    int* zmssd_best)
{
  size_t n_steps = epi_length_pyramid_ / 0.7; // one step per pixel
  const int n_steps_a = static_cast<int>(std::min(15.0, epi_length_pyramid_ca_ / 0.7));
  const int n_steps_b = static_cast<int>(std::min(15.0, epi_length_pyramid_cb_ / 0.7));
  Eigen::Vector2d step = (vk::project2(A) - vk::project2(B)) / n_steps;

  // now we sample along the epipolar line
  Eigen::Vector2d uv_C = vk::project2(C);
  Eigen::Vector2d uv = uv_C - n_steps_b * step;
  Eigen::Vector2d uv_best = uv;
  Eigen::Vector2i last_checked_pxi(0, 0);
  const int img_w = static_cast<int>(frame.cam()->imageWidth() / (1 << patch_level)) - kPatchSize;
  const int img_h = static_cast<int>(frame.cam()->imageHeight() / (1 << patch_level)) - kPatchSize;

  for (int i = 0; i < n_steps_a + n_steps_b; ++i, uv += step)
  {
    Eigen::Vector2d px;
    if (!frame.cam()->project3(vk::unproject2d(uv), &px))
      continue;
    Eigen::Vector2i pxi(px[0] / (1 << patch_level) + 0.5,
                        px[1] / (1 << patch_level) + 0.5); // +0.5 to round to closest int

    if (pxi == last_checked_pxi)
      continue;
    last_checked_pxi = pxi;

    // check if the patch is full within the new frame
    if (pxi[0] < kPatchSize || pxi[1] < kPatchSize || pxi[0] >= img_w || pxi[1] >= img_h)
      continue;

    if (updateZMSSD(frame, pxi, patch_level, patch_score, zmssd_best))
      uv_best = uv;
  }

  // convert uv_best to image coordinates
  Eigen::Vector2d projected;
  frame.cam()->project3(vk::unproject2d(uv_best), &projected);
  *image_best = projected.cast<FloatType>();
}

// 0x1800B0270
// upstream-identical (ZMSSD<4>::computeScore inlined: threshold 2000*64 = 128000).
bool Matcher::updateZMSSD(
    const Frame& frame,
    const Eigen::Vector2i& pxi,
    const int patch_level,
    const PatchScore& patch_score,
    int* zmssd_best)
{
  // TODO interpolation would probably be a good idea
  uint8_t* cur_patch_ptr = frame.img_pyr_[patch_level].data
                           + (pxi[1] - kHalfPatchSize) * frame.img_pyr_[patch_level].step
                           + (pxi[0] - kHalfPatchSize);
  int zmssd = patch_score.computeScore(cur_patch_ptr, frame.img_pyr_[patch_level].step);

  if (zmssd < *zmssd_best)
  {
    *zmssd_best = zmssd;
    return true;
  }
  else
    return false;
}

} // namespace totem
} // namespace pimax
