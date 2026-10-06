// patch_warp.cpp -- Pimax fork of svo_direct/src/patch_warp.cpp
// Object [0x1800B0450, 0x1800B1480) (COMDATs sorted by decorated name):
//   0x1800B0450 std::vector<Eigen::Vector2f>::_Resize_reallocate (lib, from warpAffine's resize)
//   0x1800B05B0 warp::getBestSearchLevel
//   0x1800B0600 warp::getWarpMatrixAffine
//   0x1800B0C10 warp::warpAffine
//   (0x1800B1450 pangolin::Handler scalar-deleting dtor -- an identical-COMDAT-folded body that
//    landed here; not patch_warp code.)

#include "direct/patch_warp.h"

#include <cmath>
#include <vector>

#include "common/frame.h"
#include "common/point.h"
#include "common/feature_wrapper.h"
#include "common/camera.h"
#include <glog/logging.h>

namespace pimax {
namespace totem {
namespace warp {

// 0x1800B0600
// upstream-modified:
//  * kHalfPatchSize is 8 here (upstream 5) -> the du/dv offsets are 8 px (constants
//    0x1803B2990 = {8.0, 0.0}, 0x1803B29A0 = {0.0, 8.0}, divisor 0x1803B29B0 = {8.0, 8.0}).
//  * CHECK_NOTNULL(A_cur_ref) removed.
//  * px_ref / f_ref are float (Pimax FloatType); xyz_ref is computed in float
//    (f_ref * (float)depth_ref) and widened to double for the transform.
void getWarpMatrixAffine(
    const CameraPtr& cam_ref,
    const CameraPtr& cam_cur,
    const Eigen::Ref<Keypoint>& px_ref,
    const Eigen::Ref<BearingVector>& f_ref,
    const double depth_ref,
    const Transformation& T_cur_ref,
    const int level_ref,
    AffineTransformation2* A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  const int kHalfPatchSize = 8;
  const BearingVector xyz_ref = f_ref * depth_ref;     // float math
  Eigen::Vector3d xyz_du_ref, xyz_dv_ref;
  // NOTE: project3 has no guarantee that the returned vector is unit length
  // - for pinhole: z component is 1 (unit plane)
  // - for omnicam: norm is 1 (unit sphere)
  cam_ref->backProject3(px_ref.cast<double>() + Eigen::Vector2d(kHalfPatchSize, 0) * (1 << level_ref), &xyz_du_ref);
  cam_ref->backProject3(px_ref.cast<double>() + Eigen::Vector2d(0, kHalfPatchSize) * (1 << level_ref), &xyz_dv_ref);
  if (cam_ref->getType() == Camera::Type::kPinhole)   // camera+48 == 0
  {
    xyz_du_ref *= xyz_ref[2];
    xyz_dv_ref *= xyz_ref[2];
  }
  else
  {
    xyz_du_ref.normalize();
    xyz_dv_ref.normalize();
    xyz_du_ref *= depth_ref;
    xyz_dv_ref *= depth_ref;
  }

  Eigen::Vector2d px_cur, px_du_cur, px_dv_cur;
  cam_cur->project3(T_cur_ref * xyz_ref.cast<double>(), &px_cur);
  cam_cur->project3(T_cur_ref * xyz_du_ref, &px_du_cur);
  cam_cur->project3(T_cur_ref * xyz_dv_ref, &px_dv_cur);
  A_cur_ref->col(0) = (px_du_cur - px_cur) / kHalfPatchSize;
  A_cur_ref->col(1) = (px_dv_cur - px_cur) / kHalfPatchSize;
}

// 0x1800B05B0
// upstream-identical
int getBestSearchLevel(
    const AffineTransformation2& A_cur_ref,
    const int max_level)
{
  // Compute patch level in other image
  int search_level = 0;
  double D = A_cur_ref.determinant();
  while (D > 3.0 && search_level < max_level)
  {
    search_level += 1;
    D *= 0.25;
  }
  return search_level;
}

// 0x1800B0C10
// upstream-modified:
//  * NaN check kept but the LOG(WARNING) "Affine warp is NaN..." was removed (no string in binary).
//  * new early-out: the reference pixel (on its pyramid level) must lie inside the image.
//  * two passes: first all sample positions are computed into a function-local thread_local
//    std::vector<Eigen::Vector2f> (TLS slot +16, guard bit at +40, dtor 0x1803A4A80 registered via
//    __tlregdtor) with a stricter bounds test (xi <= 0 / yi <= 0 rejected); then the patch is
//    interpolated, using (int) truncation instead of floor for the integer part.
bool warpAffine(
    const AffineTransformation2& A_cur_ref,
    const cv::Mat& img_ref,
    const Eigen::Ref<Keypoint>& px_ref,
    const int level_ref,
    const int search_level,
    const int halfpatch_size,
    uint8_t* patch)
{
  Eigen::Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>() * (1 << search_level);
  if (std::isnan(A_ref_cur(0, 0)))
    return false;

  thread_local std::vector<Eigen::Vector2f> px_buffer;
  const int n_pixels = 2 * halfpatch_size * 2 * halfpatch_size;
  px_buffer.resize(n_pixels);

  const Eigen::Vector2f px_ref_pyr = px_ref.cast<float>() / (1 << level_ref);
  const int stride = img_ref.step.p[0];
  // written as rejections: NaN coordinates pass this test in the binary (comiss/ja, jnb)
  if (px_ref_pyr[0] < 0.0f || px_ref_pyr[1] < 0.0f
      || px_ref_pyr[0] >= img_ref.cols || px_ref_pyr[1] >= img_ref.rows)
    return false;

  // Pass 1: sample positions (on the reference pyramid level).
  Eigen::Vector2f* px_ptr = px_buffer.data();
  for (int y = -halfpatch_size; y < halfpatch_size; ++y)
  {
    for (int x = -halfpatch_size; x < halfpatch_size; ++x, ++px_ptr)
    {
      const Eigen::Vector2f px_patch(x, y);
      const Eigen::Vector2f px(A_ref_cur * px_patch + px_ref_pyr);
      const int xi = std::floor(px[0]);
      const int yi = std::floor(px[1]);
      if (xi <= 0 || yi <= 0 || xi + 1 >= img_ref.cols || yi + 1 >= img_ref.rows)
        return false;
      *px_ptr = px;
    }
  }

  // Pass 2: bilinear interpolation.
  uint8_t* patch_ptr = patch;
  const Eigen::Vector2f* pxs = px_buffer.data();
  for (int i = 0; i < n_pixels; ++i, ++patch_ptr)
  {
    const Eigen::Vector2f& px = pxs[i];
    const int xi = static_cast<int>(px[0]);
    const int yi = static_cast<int>(px[1]);
    const float subpix_x = px[0] - xi;
    const float subpix_y = px[1] - yi;
    const float w00 = (1.0f - subpix_x) * (1.0f - subpix_y);
    const float w01 = (1.0f - subpix_x) * subpix_y;
    const float w10 = subpix_x * (1.0f - subpix_y);
    const float w11 = 1.0f - w00 - w01 - w10;
    const uint8_t* const ptr = img_ref.data + yi * stride + xi;
    *patch_ptr = static_cast<uint8_t>(w00 * ptr[0] + w01 * ptr[stride] + w10 * ptr[1] + w11 * ptr[stride + 1]);
  }
  return true;
}

} // namespace warp
} // namespace totem
} // namespace pimax
