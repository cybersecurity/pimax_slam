// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit), object ncamera.obj
// (0x1801B3F40 .. 0x1801B41FB). Upstream: rpg_svo_pro_open/vikit/vikit_cameras/src/ncamera.cpp
//
// 0x1801B3F40 is the template instantiation
//   std::vector<Transformation, Eigen::aligned_allocator<Transformation>>::vector(const vector&)
// (lib, emitted in this object; malloc + "System's malloc returned an unaligned pointer" assert).
#include <vikit/cameras/ncamera.h>

#include <string>
#include <utility>

#include <vikit/cameras/camera_geometry_base.h>

namespace vk {
namespace cameras {

// 0x1801B4050  -- upstream-modified: extra T_B_C argument/member; initInternal() (CHECK_EQ on
// sizes, CHECK_NOTNULL on cameras) is NOT called (no such checks in the binary).
NCamera::NCamera(
    const TransformationVector& T_C_B,
    const TransformationVector& T_B_C,
    const std::vector<Camera::Ptr>& cameras,
    const std::string& label)
    : T_C_B_(T_C_B)
    , T_B_C_(T_B_C)
    , cameras_(cameras)
    , label_(label)
{}

// 0x1801B41F0  -- upstream-modified: CHECK_LT(camera_index, cameras_.size()) removed
const Transformation& NCamera::get_T_C_B(size_t camera_index) const
{
  return T_C_B_[camera_index];
}

// 0x1801B41E0  -- pimax-new (returns element of the second vector at +24).
// Name inferred from in-object ordering (functions are sorted by decorated name:
// ?getCameraShared < ?get_T_B_C < ?get_T_C_B).  // TODO(verify) exact name
const Transformation& NCamera::get_T_B_C(size_t camera_index) const
{
  return T_B_C_[camera_index];
}

// 0x1801B41A0  -- upstream-modified: CHECK_LT removed. Only one getCameraShared overload is
// present in the image (const vs non-const indistinguishable from code).  // TODO(verify)
Camera::Ptr NCamera::getCameraShared(size_t camera_index)
{
  return cameras_[camera_index];
}

} // namespace cameras
} // namespace vk
