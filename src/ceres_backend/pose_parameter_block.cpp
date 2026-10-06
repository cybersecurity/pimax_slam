// pimax_slam.pi.dll -- src/ceres_backend/pose_parameter_block.cpp  (draft c05)
//
// Object 0x18008D960..0x18008DB80.  Port of svo_ceres_backend/src/pose_parameter_block.cpp --
// upstream-identical.  The default constructor is not emitted (unreferenced).
#include "ceres_backend/pose_parameter_block.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

// Default constructor (assumes not fixed).   (not present in the binary)
PoseParameterBlock::PoseParameterBlock()
    : ParameterBlock::ParameterBlock()
{
  setFixed(false);
}

// 0x18008D9C0
// Trivial destructor.
PoseParameterBlock::~PoseParameterBlock() {}

// 0x18008D960  upstream-identical
// Constructor with estimate.
PoseParameterBlock::PoseParameterBlock(const Transformation& T_WS, uint64_t id)
{
  setEstimate(T_WS);
  setId(id);
  setFixed(false);
}

// 0x18008DB00  upstream-identical
// setters
// Set estimate of this parameter block.
void PoseParameterBlock::setEstimate(const Transformation& T_WS)
{
  const Eigen::Vector3d& r = T_WS.getPosition();
  const Eigen::Quaterniond& q = T_WS.getRotation().toImplementation();
  parameters_[0] = r[0];
  parameters_[1] = r[1];
  parameters_[2] = r[2];
  parameters_[3] = q.x();  // x
  parameters_[4] = q.y();  // y
  parameters_[5] = q.z();  // z
  parameters_[6] = q.w();  // w
}

// 0x18008DA10  upstream-identical (RotationQuaternion(const Eigen::Quaterniond&) = 0x1800089C0,
// with the Pimax-minkindr EPS norm CHECKs).
// getters
// Get estimate.
Transformation PoseParameterBlock::estimate() const
{
  return Transformation(
      Eigen::Vector3d(parameters_[0], parameters_[1], parameters_[2]),
      Eigen::Quaterniond(parameters_[6], parameters_[3], parameters_[4],
                         parameters_[5]));
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
