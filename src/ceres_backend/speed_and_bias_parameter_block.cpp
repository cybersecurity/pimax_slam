// pimax_slam.pi.dll -- src/ceres_backend/speed_and_bias_parameter_block.cpp  (draft c05)
//
// Object 0x18008FE60..0x180090180.  upstream-identical.  The default ctor is not emitted.
#include "ceres_backend/speed_and_bias_parameter_block.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

// Default constructor (assumes not fixed).   (not present in the binary)
SpeedAndBiasParameterBlock::SpeedAndBiasParameterBlock()
    : ParameterBlock::ParameterBlock()
{
  setFixed(false);
}

// 0x18008FE60  upstream-identical
// Constructor with estimate.
SpeedAndBiasParameterBlock::SpeedAndBiasParameterBlock(
    const SpeedAndBias& speed_and_bias, uint64_t id)
{
  setEstimate(speed_and_bias);
  setId(id);
  setFixed(false);
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
