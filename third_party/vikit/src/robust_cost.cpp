// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit), object robust_cost.obj
// (0x1801B5C30 .. 0x1801B5DEA). Upstream: rpg_svo_pro_open/vikit/vikit_solver/src/robust_cost.cpp
//
// Unit/NormalDistribution scale estimators and Unit/Huber weight functions are not in the image
// and have been removed.
#include "vikit/solver/robust_cost.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <glog/logging.h>

namespace vk {
namespace solver {

/* ************************************************************************* */
// Scale Estimators
/* ************************************************************************* */

// 0x1801B5C50  -- upstream-modified: CHECK(!errors.empty()) << "Error vector is empty." removed
// (string absent from the image). Index computed exactly as upstream:
// floor((double)(size_t)(size/2)) -> cvttsd2si 64-bit; std::nth_element (MSVC: introselect with
// _Partition_by_median_guess_unchecked 0x180090AD0, insertion sort for <= 32 elements);
// result * 1.48f (float constant 0x1803BFF90).
// NOTE (quirk): with the CHECK gone, an empty vector makes it == end() and *it dereferences
// end() (nullptr if the vector never allocated) -> UB; binary does exactly that.
float MADScaleEstimator::compute(std::vector<float>& errors) const
{
  auto it = errors.begin()+std::floor(errors.size()/2);
  std::nth_element(errors.begin(), it, errors.end()); // compute median
  return 1.48f * (*it); // 1.48f / 0.6745
}

/* ************************************************************************* */
// Weight Functions
/* ************************************************************************* */

// 0x1801B5C30  -- upstream-identical (b*b in float; caller 0x180132D90 = PoseOptimizer)
TukeyWeightFunction::TukeyWeightFunction(const float b)
  : b_square_(b*b)
{}

// 0x1801B5DC0  -- upstream-identical (comiss b_square_, x^2; jb -> 0)
float TukeyWeightFunction::weight(const float& error) const
{
  const float x_square = error * error;
  if(x_square <= b_square_)
  {
    const float tmp = 1.0f - x_square / b_square_;
    return tmp * tmp;
  }
  else
  {
    return 0.0f;
  }
}

} // namespace solver
} // namespace vk
