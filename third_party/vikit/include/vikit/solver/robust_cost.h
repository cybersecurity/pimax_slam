// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit).
// Upstream: rpg_svo_pro_open/vikit/vikit_solver/include/vikit/solver/robust_cost.h  -- identical
// declarations, minus UnitScaleEstimator / NormalDistributionScaleEstimator / UnitWeightFunction /
// HuberWeightFunction, which are not in the image (removed, PIMAX_PATCH.md).
// Only TukeyWeightFunction and MADScaleEstimator have vtables in the image:
//   vk::solver::MADScaleEstimator::`vftable'   @0x1803B7300  [0]=deleting dtor (generic, ICF-folded)
//                                                            [1]=compute 0x1801B5C50
//   vk::solver::TukeyWeightFunction::`vftable' @0x1803BFF80  [0]=deleting dtor (ICF-folded)
//                                                            [1]=weight 0x1801B5DC0
// TukeyWeightFunction layout: +0 vptr, +8 float b_square_  (sizeof 16).
// MADScaleEstimator layout:   +0 vptr                      (sizeof 8).
#ifndef VIKIT_ROBUST_COST_H_
#define VIKIT_ROBUST_COST_H_

#include <vector>
#include <memory>

namespace vk {
namespace solver {

/// Scale Estimators to estimate standard deviation of a distribution of errors.
class ScaleEstimator
{
public:
  virtual ~ScaleEstimator() = default;
  /// Errors must be absolute values!
  virtual float compute(std::vector<float>& errors) const = 0;
};
typedef std::shared_ptr<ScaleEstimator> ScaleEstimatorPtr;

// estimates scale by computing the median absolute deviation
class MADScaleEstimator : public ScaleEstimator
{
public:
  using ScaleEstimator::ScaleEstimator;
  virtual ~MADScaleEstimator() = default;
  virtual float compute(std::vector<float>& errors) const;
};

/// Weight-Functions for M-Estimators
class WeightFunction
{
public:
  WeightFunction() = default;
  virtual ~WeightFunction() = default;
  virtual float weight(const float& error) const = 0;
};
typedef std::shared_ptr<WeightFunction> WeightFunctionPtr;

class TukeyWeightFunction : public WeightFunction
{
public:
  TukeyWeightFunction(const float b = 4.6851f);
  virtual ~TukeyWeightFunction() = default;
  virtual float weight(const float& error) const;
private:
  float b_square_;
};

} // namespace solver
} // namespace vk
#endif // VIKIT_ROBUST_COST_H_
