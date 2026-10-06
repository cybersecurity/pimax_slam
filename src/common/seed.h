// pimax_slam.pi.dll -- src/common/seed.h
// Pimax fork of svo_common/include/svo/common/seed.h (inverse-depth branch only), chunk c06.  All functions are inline; they were checked against their
// inlined copies in depth_filter.cpp (0x1800A0AB0 initializeSeeds, 0x1800A2AC0 updateSeed) and
// against the one out-of-line COMDAT copy 0x1800A0A60 (getSigma2FromDepthSigma).
// FloatType == float, SeedState == Eigen::Vector4f.
#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>

#include "common/types.h"

namespace pimax {
namespace totem {
namespace seed {

enum SeedStateIndex
{
  kMu,
  kSigma2,
  kA,
  kB,
};

inline FloatType mu(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(0);
}

inline FloatType sigma2(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(1);
}

inline FloatType a(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(2);
}

inline FloatType b(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(3);
}

// -----------------------------------------------------------------------------
// Inverse Depth Parametrization

inline FloatType getDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return 1.0 / mu_sigma2_a_b(0);              // double division, narrowed to float
}

inline FloatType getInvDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(0);
}

inline FloatType getInvMinDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(0) + std::sqrt(mu_sigma2_a_b(1));
}

inline FloatType getInvMaxDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  // Pimax: lower bound 0.01f (upstream 0.00000001).  Constant 0x1803B1F74.
  return std::max(mu_sigma2_a_b(0) - std::sqrt(mu_sigma2_a_b(1)), 0.01f);
}

inline FloatType getMeanFromDepth(FloatType depth)
{
  return 1.0 / depth;
}

inline FloatType getMeanRangeFromDepthMinMax(FloatType depth_min, FloatType /*depth_max*/)
{
  return 1.0 / depth_min;
}

inline FloatType getInitSigma2FromMuRange(FloatType mu_range)
{
  return mu_range * mu_range / 36.0;          // float product, double division
}

inline bool isConverged(const Eigen::Ref<const SeedState>& mu_sigma2_a_b,
                        FloatType mu_range,
                        FloatType sigma2_convergence_threshold)
{
  // If initial uncertainty was reduced by factor sigma2_convergence_threshold
  // we accept the seed as converged.
  const FloatType thresh = mu_range / sigma2_convergence_threshold;
  // Pimax: extra factor 0.5 (upstream: thresh * thresh).  Evaluated in double.
  return (mu_sigma2_a_b(1) < thresh * 0.5 * thresh);
}

// Out-of-line COMDAT copy at 0x1800A0A60.
inline FloatType getSigma2FromDepthSigma(FloatType depth, FloatType depth_sigma)
{
  const FloatType sigma = 0.5 * (1.0 / std::max(0.000000000001f, depth - depth_sigma)
                               - 1.0 / (depth + depth_sigma));
  return sigma * sigma;
}

// -----------------------------------------------------------------------------
// Utils

inline void increaseOutlierProbability(Eigen::Ref<SeedState>& mu_sigma2_a_b)
{
  mu_sigma2_a_b(3) += 1;
}

} // namespace seed
} // namespace totem
} // namespace pimax
