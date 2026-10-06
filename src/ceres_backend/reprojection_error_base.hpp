// pimax_slam.pi.dll -- src/ceres_backend/reprojection_error_base.hpp (from draft c00; Pimax variant)
//
// upstream-modified:
//   * the landmark parameter block is Euclidean 3-D (SizedCostFunction<2, 7, 3, 7>), upstream uses a
//     homogeneous 4-vector (<2, 7, 4, 7>).
//   * two extra virtual getters (vtable slots 5 and 6) returning the cached weighted error (+0x50)
//     and the raw error (+0x60) of the last evaluation.
//
// Primary vtable (0x1803AA288, 10 slots) = ceres::CostFunction slots + new virtuals:
//   [0] ~ReprojectionError            0x18000C020
//   [1] Evaluate                      0x18000C460
//   [2] setMeasurement                0x180015DB0
//   [3] setInformation                0x180015C30
//   [4] measurement()      -> +0x40   0x180014670
//   [5] weightedError()    -> +0x50   0x180016B50   (Pimax, name TODO(verify))
//   [6] error()            -> +0x60   0x180013020   (Pimax, name TODO(verify))
//   [7] information()      -> +0x80   0x180013030
//   [8] covariance()       -> +0xC0   0x180012C00
//   [9] EvaluateMinimal    (ReprojectionError)      0x18000E0A0
// Secondary vtable (0x1803AA2E0, ErrorInterface at +0x28):
//   [0] dtor thunk 0x18000BE8C, [1] residualDim 0x180014A10 (folded "return 2"),
//   [2] parameterBlocks 0x180014980, [3] parameterBlockDim 0x180014950,
//   [4] EvaluateWithMinimalJacobians 0x18000C490, [5] typeInfo 0x180016740 (returns 1).
#pragma once

#include <ceres/ceres.h>

#include "ceres_backend/error_interface.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

class ReprojectionErrorBase :
    public ceres::SizedCostFunction<
    2 /* number of residuals */,
    7 /* size of first parameter (T_WS) */,
    3 /* size of second parameter (Euclidean landmark) */,
    7 /* size of third parameter (camera extrinsics T_SC) */>,
    public ErrorInterface
{
 public:
  typedef Eigen::Vector2d measurement_t;
  typedef Eigen::Matrix2d covariance_t;

  virtual ~ReprojectionErrorBase() = default;

  virtual void setMeasurement(const measurement_t& measurement) = 0;
  virtual void setInformation(const covariance_t& information) = 0;
  virtual const measurement_t& measurement() const = 0;
  virtual const measurement_t& weightedError() const = 0;   // Pimax, TODO(verify) name
  virtual const measurement_t& error() const = 0;           // Pimax, TODO(verify) name
  virtual const covariance_t& information() const = 0;
  virtual const covariance_t& covariance() const = 0;
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
