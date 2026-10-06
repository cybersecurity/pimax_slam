// pimax_slam.pi.dll -- src/ceres_backend/error_interface.cpp  (draft c00)
//
// ceres_backend/error_interface.cpp -- object #3 in link order (TU3).
// Dynamic initializer 0x180001030 (.CRT$XCU #4), atexit 0x1803A47B0.
// Upstream: std::map; Pimax: std::unordered_map and the extra GroundPlaneError entry.
// The initializer builds a temporary initializer_list<pair<const ErrorType, std::string>> (8 x 0x28
// bytes) exactly in this order, then default-constructs the hash table (8 buckets, max_load_factor 1.0)
// and inserts the range (0x180020440).
#include "ceres_backend/error_interface.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x18047D9F0 (initializer 0x180001030)
const std::unordered_map<ErrorType, std::string> kErrorToStr
{
  {ErrorType::kHomogeneousPointError, std::string("HomogeneousPointError") },
  {ErrorType::kReprojectionError, std::string("ReprojectionError") },
  {ErrorType::kSpeedAndBiasError, std::string("SpeedAndBiasError") },
  {ErrorType::kMarginalizationError, std::string("MarginalizationError") },
  {ErrorType::kPoseError, std::string("PoseError") },
  {ErrorType::kIMUError, std::string("IMUError") },
  {ErrorType::kRelativePoseError, std::string("RelativePoseError") },
  {ErrorType::kGroundPlaneError, std::string("GroundPlaneError") },
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
