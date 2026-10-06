// pimax_slam.pi.dll -- src/interface/ceres_backend_factory.cpp  (draft c13, phase B3)
//
// Pimax port of svo_ros/src/ceres_backend_factory.cpp: ROS parameters replaced by constants,
// no motion-detector options, no startThread().  Object TU38, code 0x180158F30..0x1801590E0
// (+ the _Ref_count_obj2<CeresBackendInterface> members 0x180158F30 / 0x180158F60).
// The original file has the VLOG on line 13 (LogMessage(__FILE__, 13) in 0x180158F70): forced
// with #line.  The namespace spelling (pimax::totem::ceres_backend_factory) is a guess:
// TODO(verify).
//
// The binary stores the options field by field (all fields, so the order of the assignments
// below is not observable); stores in the binary: optimizer word+16 (verbose/marginalize), +8,
// +12, +32, +24, +0, word+40, +48; interface +0, +8, word+16, +24, +32, +40, +56, +48 -- the
// assignment order below follows that store order.
#include "interface/ceres_backend_factory.h"

#include <cmath>
#include <memory>

#include <glog/logging.h>

#include "ceres_backend/ceres_backend_interface.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend_factory {

// 0x180158F70  upstream-modified (constants instead of ROS params, no motion detector options,
// no startThread()).
CeresBackendInterface::Ptr makeBackend(const CameraBundlePtr& camera_bundle)
{
#line 13
  VLOG(1) << "Initialize Backend.";   // ceres_backend_factory.cpp:13
  CeresBackendOptions backend_options;
  backend_options.verbose = false;
  backend_options.marginalize = true;
  backend_options.num_iterations = 5;
  backend_options.num_threads = 1;
  backend_options.num_imu_frames = 1;
  backend_options.num_keyframes = 8;
  backend_options.max_iteration_time = -1.0;
  backend_options.remove_marginalization_term_after_correction_ = true;
  backend_options.recalculate_imu_terms_after_loop = true;
  backend_options.remove_fixation_min_num_fixed_landmarks_ = 10u;

  CeresBackendInterfaceOptions ba_interface_options;
  ba_interface_options.min_num_obs = 2u;
  ba_interface_options.min_parallax_thresh = 2.0 / 180.0 * M_PI;   // 0x3FA1DF46A2529D39
  ba_interface_options.only_use_corners = false;
  ba_interface_options.use_zero_motion_detection = true;
  ba_interface_options.backend_zero_motion_check_n_frames = 5;
  ba_interface_options.use_outlier_rejection = true;
  ba_interface_options.outlier_rejection_px_threshold = 2.75;
  ba_interface_options.skip_optimization_when_tracking_bad = true;
  ba_interface_options.min_added_measurements = 5;

  // make_shared block (_Ref_count_obj2<CeresBackendInterface>), ctor 0x180008CD0
  CeresBackendInterface::Ptr ba_interface =
      std::make_shared<CeresBackendInterface>(ba_interface_options, backend_options, camera_bundle);

  return ba_interface;
}

}  // namespace ceres_backend_factory
}  // namespace totem
}  // namespace pimax
