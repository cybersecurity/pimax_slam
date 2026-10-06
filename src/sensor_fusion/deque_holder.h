// src/sensor_fusion/deque_holder.h (TODO(verify) name/path)
// Unknown class owning three common::Deque<Eigen::Vector3d>; identical layout to the
// LedObjectPoseEstimator one (dtor 0x180106640 there).
// Only its destructor survived: 0x1801A3E00, called from SLAMManager code
// (0x1801663F0 when replacing SLAMManager+2840, 0x1801671D0 = unique_ptr deleter,
// 0x180167320 = ~SLAMManager).  The object is never allocated anywhere in the binary
// (c14 says 0x200 bytes -- TODO(verify)), so the ctor/size are unknown.
//
// TODO(verify): real class name, members before +176 (all trivially
// destructible), and the gap +304..+400.  The Deque capacities are unknown.
#pragma once

#include <cstddef>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "sensor_fusion/deque.h"

namespace pimax {

class DequeHolder {  // TODO(verify): name
 public:
  // 0x1801A3E00 is the compiler-generated destructor of this layout.
  ~DequeHolder() = default;

 private:
  unsigned char pod_0_[176];                       // +0   unknown POD
  std::string name_;                               // +176
  unsigned char pod_208_[8];                       // +208 unknown POD (c17 draft lacked it; the
                                                   //      dtor 0x1801A3E00 puts the first Deque at +216)
  common::Deque<Eigen::Vector3d> deque_216_{1};  // TODO(verify): capacity unknown    // +216
  unsigned char pod_248_[24];                      // +248 unknown POD
  common::Deque<Eigen::Vector3d> deque_272_{1};  // TODO(verify): capacity unknown    // +272
  unsigned char pod_304_[96];                      // +304 unknown POD
  std::vector<Eigen::Vector3d> vec_400_;           // +400 (24-byte elements, dtor 0x1800E4150)
  common::Deque<Eigen::Vector3d> deque_424_{1};  // TODO(verify): capacity unknown    // +424
  // c14 records unique_ptr<Unknown2840> with 0x200 bytes; the members known from the dtor end at
  // 456.  TODO(verify) trailing members (no allocation of this type exists in the image).

  friend struct DequeHolderLayout;
};

struct DequeHolderLayout {
  static_assert(offsetof(DequeHolder, name_) == 176, "DequeHolder::name_");
  static_assert(offsetof(DequeHolder, deque_216_) == 216, "DequeHolder::deque_216_");
  static_assert(offsetof(DequeHolder, deque_272_) == 272, "DequeHolder::deque_272_");
  static_assert(offsetof(DequeHolder, vec_400_) == 400, "DequeHolder::vec_400_");
  static_assert(offsetof(DequeHolder, deque_424_) == 424, "DequeHolder::deque_424_");
};

}  // namespace pimax
