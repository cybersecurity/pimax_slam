// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

// pimax_slam.pi.dll -- src/direct/patch_utils.h
// upstream (svo_direct/include/svo/direct/patch_utils.h), namespace pimax::totem.
// createPatchFromPatchWithBorder emitted out of line at 0x1800AE5A0 (matcher.obj), identical.
// The debug helpers patchToMat / normalizeAndUpsamplePatch / concatenatePatches are not in the
// image (never called) and were dropped.
#pragma once

#include "common/types.h"

namespace pimax {
namespace totem {
namespace patch_utils {

inline void createPatchFromPatchWithBorder(
    const uint8_t* const patch_with_border,
    const int patch_size,
    uint8_t* patch)
{
  uint8_t* patch_ptr = patch;
  for(int y=1; y<patch_size+1; ++y, patch_ptr += patch_size)
  {
    const uint8_t* ref_patch_border_ptr = patch_with_border + y*(patch_size+2) + 1;
    for(int x=0; x<patch_size; ++x)
      patch_ptr[x] = ref_patch_border_ptr[x];
  }
}

} // namespace patch_utils
} // namespace totem
} // namespace pimax
