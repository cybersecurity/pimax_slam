// pimax_slam.pi.dll -- src/loop_closing/geometric_verification.cpp
// (draft c15_platmap/loop_closing/geometric_verification.cpp)
//
// Only commonLandMarkCheck of upstream svo_online_loopclosing/src/geometric_verification.cpp
// survives in the binary (the rest was removed together with loop detection or dropped by
// /OPT:REF).  TODO(verify) file split: in the image 0x180178e40 lies inside the code range that
// is otherwise used by loop_closing.cpp.
#include <vector>

#include "loop_closing/loop_closing.h"

namespace pimax {
namespace totem {

// 0x180178e40   upstream-modified
// Caller: LoopClosing::addFrameToPR 0x180185e40.
// Differences: early "return false" for an empty track_IDs1 (upstream: 0/0 = NaN ->
// returns true) and no std::isnan() term.
bool commonLandMarkCheck(const std::vector<int>& track_IDs1,
                         const std::vector<int>& track_IDs2, const double th)
{
  if (track_IDs1.empty())
  {
    return false;
  }
  int common_landmarks = 0;
  for (size_t i = 0; i < track_IDs1.size(); i++)
  {
    for (size_t j = 0; j < track_IDs2.size(); j++)
    {
      if (track_IDs1[i] == track_IDs2[j])
      {
        common_landmarks++;
        break;
      }
    }
  }
  double common_landmark_percentage = double(common_landmarks) / track_IDs1.size();
  return common_landmark_percentage < th;
}

}  // namespace totem
}  // namespace pimax
