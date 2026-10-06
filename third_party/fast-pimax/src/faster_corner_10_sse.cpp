// third_party/fast/src/faster_corner_10_sse.cpp -- uzh-rpg/fast (https://github.com/uzh-rpg/fast),
// Windows/MSVC port as found in pimax_slam.pi.dll.
//
// The other `fast` objects are upstream code and are NOT reconstructed here:
//   0x1801A8170  fast::fast_corner_detect_10   (src/fast_10.cpp, mechanically generated)
//   0x1801AB160  fast::fast_corner_score_10(const fast_byte*, const int[], int)
//                                             (src/fast_10_score.cpp, generated, static inline)
//   0x1801AF880  fast::fast_corner_score_10(img, stride, corners, threshold, scores)
//   0x1801AFA50  fast::fast_nonmax_3x3        (src/nonmax_3x3.cpp)
// Upstream's faster_corner_10_sse.cpp #errors without __SSE2__ (MSVC never defines it), so the
// DLL was built with a modified wrapper.  0x1801AFA30 (27 bytes) is:
//     if (img_width < 22 || img_height >= 7) tail-call fast_corner_detect_10(...)
//     else return;
// i.e. the SSE2 path was replaced by the plain detector.  TODO(verify) exact source form.
#include <fast/fast.h>

#include <vector>

namespace fast {

// 0x1801AFA30
void fast_corner_detect_10_sse2(const fast_byte* img, int img_width, int img_height,
                                int img_stride, short barrier, std::vector<fast_xy>& corners) {
  if (img_width < 22) {
    fast_corner_detect_10(img, img_width, img_height, img_stride, barrier, corners);
    return;
  } else if (img_width < 22 || img_height < 7)
    return;

  // SSE2 implementation not available with MSVC: fall back to the plain detector.
  fast_corner_detect_10(img, img_width, img_height, img_stride, barrier, corners);
}

}  // namespace fast
