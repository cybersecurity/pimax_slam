# fast — Pimax in-tree copy

**Layout in this repository:** `third_party/fast` is the pristine uzh-rpg/fast submodule (1153981);
CMake compiles `fast_10.cpp`, `fast_10_score.cpp`, `nonmax_3x3.cpp` from it and the modified
`faster_corner_10_sse.cpp` from this directory.


Base: uzh-rpg/fast (`reference/fast`). Objects in the image (c17): `fast_corner_detect_10`
0x1801A8170 (fast_10.cpp, generated), static `fast_corner_score_10(p, pixel[], bstart)` 0x1801AB160
and `fast_corner_score_10(img, stride, corners, threshold, scores)` 0x1801AF880 (fast_10_score.cpp),
`fast_corner_detect_10_sse2` 0x1801AFA30, `fast_nonmax_3x3` 0x1801AFA50 (nonmax_3x3.cpp).
fast_10.cpp, fast_10_score.cpp, nonmax_3x3.cpp: identical to the reference (spot-checked by c17).

## Change: src/faster_corner_10_sse.cpp

Upstream `#error`s without `__SSE2__` (MSVC never defines it). The binary's wrapper (27 bytes,
disassembly checked):

    if (img_width < 22) -> tail call fast_corner_detect_10(...)
    else if (img_height < 7) -> return
    else -> tail call fast_corner_detect_10(...)

i.e. the SSE2 detector was replaced by the plain one (c17 draft). TODO(verify) exact source form.

## Removed

`include/fast/corner_9.h` (no FAST-9 in the image), `include/fast/corner_10.h` and
`include/fast/faster_corner_utilities.h` (only used by the removed SSE2 code). `fast.h` unchanged
(still declares the unused `fast_corner_detect_9_neon`).
