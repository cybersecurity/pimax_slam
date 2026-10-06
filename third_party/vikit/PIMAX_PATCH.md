# vikit — Pimax in-tree copy

Original tree in the binary: `E:\code_codex\pimax_slam\beta111_5a7902_dll\thirdparty\vikit\{vikit_cameras,vikit_common,vikit_solver}\...`
(e.g. `thirdparty\vikit\vikit_solver\include\vikit\solver\implementation/mini_least_squares_solver.hpp`).
Here the three packages are flattened into `include/vikit/...` + `src/*.cpp` (CMake globs
`third_party/vikit/*.cpp`). Base: `reference/rpg_svo_pro_open/vikit`. Objects in the image
(c18): `camera_geometry_base.obj` 0x1801B3C00, `ncamera.obj` 0x1801B3F40, `performance_monitor.obj`
0x1801B4200, `sample.obj` 0x1801B5A80, `robust_cost.obj` 0x1801B5C30 (plus the header templates
instantiated in project objects: CameraGeometry/PinholeProjection/EquidistantDistortion in the
factory TU (c14), MiniLeastSquaresSolver in pose_optimizer.obj (c12)).

## Files kept / removed

Kept: `cameras/{camera_geometry_base.h, camera_geometry.h, pinhole_projection.h,
equidistant_distortion.h, ncamera.h, implementation/*.hpp}`, `math_utils.h`, `timer.h`,
`performance_monitor.h`, `sample.h`, `solver/{mini_least_squares_solver.h, robust_cost.h,
implementation/mini_least_squares_solver.hpp}`; `src/{camera_geometry_base, ncamera,
performance_monitor, sample, robust_cost}.cpp`.

Removed (not in the image, no user in the drafts; they also needed aslam/yaml-cpp):
`cameras.h`, `cameras/{atan_distortion, camera_factory, equidistant_fisheye_geometry,
equidistant_fisheye_projection, no_distortion, omni_geometry, omni_projection,
radial_tangential_distortion}.h`, `cameras/yaml/*`, `backtrace.h`, `blender_utils.h`,
`csv_utils.h`, `homography*.h`, `path_utils.h`, `ringbuffer.h`, `user_input_thread.h`,
`vision.h` (the matcher only needed `unproject2d`, which is in math_utils.h).
In `robust_cost.{h,cpp}` only `ScaleEstimator`/`MADScaleEstimator` and
`WeightFunction`/`TukeyWeightFunction` remain (the Unit/NormalDistribution/Huber classes have no
vtable or code in the image).

## Changes (with evidence)

### cameras (c18 drafts + c14)
* `CameraGeometryBase` (sizeof 152, layout pinned by static_asserts): aslam types replaced by
  minkindr, no yaml loading, `loadMask`/`isMasked`/`createRandomKeypoint` absent;
  `backProject3(Matrix2Xd)` (0x1801B3CE0) without its two `CHECK_NOTNULL`; `setMask` (0x1801B3F30)
  without its three `CHECK_EQ`. Vtable order note: MSVC puts the `Matrix2Xd` overload in slot 1
  and the pure `Vector2d` overload in slot 2 (c11 saw "8 bytes later than upstream").
* `NCamera` (sizeof 104): second transformation vector `T_B_C_` (+24) and ctor argument;
  `get_T_B_C()` (0x1801B41E0) added; no `initInternal()` and no `CHECK_LT`s
  (0x1801B4050, 0x1801B41A0, 0x1801B41F0).
* `CameraGeometry::backProject3` (0x18015B690): no `CHECK_NOTNULL`.
* `PinholeProjection::backProject3` (inlined in 0x18015B690): if the normalised coordinates are
  exactly (0,0) the output is (0,0,1) without calling `undistort`; otherwise
  `distortion_.undistort(x, y, fx_, fy_)` (Pimax signature).
* `EquidistantDistortion` = Pimax "Fisheye62" (c14 draft, 0x18015B760 distort, 0x18015B960 jacobian,
  0x18015BD50 undistort, print in 0x180161900, getDistortionParameters 0x18015CA90): k1..k4 + p1, p2,
  constants 1e-8 / π / 1e-8 (sizeof 72); Gauss-Newton undistort (max 10 its, |step|² < 1e-16,
  skip within 4 px of the principal point, -1000 on failure). Only this distortion is
  instantiated in the image.

### math_utils.h
* `getMedian()` (0x180091700): no `assert(!data_vec.empty())` (asserts are live in this build,
  the binary has no `_wassert` there).

### timer.h
* Upstream-identical (c18). c16 claimed `Timer::stop()` (0x18002CC00) truncates the duration to
  `int`; the disassembly shows `cvtsi2sd xmm0, rax` (64-bit), i.e. `(double)int64 * 1e-9` =
  upstream. No change.

### performance_monitor (c18)
* `std::map` → `std::unordered_map` for `timers_` / `logs_` (FNV-1a hashing inlined, node sizes
  0x48/0x40); `init()`/`trace()`/`traceHeader()` do not throw when the trace file cannot be opened.

### sample.cpp
* Upstream-identical; only the two engine initialisers are in the image.

### solver (c12 drafts)
* `mini_least_squares_solver.h/.hpp`: upstream code, re-flowed so that the glog `__LINE__`s match
  the binary (GN singular 59, GN failure 65, GN success 78, GN converged 84, LM init 98,
  LM singular 150, LM success 162, LM failure 172; verified by preprocessing); the GN "close to
  singular" warning has no `H = ... g = ...` dump.
* `robust_cost.cpp`: `MADScaleEstimator::compute` without `CHECK(!errors.empty())`
  (0x1801B5C50).
