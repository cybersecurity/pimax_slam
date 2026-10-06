# pimax_slam – reconstructed source

Source reconstruction of `pimax_slam.pi.dll` (`Pimax_SLAM_2.0.0.1`, x64 MSVC, sha256 `175f94ad…b7600d4118`),
the Pimax headset inside-out tracker loaded by `pi_server.exe`. It takes the four 640×480 tracking-camera
images and the HMD IMU stream and outputs the 6-DoF head pose at IMU rate.

The engine is a heavily modified fork of **SVO Pro** ([rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)):
- **Frontend:** SVO's semi-direct frontend (sparse image alignment, depth filter with seeds, corner/edgelet
  features, reprojection, pose optimizer on vikit's `MiniLeastSquaresSolver`).
- **Backend:** the OKVIS-derived Ceres fixed-lag backend (IMU errors, marginalization).
- **Pimax additions:**
  - VINS-Mono-style visual-inertial initialization (`IntegrationBase`, `IMUFactor`, `VisualIMUAlignment`);
  - ground-plane estimation (a Kimera-VIO mesher and plane code, plus a ground-plane error in the backend);
  - a tracking-health watchdog;
  - map persistence and relocalization: `PlatMap` / `KeyFrame` via boost::serialization, DBoW2 with a
    DBoW3 binary vocabulary loader, BEBLID descriptors, PnP;
  - a 3-DoF IMU fallback (Madgwick filter);
  - the `Headset*` C API.

The original tree was `E:\code_codex\pimax_slam\beta111_5a7902_dll\` with namespaces `pimax::totem`,
`pimax::totem::ceres_backend` and `pimax::ThreeDof`.

## Status

- **Code size:** about 37k lines of C++ in `src/`, plus what Pimax changed in the bundled libraries:
  - vikit (vendored in `third_party/vikit`, heavily modified);
  - Ceres, DBoW2 (+ a DBoW3-style vocabulary loader), minkindr and FAST, as small overlays
    (`third_party/*-pimax/`) on top of pristine upstream submodules.
- **Function coverage:** all 8,185 functions of the binary were classified (`notes/c*.md`;
  `tools/classify.py` regenerates the raw per-function table from the IDA dumps).
  - 703 of the 709 functions marked `project` in the chunk tables carry their `// 0x1800XXXXX` address comment
    in the source.
  - The six without one are compiler-generated destructors, serialize templates that live in headers, a
    library deque helper and an inlined `Matcher` construction.
- **Library code** is not reconstructed: STL, Eigen, Ceres internals, boost, OpenCV inline helpers, the DBoW2
  templates, glog/gflags, GLEW and zlib. It comes from the pinned dependencies below.
- **Behaviour:** the rebuilt DLL matches the original in every differential scenario tried (see
  [Verification](#verification)).
- **Layouts:** class layouts are pinned with `static_assert`s on size and field offsets (MSVC x64), e.g.:

  | Class | Size |
  |---|---|
  | `FrameProcessorBase` | 4032 |
  | `FrameProcessor` | 4080 |
  | `Frame` | 0x340 |
  | `FrameBundle` | 0x100 |
  | `Estimator` | 0x300 |
  | `ceres_backend::Map` | 0x580 |
  | `ImuError` | 0x18B0 |
  | `CeresBackendInterface` | 0x4F0 |
  | `IntegrationBase` | 10576 |
  | `LoopClosing` | 0xC10 |
  | `SLAMManager` | 0xB20 |
  | `Matcher` | 352 |

## Build

The DLL links the C++ APIs of the prebuilt OpenCV 4.5.3 DLLs that ship with the Pimax runtime, so it must be
built with the MSVC ABI.

Dependencies are git submodules pinned to the exact upstream versions (shallow):

```sh
git clone https://github.com/cybersecurity/pimax_slam && cd pimax_slam
git submodule update --init --depth 1

# Windows (Visual Studio 2019/2022, x64)
cmake -S . -B build -A x64 && cmake --build build --config Release

# Linux cross build: clang-cl + lld-link + an xwin-splatted MSVC CRT/Windows SDK (~/.xwin)
xwin --accept-license --arch x86_64 splat --output ~/.xwin
cmake -S . -B build-win -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/clang-cl-xwin.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-win          # -> build-win/pimax_slam.pi.dll
```

**Run-time dependencies:** `opencv_{core,imgproc,calib3d,features2d,imgcodecs,highgui,aruco}453.dll` from the
Pimax runtime.

**Linked statically, as in the original:** Ceres, glog/gflags, boost, tinyxml2, DBoW2. The OpenCV import
libraries are generated at build time from `third_party/implib/*.def` (the DLLs' export tables) with
`lib.exe` / `llvm-lib`; only the OpenCV headers are used from the submodules.

**Build flags:**
- As in the original: optimized, with **asserts enabled** (Eigen asserts call `_wassert`).
- The original used MSVC's default `/std:c++14`, this build uses C++17.

**Exports:** `src/pimax_slam.def` reproduces the ten `Headset*` exports with their original ordinals (323–332).

**Test-only CMake options** (all off by default):

| Option | Effect |
|---|---|
| `PIMAX_SLAM_TEST_DETERMINISTIC` | no wall-clock solver budget, fixed shuffle seed |
| `PIMAX_SLAM_TEST_LOG_LEVEL=N` | initial level of the printf logger |
| `PIMAX_SLAM_DEBUG_INFO` | PDB output, for symbolized wine backtraces |

`tools/cc.sh <file>` compiles single translation units with the DLL's exact flags.

## Dependencies (pinned to what the binary was built with)

| Library | Version | How it was identified |
|---|---|---|
| Ceres Solver | 2.1.0, static, **with Pimax patches** (submodule + `third_party/ceres-pimax`) | `Solver::VersionString()` in the binary: `2.1.0-eigen-(3.4.0)-no_lapack-eigensparse-no_openmp`. MINIGLOG with `MAX_LOG_LEVEL=2`. RTTI shows all Schur specializations. The patches are listed in `third_party/ceres-pimax/PIMAX_PATCH.md`: minimal-Jacobian parameter blocks in `parameter_block.h` / `residual_block*.cc`, and removed log sites. `tools/ceres_line_check.py` compares every miniglog `__LINE__`. |
| Eigen | 3.4.0 (submodule) | Ceres version string; 144/148 assert line numbers match 3.4.0; 3.4-style tridiagonal deflation test. The assert paths point to a basalt-headers copy inside the LedObjectTracking tree. |
| OpenCV | 4.5.3 (DLLs; headers from the `opencv` / `opencv_contrib` submodules, generated config headers in `third_party/opencv-config`) | `opencv_*453.dll` imports, `E:\software_x86\opencv-4.5.3\…` asserts |
| glog / gflags | 0.5.0 / 2.2.2, static (submodules) | `E:\software_x86\glog-0.5.0`, `gflags-2.2.2` source paths |
| Boost | 1.74 (serialization, filesystem), static; `third_party/boost` is the used subset | `C:\Boost\include\boost-1_74\…` asserts, archive library version 18 |
| tinyxml2 | 9.0.0 (submodule) | `XMLDocument` layout (`_parsingDepth`, nesting limit 100), identical function bodies |
| DBoW2 | dorian3d/DBoW2 3924753 (submodule) + DBoW3 binary loader (`third_party/DBoW2-pimax`) + QuickLZ 1.5 level 1 (DBow3 submodule) | `third_party/DBoW2-pimax/PIMAX_PATCH.md`; `voc_GEN_8X4.dbow` is the DBoW3 binary format |
| vikit, minkindr | vikit: vendored Pimax-modified copy; minkindr: submodule 564f126 + `third_party/minkindr-pimax` (see `PIMAX_PATCH.md` in each) | vikit: `NCamera` gains `T_B_C_`, Fisheye62 distortion, `unordered_map` PerformanceMonitor. minkindr renormalizes after `operator*` / `inverse()`. |
| FAST | uzh-rpg/fast 1153981 (submodule) with an MSVC (non-SSE) wrapper | `third_party/fast-pimax` |
| GLEW 1.12, zlib 1.2.8, Pangolin statics | dead code in the original, not rebuilt | see Known deviations |

## API

`include/pimax_slam.h` documents the exported C interface. Every function returns −1 while no system exists.

| Export | Notes |
|---|---|
| `HeadsetInitialImpl(calib, out_dir, voc, unused, loc_mode)` | `calib` = `device_calibration.xml` (tinyxml2, Fisheye62 cameras, `SFConfig/Stateinit` extrinsics). The device type comes from the `deviceUID` prefix. `out_dir` receives the log files, `pimax_prior_position.txt` and the PlatMap. `voc` = `voc_GEN_8X4.dbow`. `loc_mode` is 0 kNormal, 1 kLabMap or 2 kLabLoc. |
| `HeadsetPutCameraImage(img0..img3)` | 40-byte headers: ns timestamp, shutter, gain, width/height/stride at +20/+24/+28, data at +32. Images must be 640×480 and the first 5 calls are dropped. Frames are queued for the `SLAMManager_Thread`. |
| `HeadsetPutHmdImuData(imu, pose)` | 40-byte IMU record in, 104-byte pose out: IMU-rate prediction from the last vision state, or the 3-DoF fallback |
| `HeadsetGetGroundState`, `HeadsetGetLocModeState`, `HeadsetSetTrackingMode`, `HeadsetSetDeviceEnable`, `HeadsetLeftImageNumber`, `HeadsetReleaseImpl`, `HeadsetGetVersion` | |

## Layout

| Path | Contents |
|---|---|
| `src/interface/` | the C API (`headset_api.cpp`). `SLAMManager` (0xB20): image queue and processing thread, IMU-rate pose prediction, 3-DoF fallback. Option factories and the calibration parser (`svo_factory.cpp`). `ceres_backend_factory.cpp`. |
| `src/frontend/` | `FrameProcessorBase` / `FrameProcessor`. Per-file roles: |
| | `frame_processor_base.cpp`: frame bundles, motion priors, watchdog, resets, map projection, structure optimization. |
| | `_imu_init.cpp`: VI initialization. |
| | `_ground.cpp`: ground planes and mesh. |
| | `ImuProcessor`, `StereoInit`, frontend `Map`, `PoseOptimizer`, `Reprojector`, `StereoTriangulation`, `VisualIMUAlignment`, `IntegrationBase` / `IMUFactor`. |
| `src/ceres_backend/` | `CeresBackendInterface`, `Estimator`, `ceres_backend::Map`, `ImuError`, `MarginalizationError`. The pose / speed-and-bias / gravity / 3-D point blocks, errors, local parameterizations and `GroundPlaneError`. |
| `src/direct/` | `Matcher`, `DepthFilter`, feature detection / alignment, patch warp |
| `src/common/` | `Frame`/`FrameBundle`, `Point`, seeds, feature wrappers, the printf `Logger` (`LOGD/LOGI/LOGW/LOGE`), Sophus-style SO(3) helpers, the portable binary archive |
| `src/loop_closing/` | `LoopClosing` (relocalization pipeline `ReLoc-1..5`, worker threads), `PlatMap`/`KeyFrame` persistence, BoW feature extraction, BEBLID, `MapAlignmentSE3` |
| `src/plane/`, `src/sensor_fusion/`, `src/tracker/` | Kimera-derived mesher / histogram / mesh; 3-DoF `ImuFilter`, `GyroscopeBiasEstimator`, `common::Deque`; svo `FeatureTracker` |
| `include/pimax_slam.h` | the exported C API |
| `notes/c00..c18_*.md` | per-chunk analysis: function tables for every function (project or library, upstream status), type layouts, externals, constants, quirks. The source was first drafted per chunk and then integrated; paths of the form `draft/<chunk>/...` in the notes refer to those (removed) drafts. |
| `notes/integration_*.md`, `notes/verification.md` | how the chunk drafts were merged (conflicts resolved, with evidence), and the differential-testing log |
| `third_party/` | submodules (pristine upstream at the pinned versions), the `*-pimax` overlays with Pimax's changes, the vendored vikit, the used Boost 1.74 subset, `ceres-config`, `opencv-config`, `implib/*.def` |
| `tools/` | `ida_dump.py` (query tool for the dumps), `classify.py`, `cc.sh`, `ceres_line_check.py`, `parse_platmap.py`, `sym.py`, `numdiff.py` |
| `re_dump/` | headless IDA dump scripts (`dump.py`, `dumpasm.py`, `vtables.py`); the dumps themselves are not in the repo |
| `test/` | differential test harness (below) |

The IDA database and the dumps (`re_dump/all.c`, `all.asm`, `funcs.json`, `strings.txt`, `vtables.json`,
`names.json`) are derived from the original binary and are not committed. Regenerate them with IDA 9.x
(Hex-Rays) from a copy of `pimax_slam.pi.dll` (sha256 above) placed at `re_dump/orig.dll`:

```sh
cd re_dump
idat -A -Sdump.py -oslam.i64 orig.dll     # all.c, funcs.json, strings.txt
idat -A -Sdumpasm.py slam.i64             # all.asm
idat -A -Svtables.py slam.i64             # vtables.json, names.json
cd .. && python3 tools/ida_dump.py fn 0x180162390      # HeadsetInitialImpl
```

`reference/` (not committed) held the upstream checkouts used for the comparison:
- rpg_svo_pro_open, rpg_vikit, minkindr, DBoW2, DBow3
- Kimera-VIO, uzh-rpg/fast, imu_tools
- ceres-solver 2.1.0, Boost 1.74.0, OpenCV / opencv_contrib 4.5.3

## Verification

**Test scenes.** `test/gen_scenario.py` renders a textured room into the four calibrated fisheye cameras
(`test/data/device_calibration.xml`). This is the shipped calibration with a Crystal Light `deviceUID`. Each
scene follows a known head trajectory and has a matching 1 kHz IMU stream. `test/trace_edit.py` derives variants:
- camera blackouts and frozen images;
- IMU gaps;
- tracking-mode / device-enable switches.

**Running a trace.** `test/replay.c` drives either DLL through the C API under wine and records every
`HeadsetPutHmdImuData` pose and per-frame state. `test/ab_log_diff.sh` / `test/ab_all.sh` run both DLLs on a
trace and compare:
- the info-level logs;
- stdout/stderr;
- every pose record;
- the trajectory error against ground truth (ATE).

**Determinism.** The original is not run-to-run deterministic, for three reasons:
- Ceres gets a 25 ms wall-clock budget per backend solve.
- Stereo triangulation seeds `std::shuffle` with `std::random_device`.
- The per-camera reprojection threads race.

`test/make_deterministic_orig.py` therefore makes a test copy with the time limit lifted and a fixed seed
(optionally also a lower logger level). The rebuilt test build does the same.

**Debugging aid.** `test/gdb_probe.sh` dumps internal state of either DLL at any function (winedbg `--gdb` +
python probes in `test/probes/`). That is how the bugs listed in `notes/verification.md` were located.

**Results.** For each trace, a second run of the original measures its own determinism (the "orig vs orig" column).

| Trace | Info log | Stdout/stderr | Bit-exact pose prefix, orig vs new | Orig vs orig | Max \|dp\| orig vs new | ATE RMSE orig / new |
|---|---|---|---|---|---|---|
| 4 s, look around | identical | identical | 3272 | 3272 | 2.1 mm | 17.1 / 17.4 mm |
| 6 s, still | identical | identical | all 6472 | all | 0 | 0 / 0 |
| 8 s | identical | identical | 3272 | 2406 | 2.6 mm | 15.9 / 15.6 mm |
| 8 s, 2× speed | identical | identical | 3206 | 3206 | 0.6 mm | 5.5 / 5.5 mm |
| 8 s, IMU noise | identical | identical | 5006 | 4806–5906 | 0.7 mm | 5.3 / 5.4 mm |
| 6 s, 4× speed | identical | identical | 2206 | 2206 | 0.4 mm | 6.4 / 6.5 mm |
| 6 s, no still start | identical | identical | 2639 | 2639 | 0.16 mm | 3.0 / 3.0 mm |
| 20 s | identical | identical | 3206 | 3206 | 3.1 mm | 4.7 / 4.9 mm |
| 20 s, cameras blacked out | values differ after the prefix, as orig vs orig | identical | 3206 | 3206 | 1.7 mm | 71.2 / 71.3 mm |
| 20 s, 3-DoF / enable switches | identical | identical | 3206 | 3206 | 0.02 mm | 133 / 133 mm |

The rebuilt DLL agrees with the original exactly as far as the original agrees with itself. After that the
differences stay within the original's own run-to-run spread. Localization mode (`loc_mode` 1) and a run
with a pre-seeded map directory also matched.

**Not covered by the synthetic traces:**
- PlatMap saving (it needs 200 loop-closing keyframes);
- relocalization against a stored map;
- the lab/tag modes.

These paths are reconstructed and compile, but have not been exercised. `tools/parse_platmap.py`
documents and parses the map format.

```sh
python3 test/gen_scenario.py --slam ../slam --out trace.bin --truth truth.npy --seconds 8
cmake -S . -B build-win-det -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/clang-cl-xwin.cmake -DCMAKE_BUILD_TYPE=Release \
      -DPIMAX_SLAM_TEST_DETERMINISTIC=ON -DPIMAX_SLAM_TEST_LOG_LEVEL=2 && cmake --build build-win-det
RUNTIME=<Pimax runtime dir> test/ab_all.sh trace.bin
```

## Quirks preserved from the original (selection; full lists in `notes/*.md`)

**API and threading**
- `HeadsetPutCameraImage` drops the first 5 calls of the process and reports wrong-size images as "cameraN
  empty image". It warns, but still uses the frame, when shutter < 100 µs.
- `SetDeviceEnable(false)` and `SetTrackingMode(kDof3)` stop the vision thread for good; re-enabling only
  clears the flag.
- The pose output's padding bytes are an uninitialized stack copy.

**Backend**
- `Estimator::optimize` gives Ceres a fixed 25 ms budget.
- Landmarks are frozen after 10 calls, based on parallax, observation count or inlier ratio.
- Points whose depth jumped by more than 1 m are dropped.
- The ground-plane residual is added and removed around every solve.
- The reprojection loss is `HuberLoss(0.5)`, not upstream's Cauchy loss.
- Pimax's "minimal Jacobian" hack makes the local parameterizations report `GlobalSize() == 0`, which needed
  three Ceres patches.
- `ImuError::propagation` reads timestamps from its argument but samples from its own buffer. Its saturation
  warnings never fire.
- `IMUFactor::Evaluate` overwrites the global gravity vector `G` on every call and throws away the results of
  its `normalized()` calls.

**IMU initialization**
- `initializeImu` overwrites the `IntegrationBase` noise parameters after construction, so the defaults are
  used.
- Its replay loop runs exactly once.
- Translations of its BA blocks are rounded through `float`.

**Relocalization**
- No DBoW2 score threshold is applied.
- The "spatial grouping" stage is empty.
- Nine `PIMAX_LC_*` environment variables are re-read on every call.
- `getReLocCorrection`'s averaging loop never advances its iterator.

**Map persistence**
- `PlatMap::save` writes a `-bk` temporary but the stale-backup check looks for `.bk`.
- The portable `.pba` loader exists but nothing writes `.pba` files.

**Depth filter and matching**
- `DepthFilter::updateSeed` starts from depth 100 and marks every failed match as an outlier.
- It uses the first camera's mask for all cameras.
- The edgelet non-max suppression races inside `cv::parallel_for_`.
- `align1D` uses a diagonal-pair gradient kernel.

**Reprojector**
- `matchCandidates`' rotation-histogram filter keeps the three *least* populated bins.

## Known deviations

- Built with clang-cl and C++17 instead of MSVC `/std:c++14`:
  - Logic and control flow match.
  - Floating-point results may differ in the last bits.
  - In a few glog `<<` chains, side-effect-free lookups are evaluated in a different order.
- `__FILE__` strings in assert/CHECK messages differ (different source paths). The glog / miniglog
  `__LINE__`s are reproduced with `#line` where needed.
- The original's export table also lists 322 boost::serialization singletons and the GLEW symbols, which came
  from dllexport-ed static libraries. Only the 10 `Headset*` exports are reproduced; `pi_server.exe` imports
  by name.
- GLEW 1.12, zlib 1.2.8 and Pangolin's static `Handler` objects are linked into the original but never called.
  They are not part of this build.
- `PlatMap::save_bin` (portable archive) was dead-stripped in the original; its body here is a reconstruction
  of the obvious counterpart of `load_bin`.

## License

GPLv3 (see `LICENSE`). The engine is a reconstruction of a fork of SVO Pro (rpg_svo_pro_open, GPLv3), and the
code in `src/` is derived from it and from the other upstream projects listed under Dependencies.
Third-party code keeps its own license: the vendored vikit (GPLv3, `third_party/vikit/LICENSE`), the Boost
subset (Boost Software License, `third_party/boost/LICENSE_1_0.txt`), the `*-pimax` overlays (the licenses of
the respective submodules: Ceres BSD-3-Clause, DBoW2 BSD-style, minkindr BSD-3-Clause, FAST BSD), and the
submodules themselves.

This repository contains no files from the Pimax software: the original DLL, its calibration, vocabulary
and map files and the IDA databases are not included and must be supplied from a Pimax installation to run
the tests.
