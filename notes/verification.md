# pimax_slam.pi.dll -- differential verification (phase 3)

The rebuilt DLL (`build-win-det`, `-DPIMAX_SLAM_TEST_DETERMINISTIC=ON -DPIMAX_SLAM_TEST_LOG_LEVEL=2`) is run
side by side with the deterministic test copy of the original (`test/make_deterministic_orig.py`) on synthetic
traces (`test/gen_scenario.py`) under wine, and the info-level logs and the per-IMU-record pose outputs are
compared (`test/ab_log_diff.sh`, `test/compare.py`, `test/firstdiff.py`, `test/truth_err.py`).

## Tooling added in this phase

* `test/gdb_probe.sh <dll> <trace> <workdir> <probe.py>` + `test/gdb_probe_lib.py` + `test/replay_dbg.c`:
  runs a DLL under `winedbg --gdb` and executes a gdb-python probe script. The replayer variant executes
  `int3` right after `LoadLibrary` (env `PS_BREAK`) so that the probe can read the load address (`rbx`) and
  plant breakpoints at relocated image VAs (`probe(va, cb)`, `probe_call(va, on_entry, on_exit)` for
  entry + return, helpers to read registers / MS-x64 arguments / memory / MSVC `std::map` / `std::vector` /
  Eigen dynamic vectors). This makes it possible to dump intermediate state of the *original* DLL at any
  function (all differences below were found this way: same probe on both DLLs, `tools/numdiff.py` on the
  two outputs). Probes for the rebuilt DLL get their addresses from the PDB with `tools/sym.py <substring>`
  (the probe scripts take them from env variables, e.g. `FN=$(python3 tools/sym.py "?optimize@Estimator@totem" | cut -d" " -f1)`).
  The probe scripts used for this phase are kept in `test/probes/` (original addresses as defaults).
  Breakpoints that fire very often (e.g. `ceres::internal::ResidualBlock::Evaluate`) should be created
  disabled and only enabled inside the call of interest (see `test/probes/rb.py`), otherwise the
  replay becomes very slow.
* `tools/numdiff.py a b`: line-by-line numeric diff (max relative difference per line).
* `test/poses.py`: the `HeadsetPose` dtype was missing the 4 padding bytes at +76 (timestamp is at +80), so
  `pts`, `state`, `flag`, `conf`, `six` were read 4 bytes off. Fixed; `test/compare.py` now compares the
  meaningful fields only: the padding bytes (+76..79, +90..91, +97..103) come from an uninitialised stack
  temporary (the RVO slot of `SLAMManager::PredictPose` 0x18016B940, copied as a whole into the TLS pose
  in `PutImu` 0x180168640) and differ between the two DLLs for no behavioural reason.
* `test/firstdiff.py a b`: first differing record per pose field.
* `test/truth_err.py truth.npy poses.bin...`: ATE (rigid Umeyama alignment) of the 6DoF outputs at frame times.
* `test/ab_log_diff.sh`: also drops `total time` lines (`vi-estimator total time %.2f ms` is wall-clock).

Example probe (argument dump at the entry of `ImuInitializer::VisualIMUAlignment`, original address; the
same file with the address from `tools/sym.py VisualIMUAlignment` runs on the rebuilt DLL):

```python
def via():
    m = arg(1)                                   # std::map<double, ImageFrame>&
    for v in msvc_map_nodes(m):                  # key at v, ImageFrame at v + 16
        f = v + 16
        out("t=%.17g R=%s T=%s" % (f64(v), fmt(f64(f + 24, 9)), fmt(f64(f + 96, 3))))
    return False
probe(0x180157240, via)
```

## Bugs fixed

### 1. Ceres: "minimal Jacobian" parameter blocks evaluated with the ambient size (third_party/ceres-pimax)

* Symptom: the visual BA of `FrameProcessorBase::initializeImu` (phase B) did nothing (poses moved by 1e-9
  instead of ~1e-4), every ReprojectionError evaluation failed with Ceres' "Error in evaluating the
  ResidualBlock" warning (2x7 Jacobian with 2 entries "Uninitialized"); downstream `gyroscope_bias=0`,
  `estimated_scale=-60159213`.
* Root cause: Pimax's local parameterizations report `GlobalSize() == 0` while their "minimal Jacobian"
  flag is set, and the cost functions then write a `num_residuals x TangentSize()` Jacobian. Pimax patched
  Ceres' `residual_block.cc` / `residual_block_utils.cc` for this, the integration only had the
  `parameter_block.h` part.
* Binary evidence: `ResidualBlock::Evaluate` 0x1801FB7D0 uses the scratch (ambient) Jacobian and the
  plus-Jacobian multiplication only when `jacobians[i] && plus_jacobian_ && manifold_->AmbientSize()`;
  `InvalidateEvaluation` 0x180219340 and `IsEvaluationValid` 0x180219410 use
  `(manifold == nullptr || manifold->AmbientSize()) ? size_ : manifold->TangentSize()` as the per-block
  Jacobian width (`EvaluationToString` 0x180218B80 and `NumScratchDoublesForEvaluate` 0x1801FC580 are
  unchanged).
* Files: `third_party/ceres-pimax/internal/ceres/residual_block.cc`, `residual_block_utils.cc`,
  `third_party/ceres-pimax/PIMAX_PATCH.md` (items 4, 5).

### 2. initializeImu phase B: translations of the BA parameter blocks go through float

* Symptom (after 1.): `INFO=ImuInitialRefine||estimated_scale` differed in the 7th digit; the ReprojectionError
  inputs of the visual BA differed in the extrinsic/pose translations (e.g. 0.10810510814189911 vs
  0.10810510525698519 = float rounding).
* Root cause / evidence: 0x180111AC9..0x180111AF0 (`cvtsd2ss`, `cvtsd2ss`, `cvtpd2ps`) for the camera
  extrinsics `T_body_cam_` and 0x180111DCD..0x180111DF5 (3x `cvtpd2ps`) for the window poses
  `T_world_imu()`: the translation is a `Vector3f` local that is later stored into the `double` parameter
  arrays; the quaternions stay double.
* File: `src/frontend/frame_processor_base_imu_init.cpp`.

### 3. Estimator::addStates: extrinsics parameter blocks use `NCamera::get_T_B_C`

* Symptom (after 1.+2. the IMU initialisation matched): every backend reprojection residual was ~100 px,
  the first `Estimator::optimize` removed almost all landmarks (`in_ba_graph_` cleared), the frontend then
  printed `Too few visual measurements, skip optimization once.` every frame.
* Root cause: source used `camera_rig_->get_T_C_B(i)` "without .inverse()" for `T_S_C`.
* Evidence: `Estimator::addStates` 0x180026100 calls 0x1801B41E0 (`return *(a1+24) + (i << 6)` = the
  Pimax-added second vector `T_B_C_`), not 0x1801B41F0 (`get_T_C_B`, +0). Probed: the extrinsic block of
  the original holds T_imu_cam (= T_B_C).
* File: `src/ceres_backend/estimator.cpp`.

### 4. Estimator::addObservation: reprojection residuals use the Huber loss

* Symptom (after 3.): logs identical, poses bit-exact up to t=11.206 s, then small differences; the first
  backend solve already had a different initial cost (1e-10 relative): per-residual costs differed by up to
  1e-3 relative although the raw residuals were bit-identical.
* Root cause: source passed `cauchy_loss_function_ptr_` (upstream). The original's cost was exactly
  `0.5*|r|^2` (Huber region) and the residuals were not rescaled by the Corrector.
* Evidence: `addObservation` 0x180010C20 loads the loss as `a1[73]` = Estimator+0x248; the ctor 0x180022EF0
  stores the `CauchyLoss` at +0x238 (568) and `HuberLoss(0.5)` at +0x248 (584), `HuberLoss(1.5)` at +0x258.
* File: `src/ceres_backend/estimator_impl.hpp`.

### 5. checkImuMotion: the stationarity window is filled oldest-sample-first

* Symptom (trace with IMU noise, `t8n`): `INFO=ImuInitial||...gyroscope_bias` differed. The initial gyro bias
  (`ImuProcessor::omega_bias_` = mean of the gyro window while static) differed from the first frame on.
* Root cause: `imu_window_.insert(end, measurements.begin(), measurements.end())`; the bundle's
  `ImuMeasurements` deque is newest-first, the original appends it reversed, so the `size() > 500` trimming
  pops different samples.
* Evidence: `checkImuMotion` 0x1800FE320 passes `first = {container, off + size}`, `last = {container, off}`
  to the deque range-insert 0x1800D0A40 (reverse iterators); probed window contents: original ascending in
  time (9.7035 ... 10.2005), rebuilt descending per bundle. The running sums are still accumulated in
  forward (newest-first) order, as in the source.
* File: `src/frontend/frame_processor_base.cpp`.

### 6. Ceres: log sites that do not exist in the binary

* Symptom: the rebuilt DLL wrote ~1400 miniglog lines per 8 s run to stderr (`callbacks.cc:125` iteration
  table of the two IMU-initialisation solves, `trust_region_minimizer.cc:659/743 Terminating: ...`,
  `detect_structure.cc:64/75/94 Dynamic ... block size`, `schur_eliminator.cc:156 Template specializations not
  found`); the original writes none (probed: its miniglog `LogMessage` dtor 0x1801B8DD0 is never called on the
  test traces).
* Evidence: `LoggingCallback::operator()` 0x18020E490 only has the `std::cout` branch;
  `MaxSolverIterationsReached` 0x180209900 has no "Terminating" VLOG (the GradientToleranceReached one,
  line 679 = 0x2A7, is there); the "Terminating: " references in `TrustRegionMinimizer::Minimize`
  0x18020B4B0 are lines 71, 93, 639, 698, 721 (no 743); the strings "Dynamic row/e/f block size ..." and the
  file name `schur_eliminator.cc` are not in the binary.
* Files: `third_party/ceres-pimax/internal/ceres/{callbacks.cc, trust_region_minimizer.cc, detect_structure.cc,
  schur_eliminator.cc}` (line numbers of all remaining sites unchanged), PIMAX_PATCH.md item 6.
* `test/ab_log_diff.sh` now also compares the replayer's stdout/stderr.

## Notes on the original's own nondeterminism

The deterministic test copy of the original is still not run-to-run deterministic after a few seconds of
motion (thread timing). On `t8n` three runs of the original agree with each other only up to records 4806 /
5906 (pairwise), the rebuilt DLL agrees with them up to 4806/5006, i.e. inside the original's own spread.
While probing (`addObservation` measurements) the original produced a 1-ulp different projection for
identical inputs in one run and the same value as the rebuilt DLL in another run, so the divergence source
is not in the code paths compared here.

## Additional test tooling (traces / runs)

* `test/trace_edit.py in out [--blackout T0:T1] [--freeze T0:T1] [--drop-imu T0:T1] [--event T:M:<mode>|T:E:<0|1>]
  [--settle-ms N]`: derive traces that cover tracking loss (covered cameras), frozen images, IMU gaps and the
  `HeadsetSetTrackingMode` / `HeadsetSetDeviceEnable` API.
* `test/run_one.sh`: `PRESEED=<dir>` copies files into the output directory before the run (map loading tests).
* `test/ab_log_diff.sh`: `LOC=<loc_mode>`, `LOGLEVEL=1 NEW_DLL=<dll built with -DPIMAX_SLAM_TEST_LOG_LEVEL=1>`
  for debug-level log comparison (`build-win-det-l1/` is configured that way); embedded time stamps (a log
  line without `\n` followed by the next line's header) are normalised.
* `test/ab_all.sh <trace>...`: original + rebuilt + second original run per trace and a summary
  (log identity, bit-exact pose prefix, max/mean |dp|, ATE of all three); `test/logstats.sh`: counters of a
  run's 6DOF log (IMU-init attempts/successes, resets, loop-closing resets).
* `test/kill_probes.sh`: kills leftover winedbg/gdb probe processes (do not use `pkill -f` patterns that
  match the calling shell's own command line).
* gdb probes: return breakpoints are shared per return address (one `gdb.Breakpoint` per call site,
  keyed by rsp), otherwise thousands of breakpoints accumulate and the replay crawls. Breakpoints in code
  that runs concurrently on several threads (e.g. `reprojector_utils::matchCandidates`, executed per camera
  in parallel) can trigger a gdb internal error (`finish_step_over: Assertion ... trap_expected`); probe
  such code from a single-threaded caller instead.

## Known nondeterminism of the original (not reproduced exactly by design)

* Run-to-run: the deterministic test copy of the original agrees with itself only up to a trace-dependent
  record (2206 ... 5906 on the traces below); the rebuilt DLL always agreed bit-exactly with the original
  up to that horizon.
* `debug]ave_success_num` (LOGD, `FrameProcessorBase` `sum_lm_succeeded_reproj / n_matches`) differs
  between two runs of the *original* from the first frames on: `matchCandidates` runs per camera on
  parallel threads and increments/reads the shared `Point::n_succeeded_reproj_` (data race). All other
  debug-level lines (`LOGLEVEL=1`, traces t4s, t8f, t8n, t20) are identical up to the pose horizon except
  for thread interleaving of lines (DepthFilter thread) and, once in t8f, a tracking_result 399/408 vs
  400/409 that two further runs of the rebuilt DLL did not reproduce (both 400/409 as the original).
* Blackout trace (t20b): whether the second IMU initialisation after the tracking loss succeeds varies
  between runs of the original itself (2 successes in one run, 1 in another); the rebuilt DLL shows the
  same spread.

## Results per trace

Traces (all `--settle-ms 150`, `test/gen_scenario.py`; truth `build-test/<name>_truth.npy`):

| trace | generation | content |
|---|---|---|
| t4s | (given) 4 s, speed 1, 1 s still start, no IMU noise | |
| t6still | `--seconds 6 --still --seed 3` | no motion at all |
| t8 | `--seconds 8` | |
| t8f | `--seconds 8 --speed 2 --seed 2` | faster motion |
| t8n | `--seconds 8 --still-start 3 --seed 4 --gyro-noise 0.002 --acc-noise 0.02` | IMU noise, long still start |
| t6x | `--seconds 6 --speed 4 --seed 6 --gyro-noise 0.002 --acc-noise 0.02` | very fast motion |
| t6m | `--seconds 6 --speed 1.5 --still-start 0 --seed 7 --gyro-noise 0.002 --acc-noise 0.02` | moving from the first frame (IMU init during motion) |
| t20 | `--seconds 20 --speed 1.5 --seed 5 --gyro-noise 0.002 --acc-noise 0.02` | |
| t20b | `trace_edit.py t20.bin t20b.bin --blackout 15.0:15.7` | cameras covered -> tracking lost, reset, re-init |
| t20m | `trace_edit.py t20.bin t20m.bin --event 16.0:M:0 --event 17.0:M:1 --event 18.0:E:0 --event 18.5:E:1` | 3DoF mode switch (stops vision for good), device disable/enable |

Final A/B (`test/ab_all.sh`, final source; "orig vs orig" = second run of the original; ATE = rigid-aligned
RMSE / max of the 6DoF outputs at frame times, original / rebuilt / original 2nd run):

| trace | info log | stdout/stderr | bit-exact prefix orig vs new | orig vs orig | max / mean \|dp\| orig vs new | max / mean \|dp\| orig vs orig | ATE rmse [mm] orig / new / orig2 |
|---|---|---|---|---|---|---|---|
| t4s | identical (172) | identical | 3272/4472 (12.772 s) | 3272 | 2.14 / 0.20 mm | 2.10 / 0.19 mm | 17.1 / 17.4 / 17.4 |
| t6still | identical (232) | identical | 6472/6472 (all) | all | 0 / 0 | 0 / 0 | 0 / 0 / 0 |
| t8 | identical (292) | identical | 3272/8472 (12.772 s) | 2406 | 2.62 / 0.60 mm | 4.66 / 0.89 mm | 15.9 / 15.6 / 15.7 |
| t8f | identical (292) | identical | 3206/8472 (12.706 s) | 3206 | 0.62 / 0.14 mm | 0.69 / 0.15 mm | 5.5 / 5.5 / 5.5 |
| t8n | identical (291) | identical | 5006/8472 (14.506 s) | 5906 (other runs: 4806) | 0.71 / 0.12 mm | 1.18 / 0.18 mm | 5.3 / 5.4 / 5.4 |
| t6x | identical (231) | identical | 2206/6472 (11.706 s) | 2206 | 0.43 / 0.11 mm | 0.90 / 0.17 mm | 6.4 / 6.5 / 6.3 |
| t6m | identical (259) | identical | 2639/6472 (12.139 s) | 2639 | 0.16 / 0.04 mm | 0.17 / 0.04 mm | 3.0 / 3.0 / 3.0 |
| t20 | identical (650) | identical | 3206/20472 (12.706 s) | 3206 | 3.12 / 0.52 mm | 2.44 / 0.21 mm | 4.7 / 4.9 / 4.8 |
| t20b | same length (1242), numbers differ after the horizon (as orig vs orig) | identical | 3206/20472 (12.706 s) | 3206 | 1.71 / 0.12 mm | 0.86 / 0.07 mm | 71.2 / 71.3 / 71.3 |
| t20m | identical (232) | identical | 4706/20472 (first diff 3206, identical again after the switch to 3DoF) | same | 0.02 / 0.004 mm | 0.99 / 0.54 mm | 133.1 / 133.1 / 133.1 |

Also checked (8 s trace t8): `LOC=1` (localisation mode): logs identical, prefix 3272; with a pre-seeded real
`pimax_database.bin` (`PRESEED`): logs identical, prefix 2406 = original's own horizon in that setting (no
`platMap_*.yaml` index for the synthetic device, so no map is actually loaded). No crashes, no CHECK
failures and no Ceres "Error in evaluating the ResidualBlock" in any run of the rebuilt DLL. Debug-level
(`LOGLEVEL=1`) comparisons: see the nondeterminism section above. The preintegration / alignment state
handed to `ImuInitializer::VisualIMUAlignment` (frames' R, T, all `IntegrationBase` members incl. the
Jacobian blocks) was verified bit-identical by probe on t4s, t8n, t20 and on t6m (5 initialisation attempts
during motion).

Not covered by the synthetic traces: PlatMap saving / loading and relocalisation against a stored map
(the map is only pushed/saved after 200 loop-closing keyframes; a 20 s trace produces ~18 keyframes), the
lab/tag modes, controller/LED paths. `build-test/t20b.bin` / `t20m.bin` were deleted after the runs (disk
space); regenerate them with the commands above.
