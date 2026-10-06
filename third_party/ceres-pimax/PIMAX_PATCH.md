# Ceres Solver 2.1.0 as compiled into pimax_slam.pi.dll

**Layout in this repository:** `third_party/ceres-solver` is the pristine upstream 2.1.0 submodule
(commit f68321e). This directory holds only the files Pimax changed; CMake compiles them instead of the
upstream copies (`cmake/ceres_sources.cmake`) and puts `ceres-pimax/internal` in front of
`ceres-solver/internal` on the include path (for `parameter_block.h`). `diff -r` against the submodule shows
the complete patch.


Upstream `ceres-solver` tag 2.1.0 (`include/`, `internal/ceres/` incl. `generated/` and `miniglog/`; tests,
benchmarks and the other threading back ends removed). Pimax compiled it in-tree
(`E:\code_codex\pimax_slam\beta111_5a7902_dll\thirdparty\ceres\...`) with MINIGLOG, EIGEN_SPARSE, C++ threads
and all Schur specializations (see `third_party/ceres-config`). The binary's `Solver::VersionString()` is
`2.1.0-eigen-(3.4.0)-no_lapack-eigensparse-no_openmp`.

## Changes found in the binary

1. `internal/ceres/parameter_block.h`, `ParameterBlock::SetManifold` (inlined copy at 0x1801CBBD0): the
   `CHECK_EQ(new_manifold->AmbientSize(), size_)` is gone (its message strings are not in the binary) and
   `plus_jacobian_` is only allocated / `UpdatePlusJacobian()` only checked when `AmbientSize() != 0`.
   pimax_slam's local parameterizations report `GlobalSize() == 0` while the backend's
   "minimal Jacobian" flag is set (`ceres_backend::Map::applyFlag` 0x18001A980), and parameter blocks are
   added while it is set, so stock 2.1.0 aborts here. The two remaining CHECKs sit on lines 180 and 190, as
   in the binary.
2. Same file, `ParameterBlock::UpdatePlusJacobian` (0x1801CD640): returns `true` early when
   `manifold_->AmbientSize() == 0` (no plus-Jacobian to update). Its LOG(WARNING) is on line 329, as in the
   binary.
3. `internal/ceres/detect_structure.cc` (`VLOG(1) << "Schur complement static structure <"...`) and
   `internal/ceres/block_sparse_matrix.cc` (`VLOG(2) << "Allocating values array with "...`): removed. Neither
   string exists in the binary, while other VLOG(1)/VLOG(2) strings of the same build do (and no VLOG(3)
   string does, i.e. miniglog's `MAX_LOG_LEVEL` is Ceres' default 2, set in CMakeLists.txt). Line numbers of the
   surrounding code are unchanged.

`tools/ceres_line_check.py` compares the `__LINE__` of every miniglog log site in the binary with this tree
(all .cc sites match; the remaining reports are multi-line statements / inlined callees).
4. `internal/ceres/residual_block.cc`, `ResidualBlock::Evaluate` (0x1801FB7D0): a parameter block whose
   manifold reports `AmbientSize() == 0` (pimax "minimal Jacobian" mode) is evaluated directly into the
   caller's `num_residuals x TangentSize()` Jacobian: the scratch (ambient) Jacobian and the multiplication
   with the plus-Jacobian are only used when `jacobians[i] && PlusJacobian() && manifold()->AmbientSize()`
   (both places). `NumScratchDoublesForEvaluate` (0x1801FC580) is unchanged.
5. `internal/ceres/residual_block_utils.cc`, `InvalidateEvaluation` (0x180219340) and `IsEvaluationValid`
   (0x180219410): the Jacobian width of block i is
   `(manifold == nullptr || manifold->AmbientSize()) ? Size() : manifold->TangentSize()` (stock: `Size()`).
   Without 4./5. every pimax cost function evaluated in minimal mode fails with "Error in evaluating the
   ResidualBlock" (the last `num_residuals` Jacobian entries stay "Uninitialized"). `EvaluationToString`
   (0x180218B80) still prints `Size()` columns.
6. Log sites that are not in the binary (all other `__LINE__`s unchanged): `callbacks.cc` `LoggingCallback`
   has no `else VLOG(1) << output;` (0x18020E490 only writes to `std::cout` when `log_to_stdout_`);
   `trust_region_minimizer.cc` `MaxSolverIterationsReached` (0x180209900) and `FunctionToleranceReached`
   have no `VLOG(1) << "Terminating: "` (the remaining sites 71, 93, 639, 679, 698, 721 are in the binary);
   `detect_structure.cc` VLOG(2) "Dynamic row/e/f block size ..." and `schur_eliminator.cc` VLOG(1)
   "Template specializations not found for <...>" removed (strings / file name absent; the
   `partitioned_matrix_view.cc:177` one exists). Without this the rebuilt DLL writes ~1400 lines per 8 s
   replay to stderr where the original writes none.
