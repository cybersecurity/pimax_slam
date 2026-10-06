# minkindr — Pimax in-tree copy

**Layout in this repository:** `third_party/minkindr` is the pristine ethz-asl/minkindr submodule (564f126);
this directory holds the two implementation headers Pimax changed and comes first on the include path
(the upstream headers include them with `<kindr/minimal/implementation/...>`).


Original path in the binary: `E:\code_codex\pimax_slam\beta111_5a7902_dll\thirdparty\minkindr\minkindr\include\kindr/minimal/...`
(header-only; every function below is an instantiation emitted inside a project object).

Base: ethz-asl/minkindr at 564f126 (the `third_party/minkindr` submodule). The Pimax
copy is an unknown, reformatted revision: the glog `__LINE__` values in the binary match none of
the 161 commits of the upstream history (checked by cloning the full history and scanning the
`CHECK_NEAR` / `CHECK(isValidRotationMatrix)` / `VLOG(200)` lines). The line numbers are therefore
forced with `#line` directives (see 3.).

## 1. `QuatTransformationTemplate::operator*` renormalises its result

`implementation/quat-transformation-inl.h`

Binary 0x180009B80 (`QuatTransformationTemplate<double>::operator*(const QuatTransformationTemplate&)`):
`rhs.t` is rotated first (0x180014A20, MSVC right-to-left argument evaluation), the quaternion
product is the normal `RotationQuaternion::operator*` (0x180009C80: Hamilton product,
`normalizationHelper` = renormalise only if `| |q|^2 - 1 | > 1e-4`, checking ctor 0x1800089C0),
and then the composed quaternion is renormalised **unconditionally** with Eigen's
`normalize()` (`n = |q|^2; if (n > 0) q /= sqrt(n)`), inline, before returning (NRVO). Upstream
returns the composition directly. Reported by c00 (§2, quirks), c01, c06, c10 (quirk 11), c16.

## 2. `QuatTransformationTemplate::inverse()` renormalises its result

Binary 0x180013040: position `-q.inverseRotate(t)` (Eigen `Quaternion::inverse()` 0x1800132A0,
which divides by `|q|^2`, then rotate), rotation `q.inverse()` = `conjugated()` built through the
checking `Implementation` ctor (0x1800089C0), then the same unconditional `normalize()` as in 1.
Reported by c01 ("normalizes result"), c10 (quirk 11).

## 3. glog line numbers (forced with `#line`)

`implementation/rotation-quaternion-inl.h`

| statement | binary | line in binary | upstream line |
|---|---|---|---|
| `CHECK_NEAR(squaredNorm(), 1, EPS::normalization_value())` in `RotationQuaternionTemplate(w,x,y,z)` | 0x180132B70 | 59 | 63 |
| same in `RotationQuaternionTemplate(const Implementation&)` (double 0x1800089C0, float 0x18008A610) | | 73 | 82 |
| `CHECK(isValidRotationMatrix(matrix)) << matrix` in `RotationQuaternionTemplate(const RotationMatrix&)` | 0x1800DEA10 | 102 | 115 |
| `VLOG(200)` ×4 in `isValidRotationMatrix(matrix, threshold)` | 0x180115670 | 488, 489, 493, 494 | 557, 558, 563, 564 |

The two `CHECK_NEAR`s were joined onto one line (clang uses the line of the macro name; MSVC
would use a later line for a multi-line invocation), and the second `if` of
`isValidRotationMatrix` was joined onto one line (the binary has a gap of 4 between the 2nd and 3rd
VLOG, upstream 5). The `CHECK_NEAR` text in the binary is the upstream expression
(`(squaredNorm()) <= (static_cast<Scalar>(1.0))+(EPS<Scalar>::normalization_value())`), so the
`EPS<Scalar>` helper exists in the Pimax copy.

## 4. Things that are NOT changed (checked)

* `RotationQuaternionTemplate::exp()` is upstream: threshold `isLessThenEpsilons4thRoot`
  (function-static `pow(DBL_EPSILON, 0.25)`, 0x18012FE00) and the checking `(w,x,y,z)` ctor
  (0x180132B70, line 59). Used by `Transformation::exp` in `PoseOptimizer::update` 0x18013F830.
  The *1e-12* small-angle exp that several chunk notes attribute to "the Pimax minkindr"
  (c05 PoseLocalParameterization::plus 0x18008D270, c09 getMotionPrior 0x18010A690,
  c11 getRelativeRotationPrior 0x18012AB60) is a different, Eigen-quaternion based helper
  without CHECK: it is `pimax::totem::quaternionExp()` in `src/common/transformation.h`.
* `RotationQuaternionTemplate<double>::operator*` (0x180009C80): upstream (`normalizationHelper`
  + checking ctor).
* `cast<float>()` (0x18008A270 / 0x18008A460): upstream (`constructAndRenormalize` of the cast
  rotation matrix).
* `transform()` (0x180024870), `setIdentity()` (0x180015A30), default ctor (0x180008930),
  `getTransformationMatrix()` (0x1801239E0), `operator<<` (0x180020CE0): upstream.
