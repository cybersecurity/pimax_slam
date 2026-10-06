# Chunk `c18_tail_vikit` — 0x1801B0310 .. 0x1801B7840 (+ survey of 0x1801B7840 .. end of .text)

## Summary

Part 1 has 138 functions. They come from five library and project areas, in object order:

| range | object(s) | kind | reconstructed? |
|---|---|---|---|
| 0x1801B0310–0x1801B3BF7 | `tinyxml2.cpp` (tail). The object **starts in the previous chunk at 0x1801AFEF0**: `CreateUnlinkedNode<T>` ×5 and the `XMLDocument` ctor at 0x1801B0190. | lib: tinyxml2 **9.0.0, unmodified** | no |
| 0x1801B3C00–0x1801B3F38 | vikit `camera_geometry_base.cpp` | project (vendored vikit) | yes |
| 0x1801B3F40–0x1801B41FB | vikit `ncamera.cpp` | project | yes |
| 0x1801B4200–0x1801B59E6 | vikit `performance_monitor.cpp` | project, plus STL instantiations | yes |
| 0x1801B5A80–0x1801B5C21 | vikit `sample.cpp`. Only the `std::ranlux24_base` seeding helper is here. | lib: STL | no (sample.cpp copied for completeness) |
| 0x1801B5C30–0x1801B5DEA | vikit `robust_cost.cpp` | project | yes |
| 0x1801B5DF0–0x1801B77A8 | DBoW2 (`BowVector`, `FeatureVector`, `ScoringObject`, a DescManip-style stream reader, QuickLZ, `FORB`), plus two OpenCV aruco import stubs | lib: DBoW2 fork | no |
| 0x1801B77B0–0x1801B7837 | Ceres `cost_function.cc` (`CostFunction` ctor/dtor). This is the **start of Ceres**. | lib: ceres | no |

### Conventions that hold across the image

- **Within one object file, functions appear sorted by their decorated (mangled) name, not in source order.** For example `??$..` < `??0` < `??1` < `??_G` < `?A..` < `?a..` < `_..`. Every object in this chunk follows the rule:
  - tinyxml2: `??1?$MemPoolT@$0FA@` … `?TIXML_VSNPRINTF`, then `__local_stdio_printf_options`.
  - DBoW2 scoring: Bhattacharyya < ChiSquare < DotProduct < KL < L1 < L2.
  - BowVector: `??0` < `addIfNotExist` < `addWeight` < `normalize`.
  - performance_monitor: `??0` < `??1` < `addLog` < `addTimer` < `c_str` < `init` < `log` < `startTimer` < `stopTimer` < `writeToFile`.

  Use this to name neighbouring functions. It also explains why the `.text$di` initializer for `vk::Sample::gen_int` (0x180003110) comes before the one for `gen_real` (0x180003150).
- MSVC groups virtual overloads in **reverse declaration order** in the vtable. As a result, `CameraGeometryBase::backProject3(Matrix2Xd)` sits in slot 1 and the pure `backProject3(Vector2d)` in slot 2, even though the upstream header declares them in the other order.
- Asserts: Eigen `eigen_assert` and Ceres `assert` (casts.h:102) are live, but tinyxml2's `TIXMLASSERT` is compiled out. That means `NDEBUG` is **not** defined and `_DEBUG` / `TINYXML2_DEBUG` are **not** defined either.
- The CRT is dynamic (`/MD`). Imports include `VCRUNTIME140.dll`, `VCRUNTIME140_1.dll` (`__CxxFrameHandler4`, so VS2019 or later), `MSVCP140.dll` and `api-ms-win-crt-*`. ConcRT is **not** linked. IDA's `Concurrency::*` / `?dtor$..@__acrt..` names in the tail are FLIRT false positives.

### Version findings

| library | verdict | evidence |
|---|---|---|
| **tinyxml2** | **9.0.0, identical to `../LedObjectPoseEstimator/third_party/tinyxml2`** | `XMLDocument` layout matches 9.0.0 exactly (see Types), including `_parsingDepth` @164 and the "Element nesting is too deep." check at depth 100 (`TINYXML2_MAX_ELEMENT_DEPTH`, error 18). `MemPoolT::ITEMS_PER_BLOCK = 4096/size`: blocks of 0xFF0 for 80/120, 0xFD8 for 104, 0xFC0 for 112. Other matches: the `ParseDeep` "well located declaration" optimisation, the `SetError` format string, `LoadFile(FILE*)` with `_fseeki64`/`_ftelli64`, and `GetCharacterRef` with no code-point clamp. The entities table has 5 entries and an unknown entity does `++p; ++q` (the "fixme"). No differences were found in any of the ~70 functions. |
| **Eigen** | **3.4.0** | Ceres `VersionString()` (0x1801EDC50) builds `"2.1.0" + "-eigen-(" + "3.4.0" + ")" + "-no_lapack" + "-eigensparse" + "-no_openmp"`. Of the 148 distinct `wassert` (file, line) pairs in all.c, 144 match the 3.4.0 sources and only 81 match 3.3.9. Examples: PlainObjectBase.h:277, Memory.h:185 with the "…handmade **aligned** memory allocator" text (3.3.9 reads "alignd"), SolverBase.h:150/151, SVDBase.h:258–266, Redux.h:200/411, Transpose.h:438. The tridiagonal QR deflation test in 0x180066540 (`SelfAdjointEigenSolver::computeFromTridiagonal_impl`, 3×3) is the 3.4 form: `if (|d_i|+|d_i+1| >= (e_i*4.503599627370496e15)^2) e_i = 0`, after `if (|e_i| < DBL_MIN) e_i = 0`. That is `precision_inv = 1/eps`, not 3.3's `isMuchSmallerThan(…, 2*eps)`. |
| **Ceres** | **2.1.0, no modifications found** | VersionString `2.1.0-eigen-(3.4.0)-no_lapack-eigensparse-no_openmp`. The sources are in-tree at `E:\code_codex\pimax_slam\beta111_5a7902_dll\thirdparty\ceres\…`. Spot check: 1737 strings from Ceres functions were compared against `reference/ceres-2.1.0`. Every literal message was found; the only "misses" were macro-generated `CHECK_xx` / `OPTION_xx` stringifications. 190 glog `CHECK`/`LOG` `__LINE__` immediates across 29 files (problem_impl.cc:421, line_search.cc ×8, trust_region_minimizer.cc ×6, visibility_based_preconditioner.cc ×6, …) match the reference line **exactly**. The exceptions are a few multi-line statements, off by 1–5 lines: parameter_block.h:190/329/336, reorder_program.cc:284, trust_region_preprocessor.cc:331. TODO(verify) parameter_block.h:190 (reference CHECK is at 186–188). |
| **DBoW2** | dorian3d/DBoW2 as in `reference/DBoW2` (commit 3924753, 2020-05-12) **plus a DBoW3-derived binary vocabulary loader** | `BowVector::{BowVector, addWeight, addIfNotExist, normalize}`, `FeatureVector::addFeature`, all six `*Scoring::score` and `FORB::{distance, fromString, meanValue, toString}` are identical to the reference. `FORB::distance` is already the 64-bit popcount version that returns `double`. `GeneralScoring::LOG_EPS = log(DBL_EPSILON)` is initialised at 0x1800031B0. The fork adds DBoW3's `Vocabulary::load(filename)` (0x180176910: magic `0x14B1863F81` = 88877711233 → `fromStream`, otherwise `cv::FileStorage`, throwing "Vocabulary::load Could not open file :" / "Could not open file "). It also adds `fromStream` (0x180175D80, "Vocabulary::fromStream  is not of appropriate type") with QuickLZ-compressed node blocks (`qlz_decompress` 0x1801B6E30, state on the stack, 36880 B), and a DescManip-style `fromStream(cv::Mat&, istream&)` (0x1801B6D90: cols, rows, type, then data). The `TemplatedVocabulary` vtable has 18 slots, the same count as the reference. The vocabulary file is `voc_GEN_8X4.dbow`. |
| **QuickLZ** | 1.5.x, `QLZ_COMPRESSION_LEVEL 1`, `QLZ_STREAMING_BUFFER 0` | `bitlut` table at 0x1803BFFA0 = {4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0}. 4096 hash entries; `hash_counter` @+0x8000; `stream_counter` @+0x9000. Header flag bit 1 selects a 9-byte or 3-byte header. Only the decompressor is linked (DBoW3 ships quicklz.c 1.5.0). |
| glog | 0.5.0 | Path strings `E:\software_x86\glog-0.5.0\src\{logging,vlog_is_on,utilities}.cc`. glog uses dbghelp `SymFromAddr` / `UnDecorateSymbolName` for its stack traces. |
| gflags | 2.2.2 | Path strings `E:\software_x86\gflags-2.2.2\src\gflags{,_reporting,_completions}.cc`. Only flag registration is linked. |
| Boost.Serialization | archive library version **18** (0x18035CA60 returns 18; signature "serialization::archive") | Version 18 is consistent with Boost 1.72–1.75, i.e. with 1.74. Uses `portable_binary_{i,o}archive` (Boost example code compiled into the project). |
| Boost.Filesystem / System | (1.74) | "boost::filesystem::{remove, rename, status, last_write_time, directory_iterator::…}". |
| zlib | 1.2.8 | " deflate 1.2.8 Copyright 1995-2013 …" and " inflate 1.2.8 …". The code at 0x1803927B0 (adler32, NMAX 5552) is only reachable through deflate's `configuration_table`, so it has no direct callers. |
| libjpeg | message table only | "Bogus JPEG colorspace", "Wrong JPEG library version …" sit in .rdata at 0x1803ED3E8.., but no jpeg code is in .text. This is probably data pulled in with GLEW/Pangolin glue. TODO(verify) |
| GLEW | **1.12.0** | `glewGetString` version "1.12.0" @0x1803D2654, `wglewContextInit`, `wglewIsSupported`. **No callers from outside the GLEW range**, so it is dead code linked as a whole object. No Pangolin code was found; GLEW is the only remnant of the viewer. |
| OpenCV | 4.5.3 DLLs | Imports from aruco, highgui, imgcodecs, calib3d, features2d, imgproc and core, all *453. Assert paths read `E:\software_x86\opencv-4.5.3\modules\core\include\opencv2/core/{mat.inl,persistence}.hpp`. |

## Function table (0x1801B0310 .. 0x1801B7840)

Kinds: `project`, `lib:<what>`. Status: `identical` / `modified: …` / `new` / `-`.

### tinyxml2 9.0.0 (library, not reconstructed; use upstream 9.0.0 sources)

| address | proposed name | signature | kind | status | summary |
|---|---|---|---|---|---|
| 1801B0310 | `tinyxml2::MemPoolT<80>::~MemPoolT` | `()` | lib:tinyxml2 | identical | Clear(): free blocks, reset counters, DynArray dtor |
| 1801B0390 | `MemPoolT<104>::~MemPoolT` | | lib:tinyxml2 | identical | |
| 1801B0410 | `MemPoolT<112>::~MemPoolT` | | lib:tinyxml2 | identical | |
| 1801B0490 | `MemPoolT<120>::~MemPoolT` | | lib:tinyxml2 | identical | |
| 1801B0510 | `XMLDocument::DepthTracker::~DepthTracker` | | lib:tinyxml2 | identical | `--_document->_parsingDepth` (+164) |
| 1801B0520 | `StrPair::~StrPair` | | lib:tinyxml2 | identical | Reset() |
| 1801B0550 | `XMLDocument::~XMLDocument` | | lib:tinyxml2 | identical | Clear(), 4 pools, _unlinked, _errorStr, ~XMLNode. Called by the calibration loader 0x18015E250 |
| 1801B05F0 | `XMLNode::~XMLNode` | | lib:tinyxml2 | identical | DeleteChildren + Unlink from parent |
| 1801B0690..0750 | `MemPoolT<80/104/112/120>::scalar deleting dtor` | | lib:tinyxml2 | identical | |
| 1801B0790 | `MemPool::scalar deleting dtor` | | lib:tinyxml2 | identical | |
| 1801B07C0 | `XMLAttribute::scalar deleting dtor` | | lib:tinyxml2 | identical | |
| 1801B0840 | `XMLComment::sdd` | | lib:tinyxml2 | identical | |
| 1801B0880 | `XMLDeclaration::sdd` | | lib:tinyxml2 | identical | |
| 1801B08C0 | `XMLDocument::sdd` | | lib:tinyxml2 | identical | |
| 1801B0980 | `XMLElement::sdd` | | lib:tinyxml2 | identical | frees attribute list via pool |
| 1801B0A30 | `XMLNode::sdd` | | lib:tinyxml2 | identical | |
| 1801B0A70 | `XMLText::sdd` | | lib:tinyxml2 | identical | |
| 1801B0AB0 | `XMLUnknown::sdd` | | lib:tinyxml2 | identical | |
| 1801B0AF0 | `XMLComment::Accept` | `(XMLVisitor*) const` | lib:tinyxml2 | identical | |
| 1801B0B00 | `XMLDeclaration::Accept` | | lib:tinyxml2 | identical | |
| 1801B0B10 | `XMLDocument::Accept` | | lib:tinyxml2 | identical | |
| 1801B0B80 | `XMLElement::Accept` | | lib:tinyxml2 | identical | |
| 1801B0BF0 | `XMLText::Accept` | | lib:tinyxml2 | identical | |
| 1801B0C00 | `XMLUnknown::Accept` | | lib:tinyxml2 | identical | IDA: unknown_libname_5 |
| 1801B0C10 | `MemPoolT<80>::Alloc` | `void*()` | lib:tinyxml2 | identical | block 0xFF0, 51 items |
| 1801B0D40 | `MemPoolT<104>::Alloc` | | lib:tinyxml2 | identical | block 0xFD8, 39 items |
| 1801B0E60 | `MemPoolT<112>::Alloc` | | lib:tinyxml2 | identical | block 0xFC0, 36 items |
| 1801B0F80 | `MemPoolT<120>::Alloc` | | lib:tinyxml2 | identical | block 0xFF0, 34 items |
| 1801B10A0 | `XMLElement::Attribute` | `const char*(const char* name, const char* value=0) const` | lib:tinyxml2 | identical | used by the calib loader ("translation", "rowMajorRotationMat", …) |
| 1801B1150 | `XMLDocument::Clear` | `void()` | lib:tinyxml2 | identical | |
| 1801B1280 | `XMLNode::DeleteChild` | `void(XMLNode*)` | lib:tinyxml2 | identical | |
| 1801B1360 | `XMLNode::DeleteChildren` | `void()` | lib:tinyxml2 | identical | |
| 1801B1480 | `XMLNode::FirstChildElement` | `const XMLElement*(const char*) const` | lib:tinyxml2 | identical | used by calib loader |
| 1801B1520 | `MemPoolT<N>::Free` (ICF-folded, all 4 vtables) | `void(void*)` | lib:tinyxml2 | identical | |
| 1801B1540 | `XMLUtil::GetCharacterRef` | `const char*(const char*, char*, int*)` | lib:tinyxml2 | identical | ConvertUTF32ToUTF8 inlined |
| 1801B1760 | `StrPair::GetStr` | `const char*()` | lib:tinyxml2 | identical | entity table @0x1803BF590 (5 entries) |
| 1801B19E0 | `XMLDocument::Identify` | `char*(char*, XMLNode**)` | lib:tinyxml2 | identical | |
| 1801B1BC0 | `XMLNode::InsertChildPreamble` | `void(XMLNode*) const` | lib:tinyxml2 | identical | |
| 1801B1C80..1CB0 | `MemPoolT<80/104/112/120>::ItemSize` | `size_t()` | lib:tinyxml2 | identical | |
| 1801B1CC0 | `XMLDocument::LoadFile(const char*)` | `XMLError(const char*)` | lib:tinyxml2 | identical | `LoadFile(FILE*)` inlined; fopen_s "rb"; used by calib loader |
| 1801B1E60 | `XMLDocument::NewComment` | | lib:tinyxml2 | identical | |
| 1801B1F00 | `XMLDocument::NewDeclaration` | | lib:tinyxml2 | identical | default `xml version="1.0" encoding="UTF-8"` |
| 1801B1FB0 | `XMLDocument::NewElement` | | lib:tinyxml2 | identical | |
| 1801B2050 | `XMLDocument::NewText` | | lib:tinyxml2 | identical | |
| 1801B20F0 | `XMLDocument::NewUnknown` | | lib:tinyxml2 | identical | |
| 1801B2190 | `XMLNode::NextSiblingElement` | `const XMLElement*(const char*) const` | lib:tinyxml2 | identical | used by calib loader ("Camera") |
| 1801B2230 | `XMLDocument::Parse()` (private) | `void()` | lib:tinyxml2 | identical | |
| 1801B2300 | `XMLElement::ParseAttributes` | `char*(char*, int*)` | lib:tinyxml2 | identical | |
| 1801B2590 | `XMLAttribute::ParseDeep` | `char*(char*, bool, int*)` | lib:tinyxml2 | identical | |
| 1801B2710 | `XMLComment::ParseDeep` | | lib:tinyxml2 | identical | |
| 1801B27C0 | `XMLDeclaration::ParseDeep` | | lib:tinyxml2 | identical | |
| 1801B2870 | `XMLElement::ParseDeep` | | lib:tinyxml2 | identical | |
| 1801B2930 | `XMLNode::ParseDeep` | | lib:tinyxml2 | identical | DepthTracker, "Element nesting is too deep." |
| 1801B2E30 | `XMLText::ParseDeep` | | lib:tinyxml2 | identical | |
| 1801B2FC0 | `XMLUnknown::ParseDeep` | | lib:tinyxml2 | identical | |
| 1801B3070 | `StrPair::ParseName` | | lib:tinyxml2 | identical | |
| 1801B3150 | `DynArray<XMLNode*,10>::Push` | | lib:tinyxml2 | identical | called from CreateUnlinkedNode (prev chunk) |
| 1801B31F0 | `StrPair::Set` | | lib:tinyxml2 | identical | |
| 1801B3250 | `XMLDocument::SetError` | `void(XMLError, int, const char*, ...)` | lib:tinyxml2 | identical | "Error=%s ErrorID=%d (0x%x) Line number=%d" |
| 1801B33C0 | `StrPair::SetStr` | | lib:tinyxml2 | identical | |
| 1801B3460 | `MemPoolT<N>::SetTracked` (ICF-folded) | | lib:tinyxml2 | identical | |
| 1801B3470 | `XMLComment::ShallowClone` | | lib:tinyxml2 | identical | |
| 1801B34C0 | `XMLDeclaration::ShallowClone` | | lib:tinyxml2 | identical | |
| 1801B3510 | `XMLElement::ShallowClone` | | lib:tinyxml2 | identical | |
| 1801B36F0 | `XMLText::ShallowClone` | | lib:tinyxml2 | identical | |
| 1801B3740 | `XMLUnknown::ShallowClone` | | lib:tinyxml2 | identical | |
| 1801B3790 | `XMLComment::ShallowEqual` | | lib:tinyxml2 | identical | |
| 1801B3830 | `XMLDeclaration::ShallowEqual` | | lib:tinyxml2 | identical | |
| 1801B38D0 | `XMLElement::ShallowEqual` | | lib:tinyxml2 | identical | |
| 1801B39D0 | `XMLText::ShallowEqual` | | lib:tinyxml2 | identical | |
| 1801B3A70 | `XMLUnknown::ShallowEqual` | | lib:tinyxml2 | identical | |
| 1801B3B10 | `tinyxml2::TIXML_SNPRINTF` | `int(char*, size_t, const char*, ...)` | lib:tinyxml2 | identical | vsnprintf_s(_TRUNCATE) |
| 1801B3B80 | `tinyxml2::TIXML_VSNPRINTF` | | lib:tinyxml2 | identical | |
| 1801B3BF0 | `__local_stdio_printf_options` | `unsigned __int64*()` | lib:crt (header inline, emitted in tinyxml2.obj) | - | returns &_OptionsStorage (0x18047F010) |

### vikit (project — reconstructed in `draft/c18_tail_vikit/thirdparty/vikit/…`)

| address | proposed name | signature | kind | status | summary |
|---|---|---|---|---|---|
| 1801B3C00 | `vk::cameras::CameraGeometryBase::CameraGeometryBase` | `(int width, int height)` | project | identical | label_/mask_ default; called with (640, 480) by 0x18015E250 and 0x1801899C0 (inlined `CameraGeometry` ctor) |
| 1801B3C50 | `CameraGeometryBase::scalar deleting dtor` | `void*(unsigned)` | project | identical | `= default` |
| 1801B3CE0 | `CameraGeometryBase::backProject3` | `void(const Ref<const Matrix2Xd>&, Matrix3Xd*, std::vector<bool>*) const` | project | modified: both `CHECK_NOTNULL` removed | resize, loop calling vslot 2 |
| 1801B3F30 | `CameraGeometryBase::setMask` | `void(const cv::Mat&)` | project | modified: 3× `CHECK_EQ` removed | `mask_ = mask` |
| 1801B3F40 | `std::vector<Transformation, aligned_allocator>::vector(const vector&)` | | lib:std+Eigen (in ncamera.obj) | - | element 64 B, malloc + Memory.h:185 assert |
| 1801B4050 | `vk::cameras::NCamera::NCamera` | `(const TransformationVector& T_C_B, const TransformationVector& T_B_C, const std::vector<Camera::Ptr>&, const std::string& label)` | project | modified: extra vector member, no `initInternal()` | caller: `make_shared<NCamera>` 0x180159A40 (label "PiMax") |
| 1801B41A0 | `NCamera::getCameraShared` | `std::shared_ptr<Camera>(size_t)` | project | modified: no `CHECK_LT` | many callers (frontend, loop closing) |
| 1801B41E0 | `NCamera::get_T_B_C` (name inferred) | `const Transformation&(size_t) const` | project | new | `T_B_C_[i]` (+24) |
| 1801B41F0 | `NCamera::get_T_C_B` | `const Transformation&(size_t) const` | project | modified: no `CHECK_LT` | `T_C_B_[i]` (+0) |
| 1801B4200 | `std::list<pair<const string,LogItem>>::_Free_non_head` | | lib:std | - | |
| 1801B42A0 | `std::list<pair<const string,Timer>>::_Free_non_head` | | lib:std | - | |
| 1801B4340 | `std::_Hash<…string,LogItem…>::emplace(pair<string,LogItem>&&)` | | lib:std | - | node 0x40, FNV-1a, rehash 0x180185540 |
| 1801B4650 | `std::_Hash<…string,Timer…>::emplace(pair<string,Timer>&&)` | | lib:std | - | node 0x48 |
| 1801B4970 | `vk::PerformanceMonitor::PerformanceMonitor` | `()` | project | modified (container type) | 2 unordered_maps, 2 strings, ofstream |
| 1801B4B30 | `std::_Hash<…LogItem>::~_Hash` | | lib:std | - | |
| 1801B4BB0 | `std::_Hash<…Timer>::~_Hash` | | lib:std | - | |
| 1801B4C30 | `_List_node_emplace_op2<…LogItem>::~…` (TODO(verify) LogItem vs Timer; bodies identical) | | lib:std | - | EH cleanup |
| 1801B4CC0 | `_List_node_emplace_op2<…Timer>::~…` | | lib:std | - | |
| 1801B4D50 | `std::list<…LogItem>::~list` | | lib:std | - | |
| 1801B4D80 | `std::list<…Timer>::~list` | | lib:std | - | |
| 1801B4DB0 | `unordered_map<string,LogItem>::~unordered_map` (thunk) | | lib:std | - | |
| 1801B4DC0 | `unordered_map<string,Timer>::~unordered_map` (thunk) | | lib:std | - | |
| 1801B4DD0 | `PerformanceMonitor::~PerformanceMonitor` | | project | identical | flush, close, member dtors |
| 1801B4EF0 | `PerformanceMonitor::addLog` | `void(const string&)` | project | identical | |
| 1801B4F90 | `PerformanceMonitor::addTimer` | `void(const string&)` | project | identical | `Timer()` = now() |
| 1801B5070 | `std::string::c_str` | | lib:std | - | |
| 1801B5080 | `PerformanceMonitor::init` | `void(const string& trace_name, const string& trace_dir)` | project | modified: no throw on open failure | traceHeader inlined |
| 1801B55F0 | `PerformanceMonitor::log` | `void(const string&, double)` | project | identical | "Logger = %s\n", throws "Logger not registered" |
| 1801B56D0 | `PerformanceMonitor::startTimer` | `void(const string&)` | project | identical | "startTimer: Timer not registered" |
| 1801B57D0 | `PerformanceMonitor::stopTimer` | `void(const string&)` | project | identical | "stopTimer: Timer not registered" |
| 1801B58E0 | `PerformanceMonitor::writeToFile` | `void()` | project | modified: inlined trace() does not throw | |
| 1801B5A80 | `std::_Swc_base<unsigned,24,10,24,…>::_Seed<linear_congruential_engine<unsigned,40014,0,2147483563>>` | `(lcg&, bool readcy)` | lib:std (sample.obj) | - | ranlux24 seeding (mask 0xFFFFFF @0x1803BFF60); only caller: `vk::Sample::gen_real` initialiser 0x180003150 |
| 1801B5C30 | `vk::solver::TukeyWeightFunction::TukeyWeightFunction` | `(float b)` | project | identical | `b_square_ = b*b`; caller PoseOptimizer 0x180132D90 |
| 1801B5C50 | `vk::solver::MADScaleEstimator::compute` | `float(std::vector<float>&) const` | project | modified: `CHECK(!errors.empty())` removed | median via nth_element × 1.48f |
| 1801B5DC0 | `TukeyWeightFunction::weight` | `float(const float&) const` | project | identical | |

### DBoW2 fork (library, not reconstructed)

| address | proposed name | signature | kind | status | summary |
|---|---|---|---|---|---|
| 1801B5DF0 | `DBoW2::BowVector::BowVector` | `()` | lib:DBoW2 | identical | std::map header node (0x30); used at +424 of 0x1800E26D0 / 0x18017F4C0 objects (KeyFrame-like) |
| 1801B5E30 | `BowVector::addIfNotExist` | `void(WordId, WordValue)` | lib:DBoW2 | identical | |
| 1801B5F20 | `BowVector::addWeight` | `void(WordId, WordValue)` | lib:DBoW2 | identical | |
| 1801B6020 | `BowVector::normalize` | `void(LNorm)` | lib:DBoW2 | identical | L1 = 0 → Σ\|v\|, else sqrt Σv² |
| 1801B61E0 | `DBoW2::FeatureVector::addFeature` | `void(NodeId, unsigned)` | lib:DBoW2 | identical | |
| 1801B6350 | `BhattacharyyaScoring::score` | `double(const BowVector&, const BowVector&) const` | lib:DBoW2 | identical | |
| 1801B64E0 | `ChiSquareScoring::score` | | lib:DBoW2 | identical | ×2 at the end |
| 1801B6650 | `DotProductScoring::score` | | lib:DBoW2 | identical | |
| 1801B67B0 | `KLScoring::score` | | lib:DBoW2 | identical | LOG_EPS @0x18047F138 |
| 1801B6A60 | `L1Scoring::score` | | lib:DBoW2 | identical | `-score/2.0` (compiled as `(-s)*0.5`) |
| 1801B6BE0 | `L2Scoring::score` | | lib:DBoW2 | identical | `1 - sqrt(1 - s)` (1.0 if s ≥ 1) |
| 1801B6D90 | DescManip-style `fromStream(cv::Mat&, std::istream&)` (DBoW3) | `void(cv::Mat&, std::istream&)` | lib:DBoW2-fork (DBoW3 code) | new (vs DBoW2) | reads cols, rows, type, then `create(rows, cols, type)` and `elemSize*cols` bytes; caller fromStream 0x180175D80 |
| 1801B6E30 | `qlz_decompress` | `size_t(const char*, void*, qlz_state_decompress*)` | lib:QuickLZ 1.5 | - | sets `stream_counter` (+36864) = 0 |
| 1801B6EC0 | `qlz_decompress_core` (static, level 1) | | lib:QuickLZ | - | |
| 1801B70D0 | `qlz_size_decompressed` | `size_t(const char*)` | lib:QuickLZ | - | |
| 1801B70F0 | `DBoW2::FORB::distance` | `double(const cv::Mat&, const cv::Mat&)` | lib:DBoW2 | identical | 64-bit SWAR popcount over cols/8 words |
| 1801B71E0 | `FORB::fromString` | `void(cv::Mat&, const std::string&)` | lib:DBoW2 | identical | 1×32 CV_8U |
| 1801B7330 | `FORB::meanValue` | `void(const vector<const cv::Mat*>&, cv::Mat&)` | lib:DBoW2 | identical | 256-bit majority vote |
| 1801B75C0 | `FORB::toString` | `std::string(const cv::Mat&)` | lib:DBoW2 | identical | |
| 1801B779D | `cv::aruco::getPredefinedDictionary` import stub | | lib:opencv (jmp [__imp]) | - | caller 0x18017F730 |
| 1801B77A3 | `cv::aruco::DetectorParameters::create` import stub | | lib:opencv | - | caller 0x18017F730 |
| 1801B77B0 | `ceres::CostFunction::CostFunction` | `()` | lib:ceres 2.1.0 (cost_function.cc) | - | called by the project cost functors |
| 1801B77D0 | `ceres::CostFunction::~CostFunction` | | lib:ceres | - | |

## Types

### `vk::cameras::CameraGeometryBase` (sizeof 152 = 0x98; vtable @0x1803BFE58)

| off | type | name | evidence |
|---|---|---|---|
| 0 | vptr | | ctor 0x1801B3C00 |
| 8 | int | width_ | ctor (640) ✔ |
| 12 | int | height_ | ctor (480) ✔ |
| 16 | std::string | label_ | ctor/dtor, set to "camera N" by 0x18015E250 ✔ |
| 48 | Type (int) | camera_type_ | not initialised by the base ctor; the derived (inlined) ctor copies `projection_.cam_type_` (+152) ✔ |
| 56 | cv::Mat | mask_ | ctor/dtor/setMask ✔ |

vtable slots: 0 dtor, 1 backProject3(Matrix2Xd) 0x1801B3CE0, 2 backProject3(Vector2d), 3 project3, 4 printParameters, 5 errorMultiplier, 6 getIntrinsicParameters, 7 getDistortionParameters, 8 getAngleError. Slots 2–8 are pure in the base.

### `vk::cameras::CameraGeometry<PinholeProjection<EquidistantDistortion>>` (other chunk; information for the coordinator)

- vtable @0x1803B7C78: dtor 0x18015ACB0, [1] base 0x1801B3CE0, backProject3 0x18015B690, project3 0x180161BA0, printParameters 0x180161900, errorMultiplier 0x18015B750 (named `_mm_and_ps_w`, i.e. fabs), getIntrinsic 0x18015CE00, getDistortion 0x18015CA90, getAngleError 0x18015CA20.
- `make_shared` control block 0x128, so sizeof = 280. `projection_` is at +152: `int cam_type_` (+152, 0 = kPinhole), fx +160, fy +168, 1/fx +176, 1/fy +184, cx +192, cy +200, then distortion at +208, which holds **6 coefficients** (from the XML floats), 1e-8, π and 1e-8 (constants from 0x1803B8270).
- Upstream `EquidistantDistortion` has only k1..k4, so Pimax extended it (Fisheye62-like). This belongs to chunk ~0x18015xxxx.

### `vk::cameras::NCamera` (sizeof 104 = 0x68)

| off | type | name | evidence |
|---|---|---|---|
| 0 | TransformationVector | T_C_B_ | ctor copy of arg1; get_T_C_B 0x1801B41F0 ✔ |
| 24 | TransformationVector | T_B_C_ (Pimax) | ctor copy of arg2; 0x1801B41E0 ✔ (name TODO(verify)) |
| 48 | std::vector<std::shared_ptr<Camera>> | cameras_ | getCameraShared ✔ |
| 72 | std::string | label_ | ctor ✔ ("PiMax") |

`TransformationVector` is `std::vector<kindr::minimal::QuatTransformationTemplate<double>, Eigen::aligned_allocator<…>>` with a 64-byte element stride.

At the caller 0x18015E250, the vector passed as arg 1 receives `T.inverse()` (via 0x180013040) and arg 2 receives `T`, where T is built from the XML `rowMajorRotationMat` + `translation`.

### `vk::PerformanceMonitor` (sizeof 0x1C8)

| off | type | name |
|---|---|---|
| 0 | `std::unordered_map<std::string, vk::Timer>` (64 B) | timers_ |
| 64 | `std::unordered_map<std::string, vk::LogItem>` | logs_ |
| 128 | std::string | trace_name_ |
| 160 | std::string | trace_dir_ |
| 192 | std::ofstream (264 B) | ofs_ |

- Map nodes: next/prev (16 bytes), then the key string at +16, then the value at +48.
- `vk::Timer` (24 B): +0 start_time_ (steady_clock ns), +8 duration_, +16 accumulated_. Node offsets are +48, +56 and +64.
- `vk::LogItem` (16 B): +0 double data, +8 bool set.
- Upstream uses `std::map`.
- Owners: the global frontend monitor (pointer @0x18047DDA0) and the backend `Estimator`/interface member pointer @+1224.

### `vk::solver::TukeyWeightFunction`

16 bytes: vptr @0x1803BFF80, +8 `float b_square_`.

### `vk::solver::MADScaleEstimator`

8 bytes: vptr @0x1803B7300.

### Library layouts (for cross-reference only)

**`tinyxml2::XMLDocument`** (9.0.0, x64):

| offset | member |
|---|---|
| 0 | XMLNode (vptr, `_document` @8, `_parent` @16, `_value` StrPair @24, `_parseLineNum` @48, `_firstChild` @56, `_lastChild` @64, `_prev` @72, `_next` @80, `_userData` @88, `_memPool` @96) |
| 104 | `_writeBOM` |
| 105 | `_processEntities` |
| 108 | `_errorID` |
| 112 | `_whitespaceMode` |
| 120 | `_errorStr` |
| 144 | `_errorLineNum` |
| 152 | `_charBuffer` |
| 160 | `_parseCurLineNum` |
| 164 | `_parsingDepth` |
| 168 | `_unlinked` (DynArray<XMLNode*,10>) |
| 264 | `_elementPool<120>` |
| 392 | `_attributePool<80>` |
| 520 | `_textPool<112>` |
| 648 | `_commentPool<104>` |

Total size 776. Other node layouts: XMLElement `_closingType` @104, `_rootAttribute` @112. XMLText `_isCData` @104. XMLAttribute: `_name` @8, `_value` @32, `_parseLineNum` @56, `_next` @64, `_memPool` @72.

**DBoW2 `BowVector`** is `std::map<unsigned, double>`; each node is 0x30 bytes with the key @32 and the value @40. **`FeatureVector`** is `std::map<unsigned, std::vector<unsigned>>`; each node is 0x40 bytes.

The DBoW2 vocabulary node (`TemplatedVocabulary<cv::Mat,FORB>::Node`, 152 bytes), as seen in other chunks:

| offset | member |
|---|---|
| 0 | id |
| 8 | double weight |
| 16 | vector<NodeId> children |
| 40 | parent |
| 48 | cv::Mat descriptor |
| 144 | word_id |

## External interfaces

### Calls out of the chunk

| address | meaning |
|---|---|
| 0x1801AFEF0 / 0x1801AFF70 / 0x1801AFFF0 / 0x1801B0080 / 0x1801B0110 | `XMLDocument::CreateUnlinkedNode<XMLComment / XMLDeclaration / XMLElement / XMLText / XMLUnknown>(MemPoolT&)` (previous chunk, tinyxml2) |
| 0x1800178F0 | Eigen `conditional_aligned_malloc<true>` (Matrix3Xd storage) |
| 0x18011D130 | `std::vector<bool>::resize(n, false)` |
| 0x180091A60 | Eigen `MapBase` / `Ref<const Vector2d>` construction from a column (data, rows, cols) |
| 0x180008BF0 | `std::string` copy ctor |
| 0x180011570 | `std::string::assign(const char*, size_t)` |
| 0x1800FDD60 | `std::string::append(const char*, size_t)` |
| 0x180006C30 | `std::string::_Reallocate_grow_by` (append path) |
| 0x180006290 | `operator<<(std::ostream&, const char*)` |
| 0x180017630 | `std::_Insert_string(ostream&, const char*, size_t)` (`ofs_ << std::string`) |
| 0x180016B90 | `printf` (inline `_vfprintf_l(stdout, …)`) |
| 0x180016B80 | `__local_stdio_printf_options` (other copy) |
| 0x1800E3240 | `std::runtime_error::runtime_error(const char*)` |
| 0x18017AC00 | `std::_Hash<…std::string key…>::_Find_last` (shared instantiation) |
| 0x180185540 | `_Hash::_Forced_rehash` |
| 0x18000F840 | `std::vector<_List_unchecked_iterator>::_Assign_grow` (bucket init) |
| 0x180014730 | `std::chrono::steady_clock::now()` |
| 0x180097EC0 | `basic_filebuf::close` |
| 0x180097CD0 | `basic_ofstream::~basic_ofstream` |
| 0x180097A90 | `std::use_facet<std::codecvt<char,char,_Mbstatet>>` |
| 0x180090AD0 | `std::_Partition_by_median_guess_unchecked<float*, less<>>` |
| 0x1800BCE00 | `std::_Tree<map<unsigned,…>>::_Find_lower_bound` |
| 0x18000FE80 | `std::_Tree::_Insert_node` |
| 0x180062DE0 | `std::vector<unsigned>::_Emplace_reallocate` |
| 0x180159C60 | `std::basic_stringbuf::_Init(const string&, mode)` |
| 0x180009630 | `std::basic_stringbuf::~basic_stringbuf` |

OpenCV `Mat` imports: ctor, dtor, `operator=` (copy/move), release, clone, create, zeros.

### Globals and constants used

| address | meaning |
|---|---|
| 0x18047F138 | `DBoW2::GeneralScoring::LOG_EPS` (= log(DBL_EPSILON), init 0x1800031B0) |
| 0x18047DB50 | `_Stinit` mbstate constant for filebuf |
| 0x1803BF590 | tinyxml2 entities table |
| 0x1803BFF60 | 0x00FFFFFF ×4 (ranlux mask) |
| 0x1803BFFA0 | QuickLZ bitlut |
| 0x1803BFF90 | 1.48f |
| 0x1803AE9DC | 1.0f |
| 0x1803ADD98 | 1e-9 (Timer::getTime) |
| 0x18047F050.. | `vk::Sample::gen_real` (std::ranlux24) |
| 0x18046A3E0.. | std::mt19937 (very likely `vk::Sample::gen_int`) |

### Callers into the chunk (for the coordinator)

- **tinyxml2:** `LoadFile` 0x1801B1CC0, `FirstChildElement` 0x1801B1480, `NextSiblingElement` 0x1801B2190, `Attribute` 0x1801B10A0 and `~XMLDocument` 0x1801B0550 are all called only from the calibration loader 0x18015E250.
- **vikit:**
  - `CameraGeometryBase` ctor ← 0x18015E250, 0x1801899C0.
  - `setMask` ← 0x18015E250.
  - `NCamera` ctor ← 0x180159A40.
  - `getCameraShared` ← 0x180010C20, 0x1800B2460, 0x1800DFDF0, 0x1800FB650, 0x18012B4E0, 0x18017F730, 0x18018D770, 0x1801A7DF0.
  - `get_T_B_C` ← 0x180026100, 0x1800FB650.
  - `get_T_C_B` ← 0x1800FB650, 0x1801277B0.
  - PerformanceMonitor ← 0x180013680, 0x180015DC0 (backend: timers/logs "tot_time", "ceres_time", "pre_optim_time", "marginalization", "fixation", "n_fixed_lm"; `init(trace_backend, dir)`), 0x1800DFDF0 (frontend g_permon @0x18047DDA0: 10 timers + 4 logs), 0x1800FA6B0, 0x1800FB650, 0x180119260.
  - Tukey ctor ← 0x180132D90.
- **DBoW2:**
  - BowVector/FeatureVector ← `TemplatedVocabulary::transform` 0x1801788A0 / 0x180178AA0.
  - FORB ← 0x180172DA0 (HKmeansStep), 0x1801764B0 (initiateClustersKMpp), 0x180178650, 0x180176D50 (load(FileStorage)), 0x180177330 (save).
  - qlz / DescManip ← 0x180175D80 (`fromStream`).

## Constants / config defaults

- `TukeyWeightFunction` default b = 4.6851f (header). The ctor stores b*b as a float. The actual b used is set by the PoseOptimizer caller 0x180132D90.
- `MADScaleEstimator`: factor 1.48f.
- `PerformanceMonitor::trace`: precision 15, `std::ios::fixed`. Reset values: timers 0; logs `set = false`, `data = -1.0`.
- tinyxml2: `TINYXML2_MAX_ELEMENT_DEPTH` = 100; SetError buffer 1000 bytes.

## Quirks / bugs worth preserving

- `MADScaleEstimator::compute` without its CHECK dereferences `end()` on an empty vector.
- `NCamera` accessors and `getCameraShared` have no bounds checks. An out-of-range index is undefined behaviour instead of a glog FATAL.
- `PerformanceMonitor::init` does not throw if the trace file cannot be opened. It prints "Tracefile = %s\n" and continues, and every later `writeToFile` silently writes nothing (but still resets the timers and logs).
- In `CameraGeometryBase::backProject3(Matrix2Xd)` a null `out_bearing_vectors` or `success` crashes instead of FATAL-logging.
- tinyxml2 9.0.0: an unknown `&entity;` is skipped by one byte without being copied (`++p; ++q` leaves a hole). This is upstream behaviour.

## Part 2 — library survey of 0x1801B7840 .. end of .text (0x1803A70E0)

4,808 functions. Address map (boundaries marked `~` are approximate to within one or two small functions):

| range | content | evidence |
|---|---|---|
| 0x1801B77B0 – 0x180354A3F | **Ceres 2.1.0** (in-tree `thirdparty\ceres`), ≈ 1.65 MB | `__FILE__` paths for ~80 ceres files. Object order is the static-lib member pull order, not alphabetical: cost_function → solver.cc (0x1801B7840.., `Solver::Options` validation 0x1801BAF80, summary 0x1801BC0D0, Solve 0x1801BCF80) → problem_impl.cc (0x1801C3640–0x1801CC9A0) → types.cc 0x1801DB190 → detect_structure.cc 0x1801DB650 → gradient_checking_cost_function.cc 0x1801DBD30 → parameter_block_ordering.cc 0x1801E1BB0 → minimizer.cc 0x1801E46F0 → block_sparse_matrix.cc 0x1801E6360 → program.cc 0x1801EA490 → preprocessor.cc 0x1801ED020 (VersionString 0x1801EDC50) → evaluator.cc 0x1801EFAB0 → compressed_row_sparse_matrix.cc 0x1801F3BF0 → parallel_for_cxx.cc 0x1801F9200 → residual_block.cc 0x1801FB230 → gradient_checker.cc 0x1801FF760 → line_search_minimizer.cc 0x1802056E0 → trust_region_minimizer.cc 0x180207F50 → triplet_sparse_matrix.cc 0x18020C440 → callbacks.cc → line_search_preprocessor.cc → trust_region_preprocessor.cc 0x18020F7B0 → block_jacobian_writer.cc 0x180212020 → dense_sparse_matrix.cc → dynamic_compressed_row_sparse_matrix.cc → thread_token_provider.cc → corrector.cc 0x180217920 → residual_block_utils.cc → line_search.cc 0x18021A090 → line_search_direction.cc 0x1802204B0 → coordinate_descent_minimizer.cc 0x1802246D0 → file.cc → linear_solver.cc → preconditioner.cc → reorder_program.cc 0x18022F5F0 → trust_region_strategy.cc → polynomial.cc 0x18023AB40 → low_rank_inverse_hessian.cc → cgnr_solver.cc → dynamic_sparse_normal_cholesky_solver.cc → iterative_schur_complement_solver.cc → schur_complement_solver.cc 0x1802527D0 (schur_eliminator<…> specialisations 0x180256A60..) → dogleg_strategy.cc 0x18025E1F0 → levenberg_marquardt_strategy.cc → conjugate_gradients_solver.cc → subset_preconditioner.cc → dense_cholesky.cc → dense_qr.cc → schur_jacobi_preconditioner.cc → visibility_based_preconditioner.cc 0x18027E730 → block_random_access_*.cc → sparse_cholesky.cc → inner_product_computer.cc → linear_least_squares_problems.cc → partitioned_matrix_view.cc → canonical_views_clustering.cc → single_linkage_clustering.cc → visibility.cc 0x1802978C0 → generated schur_eliminator_impl.h specialisations 0x1802A98F0–0x180334190 → eigensparse.cc 0x18033A8E0 → partitioned_matrix_view_impl.h specialisations 0x18033F8D0–0x180354A3F. Eigen sparse/dense kernels used by Ceres are interleaved. |
| 0x180354A40 – ~0x18035BEFF | **glog 0.5.0**: `utilities.cc`, `vlog_is_on.cc`, `logging.cc` (0x180358370 "Mailing command", 0x180358D20 "*** Check failure stack trace: ***", 0x180359610 log file header), symbolize/stacktrace (0x18035B0A0 "%s@ %*p") | `LogMessage` entry points used by project code: 0x180354E20 / 0x180354E50 / 0x180354E80 (`LogMessage(file, line[, sev])` variants), 0x180354EB0 (`CheckOpMessageBuilder`, "Check failed: "), 0x1803551B0 / 0x180355280 (`~LogMessage` / `~LogMessageFatal`). Last ~2 KB (0x18035B6C0–0x18035BEFF, TLS helpers with no direct callers) TODO(verify) |
| ~0x18035BF20 – ~0x1803608E0 | **Boost.Serialization 1.74** (archive_exception 0x18035C120, codecvt_null, basic_iarchive/oarchive, extended_type_info, void_cast, binary_iarchive/oarchive with "endian setting"/"size of double" checks 0x18035FC10, library_version 18 0x18035CA60) | `.?AVarchive_exception@archive@boost@@` |
| ~0x1803608F0 – 0x180366369 | **Boost.Filesystem + Boost.System 1.74** (path codecvt 0x180360C50, directory_iterator 0x180363DB0 / 0x180364490, remove/rename/status/last_write_time 0x180364D90–0x180365280, error_category messages) | strings |
| 0x18036636A – 0x18036665D | **OpenCV import thunks** (`jmp [__imp_…]`, 6 bytes each, ~125 thunks: Feature2D/ORB/BFMatcher/Mat/FileStorage/imgproc/calib3d/highgui) | names |
| 0x180366660 – 0x18036704F | **gflags 2.2.2** (FlagRegisterer ctors 0x180366660 / 750 / 840 / 9F0, FlagRegistry::RegisterFlag 0x180366AF0 "ERROR: flag '%s' was defined more than once…"); called only from glog's `DEFINE_*` initializers in `.text$di` 0x1800044A0–0x180006000 | strings / xrefs |
| 0x180367050 – 0x1803927AF | **GLEW 1.12.0** (per-extension `_glewInit_*` 0x180367050.., `wglewContextInit` 0x18038C580, `wglewGetExtension` 0x18038EEF0, `wglewIsSupported` 0x18038F030). No callers from the rest of the DLL. | strings, OPENGL32 imports `glGetString`, `wglGetCurrentDC`, `wglGetProcAddress` |
| 0x1803927B0 – 0x180395A5E | **zlib 1.2.8**: adler32 0x1803927B0, crc32 0x180392AA0, deflate_stored/fast/slow 0x180392DA0 / 0x180393050 / 0x1803934C0, fill_window 0x1803939D0, trees.c (_tr_flush_block 0x180393EF0, build_tree, send_tree, compress_block …) | NMAX 5552 / BASE 65521 constants, copyright strings |
| 0x180395A5F – 0x1803974FF | **CRT / STL glue**: MSVCP140 import thunks (_Mtx_*, _Cnd_*, streambuf virtuals, _Query_perf_*), `__std_reverse_trivially_swappable_8`, `operator new`/delete wrappers 0x180395D4C / 0x180395D88, `__security_check_cookie` 0x180395F00, `__GSHandlerCheck*`, `_Init_thread_*`, scrt DllMain / dllmain_dispatch / `__scrt_*` 0x180396xxx, vcruntime / ucrt import thunks (memcpy, malloc, math functions …) 0x180397360–0x1803974AF, `_guard_*_nop` | names |
| 0x180397500 – ~0x1803A463F | **`.text$x` — C++ EH funclets** (unwind destructor funclets and catch blocks) for every object, in object order (project objects first, then libraries). Mostly 12–60 byte stubs; IDA mislabels some as `?dtor$..@__acrt_…`. | |
| ~0x1803A4640 – 0x1803A70DF | **`.text$yd` — `atexit` dynamic destructors** (`??__F…`) for statics: boost::serialization singletons (`iserializer` / `oserializer` / `extended_type_info_typeid` for `pimax::totem::KeyFrame`, `PlatMap`, `DBoW2::BowVector`, `kindr::minimal::QuatTransformation`, cv types …), gflags/glog statics, the `MemPool` … | e.g. 0x1803A53F0 resets the `iserializer<binary_iarchive, std::vector<KeyFrame*>>` singleton |

### Project-looking functions in the tail

The tail contains no free-standing project functions. Two groups of compiler-generated code belong to project functions and are reconstructed by their owners' chunks:

1. **Catch funclets with project log strings** (they call `LOGE` 0x18000C2C0 or `LOGW` 0x18000F6A0 and then rethrow or return). They are part of the owning functions' `try/catch` and are reconstructed by the owners' chunks:

   | funclet | string | owner |
   |---|---|---|
   | 0x1803A09E0 | "Standard Exception load(): %s\n" (also destroys members +0x68 / +0x80, then rethrows) | PlatMap::load 0x180182F80 |
   | 0x1803A0BB0 | "Standard Exception load(): %s\n" | 0x180183AA0 |
   | 0x1803A0AB0 | "OpenCV Exception\n" | loop_closing/platmap region (owner not resolved) |
   | 0x1803A0AF0 | "Standard Exception loadIndex() \n" | loop_closing/platmap region (owner not resolved) |
   | 0x1803A0B30 | "Unknown Exception\n" | loop_closing/platmap region (owner not resolved) |
   | 0x1803A0CA0 | "Error during saving map: %s \n", "Temporary map file removed due to error." (740 B) | PlatMap save 0x180183EC0 |
   | 0x1803A10A0 | "ReLocalize: failed to prepare frames, unknown exception" | 0x180186A50 |
   | 0x1803A10F0 | "ReLocalize: failed to prepare frames: %s" | 0x180186A50 |
   | 0x1803A1150 | "ReLocalize: failed to prepare frames, allocation failure: %s" | 0x180186A50 |
   | 0x1803A11B0 | "ReLocalize: failed to prepare frames, OpenCV exception code=%d: %s" | 0x180186A50 |
   | 0x1803A1470 | "Standard Exception tag\n" | owner not resolved |
   | 0x1803A15E0 | "ReLocalize: unknown exception" | ReLocalize 0x18018D770 (26 KB) |
   | 0x1803A1620 | "ReLocalize: exception: %s" | 0x18018D770 |
   | 0x1803A1660 | "ReLocalize: allocation failure: %s" | 0x18018D770 |
   | 0x1803A16A0 | "ReLocalize: OpenCV exception code=%d: %s" | 0x18018D770 |
   | 0x1803A21B0 | "ceres solve error: %s\n" (LOGW, returns) | 0x18018D770 |
   | 0x1803A2200 | "ceres solve error\n" | 0x18018D770 |
   | 0x1803A2270 | "OpenCV Exception\n" | owner not resolved |
   | 0x1803A22B0 | "Standard Exception tag\n" | owner not resolved |
   | 0x1803A22F0 | "Unknown Exception\n" | owner not resolved |

   For the funclets marked "owner not resolved", Hex-Rays did not inline the catch body into the parent. They lie in the loop_closing/platmap funclet run (0x1803A0A00–0x1803A2300); the parents can be found through the `.pdata` / FH4 FuncInfo handler maps. TODO(verify)

2. **`atexit` destructors** (0x1803A4640..) for boost::serialization singletons of project types. These are compiler-generated and need no reconstruction. They exist as long as the project uses `BOOST_CLASS_EXPORT` / serialization of `KeyFrame`, `PlatMap`, `std::vector<KeyFrame*>`, `std::vector<std::vector<KeyFrame*>>`, `BowVector`, `std::map<unsigned,double>`, `cv::Point_`, `cv::Point3_`, `Eigen::Matrix<double,3,1>`, `QuatTransformation`, `vector<int>` and `pair<const unsigned,double>`. That list comes from the export names (the DLL **exports** those boost serializer ctors and singletons; see the export name table at 0x18043FE6C..).

Apart from these, the tail has no project code. Ceres template instantiations used by the project (AutoDiff and the like) live in the project objects, not here.

## Draft files

- `draft/c18_tail_vikit/thirdparty/vikit/vikit_cameras/include/vikit/cameras/camera_geometry_base.h` (+ `implementation/camera_geometry_base.hpp`)
- `draft/c18_tail_vikit/thirdparty/vikit/vikit_cameras/src/camera_geometry_base.cpp`
- `draft/c18_tail_vikit/thirdparty/vikit/vikit_cameras/include/vikit/cameras/ncamera.h`, `src/ncamera.cpp`
- `draft/c18_tail_vikit/thirdparty/vikit/vikit_common/include/vikit/{performance_monitor.h,timer.h,sample.h}`, `src/{performance_monitor.cpp,sample.cpp}`
- `draft/c18_tail_vikit/thirdparty/vikit/vikit_solver/include/vikit/solver/robust_cost.h`, `src/robust_cost.cpp`

The tree layout follows the binary's `thirdparty\vikit\vikit_solver\include\vikit\solver\implementation/mini_least_squares_solver.hpp` path. With a glog stub, Eigen 3.4 and minkindr, all five `.cpp` files pass `g++ -std=c++17 -fsyntax-only`.

The libraries are not reconstructed:
- tinyxml2: use `third_party/tinyxml2` (9.0.0) unchanged.
- DBoW2: use `reference/DBoW2` plus DBoW3's `quicklz.{c,h}` (1.5.0, level 1) and the loader additions from the vocabulary chunk.
- Ceres: use reference 2.1.0 unchanged, configured with EIGEN_SPARSE, no LAPACK, no OpenMP (CXX threads).

## Open questions

- Is `NCamera`'s second vector really `T_B_C_` / `get_T_B_C`? The name is inferred from decorated-name ordering and the caller pushing `T` vs `T.inverse()`. Resolve this with the 0x18015E250 / 0x1800FB650 chunk owners.
- The two `getCameraShared` overloads (const / non-const) cannot be told apart.
- Do `loadMask`, `isMasked` and `createRandomKeypoint` exist in the Pimax source? They are unreferenced, so the binary cannot say.
- Mapping of 0x1801B4C30 / 0x1801B4CC0 to LogItem vs Timer (identical bodies).
- Owners of the catch funclets without string matches.
