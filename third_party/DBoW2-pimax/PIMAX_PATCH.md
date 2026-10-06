# DBoW2 — Pimax in-tree copy

**Layout in this repository:** `third_party/DBoW2` is the pristine dorian3d/DBoW2 submodule (3924753) and
`third_party/DBow3` the rmsalinas/DBow3 submodule that provides QuickLZ (`src/quicklz.{c,h}`). This directory
holds the Pimax files that replace or extend DBoW2 (`DBoW2.h`, `TemplatedVocabulary.h`, `DescManip.{h,cpp}`);
its `include/` and `include/DBoW2/` come first on the include path. Upstream `BowVector.cpp`,
`FeatureVector.cpp`, `FORB.cpp` and `ScoringObject.cpp` are compiled unchanged from the submodule.


Base: dorian3d/DBoW2 commit 3924753 (`reference/DBoW2`), plus a port of DBoW3's binary
vocabulary loader (`reference/DBow3`). Library code: the template bodies are not reconstructed
function by function; only the Pimax-added/modified parts were checked against the binary.
Objects in the image (c18): BowVector 0x1801B5DF0.., FeatureVector 0x1801B61E0, ScoringObject
0x1801B6350..0x1801B6BE0, DescManip::fromStream 0x1801B6D90, QuickLZ 0x1801B6E30..0x1801B70D0,
FORB 0x1801B70F0..0x1801B75C0; the `TemplatedVocabulary<cv::Mat, FORB>` instantiation lives in
the loop-closing object (0x180172460..0x180178AA0, vtable 0x1803BDB18, 18 slots, c15).

## Changes

1. `TemplatedVocabulary::load(const std::string&)` (0x180176910, c15): opens an
   `std::ifstream(filename, ios::binary)` (mode in|binary), throws
   `std::runtime_error("Vocabulary::load Could not open file :" + filename + " for reading")`
   if it fails, reads a `uint64_t` signature; `88877711233` (0x14B1863F81) → `seekg(0)` +
   `fromStream(ifile)`; otherwise the upstream `cv::FileStorage` path (throws
   `std::string("Could not open file ") + filename`, then virtual `load(fs, "vocabulary")`).
   The ifstream stays open during the FileStorage path (binary destroys it at the end).
2. New non-virtual `TemplatedVocabulary::fromStream(std::istream&)` (0x180175D80): DBoW3
   `Vocabulary::fromStream` adapted to DBoW2 members. Verified against the decompilation:
   `m_words.clear()` then `m_nodes.clear()`; signature (0-initialised) mismatch →
   `std::runtime_error("Vocabulary::fromStream  is not of appropriate type")` (two spaces);
   `bool compressed`, `uint32_t nnodes`, return if 0 (before the stringstream is built);
   compressed: zeroed `qlz_state_decompress` (0x9008 B), `std::vector<char>` 10000 / 10400
   (zero-filled), `uint32_t nChunks`, per chunk read 9-byte header, `qlz_size_compressed`,
   read the rest, `qlz_decompress`, append to a stringstream; then `m_k`, `m_L`, **`m_scoring`
   before `m_weighting`**, `createScoringObject()`, `m_nodes.resize(nnodes)`, node 0 id = 0,
   nodes 1..n-1 `(nid, parent, weight, DescManip::fromStream)` + parent's children push_back,
   then `uint32_t` word count, `m_words.resize`, `(wid, nid)` pairs.
3. New `DBoW2::DescManip::fromStream(cv::Mat&, std::istream&)` (`include/DBoW2/DescManip.h`,
   `src/DescManip.cpp`, 0x1801B6D90): DBoW3 code — `int cols, rows, type`, `m.create(rows, cols,
   type)`, read `m.elemSize()*m.cols` bytes. TODO(verify) class/file name (DBoW3's kept).
4. `DBoW2.h`: no longer includes TemplatedDatabase / QueryResults / FBrief; only the
   `OrbVocabulary` typedef is kept.
5. (none: the upstream `src/*.cpp` compile unchanged because `include/DBoW2` is on the include path)

## Not compiled (not in the image)

`include/DBoW2/TemplatedDatabase.h`, `include/DBoW2/QueryResults.h`, `src/QueryResults.cpp`, FBrief/FSurf64
(no database / query-result strings or code in the binary; the drafts only use the vocabulary,
BowVector, FeatureVector and FORB). FBrief/FSurf64 were already absent.

## Unchanged (identical to the reference, per c18)

BowVector (ctor, addWeight, addIfNotExist, normalize), FeatureVector::addFeature, all six
scoring classes (`GeneralScoring::LOG_EPS` dynamic initialiser 0x1800031B0), FORB
(distance = 64-bit SWAR popcount returning double, fromString, meanValue, toString), every other
TemplatedVocabulary member (HKmeansStep, initiateClustersKMpp, create×3, transform×5,
setNodeWeights, createScoringObject, createWords, save/load(FileStorage), stopWords, getters).
Node layout (152 B: id +0, weight +8, children +16, parent +40, descriptor +48, word_id +144) and
sizeof(TemplatedVocabulary<cv::Mat,FORB>) == 80 checked by a test instantiation.

## QuickLZ

`third_party/DBow3/src/quicklz.{c,h}` (submodule) = DBoW3's QuickLZ 1.5.0 final, byte-identical to the
code in the binary, already configured with `QLZ_COMPRESSION_LEVEL 1`,
`QLZ_STREAMING_BUFFER 0` (sizeof(qlz_state_decompress) = 0x9008 as memset in 0x180175D80).
Only `qlz_decompress` (0x1801B6E30, core 0x1801B6EC0) and `qlz_size_compressed` (0x1801B70D0 —
c18 called it `qlz_size_decompressed`, but it reads the header field at src+1, i.e. the
compressed size) are referenced; the compressor is dropped by /OPT:REF. No change.
