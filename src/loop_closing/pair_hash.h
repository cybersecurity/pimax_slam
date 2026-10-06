// pimax_slam.pi.dll -- src/loop_closing/pair_hash.h  (from draft c14)
//
// Hash functor emitted at 0x180170E50 (used by the loop-closing
// TU's std::unordered_map<Key16, std::vector<8-byte T>>: callers 0x180172460 (_Try_emplace /
// operator[]) and 0x180173D90 (rehash); the map's node list is freed by 0x180172000).
// TODO(verify): real name, file and key type. The key is 16 bytes hashed as two 8-byte halves
// with MSVC std::hash (FNV-1a over the object bytes, no -0.0 normalisation -> integer or pointer
// halves, e.g. std::pair<int64_t, int64_t> / std::pair<uint64_t, uint64_t>), combined
// boost::hash_combine style.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

// 0x180170E50  (upstream: new)
struct PairHash
{
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
        std::size_t seed = std::hash<T1>()(p.first);
        seed ^= std::hash<T2>()(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};
