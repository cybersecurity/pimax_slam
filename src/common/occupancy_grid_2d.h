// pimax_slam.pi.dll -- src/common/occupancy_grid_2d.h
// Pimax fork of svo_common/include/svo/common/occupancy_grid_2d.h (chunk c06).  sizeof == 72:
//   +0 cell_size, +4 n_cols, +8 n_rows (ints), +16 occupancy_ (std::vector<bool>: words +16,
//   size +40), +48 feature_occupancy_ (std::vector<Keypoint>, Keypoint == Vector2f).
// Out-of-line copies in this chunk: ctor 0x1800AA2F0, dtor 0x1800AA440, reset 0x1800AAB80.
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>

#include "common/types.h"

namespace pimax {
namespace totem {

class OccupandyGrid2D
{
public:
  using Grid = std::vector<bool>;
  using FeatureGrid = std::vector<Keypoint>;

  // 0x1800AA2F0 (upstream-identical)
  OccupandyGrid2D(int cell_size, int n_cols, int n_rows)
    : cell_size(cell_size)
    , n_cols(n_cols)
    , n_rows(n_rows)
    , occupancy_(n_cols * n_rows, false)
    , feature_occupancy_(n_cols * n_rows, Keypoint(0, 0))
  {
  }

  OccupandyGrid2D(const OccupandyGrid2D& rhs)
    : cell_size(rhs.cell_size)
    , n_cols(rhs.n_cols)
    , n_rows(rhs.n_rows)
    , occupancy_(rhs.occupancy_)
    , feature_occupancy_(rhs.feature_occupancy_)
  {
  }

  ~OccupandyGrid2D() = default;   // 0x1800AA440

  inline static int getNCell(
      const int n_pixels, const int size)
  {
    return std::ceil(static_cast<double>(n_pixels)
                      / static_cast<double>(size));
  }

  const int cell_size;
  const int n_cols;
  const int n_rows;
  Grid occupancy_;
  FeatureGrid feature_occupancy_;

  // 0x1800AAB80
  inline void reset()
  {
    std::fill(occupancy_.begin(), occupancy_.end(), false);
  }

  inline size_t size()
  {
    return occupancy_.size();
  }

  inline bool empty()
  {
    return occupancy_.empty();
  }

  /// Pimax: pure integer arithmetic (upstream: floor of double division through a
  /// Vector2d overload).  Identical for non-negative pixels.
  inline size_t getCellIndex(int x, int y, int scale = 1) const
  {
    return (scale * x) / cell_size + n_cols * ((scale * y) / cell_size);
  }
};

using OccGrid2DPtr = std::shared_ptr<OccupandyGrid2D>;

static_assert(sizeof(OccupandyGrid2D) == 72, "sizeof(OccupandyGrid2D)");
static_assert(offsetof(OccupandyGrid2D, occupancy_) == 16, "OccupandyGrid2D::occupancy_");
static_assert(offsetof(OccupandyGrid2D, feature_occupancy_) == 48, "OccupandyGrid2D::feature_occupancy_");

} // namespace totem
} // namespace pimax
