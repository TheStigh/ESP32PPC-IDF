#pragma once

#include <cstdint>

#include "VL53L1X_ULD.h"

namespace ppc {

enum class Orientation : uint8_t {
  Parallel = 0,
  Perpendicular = 1,
};

struct Roi {
  uint8_t width{6};
  uint8_t height{16};
  uint8_t center{199};

  bool operator==(const Roi &rhs) const {
    return width == rhs.width && height == rhs.height && center == rhs.center;
  }

  bool operator!=(const Roi &rhs) const { return !(*this == rhs); }
};

struct Threshold {
  uint16_t idle{0};
  uint16_t min{0};
  uint16_t max{0};
  uint8_t min_percent{0};
  uint8_t max_percent{85};
};

struct RangingMode {
  const char *name;
  uint16_t timing_budget;
  uint16_t delay_between_measurements;
  EDistanceMode mode;
};

namespace Ranging {
static constexpr RangingMode kShortest{"Shortest", 15, 20, Short};
static constexpr RangingMode kShort{"Short", 20, 25, Long};
static constexpr RangingMode kMedium{"Medium", 33, 38, Long};
static constexpr RangingMode kLong{"Long", 50, 55, Long};
static constexpr RangingMode kLonger{"Longer", 100, 105, Long};
static constexpr RangingMode kLongest{"Longest", 200, 205, Long};
}  // namespace Ranging

}  // namespace ppc
