#pragma once

#include <vector>

#include "ppc_types.h"
#include "vl53l1x_device.h"

namespace ppc {

class Zone {
 public:
  explicit Zone(uint8_t id) : id_(id) {}

  void set_max_samples(uint8_t max) { max_samples_ = max; }
  void set_threshold_percentages(uint8_t min_percent, uint8_t max_percent);

  void reset_roi(uint8_t default_center);
  void roi_calibration(uint16_t entry_idle, uint16_t exit_idle, Orientation orientation);
  void calibrate_threshold(Vl53l1xDevice &sensor, int number_attempts);

  bool read_distance(Vl53l1xDevice &sensor, VL53L1_Error &status_out);
  void update_adaptive_threshold(float alpha);
  bool is_occupied() const;

  uint16_t last_distance() const { return last_distance_; }
  uint16_t min_distance() const { return min_distance_; }
  uint8_t id() const { return id_; }

  Roi roi{};
  Roi roi_override{};
  Threshold threshold{};

 private:
  int get_optimized_value(const std::vector<int> &values, int sum) const;

  uint8_t id_;
  uint8_t max_samples_{2};
  std::vector<uint16_t> samples_{};
  uint16_t last_distance_{0};
  uint16_t min_distance_{0};
};

}  // namespace ppc
