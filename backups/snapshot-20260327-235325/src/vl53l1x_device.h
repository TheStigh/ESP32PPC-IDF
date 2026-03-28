#pragma once

#include <optional>

#include "ppc_types.h"

namespace ppc {

class Vl53l1xDevice {
 public:
  bool begin(uint8_t sda_pin,
             uint8_t scl_pin,
             uint32_t i2c_frequency,
             uint8_t address,
             uint16_t timeout_ms,
             std::optional<int16_t> offset_mm,
             std::optional<uint16_t> xtalk_cps);

  bool set_ranging_mode(const RangingMode *mode);
  const RangingMode *get_ranging_mode() const { return ranging_mode_; }

  void set_ranging_mode_override(const RangingMode *mode) {
    ranging_mode_override_ = mode;
  }

  const RangingMode *get_ranging_mode_override() const {
    return ranging_mode_override_.has_value() ? ranging_mode_override_.value() : nullptr;
  }

  bool read_distance(const Roi &roi, uint16_t &distance_out, VL53L1_Error &status_out);

 private:
  bool wait_for_boot();
  bool init();

  VL53L1X_ULD sensor_;
  uint8_t address_{0x29};
  uint16_t timeout_ms_{1000};

  const RangingMode *ranging_mode_{nullptr};
  std::optional<const RangingMode *> ranging_mode_override_{};
  std::optional<int16_t> offset_mm_{};
  std::optional<uint16_t> xtalk_cps_{};

  Roi last_roi_{};
  bool has_last_roi_{false};
};

}  // namespace ppc
