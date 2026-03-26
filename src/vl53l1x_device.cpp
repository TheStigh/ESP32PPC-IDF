#include "vl53l1x_device.h"

#include <Arduino.h>

#include "VL53L1X_i2ccoms.h"

namespace ppc {

bool Vl53l1xDevice::begin(uint8_t sda_pin,
                          uint8_t scl_pin,
                          uint32_t i2c_frequency,
                          uint8_t address,
                          uint16_t timeout_ms,
                          std::optional<int16_t> offset_mm,
                          std::optional<uint16_t> xtalk_cps) {
  address_ = address;
  timeout_ms_ = timeout_ms;
  offset_mm_ = offset_mm;
  xtalk_cps_ = xtalk_cps;

  vl53l1x_configure_i2c(sda_pin, scl_pin, i2c_frequency);
  if (i2c_init() != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] I2C init failed (SDA=%u SCL=%u F=%lu)\n", sda_pin, scl_pin, i2c_frequency);
    return false;
  }

  if (!init()) {
    return false;
  }

  if (offset_mm_.has_value()) {
    VL53L1_Error status = sensor_.SetOffsetInMm(offset_mm_.value());
    if (status != VL53L1_ERROR_NONE) {
      Serial.printf("[VL53L1X] SetOffsetInMm failed: %d\n", status);
      return false;
    }
  }

  if (xtalk_cps_.has_value()) {
    VL53L1_Error status = sensor_.SetXTalk(xtalk_cps_.value());
    if (status != VL53L1_ERROR_NONE) {
      Serial.printf("[VL53L1X] SetXTalk failed: %d\n", status);
      return false;
    }
  }

  return true;
}

bool Vl53l1xDevice::wait_for_boot() {
  delay(2);

  uint8_t device_state = 0;
  uint32_t start = millis();
  while ((millis() - start) < timeout_ms_) {
    VL53L1_Error status = sensor_.GetBootState(&device_state);
    if (status != VL53L1_ERROR_NONE) {
      Serial.printf("[VL53L1X] GetBootState failed: %d\n", status);
      return false;
    }

    if ((device_state & 0x01U) == 0x01U) {
      return true;
    }
    delay(1);
    yield();
  }

  Serial.println("[VL53L1X] Timed out waiting for sensor boot");
  return false;
}

bool Vl53l1xDevice::init() {
  if (address_ != (sensor_.GetI2CAddress() >> 1)) {
    VL53L1_Error status = sensor_.SetI2CAddress(address_ << 1);
    if (status != VL53L1_ERROR_NONE) {
      Serial.printf("[VL53L1X] SetI2CAddress failed: %d\n", status);
      return false;
    }
  }

  if (!wait_for_boot()) {
    return false;
  }

  VL53L1_Error status = sensor_.Init();
  if (status != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] Init failed: %d\n", status);
    return false;
  }

  return true;
}

bool Vl53l1xDevice::set_ranging_mode(const RangingMode *mode) {
  if (mode == nullptr) {
    return false;
  }

  VL53L1_Error status = sensor_.SetDistanceMode(mode->mode);
  if (status != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] SetDistanceMode(%s) failed: %d\n", mode->name, status);
    return false;
  }

  status = sensor_.SetTimingBudgetInMs(mode->timing_budget);
  if (status != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] SetTimingBudgetInMs(%u) failed: %d\n", mode->timing_budget, status);
    return false;
  }

  status = sensor_.SetInterMeasurementInMs(mode->delay_between_measurements);
  if (status != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] SetInterMeasurementInMs(%u) failed: %d\n", mode->delay_between_measurements, status);
    return false;
  }

  ranging_mode_ = mode;
  Serial.printf("[VL53L1X] Ranging mode set: %s\n", mode->name);
  return true;
}

bool Vl53l1xDevice::read_distance(const Roi &roi, uint16_t &distance_out, VL53L1_Error &status_out) {
  if (!has_last_roi_ || roi != last_roi_) {
    status_out = sensor_.SetROI(roi.width, roi.height);
    if (status_out != VL53L1_ERROR_NONE) {
      Serial.printf("[VL53L1X] SetROI failed: %d\n", status_out);
      return false;
    }

    status_out = sensor_.SetROICenter(roi.center);
    if (status_out != VL53L1_ERROR_NONE) {
      Serial.printf("[VL53L1X] SetROICenter failed: %d\n", status_out);
      return false;
    }

    last_roi_ = roi;
    has_last_roi_ = true;
  }

  status_out = sensor_.StartRanging();
  if (status_out != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] StartRanging failed: %d\n", status_out);
    return false;
  }

  uint8_t data_ready = false;
  while (!data_ready) {
    status_out = sensor_.CheckForDataReady(&data_ready);
    if (status_out != VL53L1_ERROR_NONE) {
      Serial.printf("[VL53L1X] CheckForDataReady failed: %d\n", status_out);
      sensor_.StopRanging();
      return false;
    }
    delay(1);
    yield();
  }

  status_out = sensor_.GetDistanceInMm(&distance_out);
  if (status_out != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] GetDistanceInMm failed: %d\n", status_out);
    sensor_.StopRanging();
    return false;
  }

  status_out = sensor_.ClearInterrupt();
  if (status_out != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] ClearInterrupt failed: %d\n", status_out);
    sensor_.StopRanging();
    return false;
  }

  status_out = sensor_.StopRanging();
  if (status_out != VL53L1_ERROR_NONE) {
    Serial.printf("[VL53L1X] StopRanging failed: %d\n", status_out);
    return false;
  }

  return true;
}

}  // namespace ppc
