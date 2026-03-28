#include "ppc_zone.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace ppc {

void Zone::set_threshold_percentages(uint8_t min_percent, uint8_t max_percent) {
  threshold.min_percent = min_percent;
  threshold.max_percent = max_percent;
}

bool Zone::read_distance(Vl53l1xDevice &sensor, VL53L1_Error &status_out) {
  uint16_t distance = 0;
  if (!sensor.read_distance(roi, distance, status_out)) {
    return false;
  }

  last_distance_ = distance;
  samples_.insert(samples_.begin(), distance);
  if (samples_.size() > max_samples_) {
    samples_.pop_back();
  }

  min_distance_ = *std::min_element(samples_.begin(), samples_.end());
  return true;
}

void Zone::reset_roi(uint8_t default_center) {
  if (roi_override.width != 0) {
    roi.width = roi_override.width;
  } else {
    roi.width = 6;
  }

  if (roi_override.height != 0) {
    roi.height = roi_override.height;
  } else {
    roi.height = 16;
  }

  if (roi_override.center != 0) {
    roi.center = roi_override.center;
  } else {
    roi.center = default_center;
  }

  Serial.printf("[Zone] %s reset ROI {w:%u h:%u c:%u}\n", id_ == 0 ? "Entry" : "Exit", roi.width, roi.height,
                roi.center);
}

bool Zone::calibrate_threshold(Vl53l1xDevice &sensor, int number_attempts) {
  std::vector<int> values;
  values.reserve(number_attempts);
  int sum = 0;

  for (int i = 0; i < number_attempts; ++i) {
    VL53L1_Error status = VL53L1_ERROR_NONE;
    if (!read_distance(sensor, status)) {
      continue;
    }
    values.push_back(static_cast<int>(last_distance_));
    sum += static_cast<int>(last_distance_);
    delay(2);
  }

  if (values.size() < 3) {
    Serial.printf("[Zone] %s threshold calibration failed: only %u valid samples\n",
                  id_ == 0 ? "Entry" : "Exit",
                  static_cast<unsigned>(values.size()));
    return false;
  }

  threshold.idle = static_cast<uint16_t>(get_optimized_value(values, sum));
  threshold.max = static_cast<uint16_t>((static_cast<uint32_t>(threshold.idle) * threshold.max_percent) / 100U);
  threshold.min = static_cast<uint16_t>((static_cast<uint32_t>(threshold.idle) * threshold.min_percent) / 100U);

  Serial.printf("[Zone] %s threshold idle=%u min=%u(%u%%) max=%u(%u%%)\n",
                id_ == 0 ? "Entry" : "Exit",
                threshold.idle,
                threshold.min,
                threshold.min_percent,
                threshold.max,
                threshold.max_percent);

  if (threshold.idle == 0 || threshold.max <= threshold.min) {
    Serial.printf("[Zone] %s threshold calibration invalid: idle=%u min=%u max=%u\n",
                  id_ == 0 ? "Entry" : "Exit",
                  threshold.idle,
                  threshold.min,
                  threshold.max);
    return false;
  }

  return true;
}

void Zone::roi_calibration(uint16_t entry_idle, uint16_t exit_idle, Orientation orientation) {
  const float min_idle_m = static_cast<float>(std::min(entry_idle, exit_idle)) / 1000.0f;
  int function_of_distance = static_cast<int>(16.0f * (1.0f - (0.15f * 2.0f) / (0.34f * min_idle_m)));
  int roi_size = std::min(8, std::max(4, function_of_distance));

  if (roi_override.width != 0) {
    roi.width = roi_override.width;
  } else {
    roi.width = static_cast<uint8_t>(roi_size);
  }

  if (roi_override.height != 0) {
    roi.height = roi_override.height;
  } else {
    roi.height = static_cast<uint8_t>(roi_size * 2);
  }

  if (roi_override.center != 0) {
    roi.center = roi_override.center;
  } else {
    if (orientation == Orientation::Parallel) {
      switch (roi.width) {
        case 4:
          roi.center = id_ == 0 ? 150 : 247;
          break;
        case 5:
        case 6:
          roi.center = id_ == 0 ? 159 : 239;
          break;
        case 7:
        case 8:
        default:
          roi.center = id_ == 0 ? 167 : 231;
          break;
      }
    } else {
      switch (roi.width) {
        case 4:
          roi.center = id_ == 0 ? 193 : 58;
          break;
        case 5:
        case 6:
          roi.center = id_ == 0 ? 194 : 59;
          break;
        case 7:
        case 8:
        default:
          roi.center = id_ == 0 ? 195 : 60;
          break;
      }
    }
  }

  Serial.printf("[Zone] %s ROI calibrated {w:%u h:%u c:%u}\n", id_ == 0 ? "Entry" : "Exit", roi.width, roi.height,
                roi.center);
}

int Zone::get_optimized_value(const std::vector<int> &values, int sum) const {
  const int size = static_cast<int>(values.size());
  if (size == 0) {
    return 0;
  }

  int avg = sum / size;
  long sum_squared = 0;
  for (const int v : values) {
    sum_squared += static_cast<long>(v) * static_cast<long>(v);
    yield();
  }

  int variance = static_cast<int>(sum_squared / size - (avg * avg));
  if (variance < 0) {
    variance = 0;
  }
  int sd = static_cast<int>(std::sqrt(static_cast<float>(variance)));
  return avg - sd;
}

bool Zone::is_occupied() const {
  return min_distance_ < threshold.max && min_distance_ > threshold.min;
}

void Zone::update_adaptive_threshold(float alpha) {
  if (last_distance_ <= threshold.max || last_distance_ == 0) {
    return;
  }

  uint16_t max_reasonable_change = threshold.idle / 5;
  if (std::abs(static_cast<int>(last_distance_) - static_cast<int>(threshold.idle)) > max_reasonable_change) {
    return;
  }

  uint16_t old_idle = threshold.idle;
  threshold.idle = static_cast<uint16_t>((1.0f - alpha) * static_cast<float>(threshold.idle) +
                                         alpha * static_cast<float>(last_distance_));

  threshold.max = static_cast<uint16_t>((static_cast<uint32_t>(threshold.idle) * threshold.max_percent) / 100U);
  threshold.min = static_cast<uint16_t>((static_cast<uint32_t>(threshold.idle) * threshold.min_percent) / 100U);

  if (old_idle != threshold.idle) {
    Serial.printf("[Zone] %s adaptive idle %u -> %u\n", id_ == 0 ? "Entry" : "Exit", old_idle, threshold.idle);
  }
}

}  // namespace ppc
