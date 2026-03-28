#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <algorithm>
#include <ctime>
#include <optional>
#include <string>

#include "app_config.h"
#include "ppc_zone.h"
#include "vl53l1x_device.h"

namespace {

constexpr uint8_t kNobody = 0;
constexpr uint8_t kSomeone = 1;
constexpr char kFwVersion[] = "standalone-0.1.0";

struct RuntimeConfig {
  bool invert_direction;
  uint8_t sampling_size;
  uint8_t threshold_min_percent;
  uint8_t threshold_max_percent;
  uint32_t path_tracking_timeout_ms;
  uint32_t event_cooldown_ms;
  uint32_t peak_time_delta_ms;
  bool adaptive_threshold_enabled;
  uint32_t adaptive_threshold_interval_ms;
  float adaptive_threshold_alpha;
  bool serial_debug_enabled;
  uint32_t serial_debug_sample_interval_ms;
};

RuntimeConfig runtime_cfg{
    cfg::kInvertDirection,
    cfg::kSamplingSize,
    cfg::kThresholdMinPercent,
    cfg::kThresholdMaxPercent,
    cfg::kPathTrackingTimeoutMs,
    cfg::kEventCooldownMs,
    cfg::kPeakTimeDeltaMs,
    cfg::kAdaptiveThresholdEnabled,
    cfg::kAdaptiveThresholdIntervalMs,
    cfg::kAdaptiveThresholdAlpha,
    cfg::kSerialDebugEnabled,
    cfg::kSerialDebugSampleIntervalMs,
};

WiFiClient plain_client;
WiFiClientSecure tls_client;
PubSubClient mqtt_client(plain_client);

String base_topic;
String topic_state;
String topic_event;
String topic_cmd;
String topic_ack;
String topic_availability;
String topic_meta;

uint32_t state_seq = 0;
uint32_t last_sensor_loop_ms = 0;
uint32_t last_state_publish_ms = 0;
uint32_t last_wifi_reconnect_ms = 0;
uint32_t last_mqtt_reconnect_ms = 0;

struct PendingEvent {
  bool ready{false};
  String name{};
};

bool parse_uint32(JsonVariantConst value, uint32_t &out) {
  if (value.is<uint32_t>()) {
    out = value.as<uint32_t>();
    return true;
  }
  if (value.is<int>()) {
    int v = value.as<int>();
    if (v < 0) {
      return false;
    }
    out = static_cast<uint32_t>(v);
    return true;
  }
  return false;
}

bool parse_float(JsonVariantConst value, float &out) {
  if (value.is<float>() || value.is<double>() || value.is<int>()) {
    out = value.as<float>();
    return true;
  }
  return false;
}

class CounterEngine {
 public:
  bool begin() {
    apply_runtime_config();

    if (!sensor_.begin(cfg::kI2cSdaPin,
                       cfg::kI2cSclPin,
                       cfg::kI2cFrequency,
                       cfg::kVl53l1xAddress,
                       cfg::kVl53l1xBootTimeoutMs,
                       std::nullopt,
                       std::nullopt)) {
      Serial.println("[PPC] Sensor init failed");
      return false;
    }

    if (!recalibrate()) {
      Serial.println("[PPC] Calibration failed");
      return false;
    }

    const uint32_t now = millis();
    reset_path_tracking();
    last_state_change_time_ = now;
    last_adaptive_update_time_ = now;
    zones_empty_since_ = now;

    Serial.println("[PPC] Counter engine ready");
    return true;
  }

  bool loop_once() {
    maybe_reset_daily_totals();

    VL53L1_Error status_entry = VL53L1_ERROR_NONE;
    VL53L1_Error status_exit = VL53L1_ERROR_NONE;
    if (!entry_.read_distance(sensor_, status_entry)) {
      sensor_status_ = status_entry;
      return false;
    }
    if (!exit_.read_distance(sensor_, status_exit)) {
      sensor_status_ = status_exit;
      return false;
    }

    sensor_status_ = (status_entry != VL53L1_ERROR_NONE) ? status_entry : status_exit;
    path_tracking();

    if (runtime_cfg.adaptive_threshold_enabled) {
      update_adaptive_thresholds();
    }

    maybe_log_debug_sample(millis());
    return true;
  }

  bool recalibrate() {
    Serial.println("[PPC] Recalibrating zones");

    if (orientation_ == ppc::Orientation::Parallel) {
      entry_.reset_roi(167);
      exit_.reset_roi(231);
    } else {
      entry_.reset_roi(195);
      exit_.reset_roi(60);
    }

    if (!calibrate_distance()) {
      return false;
    }

    entry_.roi_calibration(entry_.threshold.idle, exit_.threshold.idle, orientation_);
    if (!entry_.calibrate_threshold(sensor_, 20)) {
      return false;
    }

    exit_.roi_calibration(entry_.threshold.idle, exit_.threshold.idle, orientation_);
    if (!exit_.calibrate_threshold(sensor_, 20)) {
      return false;
    }

    return true;
  }

  void set_people_counter(int value) {
    if (clamped_mode_ && value < 0) {
      value = 0;
    }
    people_counter_ = value;
  }

  void set_clamped_mode(bool enabled) {
    clamped_mode_ = enabled;
    if (clamped_mode_ && people_counter_ < 0) {
      people_counter_ = 0;
    }
  }

  void apply_runtime_config() {
    entry_.set_max_samples(runtime_cfg.sampling_size);
    exit_.set_max_samples(runtime_cfg.sampling_size);
    entry_.set_threshold_percentages(runtime_cfg.threshold_min_percent, runtime_cfg.threshold_max_percent);
    exit_.set_threshold_percentages(runtime_cfg.threshold_min_percent, runtime_cfg.threshold_max_percent);
  }

  bool clamped_mode() const { return clamped_mode_; }
  int people_counter() const { return people_counter_; }
  bool presence() const { return presence_; }
  uint16_t distance_entry() const { return entry_.last_distance(); }
  uint16_t distance_exit() const { return exit_.last_distance(); }
  uint16_t min_threshold_entry() const { return entry_.threshold.min; }
  uint16_t max_threshold_entry() const { return entry_.threshold.max; }
  uint16_t min_threshold_exit() const { return exit_.threshold.min; }
  uint16_t max_threshold_exit() const { return exit_.threshold.max; }
  uint16_t roi_width_entry() const { return entry_.roi.width; }
  uint16_t roi_height_entry() const { return entry_.roi.height; }
  uint16_t roi_width_exit() const { return exit_.roi.width; }
  uint16_t roi_height_exit() const { return exit_.roi.height; }
  VL53L1_Error sensor_status() const { return sensor_status_; }
  const char *last_direction() const { return last_direction_.c_str(); }
  uint32_t total_entry_today() const { return total_entry_today_; }
  uint32_t total_exit_today() const { return total_exit_today_; }

  bool pop_event(PendingEvent &out) {
    if (!pending_event_.ready) {
      return false;
    }
    out = pending_event_;
    pending_event_.ready = false;
    pending_event_.name = "";
    return true;
  }

 private:
  static constexpr int kLeftState = 1;
  static constexpr int kRightState = 2;
  static constexpr int kBothState = 3;
  static constexpr int kBalanceDeltaMinMm = 120;

  bool calibrate_distance() {
    const ppc::RangingMode *initial = sensor_.get_ranging_mode_override();
    if (initial == nullptr) {
      initial = &ppc::Ranging::kLongest;
    }

    if (!sensor_.set_ranging_mode(initial)) {
      return false;
    }

    if (!entry_.calibrate_threshold(sensor_, 20) || !exit_.calibrate_threshold(sensor_, 20)) {
      return false;
    }

    if (sensor_.get_ranging_mode_override() != nullptr) {
      return true;
    }

    const ppc::RangingMode *mode = determine_ranging_mode(entry_.threshold.idle, exit_.threshold.idle);
    if (mode != initial) {
      return sensor_.set_ranging_mode(mode);
    }

    return true;
  }

  const ppc::RangingMode *determine_ranging_mode(uint16_t entry_idle, uint16_t exit_idle) const {
    const uint16_t min_d = std::min(entry_idle, exit_idle);
    const uint16_t max_d = std::max(entry_idle, exit_idle);

    if (min_d <= 1300) {
      return &ppc::Ranging::kShort;
    }
    if (max_d > 1300 && min_d <= 2000) {
      return &ppc::Ranging::kMedium;
    }
    if (max_d > 2000 && min_d <= 2700) {
      return &ppc::Ranging::kLong;
    }
    if (max_d > 2700 && min_d <= 3400) {
      return &ppc::Ranging::kLonger;
    }
    return &ppc::Ranging::kLongest;
  }

  void queue_event(const char *name) {
    pending_event_.ready = true;
    pending_event_.name = name;
    last_direction_ = name;
    if (runtime_cfg.serial_debug_enabled) {
      Serial.printf("[PPC][EVENT] %s count=%d\n", name, people_counter_);
    }
  }

  void reset_path_tracking() {
    tracking_active_ = false;
    prev_combined_state_ = 0;
    first_single_state_ = 0;
    last_single_state_ = 0;
    saw_both_state_ = false;
    left_peak_distance_ = 0xFFFF;
    right_peak_distance_ = 0xFFFF;
    left_peak_time_ms_ = 0;
    right_peak_time_ms_ = 0;
    have_balance_window_ = false;
    start_balance_mm_ = 0;
    last_balance_mm_ = 0;
    last_state_change_time_ = millis();
  }

  void update_counter(int delta) {
    int next = people_counter_ + delta;
    if (clamped_mode_ && next < 0) {
      next = 0;
    }
    people_counter_ = next;
  }

  void maybe_log_debug_sample(uint32_t now) {
    if (!runtime_cfg.serial_debug_enabled) {
      return;
    }
    if ((now - last_debug_sample_ms_) < runtime_cfg.serial_debug_sample_interval_ms) {
      return;
    }
    last_debug_sample_ms_ = now;

    Serial.printf("[PPC][DBG] d0=%u d1=%u th0=%u-%u th1=%u-%u occ0=%d occ1=%d cnt=%d pres=%d dir=%s\n",
                  entry_.last_distance(),
                  exit_.last_distance(),
                  entry_.threshold.min,
                  entry_.threshold.max,
                  exit_.threshold.min,
                  exit_.threshold.max,
                  entry_.is_occupied() ? 1 : 0,
                  exit_.is_occupied() ? 1 : 0,
                  people_counter_,
                  presence_ ? 1 : 0,
                  last_direction_.c_str());
  }

  int combined_state(bool &left_occupied,
                     bool &right_occupied,
                     uint16_t &left_distance,
                     uint16_t &right_distance) const {
    if (runtime_cfg.invert_direction) {
      left_occupied = exit_.is_occupied();
      right_occupied = entry_.is_occupied();
      left_distance = exit_.min_distance();
      right_distance = entry_.min_distance();
    } else {
      left_occupied = entry_.is_occupied();
      right_occupied = exit_.is_occupied();
      left_distance = entry_.min_distance();
      right_distance = exit_.min_distance();
    }

    return (left_occupied ? kLeftState : 0) + (right_occupied ? kRightState : 0);
  }

  int resolve_event_delta(uint32_t now) const {
    int delta = 0;
    const char *reason = "none";

    if (saw_both_state_ && first_single_state_ != 0 && last_single_state_ != 0 &&
        first_single_state_ != last_single_state_) {
      if (first_single_state_ == kRightState && last_single_state_ == kLeftState) {
        delta = 1;
      } else if (first_single_state_ == kLeftState && last_single_state_ == kRightState) {
        delta = -1;
      }
      if (delta != 0) {
        reason = "first_last";
      }
    }

    if (delta == 0 && saw_both_state_ && left_peak_time_ms_ != 0 && right_peak_time_ms_ != 0) {
      const uint32_t peak_dt = (left_peak_time_ms_ > right_peak_time_ms_) ? (left_peak_time_ms_ - right_peak_time_ms_)
                                                                           : (right_peak_time_ms_ - left_peak_time_ms_);
      if (peak_dt >= runtime_cfg.peak_time_delta_ms) {
        if (right_peak_time_ms_ < left_peak_time_ms_) {
          delta = 1;
        } else {
          delta = -1;
        }
        reason = "peak_time";
      }
    }

    if (delta == 0 && saw_both_state_ && have_balance_window_) {
      const int abs_start = (start_balance_mm_ < 0) ? -start_balance_mm_ : start_balance_mm_;
      const int abs_end = (last_balance_mm_ < 0) ? -last_balance_mm_ : last_balance_mm_;
      if (abs_start >= kBalanceDeltaMinMm && abs_end >= kBalanceDeltaMinMm) {
        if (start_balance_mm_ < 0 && last_balance_mm_ > 0) {
          delta = 1;
          reason = "balance_cross";
        } else if (start_balance_mm_ > 0 && last_balance_mm_ < 0) {
          delta = -1;
          reason = "balance_cross";
        }
      }
    }

    if (runtime_cfg.serial_debug_enabled) {
      Serial.printf("[PPC][TRACE] finalize reason=%s first=%d last=%d both=%d peakL=%lu peakR=%lu bal=%d->%d now=%lu delta=%d\n",
                    reason,
                    first_single_state_,
                    last_single_state_,
                    saw_both_state_ ? 1 : 0,
                    left_peak_time_ms_,
                    right_peak_time_ms_,
                    start_balance_mm_,
                    last_balance_mm_,
                    now,
                    delta);
    }

    return delta;
  }
  void commit_event_delta(int delta, uint32_t now) {
    if (delta == 0) {
      return;
    }

    if ((now - last_count_event_ms_) < runtime_cfg.event_cooldown_ms) {
      if (runtime_cfg.serial_debug_enabled) {
        Serial.printf("[PPC][TRACE] %s ignored by cooldown (%lu ms)\n",
                      delta > 0 ? "Entry" : "Exit",
                      now - last_count_event_ms_);
      }
      return;
    }

    if (delta > 0) {
      total_entry_today_++;
      update_counter(1);
      queue_event("Entry");
    } else {
      total_exit_today_++;
      update_counter(-1);
      queue_event("Exit");
    }
    last_count_event_ms_ = now;
  }

  void path_tracking() {
    const uint32_t now = millis();
    bool left_occupied = false;
    bool right_occupied = false;
    uint16_t left_distance = 0;
    uint16_t right_distance = 0;
    const int state = combined_state(left_occupied, right_occupied, left_distance, right_distance);

    presence_ = (state != 0);

    if (!tracking_active_) {
      if (state == 0) {
        return;
      }
      reset_path_tracking();
      tracking_active_ = true;
      prev_combined_state_ = state;
      last_state_change_time_ = now;
    }

    if (runtime_cfg.path_tracking_timeout_ms > 0 && (now - last_state_change_time_) > runtime_cfg.path_tracking_timeout_ms) {
      queue_event("Timeout");
      if (runtime_cfg.serial_debug_enabled) {
        Serial.printf("[PPC][TRACE] timeout state=%d age=%lu\n", prev_combined_state_, now - last_state_change_time_);
      }
      reset_path_tracking();
      return;
    }

    if (state == kBothState) {
      saw_both_state_ = true;
    }

    if (state == kLeftState || state == kRightState) {
      if (first_single_state_ == 0) {
        first_single_state_ = state;
      }
      last_single_state_ = state;
    }

    if (left_occupied && left_distance > 0 && left_distance < left_peak_distance_) {
      left_peak_distance_ = left_distance;
      left_peak_time_ms_ = now;
    }
    if (right_occupied && right_distance > 0 && right_distance < right_peak_distance_) {
      right_peak_distance_ = right_distance;
      right_peak_time_ms_ = now;
    }
    if (state != 0 && left_distance > 0 && right_distance > 0) {
      const int balance_mm = static_cast<int>(right_distance) - static_cast<int>(left_distance);
      if (!have_balance_window_) {
        start_balance_mm_ = balance_mm;
        have_balance_window_ = true;
      }
      last_balance_mm_ = balance_mm;
    }

    if (state != prev_combined_state_) {
      if (runtime_cfg.serial_debug_enabled) {
        Serial.printf("[PPC][TRACE] state %d->%d left=%d right=%d first=%d last=%d both=%d\n",
                      prev_combined_state_,
                      state,
                      left_occupied ? 1 : 0,
                      right_occupied ? 1 : 0,
                      first_single_state_,
                      last_single_state_,
                      saw_both_state_ ? 1 : 0);
      }
      prev_combined_state_ = state;
      last_state_change_time_ = now;
    }

    if (state == 0) {
      const int delta = resolve_event_delta(now);
      commit_event_delta(delta, now);
      reset_path_tracking();
    }
  }

  void update_adaptive_thresholds() {
    const uint32_t now = millis();
    const bool both_zones_empty = !entry_.is_occupied() && !exit_.is_occupied();

    if (both_zones_empty) {
      if (zones_were_occupied_) {
        zones_empty_since_ = now;
        zones_were_occupied_ = false;
      }

      const uint32_t empty_duration = now - zones_empty_since_;
      if (empty_duration >= runtime_cfg.adaptive_threshold_interval_ms &&
          (now - last_adaptive_update_time_) >= runtime_cfg.adaptive_threshold_interval_ms) {
        entry_.update_adaptive_threshold(runtime_cfg.adaptive_threshold_alpha);
        exit_.update_adaptive_threshold(runtime_cfg.adaptive_threshold_alpha);
        last_adaptive_update_time_ = now;
      }
    } else {
      zones_were_occupied_ = true;
    }
  }

  void maybe_reset_daily_totals() {
    time_t now = time(nullptr);
    if (now < 100000) {
      return;
    }

    struct tm local_tm;
    if (localtime_r(&now, &local_tm) == nullptr) {
      return;
    }

    if (last_reset_day_of_year_ < 0) {
      last_reset_day_of_year_ = local_tm.tm_yday;
      return;
    }

    if (local_tm.tm_yday != last_reset_day_of_year_) {
      last_reset_day_of_year_ = local_tm.tm_yday;
      total_entry_today_ = 0;
      total_exit_today_ = 0;
    }
  }

  ppc::Vl53l1xDevice sensor_{};
  ppc::Zone entry_{0};
  ppc::Zone exit_{1};

  ppc::Orientation orientation_{ppc::Orientation::Parallel};

  int people_counter_{0};
  bool clamped_mode_{cfg::kClampCounterAtZero};
  bool presence_{false};
  String last_direction_{"None"};
  VL53L1_Error sensor_status_{VL53L1_ERROR_NONE};

  uint32_t total_entry_today_{0};
  uint32_t total_exit_today_{0};
  int last_reset_day_of_year_{-1};

  bool tracking_active_{false};
  int prev_combined_state_{0};
  int first_single_state_{0};
  int last_single_state_{0};
  bool saw_both_state_{false};
  uint16_t left_peak_distance_{0xFFFF};
  uint16_t right_peak_distance_{0xFFFF};
  uint32_t left_peak_time_ms_{0};
  uint32_t right_peak_time_ms_{0};
  bool have_balance_window_{false};
  int start_balance_mm_{0};
  int last_balance_mm_{0};
  uint32_t last_state_change_time_{0};

  uint32_t last_adaptive_update_time_{0};
  uint32_t zones_empty_since_{0};
  bool zones_were_occupied_{false};
  uint32_t last_count_event_ms_{0};

  uint32_t last_debug_sample_ms_{0};

  PendingEvent pending_event_{};
};

CounterEngine engine;

String iso_utc_now() {
  time_t now = time(nullptr);
  if (now < 100000) {
    return String("0");
  }

  struct tm utc_tm;
  if (gmtime_r(&now, &utc_tm) == nullptr) {
    return String("0");
  }

  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);
  return String(buffer);
}

void publish_json(const String &topic, JsonDocument &doc, bool retained) {
  String payload;
  serializeJson(doc, payload);
  mqtt_client.publish(topic.c_str(), payload.c_str(), retained);
}

void publish_ack(const String &request_id, bool ok, const String &message) {
  JsonDocument doc;
  doc["ts"] = iso_utc_now();
  doc["request_id"] = request_id;
  doc["ok"] = ok;
  doc["message"] = message;
  publish_json(topic_ack, doc, false);
}

void append_runtime_config(JsonObject obj) {
  obj["invert_direction"] = runtime_cfg.invert_direction;
  obj["sampling"] = runtime_cfg.sampling_size;
  obj["threshold_min_percent"] = runtime_cfg.threshold_min_percent;
  obj["threshold_max_percent"] = runtime_cfg.threshold_max_percent;
  obj["path_tracking_timeout_ms"] = runtime_cfg.path_tracking_timeout_ms;
  obj["event_cooldown_ms"] = runtime_cfg.event_cooldown_ms;
  obj["peak_time_delta_ms"] = runtime_cfg.peak_time_delta_ms;

  JsonObject adaptive = obj["adaptive"].to<JsonObject>();
  adaptive["enabled"] = runtime_cfg.adaptive_threshold_enabled;
  adaptive["interval_ms"] = runtime_cfg.adaptive_threshold_interval_ms;
  adaptive["alpha"] = runtime_cfg.adaptive_threshold_alpha;

  JsonObject debug = obj["debug"].to<JsonObject>();
  debug["serial_enabled"] = runtime_cfg.serial_debug_enabled;
  debug["sample_interval_ms"] = runtime_cfg.serial_debug_sample_interval_ms;
}

void publish_meta() {
  JsonDocument doc;
  doc["fw_version"] = kFwVersion;
  doc["device_id"] = cfg::kDeviceId;
  doc["customer_id"] = cfg::kCustomerId;
  doc["mqtt_tls"] = cfg::kMqttUseTls;
  doc["schema"] = "ppc/v1";
  doc["features"]["commands"] = true;
  doc["features"]["daily_totals"] = true;
  doc["features"]["adaptive_threshold"] = true;
  doc["features"]["dynamic_config"] = true;
  doc["features"]["serial_debug"] = true;
  publish_json(topic_meta, doc, true);
}

void publish_state() {
  JsonDocument doc;
  doc["ts"] = iso_utc_now();
  doc["seq"] = state_seq++;
  doc["people_count"] = engine.people_counter();
  doc["presence"] = engine.presence();
  doc["last_direction"] = engine.last_direction();

  JsonObject today = doc["today"].to<JsonObject>();
  today["entry"] = engine.total_entry_today();
  today["exit"] = engine.total_exit_today();

  JsonObject zones = doc["zones"].to<JsonObject>();
  zones["z0_mm"] = engine.distance_entry();
  zones["z1_mm"] = engine.distance_exit();
  zones["min0"] = engine.min_threshold_entry();
  zones["max0"] = engine.max_threshold_entry();
  zones["min1"] = engine.min_threshold_exit();
  zones["max1"] = engine.max_threshold_exit();
  zones["roi0_w"] = engine.roi_width_entry();
  zones["roi0_h"] = engine.roi_height_entry();
  zones["roi1_w"] = engine.roi_width_exit();
  zones["roi1_h"] = engine.roi_height_exit();

  JsonObject health = doc["health"].to<JsonObject>();
  health["rssi"] = WiFi.RSSI();
  health["uptime_s"] = millis() / 1000UL;
  health["sensor_status"] = static_cast<int>(engine.sensor_status());

  JsonObject config = doc["config"].to<JsonObject>();
  append_runtime_config(config);

  publish_json(topic_state, doc, true);
}

void publish_event(const PendingEvent &ev) {
  JsonDocument doc;
  doc["ts"] = iso_utc_now();
  doc["event"] = ev.name;
  doc["count_after"] = engine.people_counter();
  publish_json(topic_event, doc, false);
}

void setup_topics() {
  base_topic = String("ppc/v1/c/") + cfg::kCustomerId + "/d/" + cfg::kDeviceId;
  topic_state = base_topic + "/state";
  topic_event = base_topic + "/event";
  topic_cmd = base_topic + "/cmd";
  topic_ack = base_topic + "/ack";
  topic_availability = base_topic + "/availability";
  topic_meta = base_topic + "/meta";
}

void connect_wifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if ((now - last_wifi_reconnect_ms) < 5000) {
    return;
  }
  last_wifi_reconnect_ms = now;

  Serial.printf("[WiFi] Connecting to %s\n", cfg::kWifiSsid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg::kWifiSsid, cfg::kWifiPassword);
}

bool apply_runtime_config_from_json(JsonObjectConst cfg_obj, String &error_message) {
  bool changed_any = false;
  bool needs_recalibration = false;

  JsonVariantConst v_sampling = cfg_obj["sampling"];
  if (!v_sampling.isNull()) {
    uint32_t sampling = 0;
    if (!parse_uint32(v_sampling, sampling) || sampling < 1 || sampling > 16) {
      error_message = "invalid_sampling";
      return false;
    }
    runtime_cfg.sampling_size = static_cast<uint8_t>(sampling);
    changed_any = true;
    needs_recalibration = true;
  }

  JsonVariantConst v_min = cfg_obj["threshold_min_percent"];
  if (!v_min.isNull()) {
    uint32_t min_percent = 0;
    if (!parse_uint32(v_min, min_percent) || min_percent > 100) {
      error_message = "invalid_threshold_min_percent";
      return false;
    }
    runtime_cfg.threshold_min_percent = static_cast<uint8_t>(min_percent);
    changed_any = true;
    needs_recalibration = true;
  }

  JsonVariantConst v_max = cfg_obj["threshold_max_percent"];
  if (!v_max.isNull()) {
    uint32_t max_percent = 0;
    if (!parse_uint32(v_max, max_percent) || max_percent > 100) {
      error_message = "invalid_threshold_max_percent";
      return false;
    }
    runtime_cfg.threshold_max_percent = static_cast<uint8_t>(max_percent);
    changed_any = true;
    needs_recalibration = true;
  }

  if (runtime_cfg.threshold_min_percent >= runtime_cfg.threshold_max_percent) {
    error_message = "threshold_min_must_be_less_than_max";
    return false;
  }

  JsonVariantConst v_timeout = cfg_obj["path_tracking_timeout_ms"];
  if (!v_timeout.isNull()) {
    uint32_t timeout_ms = 0;
    if (!parse_uint32(v_timeout, timeout_ms) || timeout_ms > 60000) {
      error_message = "invalid_path_tracking_timeout_ms";
      return false;
    }
    runtime_cfg.path_tracking_timeout_ms = timeout_ms;
    changed_any = true;
  }

  JsonVariantConst v_event_cooldown = cfg_obj["event_cooldown_ms"];
  if (!v_event_cooldown.isNull()) {
    uint32_t cooldown_ms = 0;
    if (!parse_uint32(v_event_cooldown, cooldown_ms) || cooldown_ms > 10000) {
      error_message = "invalid_event_cooldown_ms";
      return false;
    }
    runtime_cfg.event_cooldown_ms = cooldown_ms;
    changed_any = true;
  }


  JsonVariantConst v_peak_delta = cfg_obj["peak_time_delta_ms"];
  if (!v_peak_delta.isNull()) {
    uint32_t peak_delta_ms = 0;
    if (!parse_uint32(v_peak_delta, peak_delta_ms) || peak_delta_ms > 2000) {
      error_message = "invalid_peak_time_delta_ms";
      return false;
    }
    runtime_cfg.peak_time_delta_ms = peak_delta_ms;
    changed_any = true;
  }
  JsonVariantConst v_invert = cfg_obj["invert_direction"];
  if (!v_invert.isNull()) {
    if (!v_invert.is<bool>()) {
      error_message = "invalid_invert_direction";
      return false;
    }
    runtime_cfg.invert_direction = v_invert.as<bool>();
    changed_any = true;
  }

  JsonVariantConst v_debug_enabled = cfg_obj["debug_serial"];
  if (!v_debug_enabled.isNull()) {
    if (!v_debug_enabled.is<bool>()) {
      error_message = "invalid_debug_serial";
      return false;
    }
    runtime_cfg.serial_debug_enabled = v_debug_enabled.as<bool>();
    changed_any = true;
  }

  JsonVariantConst v_debug_interval = cfg_obj["debug_interval_ms"];
  if (!v_debug_interval.isNull()) {
    uint32_t interval_ms = 0;
    if (!parse_uint32(v_debug_interval, interval_ms) || interval_ms < 20 || interval_ms > 10000) {
      error_message = "invalid_debug_interval_ms";
      return false;
    }
    runtime_cfg.serial_debug_sample_interval_ms = interval_ms;
    changed_any = true;
  }

  JsonVariantConst adaptive_obj_var = cfg_obj["adaptive"];
  if (!adaptive_obj_var.isNull()) {
    if (!adaptive_obj_var.is<JsonObjectConst>()) {
      error_message = "invalid_adaptive_object";
      return false;
    }

    JsonObjectConst adaptive_obj = adaptive_obj_var.as<JsonObjectConst>();

    JsonVariantConst v_adaptive_enabled = adaptive_obj["enabled"];
    if (!v_adaptive_enabled.isNull()) {
      if (!v_adaptive_enabled.is<bool>()) {
        error_message = "invalid_adaptive_enabled";
        return false;
      }
      runtime_cfg.adaptive_threshold_enabled = v_adaptive_enabled.as<bool>();
      changed_any = true;
    }

    JsonVariantConst v_adaptive_interval = adaptive_obj["interval_ms"];
    if (!v_adaptive_interval.isNull()) {
      uint32_t interval_ms = 0;
      if (!parse_uint32(v_adaptive_interval, interval_ms) || interval_ms < 1000 || interval_ms > 3600000) {
        error_message = "invalid_adaptive_interval_ms";
        return false;
      }
      runtime_cfg.adaptive_threshold_interval_ms = interval_ms;
      changed_any = true;
    }

    JsonVariantConst v_adaptive_alpha = adaptive_obj["alpha"];
    if (!v_adaptive_alpha.isNull()) {
      float alpha = 0.0f;
      if (!parse_float(v_adaptive_alpha, alpha) || alpha <= 0.0f || alpha > 1.0f) {
        error_message = "invalid_adaptive_alpha";
        return false;
      }
      runtime_cfg.adaptive_threshold_alpha = alpha;
      changed_any = true;
    }
  }

  if (!changed_any) {
    error_message = "no_supported_fields";
    return false;
  }

  engine.apply_runtime_config();
  if (needs_recalibration && !engine.recalibrate()) {
    error_message = "recalibration_failed_after_config_update";
    return false;
  }

  return true;
}

void mqtt_callback(char *topic, uint8_t *payload, unsigned int length) {
  if (String(topic) != topic_cmd) {
    return;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    publish_ack("", false, String("invalid_json: ") + err.c_str());
    return;
  }

  String request_id = doc["request_id"].is<const char *>() ? String(doc["request_id"].as<const char *>()) : "";
  String cmd = doc["cmd"].is<const char *>() ? String(doc["cmd"].as<const char *>()) : "";

  if (cmd == "recalibrate") {
    bool ok = engine.recalibrate();
    publish_ack(request_id, ok, ok ? "recalibrated" : "recalibration_failed");
    publish_state();
    return;
  }

  if (cmd == "get_config") {
    publish_ack(request_id, true, "config_in_state");
    publish_state();
    return;
  }

  if (cmd == "set_config") {
    JsonObjectConst cfg_obj = doc["config"].is<JsonObjectConst>() ? doc["config"].as<JsonObjectConst>()
                                                                    : doc.as<JsonObjectConst>();
    String error_message;
    bool ok = apply_runtime_config_from_json(cfg_obj, error_message);
    publish_ack(request_id, ok, ok ? "config_updated" : error_message);
    publish_state();
    return;
  }

  if (cmd == "set_people_counter") {
    if (!doc["value"].is<int>()) {
      publish_ack(request_id, false, "missing_integer_value");
      return;
    }
    engine.set_people_counter(doc["value"].as<int>());
    publish_ack(request_id, true, "people_counter_updated");
    publish_state();
    return;
  }

  if (cmd == "set_clamped") {
    if (!doc["value"].is<bool>()) {
      publish_ack(request_id, false, "missing_boolean_value");
      return;
    }
    engine.set_clamped_mode(doc["value"].as<bool>());
    publish_ack(request_id, true, "clamped_mode_updated");
    publish_state();
    return;
  }

  if (cmd == "restart") {
    publish_ack(request_id, true, "restarting");
    delay(250);
    ESP.restart();
    return;
  }

  publish_ack(request_id, false, "unsupported_command");
}

void connect_mqtt() {
  if (WiFi.status() != WL_CONNECTED || mqtt_client.connected()) {
    return;
  }

  const uint32_t now = millis();
  if ((now - last_mqtt_reconnect_ms) < 5000) {
    return;
  }
  last_mqtt_reconnect_ms = now;

  uint64_t chip = ESP.getEfuseMac();
  char client_id[80];
  snprintf(client_id, sizeof(client_id), "%s-%04X", cfg::kDeviceId, static_cast<unsigned>(chip & 0xFFFFU));

  bool connected = mqtt_client.connect(client_id,
                                       cfg::kMqttUsername,
                                       cfg::kMqttPassword,
                                       topic_availability.c_str(),
                                       1,
                                       true,
                                       "offline",
                                       true);

  if (!connected) {
    Serial.printf("[MQTT] Connect failed rc=%d\n", mqtt_client.state());
    return;
  }

  Serial.println("[MQTT] Connected");
  mqtt_client.subscribe(topic_cmd.c_str(), 1);
  mqtt_client.publish(topic_availability.c_str(), "online", true);
  publish_meta();
  publish_state();
}

void setup_mqtt_transport() {
  mqtt_client.setServer(cfg::kMqttHost, cfg::kMqttUseTls ? cfg::kMqttPortTls : cfg::kMqttPortPlain);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_client.setBufferSize(1536);

  if (cfg::kMqttUseTls) {
    if (strlen(cfg::kMqttCaCert) > 0) {
      tls_client.setCACert(cfg::kMqttCaCert);
    } else {
      Serial.println("[MQTT] TLS enabled without CA cert. Falling back to insecure TLS for testing.");
      tls_client.setInsecure();
    }

    if (strlen(cfg::kMqttClientCert) > 0 && strlen(cfg::kMqttClientKey) > 0) {
      tls_client.setCertificate(cfg::kMqttClientCert);
      tls_client.setPrivateKey(cfg::kMqttClientKey);
    }

    mqtt_client.setClient(tls_client);
  } else {
    mqtt_client.setClient(plain_client);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  setup_topics();

  configTime(cfg::kGmtOffsetSec, cfg::kDstOffsetSec, cfg::kNtpServer1, cfg::kNtpServer2);

  connect_wifi();
  setup_mqtt_transport();

  if (!engine.begin()) {
    Serial.println("[PPC] Engine init failed, rebooting in 5 seconds");
    delay(5000);
    ESP.restart();
  }

  Serial.println("[PPC] Setup complete");
}

void loop() {
  connect_wifi();
  connect_mqtt();

  if (mqtt_client.connected()) {
    mqtt_client.loop();
  }

  const uint32_t now = millis();
  if ((now - last_sensor_loop_ms) >= cfg::kSensorLoopIntervalMs) {
    last_sensor_loop_ms = now;
    engine.loop_once();

    PendingEvent ev;
    if (engine.pop_event(ev) && mqtt_client.connected()) {
      publish_event(ev);
    }
  }

  if ((now - last_state_publish_ms) >= cfg::kStatePublishIntervalMs) {
    last_state_publish_ms = now;
    if (mqtt_client.connected()) {
      publish_state();
    }
  }

  delay(2);
}


