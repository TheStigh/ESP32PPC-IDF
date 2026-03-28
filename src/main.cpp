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

class CounterEngine {
 public:
  bool begin() {
    entry_.set_max_samples(cfg::kSamplingSize);
    exit_.set_max_samples(cfg::kSamplingSize);
    entry_.set_threshold_percentages(cfg::kThresholdMinPercent, cfg::kThresholdMaxPercent);
    exit_.set_threshold_percentages(cfg::kThresholdMinPercent, cfg::kThresholdMaxPercent);

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
    last_state_change_time_ = now;
    last_adaptive_update_time_ = now;
    zones_empty_since_ = now;

    Serial.println("[PPC] Counter engine ready");
    return true;
  }

  bool loop_once() {
    maybe_reset_daily_totals();

    VL53L1_Error status = VL53L1_ERROR_NONE;
    if (!current_zone_->read_distance(sensor_, status)) {
      sensor_status_ = status;
      return false;
    }

    sensor_status_ = status;
    path_tracking(*current_zone_);

    if (cfg::kAdaptiveThresholdEnabled) {
      update_adaptive_thresholds();
    }

    current_zone_ = (current_zone_ == &entry_) ? &exit_ : &entry_;
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
    entry_.calibrate_threshold(sensor_, 20);
    exit_.roi_calibration(entry_.threshold.idle, exit_.threshold.idle, orientation_);
    exit_.calibrate_threshold(sensor_, 20);

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
  bool calibrate_distance() {
    const ppc::RangingMode *initial = sensor_.get_ranging_mode_override();
    if (initial == nullptr) {
      initial = &ppc::Ranging::kLongest;
    }

    if (!sensor_.set_ranging_mode(initial)) {
      return false;
    }

    entry_.calibrate_threshold(sensor_, 20);
    exit_.calibrate_threshold(sensor_, 20);

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
  }

  void reset_path_tracking() {
    for (int &v : path_track_) {
      v = 0;
    }
    path_track_filling_size_ = 1;
    left_previous_status_ = kNobody;
    right_previous_status_ = kNobody;
    last_state_change_time_ = millis();
  }

  void update_counter(int delta) {
    int next = people_counter_ + delta;
    if (clamped_mode_ && next < 0) {
      next = 0;
    }
    people_counter_ = next;
  }

  void path_tracking(ppc::Zone &zone) {
    int current_zone_status = kNobody;
    int all_zones_current_status = 0;
    int event_has_occurred = 0;
    uint32_t now = millis();

    if (path_track_filling_size_ > 1 && cfg::kPathTrackingTimeoutMs > 0) {
      if ((now - last_state_change_time_) > cfg::kPathTrackingTimeoutMs) {
        queue_event("Timeout");
        reset_path_tracking();
        return;
      }
    }

    if (zone.min_distance() < zone.threshold.max && zone.min_distance() > zone.threshold.min) {
      current_zone_status = kSomeone;
      presence_ = true;
    }

    bool is_left_zone = (&zone == (cfg::kInvertDirection ? &exit_ : &entry_));
    if (is_left_zone) {
      if (current_zone_status != left_previous_status_) {
        event_has_occurred = 1;
        if (current_zone_status == kSomeone) {
          all_zones_current_status += 1;
        }
        if (right_previous_status_ == kSomeone) {
          all_zones_current_status += 2;
        }
        left_previous_status_ = current_zone_status;
      }
    } else {
      if (current_zone_status != right_previous_status_) {
        event_has_occurred = 1;
        if (current_zone_status == kSomeone) {
          all_zones_current_status += 2;
        }
        if (left_previous_status_ == kSomeone) {
          all_zones_current_status += 1;
        }
        right_previous_status_ = current_zone_status;
      }
    }

    if (event_has_occurred) {
      last_state_change_time_ = now;
      if (path_track_filling_size_ < 4) {
        path_track_filling_size_++;
      }

      if (left_previous_status_ == kNobody && right_previous_status_ == kNobody) {
        if (path_track_filling_size_ == 4) {
          if (path_track_[1] == 1 && path_track_[2] == 3 && path_track_[3] == 2) {
            total_exit_today_++;
            update_counter(-1);
            queue_event("Exit");
          } else if (path_track_[1] == 2 && path_track_[2] == 3 && path_track_[3] == 1) {
            total_entry_today_++;
            update_counter(1);
            queue_event("Entry");
          }
        }
        reset_path_tracking();
      } else {
        path_track_[path_track_filling_size_ - 1] = all_zones_current_status;
      }
    }

    if (current_zone_status == kNobody && left_previous_status_ == kNobody && right_previous_status_ == kNobody) {
      presence_ = false;
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
      if (empty_duration >= cfg::kAdaptiveThresholdIntervalMs &&
          (now - last_adaptive_update_time_) >= cfg::kAdaptiveThresholdIntervalMs) {
        entry_.update_adaptive_threshold(cfg::kAdaptiveThresholdAlpha);
        exit_.update_adaptive_threshold(cfg::kAdaptiveThresholdAlpha);
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
  ppc::Zone *current_zone_{&entry_};

  ppc::Orientation orientation_{ppc::Orientation::Parallel};

  int people_counter_{0};
  bool clamped_mode_{cfg::kClampCounterAtZero};
  bool presence_{false};
  String last_direction_{"None"};
  VL53L1_Error sensor_status_{VL53L1_ERROR_NONE};

  uint32_t total_entry_today_{0};
  uint32_t total_exit_today_{0};
  int last_reset_day_of_year_{-1};

  int path_track_[4] = {0, 0, 0, 0};
  int path_track_filling_size_{1};
  int left_previous_status_{kNobody};
  int right_previous_status_{kNobody};
  uint32_t last_state_change_time_{0};

  uint32_t last_adaptive_update_time_{0};
  uint32_t zones_empty_since_{0};
  bool zones_were_occupied_{false};

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

void publish_meta() {
  JsonDocument doc;
  doc["fw_version"] = kFwVersion;
  doc["device_id"] = cfg::kDeviceId;
  doc["customer_id"] = cfg::kCustomerId;
  doc["mqtt_tls"] = cfg::kMqttUseTls;
  doc["schema"] = "ppc/v1";
  doc["features"]["commands"] = true;
  doc["features"]["daily_totals"] = true;
  doc["features"]["adaptive_threshold"] = cfg::kAdaptiveThresholdEnabled;
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

