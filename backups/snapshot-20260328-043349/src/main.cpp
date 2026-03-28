#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <algorithm>
#include <ctime>
#include <cstring>
#include <optional>
#include <string>
#include "app_config.h"
#include "ismart_logo.h"
#include "ppc_zone.h"
#include "vl53l1x_device.h"

namespace {
  constexpr uint8_t kNobody = 0;
  constexpr uint8_t kSomeone = 1;
  constexpr char kFwVersion[] = "standalone-0.1.0";
  constexpr uint16_t kPersistedSettingsSchema = 1;
  constexpr char kPrefsNamespace[] = "esp32ppc";
  constexpr char kPrefsKeySettings[] = "settings";
  constexpr uint16_t kProvisioningSchema = 1;
  constexpr char kPrefsKeyProvisioning[] = "provision";
  constexpr uint8_t kProvisionPortalDnsPort = 53;
  const IPAddress kProvisionApIp(192, 168, 4, 1);
  const IPAddress kProvisionApGateway(192, 168, 4, 1);
  const IPAddress kProvisionApSubnet(255, 255, 255, 0);
  constexpr size_t kMaxCaCertLength = 4096;
  struct PersistedCalibration {
     bool valid {
      false
    };
     char ranging_mode[16] {
      "Longest"
    };
     uint16_t z0_idle_mm {
      0
    };
     uint16_t z0_min_mm {
      0
    };
     uint16_t z0_max_mm {
      0
    };
     uint8_t z0_roi_w {
      0
    };
     uint8_t z0_roi_h {
      0
    };
     uint8_t z0_roi_center {
      0
    };
     uint16_t z1_idle_mm {
      0
    };
     uint16_t z1_min_mm {
      0
    };
     uint16_t z1_max_mm {
      0
    };
     uint8_t z1_roi_w {
      0
    };
     uint8_t z1_roi_h {
      0
    };
     uint8_t z1_roi_center {
      0
    };
    
  };
  PersistedCalibration boot_calibration {
    
  };
  bool boot_clamped_mode = cfg::kClampCounterAtZero;
  int boot_people_counter = 0;
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
     bool door_protection_enabled;
     uint16_t door_protection_mm;
    
  };
  RuntimeConfig runtime_cfg {
     cfg::kInvertDirection, cfg::kSamplingSize, cfg::kThresholdMinPercent, cfg::kThresholdMaxPercent, cfg::kPathTrackingTimeoutMs, cfg::kEventCooldownMs, cfg::kPeakTimeDeltaMs, cfg::kAdaptiveThresholdEnabled, cfg::kAdaptiveThresholdIntervalMs, cfg::kAdaptiveThresholdAlpha, cfg::kSerialDebugEnabled, cfg::kSerialDebugSampleIntervalMs, cfg::kDoorProtectionEnabled, cfg::kDoorProtectionDistanceMm,
  };
  struct ProvisioningConfig {
     String wifi_ssid;
     String wifi_password;
     String mqtt_host;
     uint16_t mqtt_port_plain;
     uint16_t mqtt_port_tls;
     bool mqtt_use_tls;
     String mqtt_username;
     String mqtt_password;
     String mqtt_ca_cert;
     String customer_id;
     String device_id;

  };
  ProvisioningConfig provisioning_cfg;
  WebServer provisioning_server(80);
  DNSServer provisioning_dns;
  bool provisioning_restart_requested = false;
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
  uint32_t last_health_sample_ms = 0;
  int cached_rssi = 0;
  uint32_t cached_uptime_s = 0;
  bool health_sample_initialized = false;
  struct PendingEvent {
     bool ready {
      false
    };
     String name {
      
    };
    
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
  bool parse_bool(JsonVariantConst value, bool &out) {
     if (value.is<bool>()) {
       out = value.as<bool>();
       return true;
      
    }
     if (value.is<int>()) {
       int v = value.as<int>();
       if (v == 0) {
         out = false;
         return true;
        
      }
       if (v == 1) {
         out = true;
         return true;
        
      }
       return false;
      
    }
     if (value.is<const char *>()) {
       String v = value.as<const char *>();
       v.toLowerCase();
       if (v == "true" || v == "1" || v == "on") {
         out = true;
         return true;
        
      }
       if (v == "false" || v == "0" || v == "off") {
         out = false;
         return true;
        
      }
      
    }
     return false;
    
  }

  String trim_copy(String value) {
     value.trim();
     return value;

  }

  bool parse_port_string(const String &value, uint16_t &out) {
     const String trimmed = trim_copy(value);
     if (trimmed.isEmpty()) {
       return false;
    }
     char *end = nullptr;
     unsigned long parsed = strtoul(trimmed.c_str(), &end, 10);
     if (end == nullptr || *end != '\0' || parsed == 0 || parsed > 65535UL) {
       return false;
    }
     out = static_cast<uint16_t>(parsed);
     return true;

  }

  String default_device_id() {
     uint64_t chip = ESP.getEfuseMac();
     char id[40];
     snprintf(id, sizeof(id), "esp32ppc-%04X", static_cast<unsigned>(chip & 0xFFFFU));
     return String(id);

  }

  void reset_provisioning_defaults() {
     provisioning_cfg.wifi_ssid = cfg::kWifiSsid;
     provisioning_cfg.wifi_password = cfg::kWifiPassword;
     provisioning_cfg.mqtt_host = cfg::kMqttHost;
     provisioning_cfg.mqtt_port_plain = cfg::kMqttPortPlain;
     provisioning_cfg.mqtt_port_tls = cfg::kMqttPortTls;
     provisioning_cfg.mqtt_use_tls = cfg::kMqttUseTls;
     provisioning_cfg.mqtt_username = cfg::kMqttUsername;
     provisioning_cfg.mqtt_password = cfg::kMqttPassword;
     provisioning_cfg.mqtt_ca_cert = cfg::kMqttCaCert;
     provisioning_cfg.customer_id = cfg::kCustomerId;
     provisioning_cfg.device_id = cfg::kDeviceId;
     if (trim_copy(provisioning_cfg.device_id).isEmpty()) {
       provisioning_cfg.device_id = default_device_id();
    }

  }

  bool provisioning_config_valid() {
     return !trim_copy(provisioning_cfg.wifi_ssid).isEmpty() && !trim_copy(provisioning_cfg.mqtt_host).isEmpty() && provisioning_cfg.mqtt_port_plain > 0 && provisioning_cfg.mqtt_port_tls > 0 && !trim_copy(provisioning_cfg.customer_id).isEmpty() && !trim_copy(provisioning_cfg.device_id).isEmpty();

  }

  String html_escape(const String &value) {
     String out;
     out.reserve(value.length() + 16);
     for (size_t i = 0; i < value.length(); ++i) {
       char c = value[i];
       if (c == '&') {
         out += "&amp;";
      }
       else if (c == '<') {
         out += "&lt;";
      }
       else if (c == '>') {
         out += "&gt;";
      }
       else if (c == '"') {
         out += "&quot;";
      }
       else {
         out += c;
      }
    }
     return out;

  }

  bool save_provisioning_settings() {
     Preferences prefs;
     if (!prefs.begin(kPrefsNamespace, false)) {
       Serial.println("[CFG] Preferences open failed (write provisioning)");
       return false;
    }

     JsonDocument doc;
     doc["schema"] = kProvisioningSchema;
     doc["wifi"]["ssid"] = provisioning_cfg.wifi_ssid;
     doc["wifi"]["password"] = provisioning_cfg.wifi_password;
     doc["mqtt"]["host"] = provisioning_cfg.mqtt_host;
     doc["mqtt"]["port_plain"] = provisioning_cfg.mqtt_port_plain;
     doc["mqtt"]["port_tls"] = provisioning_cfg.mqtt_port_tls;
     doc["mqtt"]["use_tls"] = provisioning_cfg.mqtt_use_tls;
     doc["mqtt"]["username"] = provisioning_cfg.mqtt_username;
     doc["mqtt"]["password"] = provisioning_cfg.mqtt_password;
     doc["mqtt"]["ca_cert"] = provisioning_cfg.mqtt_ca_cert;
     doc["topic"]["customer_id"] = provisioning_cfg.customer_id;
     doc["topic"]["device_id"] = provisioning_cfg.device_id;

     String payload;
     serializeJson(doc, payload);
     size_t written = prefs.putString(kPrefsKeyProvisioning, payload);
     prefs.end();
     if (written == 0) {
       Serial.println("[CFG] Persisted provisioning write failed");
       return false;
    }
     return true;

  }

  void load_provisioning_settings() {
     reset_provisioning_defaults();

     Preferences prefs;
     if (!prefs.begin(kPrefsNamespace, true)) {
       Serial.println("[CFG] Preferences open failed (read provisioning)");
       return;
    }
     String payload = prefs.getString(kPrefsKeyProvisioning, "");
     prefs.end();
     if (payload.isEmpty()) {
       return;
    }

     JsonDocument doc;
     DeserializationError err = deserializeJson(doc, payload);
     if (err) {
       Serial.printf("[CFG] Persisted provisioning parse failed: %s\n", err.c_str());
       return;
    }

     JsonObjectConst wifi_obj = doc["wifi"].as<JsonObjectConst>();
     if (!wifi_obj.isNull()) {
       if (wifi_obj["ssid"].is<const char *>()) {
         provisioning_cfg.wifi_ssid = wifi_obj["ssid"].as<const char *>();
      }
       if (wifi_obj["password"].is<const char *>()) {
         provisioning_cfg.wifi_password = wifi_obj["password"].as<const char *>();
      }
    }

     JsonObjectConst mqtt_obj = doc["mqtt"].as<JsonObjectConst>();
     if (!mqtt_obj.isNull()) {
       if (mqtt_obj["host"].is<const char *>()) {
         provisioning_cfg.mqtt_host = mqtt_obj["host"].as<const char *>();
      }
       uint32_t port_plain = 0;
       if (parse_uint32(mqtt_obj["port_plain"], port_plain) && port_plain > 0 && port_plain <= 65535) {
         provisioning_cfg.mqtt_port_plain = static_cast<uint16_t>(port_plain);
      }
       uint32_t port_tls = 0;
       if (parse_uint32(mqtt_obj["port_tls"], port_tls) && port_tls > 0 && port_tls <= 65535) {
         provisioning_cfg.mqtt_port_tls = static_cast<uint16_t>(port_tls);
      }
       bool use_tls = false;
       if (parse_bool(mqtt_obj["use_tls"], use_tls)) {
         provisioning_cfg.mqtt_use_tls = use_tls;
      }
       if (mqtt_obj["username"].is<const char *>()) {
         provisioning_cfg.mqtt_username = mqtt_obj["username"].as<const char *>();
      }
       if (mqtt_obj["password"].is<const char *>()) {
         provisioning_cfg.mqtt_password = mqtt_obj["password"].as<const char *>();
      }
       if (mqtt_obj["ca_cert"].is<const char *>()) {
         provisioning_cfg.mqtt_ca_cert = mqtt_obj["ca_cert"].as<const char *>();
      }
    }

     JsonObjectConst topic_obj = doc["topic"].as<JsonObjectConst>();
     if (!topic_obj.isNull()) {
       if (topic_obj["customer_id"].is<const char *>()) {
         provisioning_cfg.customer_id = topic_obj["customer_id"].as<const char *>();
      }
       if (topic_obj["device_id"].is<const char *>()) {
         provisioning_cfg.device_id = topic_obj["device_id"].as<const char *>();
      }
    }

     provisioning_cfg.wifi_ssid = trim_copy(provisioning_cfg.wifi_ssid);
     provisioning_cfg.mqtt_host = trim_copy(provisioning_cfg.mqtt_host);
     provisioning_cfg.customer_id = trim_copy(provisioning_cfg.customer_id);
     provisioning_cfg.device_id = trim_copy(provisioning_cfg.device_id);
     if (provisioning_cfg.device_id.isEmpty()) {
       provisioning_cfg.device_id = default_device_id();
    }

     Serial.println("[CFG] Provisioning settings loaded");
  }

  bool clear_all_persisted_data() {
     Preferences prefs;
     if (!prefs.begin(kPrefsNamespace, false)) {
       Serial.println("[CFG] Preferences open failed (factory reset)");
       return false;
    }
     bool ok = prefs.clear();
     prefs.end();
     return ok;
  }

  const char kProvisionPageTemplate[] = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>ESP32PPC Setup</title>
<style>
  :root { --blue: #1f6fdb; --blue-hover: #185fc0; --border: #d9e1ea; --bg: #ffffff; --text: #1a1f2a; }
  body { margin: 0; font-family: Segoe UI, Tahoma, sans-serif; background: var(--bg); color: var(--text); }
  .wrap { max-width: 760px; margin: 24px auto; padding: 18px; }
  .card { border: 1px solid var(--border); border-radius: 12px; padding: 18px; margin-bottom: 14px; }
  .topbar { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 10px; }
  h1 { margin: 0; font-size: 22px; }
  .brand-logo-img { height: 34px; width: auto; display: block; object-fit: contain; }
  h2 { margin: 0 0 10px 0; font-size: 18px; }
  label { display: block; margin: 8px 0 4px 0; font-weight: 600; }
  input, select, textarea { width: 100%; box-sizing: border-box; border: 1px solid #c7d1dc; border-radius: 8px; padding: 10px; font-size: 14px; }
  textarea { min-height: 150px; resize: vertical; }
  .row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  .btn { display: inline-flex; align-items: center; justify-content: center; gap: 8px; border: 0; border-radius: 8px; padding: 10px 16px; background: var(--blue); color: #fff; font-weight: 600; cursor: pointer; }
  .btn:hover { background: var(--blue-hover); }
  .btn:disabled { opacity: 0.7; cursor: not-allowed; }
  .scan-status { margin-top: 6px; min-height: 18px; }
  .spinner { width: 14px; height: 14px; border: 2px solid rgba(255,255,255,0.45); border-top-color: #ffffff; border-radius: 50%; animation: spin 0.8s linear infinite; }
  [hidden] { display: none !important; }
  @keyframes spin { from { transform: rotate(0deg); } to { transform: rotate(360deg); } }
  .muted { color: #5a6678; font-size: 13px; }
  .msg { border: 1px solid #8ac0ff; background: #eef6ff; color: #103a70; border-radius: 8px; padding: 10px; margin-bottom: 12px; }
  .check { display: flex; align-items: center; gap: 8px; margin: 10px 0; }
  .check input { width: auto; }
</style>
</head>
<body>
  <div class="wrap">
    <div class="topbar">
      <h1>ESP32PPC Provisioning</h1>
      <img class="brand-logo-img" src="/logo.png" alt="ismart logo" />
    </div>
    %MESSAGE%
    <form method="post" action="/save">
      <div class="card">
        <h2>WiFi</h2>
        <label for="wifi_ssid">Network (SSID)</label>
        <div class="row">
          <select id="wifi_list" onchange="selectWifi()"><option value="">Select scanned network</option></select>
          <button id="scan_btn" class="btn" type="button" onclick="scanWifi()">
            <span id="scan_spinner" class="spinner" hidden></span>
            <span id="scan_label">Scan WiFi</span>
          </button>
        </div>
        <div id="scan_status" class="muted scan-status">Tap Scan WiFi to list nearby networks.</div>
        <input id="wifi_ssid" name="wifi_ssid" type="text" value="%WIFI_SSID%" placeholder="WiFi SSID" required />
        <label for="wifi_password">Password</label>
        <input id="wifi_password" name="wifi_password" type="password" value="%WIFI_PASSWORD%" placeholder="WiFi password" />
      </div>

      <div class="card">
        <h2>MQTT Broker</h2>
        <label for="mqtt_host">Host/IP</label>
        <input id="mqtt_host" name="mqtt_host" type="text" value="%MQTT_HOST%" placeholder="broker.example.com" required />
        <div class="row">
          <div>
            <label for="mqtt_port_plain">Port (plain)</label>
            <input id="mqtt_port_plain" name="mqtt_port_plain" type="number" min="1" max="65535" value="%MQTT_PORT_PLAIN%" />
          </div>
          <div>
            <label for="mqtt_port_tls">Port (TLS)</label>
            <input id="mqtt_port_tls" name="mqtt_port_tls" type="number" min="1" max="65535" value="%MQTT_PORT_TLS%" />
          </div>
        </div>
        <div class="check">
          <input id="mqtt_use_tls" name="mqtt_use_tls" type="checkbox" %MQTT_TLS_CHECKED% />
          <label for="mqtt_use_tls" style="margin:0">Use TLS/SSL</label>
        </div>
        <div class="row">
          <div>
            <label for="mqtt_username">Username</label>
            <input id="mqtt_username" name="mqtt_username" type="text" value="%MQTT_USER%" />
          </div>
          <div>
            <label for="mqtt_password">Password</label>
            <input id="mqtt_password" name="mqtt_password" type="password" value="%MQTT_PASSWORD%" />
          </div>
        </div>
        <label for="ca_file">CA certificate file (.crt/.pem)</label>
        <input id="ca_file" type="file" accept=".crt,.pem,text/plain" onchange="loadCertFile(event)" />
        <label for="mqtt_ca_cert">CA certificate content</label>
        <textarea id="mqtt_ca_cert" name="mqtt_ca_cert" placeholder="-----BEGIN CERTIFICATE-----">%MQTT_CA_CERT%</textarea>
        <div class="muted">Upload file or paste certificate. Used when TLS is enabled.</div>
      </div>

      <div class="card">
        <h2>Topic Identity</h2>
        <div class="row">
          <div>
            <label for="customer_id">Customer ID</label>
            <input id="customer_id" name="customer_id" type="text" value="%CUSTOMER_ID%" required />
          </div>
          <div>
            <label for="device_id">Device ID</label>
            <input id="device_id" name="device_id" type="text" value="%DEVICE_ID%" required />
          </div>
        </div>
        <div class="muted">Topic base: ppc/v1/c/&lt;customer_id&gt;/d/&lt;device_id&gt;</div>
      </div>

      <button class="btn" type="submit">Save and Reboot</button>
    </form>
  </div>

<script>
let scanBusy = false;

function setScanUi(busy, text) {
  const btn = document.getElementById('scan_btn');
  const spinner = document.getElementById('scan_spinner');
  const label = document.getElementById('scan_label');
  const status = document.getElementById('scan_status');
  if (btn) btn.disabled = busy;
  if (spinner) spinner.hidden = !busy;
  if (label) label.textContent = busy ? 'Scanning...' : 'Scan WiFi';
  if (status && text) status.textContent = text;
}

async function scanWifi() {
  if (scanBusy) return;
  scanBusy = true;
  setScanUi(true, 'Scanning nearby networks... this can take a few seconds.');
  try {
    const res = await fetch('/scan', { cache: 'no-store' });
    if (!res.ok) throw new Error('scan_http_error');
    const data = await res.json();
    const list = document.getElementById('wifi_list');
    list.innerHTML = '<option value="">Select scanned network</option>';
    const ssids = (data.ssids || []);
    ssids.forEach(ssid => {
      const o = document.createElement('option');
      o.value = ssid;
      o.textContent = ssid;
      list.appendChild(o);
    });
    setScanUi(false, ssids.length > 0 ? `Found ${ssids.length} network(s).` : 'No networks found. Move closer to AP and scan again.');
  } catch (_) {
    setScanUi(false, 'WiFi scan failed or timed out. Wait a moment and try again.');
  } finally {
    scanBusy = false;
    const btn = document.getElementById('scan_btn');
    const spinner = document.getElementById('scan_spinner');
    const label = document.getElementById('scan_label');
    if (btn) btn.disabled = false;
    if (spinner) spinner.hidden = true;
    if (label) label.textContent = 'Scan WiFi';
  }
}

function selectWifi() {
  const list = document.getElementById('wifi_list');
  if (list.value) {
    document.getElementById('wifi_ssid').value = list.value;
  }
}

function loadCertFile(evt) {
  const file = evt.target.files && evt.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(e) {
    document.getElementById('mqtt_ca_cert').value = e.target.result || '';
  };
  reader.readAsText(file);
}
</script>
</body>
</html>
)rawliteral";

  String build_provision_page(const String &message = String()) {
     String page = kProvisionPageTemplate;
     String msg = message.isEmpty() ? String() : (String("<div class=\"msg\">") + html_escape(message) + "</div>");
     page.replace("%MESSAGE%", msg);
     page.replace("%WIFI_SSID%", html_escape(provisioning_cfg.wifi_ssid));
     page.replace("%WIFI_PASSWORD%", html_escape(provisioning_cfg.wifi_password));
     page.replace("%MQTT_HOST%", html_escape(provisioning_cfg.mqtt_host));
     page.replace("%MQTT_PORT_PLAIN%", String(provisioning_cfg.mqtt_port_plain));
     page.replace("%MQTT_PORT_TLS%", String(provisioning_cfg.mqtt_port_tls));
     page.replace("%MQTT_TLS_CHECKED%", provisioning_cfg.mqtt_use_tls ? "checked" : "");
     page.replace("%MQTT_USER%", html_escape(provisioning_cfg.mqtt_username));
     page.replace("%MQTT_PASSWORD%", html_escape(provisioning_cfg.mqtt_password));
     page.replace("%MQTT_CA_CERT%", html_escape(provisioning_cfg.mqtt_ca_cert));
     page.replace("%CUSTOMER_ID%", html_escape(provisioning_cfg.customer_id));
     page.replace("%DEVICE_ID%", html_escape(provisioning_cfg.device_id));
     return page;

  }

  void handle_provision_root() {
     provisioning_server.send(200, "text/html", build_provision_page());

  }

  void handle_provision_logo() {
     provisioning_server.send_P(200, "image/png", reinterpret_cast<const char *>(kIsmartLogoPng), kIsmartLogoPngLen);

  }
  void handle_provision_scan() {
     int count = WiFi.scanNetworks(false, false, false, 120);
     JsonDocument doc;
     JsonArray ssids = doc["ssids"].to<JsonArray>();
     for (int i = 0; i < count; ++i) {
       const String ssid = WiFi.SSID(i);
       if (!ssid.isEmpty()) {
         ssids.add(ssid);
      }
    }
     WiFi.scanDelete();
     String payload;
     serializeJson(doc, payload);
     provisioning_server.send(200, "application/json", payload);

  }

  void handle_provision_save() {
     ProvisioningConfig candidate = provisioning_cfg;
     candidate.wifi_ssid = trim_copy(provisioning_server.arg("wifi_ssid"));
     candidate.wifi_password = provisioning_server.arg("wifi_password");
     candidate.mqtt_host = trim_copy(provisioning_server.arg("mqtt_host"));
     candidate.mqtt_username = provisioning_server.arg("mqtt_username");
     candidate.mqtt_password = provisioning_server.arg("mqtt_password");
     candidate.mqtt_ca_cert = provisioning_server.arg("mqtt_ca_cert");
     candidate.customer_id = trim_copy(provisioning_server.arg("customer_id"));
     candidate.device_id = trim_copy(provisioning_server.arg("device_id"));
     candidate.mqtt_use_tls = provisioning_server.hasArg("mqtt_use_tls");

     uint16_t parsed_port = 0;
     if (parse_port_string(provisioning_server.arg("mqtt_port_plain"), parsed_port)) {
       candidate.mqtt_port_plain = parsed_port;
    }
     if (parse_port_string(provisioning_server.arg("mqtt_port_tls"), parsed_port)) {
       candidate.mqtt_port_tls = parsed_port;
    }

     candidate.mqtt_ca_cert.trim();
     if (candidate.mqtt_ca_cert.length() > kMaxCaCertLength) {
       provisioning_server.send(400, "text/plain", "certificate_too_large");
       return;
    }

     if (candidate.device_id.isEmpty()) {
       candidate.device_id = default_device_id();
    }

     provisioning_cfg = candidate;
     if (!provisioning_config_valid()) {
       provisioning_server.send(400, "text/plain", "missing_or_invalid_required_fields");
       return;
    }
     if (!save_provisioning_settings()) {
       provisioning_server.send(500, "text/plain", "persist_failed");
       return;
    }

     provisioning_server.send(200, "text/html", build_provision_page("Saved. Rebooting device now."));
     provisioning_restart_requested = true;

  }

  void handle_provision_not_found() {
     provisioning_server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
     provisioning_server.send(302, "text/plain", "");

  }

  void start_provisioning_portal() {
     WiFi.disconnect(true, true);
     delay(50);
     WiFi.mode(WIFI_AP_STA);
     WiFi.softAPConfig(kProvisionApIp, kProvisionApGateway, kProvisionApSubnet);

     uint64_t chip = ESP.getEfuseMac();
     char suffix[5];
     snprintf(suffix, sizeof(suffix), "%04X", static_cast<unsigned>(chip & 0xFFFFU));
     String ap_ssid = String(cfg::kProvisionApSsidPrefix) + "-" + suffix;
     const char *ap_password = (strlen(cfg::kProvisionApPassword) >= 8) ? cfg::kProvisionApPassword : nullptr;
     if (!WiFi.softAP(ap_ssid.c_str(), ap_password)) {
       Serial.println("[PROVISION] Failed to start AP");
       provisioning_restart_requested = true;
       return;
    }

     provisioning_dns.start(kProvisionPortalDnsPort, "*", WiFi.softAPIP());
     provisioning_server.on("/", HTTP_GET, handle_provision_root);
     provisioning_server.on("/logo.png", HTTP_GET, handle_provision_logo);
     provisioning_server.on("/scan", HTTP_GET, handle_provision_scan);
     provisioning_server.on("/save", HTTP_POST, handle_provision_save);
     provisioning_server.on("/generate_204", HTTP_GET, handle_provision_not_found);
     provisioning_server.on("/fwlink", HTTP_GET, handle_provision_not_found);
     provisioning_server.onNotFound(handle_provision_not_found);
     provisioning_server.begin();

     Serial.printf("[PROVISION] AP ready. SSID=%s IP=%s\n", ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());

  }

  void run_provisioning_portal_until_configured() {
     provisioning_restart_requested = false;
     start_provisioning_portal();
     while (!provisioning_restart_requested) {
       provisioning_dns.processNextRequest();
       provisioning_server.handleClient();
       delay(2);
    }
     provisioning_server.stop();
     provisioning_dns.stop();
     delay(300);
     ESP.restart();

  }
  const ppc::RangingMode *ranging_mode_from_name(const String &name) {
     if (name.equalsIgnoreCase(ppc::Ranging::kShortest.name)) {
       return &ppc::Ranging::kShortest;
    }
     if (name.equalsIgnoreCase(ppc::Ranging::kShort.name)) {
       return &ppc::Ranging::kShort;
    }
     if (name.equalsIgnoreCase(ppc::Ranging::kMedium.name)) {
       return &ppc::Ranging::kMedium;
    }
     if (name.equalsIgnoreCase(ppc::Ranging::kLong.name)) {
       return &ppc::Ranging::kLong;
    }
     if (name.equalsIgnoreCase(ppc::Ranging::kLonger.name)) {
       return &ppc::Ranging::kLonger;
    }
     if (name.equalsIgnoreCase(ppc::Ranging::kLongest.name)) {
       return &ppc::Ranging::kLongest;
    }
     return nullptr;
  }

  class CounterEngine {
     public: bool begin() {
       apply_runtime_config();
       if (!sensor_.begin(cfg::kI2cSdaPin, cfg::kI2cSclPin, cfg::kI2cFrequency, cfg::kVl53l1xAddress, cfg::kVl53l1xBootTimeoutMs, std::nullopt, std::nullopt)) {
         Serial.println("[PPC] Sensor init failed");
         return false;
      }
       bool calibration_loaded = false;
       if (has_boot_calibration_) {
         calibration_loaded = apply_persisted_calibration(boot_calibration_);
         if (!calibration_loaded) {
           Serial.println("[PPC] Persisted calibration invalid, recalibrating");
        }
      }
       if (!calibration_loaded && !recalibrate()) {
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
        
      }
       else {
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
     void set_boot_calibration(const PersistedCalibration *calibration) {
       if (calibration == nullptr || !calibration->valid) {
         has_boot_calibration_ = false;
         return;
      }
       boot_calibration_ = *calibration;
       has_boot_calibration_ = true;
    }
     void apply_runtime_config() {
       entry_.set_max_samples(runtime_cfg.sampling_size);
       exit_.set_max_samples(runtime_cfg.sampling_size);
       entry_.set_threshold_percentages(runtime_cfg.threshold_min_percent, runtime_cfg.threshold_max_percent);
       exit_.set_threshold_percentages(runtime_cfg.threshold_min_percent, runtime_cfg.threshold_max_percent);
      
    }
     bool clamped_mode() const {
       return clamped_mode_;
      
    }
     int people_counter() const {
       return people_counter_;
      
    }
     bool presence() const {
       return presence_;
      
    }
     uint16_t distance_entry() const {
       return entry_.last_distance();
      
    }
     uint16_t distance_exit() const {
       return exit_.last_distance();
      
    }
     uint16_t min_threshold_entry() const {
       return entry_.threshold.min;
      
    }
     uint16_t max_threshold_entry() const {
       return entry_.threshold.max;
      
    }
     uint16_t min_threshold_exit() const {
       return exit_.threshold.min;
      
    }
     uint16_t max_threshold_exit() const {
       return exit_.threshold.max;
      
    }
     uint16_t idle_threshold_entry() const {
       return entry_.threshold.idle;
      
    }
     uint16_t idle_threshold_exit() const {
       return exit_.threshold.idle;
      
    }
     uint16_t roi_width_entry() const {
       return entry_.roi.width;
      
    }
     uint16_t roi_height_entry() const {
       return entry_.roi.height;
      
    }
     uint16_t roi_width_exit() const {
       return exit_.roi.width;
      
    }
     uint16_t roi_height_exit() const {
       return exit_.roi.height;
      
    }
     uint8_t roi_center_entry() const {
       return entry_.roi.center;
      
    }
     uint8_t roi_center_exit() const {
       return exit_.roi.center;
      
    }
     const char *ranging_mode_name() const {
       const ppc::RangingMode *mode = sensor_.get_ranging_mode();
       return mode == nullptr ? "unknown" : mode->name;
      
    }
     VL53L1_Error sensor_status() const {
       return sensor_status_;
      
    }
     const char *last_direction() const {
       return last_direction_.c_str();
      
    }
     uint32_t total_entry_today() const {
       return total_entry_today_;
      
    }
     uint32_t total_exit_today() const {
       return total_exit_today_;
      
    }
     bool pop_event(PendingEvent &out) {
       if (!pending_event_.ready) {
         return false;
        
      }
       out = pending_event_;
       pending_event_.ready = false;
       pending_event_.name = "";
       return true;
      
    }
     private: static constexpr int kLeftState = 1;
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
     bool apply_persisted_calibration(const PersistedCalibration &calibration) {
       String mode_name = calibration.ranging_mode;
       const ppc::RangingMode *mode = ranging_mode_from_name(mode_name);
       if (mode == nullptr) {
         return false;
      }
       if (!sensor_.set_ranging_mode(mode)) {
         return false;
      }
       if (calibration.z0_idle_mm == 0 || calibration.z1_idle_mm == 0 || calibration.z0_max_mm <= calibration.z0_min_mm || calibration.z1_max_mm <= calibration.z1_min_mm) {
         return false;
      }
       entry_.threshold.idle = calibration.z0_idle_mm;
       entry_.threshold.min = calibration.z0_min_mm;
       entry_.threshold.max = calibration.z0_max_mm;
       entry_.roi.width = calibration.z0_roi_w;
       entry_.roi.height = calibration.z0_roi_h;
       entry_.roi.center = calibration.z0_roi_center;
       exit_.threshold.idle = calibration.z1_idle_mm;
       exit_.threshold.min = calibration.z1_min_mm;
       exit_.threshold.max = calibration.z1_max_mm;
       exit_.roi.width = calibration.z1_roi_w;
       exit_.roi.height = calibration.z1_roi_h;
       exit_.roi.center = calibration.z1_roi_center;
       Serial.printf("[PPC] Restored persisted calibration (%s)\n", calibration.ranging_mode);
       return true;
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
       Serial.printf("[PPC][DBG] d0=%u d1=%u th0=%u-%u th1=%u-%u occ0=%d occ1=%d cnt=%d pres=%d dir=%s\n", entry_.last_distance(), exit_.last_distance(), entry_.threshold.min, entry_.threshold.max, exit_.threshold.min, exit_.threshold.max, entry_.is_occupied() ? 1 : 0, exit_.is_occupied() ? 1 : 0, people_counter_, presence_ ? 1 : 0, last_direction_.c_str());
      
    }
     int combined_state(bool &left_occupied, bool &right_occupied, uint16_t &left_distance, uint16_t &right_distance) const {
       if (runtime_cfg.invert_direction) {
         left_occupied = exit_.is_occupied();
         right_occupied = entry_.is_occupied();
         left_distance = exit_.min_distance();
         right_distance = entry_.min_distance();
        
      }
       else {
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
       if (saw_both_state_ && first_single_state_ != 0 && last_single_state_ != 0 && first_single_state_ != last_single_state_) {
         if (first_single_state_ == kRightState && last_single_state_ == kLeftState) {
           delta = 1;
          
        }
         else if (first_single_state_ == kLeftState && last_single_state_ == kRightState) {
           delta = -1;
          
        }
         if (delta != 0) {
           reason = "first_last";
          
        }
        
      }
       if (delta == 0 && saw_both_state_ && left_peak_time_ms_ != 0 && right_peak_time_ms_ != 0) {
         const uint32_t peak_dt = (left_peak_time_ms_ > right_peak_time_ms_) ? (left_peak_time_ms_ - right_peak_time_ms_) : (right_peak_time_ms_ - left_peak_time_ms_);
         if (peak_dt >= runtime_cfg.peak_time_delta_ms) {
           if (right_peak_time_ms_ < left_peak_time_ms_) {
             delta = 1;
            
          }
           else {
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
            
          }
           else if (start_balance_mm_ > 0 && last_balance_mm_ < 0) {
             delta = -1;
             reason = "balance_cross";
            
          }
          
        }
        
      }
       if (runtime_cfg.serial_debug_enabled) {
         Serial.printf("[PPC][TRACE] finalize reason=%s first=%d last=%d both=%d peakL=%lu peakR=%lu bal=%d->%d now=%lu delta=%d\n", reason, first_single_state_, last_single_state_, saw_both_state_ ? 1 : 0, left_peak_time_ms_, right_peak_time_ms_, start_balance_mm_, last_balance_mm_, now, delta);
        
      }
       return delta;
      
    }
     void commit_event_delta(int delta, uint32_t now) {
       if (delta == 0) {
         return;
        
      }
       if ((now - last_count_event_ms_) < runtime_cfg.event_cooldown_ms) {
         if (runtime_cfg.serial_debug_enabled) {
           Serial.printf("[PPC][TRACE] %s ignored by cooldown (%lu ms)\n", delta > 0 ? "Entry" : "Exit", now - last_count_event_ms_);
          
        }
         return;
        
      }
       if (delta > 0) {
         total_entry_today_++;
         update_counter(1);
         queue_event("Entry");
        
      }
       else {
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
       const bool door_blocked = runtime_cfg.door_protection_enabled && ((entry_.last_distance() > 0 && entry_.last_distance() <= runtime_cfg.door_protection_mm) || (exit_.last_distance() > 0 && exit_.last_distance() <= runtime_cfg.door_protection_mm));
       if (door_blocked) {
         if (runtime_cfg.serial_debug_enabled) {
           Serial.printf("[PPC][TRACE] door-protect block d0=%u d1=%u <= %u mm\n", entry_.last_distance(), exit_.last_distance(), runtime_cfg.door_protection_mm);
          
        }
         presence_ = false;
         reset_path_tracking();
         return;
        
      }
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
           Serial.printf("[PPC][TRACE] state %d->%d left=%d right=%d first=%d last=%d both=%d\n", prev_combined_state_, state, left_occupied ? 1 : 0, right_occupied ? 1 : 0, first_single_state_, last_single_state_, saw_both_state_ ? 1 : 0);
          
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
         if (empty_duration >= runtime_cfg.adaptive_threshold_interval_ms && (now - last_adaptive_update_time_) >= runtime_cfg.adaptive_threshold_interval_ms) {
           entry_.update_adaptive_threshold(runtime_cfg.adaptive_threshold_alpha);
           exit_.update_adaptive_threshold(runtime_cfg.adaptive_threshold_alpha);
           last_adaptive_update_time_ = now;
          
        }
        
      }
       else {
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
     ppc::Vl53l1xDevice sensor_ {
      
    };
     ppc::Zone entry_ {
      0
    };
     ppc::Zone exit_ {
      1
    };
     ppc::Orientation orientation_ {
      ppc::Orientation::Parallel
    };
     PersistedCalibration boot_calibration_ {
      
    };
     bool has_boot_calibration_ {
      false
    };
     int people_counter_ {
      0
    };
     bool clamped_mode_ {
      cfg::kClampCounterAtZero
    };
     bool presence_ {
      false
    };
     String last_direction_ {
      "None"
    };
     VL53L1_Error sensor_status_ {
      VL53L1_ERROR_NONE
    };
     uint32_t total_entry_today_ {
      0
    };
     uint32_t total_exit_today_ {
      0
    };
     int last_reset_day_of_year_ {
      -1
    };
     bool tracking_active_ {
      false
    };
     int prev_combined_state_ {
      0
    };
     int first_single_state_ {
      0
    };
     int last_single_state_ {
      0
    };
     bool saw_both_state_ {
      false
    };
     uint16_t left_peak_distance_ {
      0xFFFF
    };
     uint16_t right_peak_distance_ {
      0xFFFF
    };
     uint32_t left_peak_time_ms_ {
      0
    };
     uint32_t right_peak_time_ms_ {
      0
    };
     bool have_balance_window_ {
      false
    };
     int start_balance_mm_ {
      0
    };
     int last_balance_mm_ {
      0
    };
     uint32_t last_state_change_time_ {
      0
    };
     uint32_t last_adaptive_update_time_ {
      0
    };
     uint32_t zones_empty_since_ {
      0
    };
     bool zones_were_occupied_ {
      false
    };
     uint32_t last_count_event_ms_ {
      0
    };
     uint32_t last_debug_sample_ms_ {
      0
    };
     PendingEvent pending_event_ {
      
    };
    
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
  void append_calibration_payload(JsonObject calibration) {
     calibration["ranging_mode"] = engine.ranging_mode_name();
     JsonObject z0 = calibration["z0"].to<JsonObject>();
     z0["idle_mm"] = engine.idle_threshold_entry();
     z0["min_mm"] = engine.min_threshold_entry();
     z0["max_mm"] = engine.max_threshold_entry();
     z0["roi_w"] = engine.roi_width_entry();
     z0["roi_h"] = engine.roi_height_entry();
     z0["roi_center"] = engine.roi_center_entry();
     JsonObject z1 = calibration["z1"].to<JsonObject>();
     z1["idle_mm"] = engine.idle_threshold_exit();
     z1["min_mm"] = engine.min_threshold_exit();
     z1["max_mm"] = engine.max_threshold_exit();
     z1["roi_w"] = engine.roi_width_exit();
     z1["roi_h"] = engine.roi_height_exit();
     z1["roi_center"] = engine.roi_center_exit();
  }

  bool load_persisted_calibration(JsonObjectConst calibration_obj, PersistedCalibration &out) {
     JsonVariantConst mode_var = calibration_obj["ranging_mode"];
     if (!mode_var.is<const char *>()) {
       return false;
    }
     String mode_name = mode_var.as<const char *>();
     if (ranging_mode_from_name(mode_name) == nullptr) {
       return false;
    }

     JsonObjectConst z0 = calibration_obj["z0"].as<JsonObjectConst>();
     JsonObjectConst z1 = calibration_obj["z1"].as<JsonObjectConst>();
     if (z0.isNull() || z1.isNull()) {
       return false;
    }

     auto parse_zone = [&](JsonObjectConst zone,
                           uint16_t &idle,
                           uint16_t &min_v,
                           uint16_t &max_v,
                           uint8_t &roi_w,
                           uint8_t &roi_h,
                           uint8_t &roi_c) -> bool {
       uint32_t idle_u32 = 0;
       uint32_t min_u32 = 0;
       uint32_t max_u32 = 0;
       uint32_t roi_w_u32 = 0;
       uint32_t roi_h_u32 = 0;
       uint32_t roi_c_u32 = 0;
       if (!parse_uint32(zone["idle_mm"], idle_u32) || !parse_uint32(zone["min_mm"], min_u32) || !parse_uint32(zone["max_mm"], max_u32) || !parse_uint32(zone["roi_w"], roi_w_u32) || !parse_uint32(zone["roi_h"], roi_h_u32) || !parse_uint32(zone["roi_center"], roi_c_u32)) {
         return false;
      }
       if (idle_u32 == 0 || max_u32 <= min_u32 || roi_w_u32 == 0 || roi_h_u32 == 0 || roi_w_u32 > 16 || roi_h_u32 > 16 || roi_c_u32 > 255) {
         return false;
      }
       idle = static_cast<uint16_t>(idle_u32);
       min_v = static_cast<uint16_t>(min_u32);
       max_v = static_cast<uint16_t>(max_u32);
       roi_w = static_cast<uint8_t>(roi_w_u32);
       roi_h = static_cast<uint8_t>(roi_h_u32);
       roi_c = static_cast<uint8_t>(roi_c_u32);
       return true;
    };

     PersistedCalibration parsed{};
     strncpy(parsed.ranging_mode, mode_name.c_str(), sizeof(parsed.ranging_mode) - 1);
     parsed.ranging_mode[sizeof(parsed.ranging_mode) - 1] = '\0';

     if (!parse_zone(z0, parsed.z0_idle_mm, parsed.z0_min_mm, parsed.z0_max_mm, parsed.z0_roi_w, parsed.z0_roi_h, parsed.z0_roi_center)) {
       return false;
    }
     if (!parse_zone(z1, parsed.z1_idle_mm, parsed.z1_min_mm, parsed.z1_max_mm, parsed.z1_roi_w, parsed.z1_roi_h, parsed.z1_roi_center)) {
       return false;
    }

     parsed.valid = true;
     out = parsed;
     return true;
  }

  void load_persisted_settings() {
     runtime_cfg.serial_debug_enabled = false;
     runtime_cfg.serial_debug_sample_interval_ms = cfg::kSerialDebugSampleIntervalMs;
     boot_clamped_mode = cfg::kClampCounterAtZero;
     boot_people_counter = 0;
     boot_calibration = PersistedCalibration{};

     Preferences prefs;
     if (!prefs.begin(kPrefsNamespace, true)) {
       Serial.println("[CFG] Preferences open failed (read)");
       return;
    }

     String payload = prefs.getString(kPrefsKeySettings, "");
     prefs.end();
     if (payload.isEmpty()) {
       return;
    }

     JsonDocument doc;
     DeserializationError err = deserializeJson(doc, payload);
     if (err) {
       Serial.printf("[CFG] Persisted settings parse failed: %s\n", err.c_str());
       return;
    }

     JsonObjectConst cfg_obj = doc["config"].as<JsonObjectConst>();
     if (!cfg_obj.isNull()) {
       uint32_t u32 = 0;
       bool b = false;

       if (parse_uint32(cfg_obj["sampling"], u32) && u32 >= 1 && u32 <= 16) {
         runtime_cfg.sampling_size = static_cast<uint8_t>(u32);
      }
       if (parse_uint32(cfg_obj["threshold_min_percent"], u32) && u32 <= 100) {
         runtime_cfg.threshold_min_percent = static_cast<uint8_t>(u32);
      }
       if (parse_uint32(cfg_obj["threshold_max_percent"], u32) && u32 <= 100) {
         runtime_cfg.threshold_max_percent = static_cast<uint8_t>(u32);
      }
       if (runtime_cfg.threshold_min_percent >= runtime_cfg.threshold_max_percent) {
         runtime_cfg.threshold_min_percent = cfg::kThresholdMinPercent;
         runtime_cfg.threshold_max_percent = cfg::kThresholdMaxPercent;
      }
       if (parse_uint32(cfg_obj["path_tracking_timeout_ms"], u32) && u32 <= 60000) {
         runtime_cfg.path_tracking_timeout_ms = u32;
      }
       if (parse_uint32(cfg_obj["event_cooldown_ms"], u32) && u32 <= 10000) {
         runtime_cfg.event_cooldown_ms = u32;
      }
       if (parse_uint32(cfg_obj["peak_time_delta_ms"], u32) && u32 <= 2000) {
         runtime_cfg.peak_time_delta_ms = u32;
      }
       if (parse_bool(cfg_obj["invert_direction"], b)) {
         runtime_cfg.invert_direction = b;
      }

       JsonObjectConst adaptive_obj = cfg_obj["adaptive"].as<JsonObjectConst>();
       if (!adaptive_obj.isNull()) {
         if (parse_bool(adaptive_obj["enabled"], b)) {
           runtime_cfg.adaptive_threshold_enabled = b;
        }
         if (parse_uint32(adaptive_obj["interval_ms"], u32) && u32 >= 1000 && u32 <= 3600000) {
           runtime_cfg.adaptive_threshold_interval_ms = u32;
        }
         float alpha = 0.0f;
         if (parse_float(adaptive_obj["alpha"], alpha) && alpha > 0.0f && alpha <= 1.0f) {
           runtime_cfg.adaptive_threshold_alpha = alpha;
        }
      }

       JsonObjectConst door_obj = cfg_obj["door_protection"].as<JsonObjectConst>();
       if (!door_obj.isNull()) {
         if (parse_bool(door_obj["enabled"], b)) {
           runtime_cfg.door_protection_enabled = b;
        }
         if (parse_uint32(door_obj["mm"], u32) && u32 <= 4000) {
           runtime_cfg.door_protection_mm = static_cast<uint16_t>(u32);
        }
      }

       if (parse_bool(cfg_obj["clamped_mode"], b)) {
         boot_clamped_mode = b;
      }
       if (cfg_obj["people_counter"].is<int>()) {
         boot_people_counter = cfg_obj["people_counter"].as<int>();
      }
    }

     JsonObjectConst calibration_obj = doc["calibration"].as<JsonObjectConst>();
     if (!calibration_obj.isNull()) {
       PersistedCalibration loaded{};
       if (load_persisted_calibration(calibration_obj, loaded)) {
         boot_calibration = loaded;
      }
       else {
         Serial.println("[CFG] Persisted calibration ignored (invalid)");
      }
    }

     Serial.println("[CFG] Persisted settings loaded");
  }

  bool save_persisted_settings() {
     Preferences prefs;
     if (!prefs.begin(kPrefsNamespace, false)) {
       Serial.println("[CFG] Preferences open failed (write)");
       return false;
    }

     JsonDocument doc;
     doc["schema"] = kPersistedSettingsSchema;

     JsonObject cfg_obj = doc["config"].to<JsonObject>();
     cfg_obj["invert_direction"] = runtime_cfg.invert_direction;
     cfg_obj["sampling"] = runtime_cfg.sampling_size;
     cfg_obj["threshold_min_percent"] = runtime_cfg.threshold_min_percent;
     cfg_obj["threshold_max_percent"] = runtime_cfg.threshold_max_percent;
     cfg_obj["path_tracking_timeout_ms"] = runtime_cfg.path_tracking_timeout_ms;
     cfg_obj["event_cooldown_ms"] = runtime_cfg.event_cooldown_ms;
     cfg_obj["peak_time_delta_ms"] = runtime_cfg.peak_time_delta_ms;

     JsonObject adaptive_obj = cfg_obj["adaptive"].to<JsonObject>();
     adaptive_obj["enabled"] = runtime_cfg.adaptive_threshold_enabled;
     adaptive_obj["interval_ms"] = runtime_cfg.adaptive_threshold_interval_ms;
     adaptive_obj["alpha"] = runtime_cfg.adaptive_threshold_alpha;

     JsonObject door_obj = cfg_obj["door_protection"].to<JsonObject>();
     door_obj["enabled"] = runtime_cfg.door_protection_enabled;
     door_obj["mm"] = runtime_cfg.door_protection_mm;

     cfg_obj["clamped_mode"] = engine.clamped_mode();
     cfg_obj["people_counter"] = engine.people_counter();

     JsonObject calibration = doc["calibration"].to<JsonObject>();
     append_calibration_payload(calibration);

     String payload;
     serializeJson(doc, payload);
     size_t written = prefs.putString(kPrefsKeySettings, payload);
     prefs.end();

     if (written == 0) {
       Serial.println("[CFG] Persisted settings write failed");
       return false;
    }
     return true;
  }

  void publish_recalibration_ack(const String &request_id, bool ok, const String &message) {
     JsonDocument doc;
     doc["ts"] = iso_utc_now();
     doc["request_id"] = request_id;
     doc["ok"] = ok;
     doc["message"] = message;
     if (ok) {
       JsonObject calibration = doc["calibration"].to<JsonObject>();
       calibration["ranging_mode"] = engine.ranging_mode_name();
       JsonObject z0 = calibration["z0"].to<JsonObject>();
       z0["idle_mm"] = engine.idle_threshold_entry();
       z0["min_mm"] = engine.min_threshold_entry();
       z0["max_mm"] = engine.max_threshold_entry();
       z0["roi_w"] = engine.roi_width_entry();
       z0["roi_h"] = engine.roi_height_entry();
       z0["roi_center"] = engine.roi_center_entry();
       JsonObject z1 = calibration["z1"].to<JsonObject>();
       z1["idle_mm"] = engine.idle_threshold_exit();
       z1["min_mm"] = engine.min_threshold_exit();
       z1["max_mm"] = engine.max_threshold_exit();
       z1["roi_w"] = engine.roi_width_exit();
       z1["roi_h"] = engine.roi_height_exit();
       z1["roi_center"] = engine.roi_center_exit();
      
    }
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
     obj["clamped_mode"] = engine.clamped_mode();
     JsonObject adaptive = obj["adaptive"].to<JsonObject>();
     adaptive["enabled"] = runtime_cfg.adaptive_threshold_enabled;
     adaptive["interval_ms"] = runtime_cfg.adaptive_threshold_interval_ms;
     adaptive["alpha"] = runtime_cfg.adaptive_threshold_alpha;
     JsonObject debug = obj["debug"].to<JsonObject>();
     debug["serial_enabled"] = runtime_cfg.serial_debug_enabled;
     debug["sample_interval_ms"] = runtime_cfg.serial_debug_sample_interval_ms;
     JsonObject door_protection = obj["door_protection"].to<JsonObject>();
     door_protection["enabled"] = runtime_cfg.door_protection_enabled;
     door_protection["mm"] = runtime_cfg.door_protection_mm;
    
  }
  void publish_meta() {
     JsonDocument doc;
     doc["fw_version"] = kFwVersion;
     doc["device_id"] = provisioning_cfg.device_id;
     doc["customer_id"] = provisioning_cfg.customer_id;
     doc["mqtt_tls"] = provisioning_cfg.mqtt_use_tls;
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
     const bool include_live_distances = engine.presence() || runtime_cfg.serial_debug_enabled;
     if (include_live_distances) {
       zones["z0_mm"] = engine.distance_entry();
       zones["z1_mm"] = engine.distance_exit();
      
    }
     zones["min0"] = engine.min_threshold_entry();
     zones["max0"] = engine.max_threshold_entry();
     zones["min1"] = engine.min_threshold_exit();
     zones["max1"] = engine.max_threshold_exit();
     zones["roi0_w"] = engine.roi_width_entry();
     zones["roi0_h"] = engine.roi_height_entry();
     zones["roi1_w"] = engine.roi_width_exit();
     zones["roi1_h"] = engine.roi_height_exit();
     const uint32_t now_ms = millis();
     if (!health_sample_initialized || (now_ms - last_health_sample_ms) >= 1000) {
       last_health_sample_ms = now_ms;
       cached_rssi = WiFi.RSSI();
       cached_uptime_s = now_ms / 1000UL;
       health_sample_initialized = true;
      
    }
     JsonObject health = doc["health"].to<JsonObject>();
     health["rssi"] = cached_rssi;
     health["uptime_s"] = cached_uptime_s;
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
     base_topic = String("ppc/v1/c/") + provisioning_cfg.customer_id + "/d/" + provisioning_cfg.device_id;
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
     if (trim_copy(provisioning_cfg.wifi_ssid).isEmpty()) {
       return;
      
    }
     const uint32_t now = millis();
     if ((now - last_wifi_reconnect_ms) < 5000) {
       return;
      
    }
     last_wifi_reconnect_ms = now;
     Serial.printf("[WiFi] Connecting to %s\n", provisioning_cfg.wifi_ssid.c_str());
     WiFi.mode(WIFI_STA);
     WiFi.begin(provisioning_cfg.wifi_ssid.c_str(), provisioning_cfg.wifi_password.c_str());
    
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
     JsonVariantConst v_door_enabled = cfg_obj["door_protection_enabled"];
     if (!v_door_enabled.isNull()) {
       bool enabled = false;
       if (!parse_bool(v_door_enabled, enabled)) {
         error_message = "invalid_door_protection_enabled";
         return false;
        
      }
       runtime_cfg.door_protection_enabled = enabled;
       changed_any = true;
      
    }
     JsonVariantConst v_door_mm = cfg_obj["door_protection_mm"];
     if (!v_door_mm.isNull()) {
       uint32_t mm = 0;
       if (!parse_uint32(v_door_mm, mm) || mm > 4000) {
         error_message = "invalid_door_protection_mm";
         return false;
        
      }
       runtime_cfg.door_protection_mm = static_cast<uint16_t>(mm);
       changed_any = true;
      
    }
     JsonVariantConst door_obj_var = cfg_obj["door_protection"];
     if (!door_obj_var.isNull()) {
       if (!door_obj_var.is<JsonObjectConst>()) {
         error_message = "invalid_door_protection_object";
         return false;
        
      }
       JsonObjectConst door_obj = door_obj_var.as<JsonObjectConst>();
       JsonVariantConst v_door_obj_enabled = door_obj["enabled"];
       if (!v_door_obj_enabled.isNull()) {
         bool enabled = false;
         if (!parse_bool(v_door_obj_enabled, enabled)) {
           error_message = "invalid_door_protection_enabled";
           return false;
          
        }
         runtime_cfg.door_protection_enabled = enabled;
         changed_any = true;
        
      }
       JsonVariantConst v_door_obj_mm = door_obj["mm"];
       if (!v_door_obj_mm.isNull()) {
         uint32_t mm = 0;
         if (!parse_uint32(v_door_obj_mm, mm) || mm > 4000) {
           error_message = "invalid_door_protection_mm";
           return false;
          
        }
         runtime_cfg.door_protection_mm = static_cast<uint16_t>(mm);
         changed_any = true;
        
      }
      
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
       runtime_cfg.serial_debug_enabled = cfg::kSerialDebugEnabled;
       runtime_cfg.serial_debug_sample_interval_ms = cfg::kSerialDebugSampleIntervalMs;
       bool ok = engine.recalibrate();
       if (ok) {
         save_persisted_settings();
      }
       publish_recalibration_ack(request_id, ok, ok ? "recalibrated" : "recalibration_failed");
       publish_state();
       return;
      
    }
     if (cmd == "get_config") {
       publish_ack(request_id, true, "config_in_state");
       publish_state();
       return;
      
    }
     if (cmd == "set_config") {
       JsonObjectConst cfg_obj = doc["config"].is<JsonObjectConst>() ? doc["config"].as<JsonObjectConst>() : doc.as<JsonObjectConst>();
       String error_message;
       bool ok = apply_runtime_config_from_json(cfg_obj, error_message);
       if (ok) {
         save_persisted_settings();
      }
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
       save_persisted_settings();
       publish_ack(request_id, true, "people_counter_updated");
       publish_state();
       return;
      
    }
     if (cmd == "set_clamped" || cmd == "set_clamped_mode") {
       JsonVariantConst clamped_var = doc["value"];
       if (clamped_var.isNull()) {
         clamped_var = doc["clamped"];
        
      }
       bool clamped = false;
       if (!parse_bool(clamped_var, clamped)) {
         publish_ack(request_id, false, "missing_or_invalid_clamped_value");
         return;
        
      }
       engine.set_clamped_mode(clamped);
       save_persisted_settings();
       publish_ack(request_id, true, "clamped_mode_updated");
       publish_state();
       return;
      
    }
     if (cmd == "set_door_protection") {
       JsonVariantConst v_enabled = doc["enabled"];
       if (v_enabled.isNull()) {
         v_enabled = doc["value"];
        
      }
       JsonVariantConst v_mm = doc["mm"];
       if (v_mm.isNull()) {
         v_mm = doc["distance_mm"];
        
      }
       if (v_enabled.isNull() && v_mm.isNull()) {
         publish_ack(request_id, false, "missing_enabled_or_mm");
         return;
        
      }
       if (!v_enabled.isNull()) {
         bool enabled = false;
         if (!parse_bool(v_enabled, enabled)) {
           publish_ack(request_id, false, "invalid_door_protection_enabled");
           return;
          
        }
         runtime_cfg.door_protection_enabled = enabled;
        
      }
       if (!v_mm.isNull()) {
         uint32_t mm = 0;
         if (!parse_uint32(v_mm, mm) || mm > 4000) {
           publish_ack(request_id, false, "invalid_door_protection_mm");
           return;
          
        }
         runtime_cfg.door_protection_mm = static_cast<uint16_t>(mm);
        
      }
       save_persisted_settings();
       publish_ack(request_id, true, "door_protection_updated");
       publish_state();
       return;
      
    }
     if (cmd == "factory_reset") {
       JsonVariantConst confirm_var = doc["confirm"];
       if (confirm_var.isNull()) {
         confirm_var = doc["value"];
      }
       bool confirmed = false;
       if (!parse_bool(confirm_var, confirmed) && confirm_var.is<const char *>()) {
         String token = confirm_var.as<const char *>();
         token.toUpperCase();
         confirmed = (token == "ERASE_NVS" || token == "FACTORY_RESET");
      }
       if (!confirmed) {
         publish_ack(request_id, false, "factory_reset_confirmation_required");
         return;
      }
       if (!clear_all_persisted_data()) {
         publish_ack(request_id, false, "factory_reset_failed");
         return;
      }
       publish_ack(request_id, true, "factory_reset_restarting");
       delay(300);
       ESP.restart();
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
     char client_id[96];
     snprintf(client_id, sizeof(client_id), "%s-%04X", provisioning_cfg.device_id.c_str(), static_cast<unsigned>(chip & 0xFFFFU));
     const char *mqtt_user = provisioning_cfg.mqtt_username.isEmpty() ? nullptr : provisioning_cfg.mqtt_username.c_str();
     const char *mqtt_pass = provisioning_cfg.mqtt_password.isEmpty() ? nullptr : provisioning_cfg.mqtt_password.c_str();
     bool connected = mqtt_client.connect(client_id, mqtt_user, mqtt_pass, topic_availability.c_str(), 1, true, "offline", true);
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
     mqtt_client.setServer(provisioning_cfg.mqtt_host.c_str(), provisioning_cfg.mqtt_use_tls ? provisioning_cfg.mqtt_port_tls : provisioning_cfg.mqtt_port_plain);
     mqtt_client.setCallback(mqtt_callback);
     mqtt_client.setBufferSize(1536);
     if (provisioning_cfg.mqtt_use_tls) {
       if (!provisioning_cfg.mqtt_ca_cert.isEmpty()) {
         tls_client.setCACert(provisioning_cfg.mqtt_ca_cert.c_str());
         
      }
       else {
         Serial.println("[MQTT] TLS enabled without CA cert. Falling back to insecure TLS for testing.");
         tls_client.setInsecure();
         
      }
       if (strlen(cfg::kMqttClientCert) > 0 && strlen(cfg::kMqttClientKey) > 0) {
         tls_client.setCertificate(cfg::kMqttClientCert);
         tls_client.setPrivateKey(cfg::kMqttClientKey);
         
      }
       mqtt_client.setClient(tls_client);
       
    }
     else {
       mqtt_client.setClient(plain_client);
       
    }
     
  }
  
}
// namespace

void setup() {
   Serial.begin(115200);
   delay(200);

   load_provisioning_settings();
   if (!provisioning_config_valid()) {
     Serial.println("[PROVISION] Missing WiFi/MQTT settings. Starting setup AP.");
     run_provisioning_portal_until_configured();
     return;
  }

   setup_topics();
   configTime(cfg::kGmtOffsetSec, cfg::kDstOffsetSec, cfg::kNtpServer1, cfg::kNtpServer2);

   load_persisted_settings();
   engine.set_boot_calibration(boot_calibration.valid ? &boot_calibration : nullptr);
   engine.set_clamped_mode(boot_clamped_mode);
   engine.set_people_counter(boot_people_counter);

   connect_wifi();
   setup_mqtt_transport();
   if (!engine.begin()) {
     Serial.println("[PPC] Engine init failed, rebooting in 5 seconds");
     delay(5000);
     ESP.restart();
    
  }
   save_persisted_settings();
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























































