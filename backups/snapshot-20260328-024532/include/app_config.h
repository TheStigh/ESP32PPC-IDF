#pragma once

// Edit values in this file for your installation.
// Keep secrets out of git if you plan to publish the repository.
namespace cfg {

// Device identity and topic path: ppc/v1/c/{customer_id}/d/{device_id}
static constexpr char kDeviceId[] = "esp32ppc-c5-01";
static constexpr char kCustomerId[] = "customer-demo";

// Wi-Fi
static constexpr char kWifiSsid[] = "TheStighMesh";
static constexpr char kWifiPassword[] = "AarsteinN3T99#";

// MQTT broker
static constexpr char kMqttHost[] = "192.168.11.200";
static constexpr uint16_t kMqttPortPlain = 1883;
static constexpr uint16_t kMqttPortTls = 8883;
static constexpr bool kMqttUseTls = false;
static constexpr char kMqttUsername[] = "mqtt_thestigh";
static constexpr char kMqttPassword[] = "90Hester!";

// Optional TLS assets. Leave empty to use insecure TLS (testing only).
static constexpr char kMqttCaCert[] = "";
static constexpr char kMqttClientCert[] = "";
static constexpr char kMqttClientKey[] = "";

// Sensor bus and timing
static constexpr uint8_t kI2cSdaPin = 23;
static constexpr uint8_t kI2cSclPin = 24;
static constexpr uint32_t kI2cFrequency = 400000;
static constexpr uint8_t kVl53l1xAddress = 0x29;
static constexpr uint16_t kVl53l1xBootTimeoutMs = 1000;

// Counter tuning aligned with current ESPHome defaults
static constexpr bool kInvertDirection = true;
static constexpr uint8_t kSamplingSize = 2;
static constexpr uint8_t kThresholdMinPercent = 0;
static constexpr uint8_t kThresholdMaxPercent = 85;
static constexpr uint32_t kPathTrackingTimeoutMs = 3000;
static constexpr uint32_t kEventCooldownMs = 700;
static constexpr uint32_t kPeakTimeDeltaMs = 120;
static constexpr bool kAdaptiveThresholdEnabled = true;
static constexpr uint32_t kAdaptiveThresholdIntervalMs = 60000;
static constexpr float kAdaptiveThresholdAlpha = 0.05f;
static constexpr bool kClampCounterAtZero = true;
static constexpr bool kDoorProtectionEnabled = false;
static constexpr uint16_t kDoorProtectionDistanceMm = 100;

// Publish intervals
static constexpr uint32_t kSensorLoopIntervalMs = 40;
static constexpr uint32_t kStatePublishIntervalMs = 1000;
static constexpr uint32_t kWifiRssiPublishIntervalMs = 10000;

// Serial diagnostics
static constexpr bool kSerialDebugEnabled = false;
static constexpr uint32_t kSerialDebugSampleIntervalMs = 200;

// Time sync for daily totals reset
static constexpr char kNtpServer1[] = "pool.ntp.org";
static constexpr char kNtpServer2[] = "time.nist.gov";
static constexpr long kGmtOffsetSec = 3600;  // Europe/Oslo winter offset
static constexpr int kDstOffsetSec = 3600;

}  // namespace cfg

