# ESP32PPC Standalone Firmware Documentation

## 1. Overview

This project is a standalone PlatformIO/Arduino firmware for a people counter based on:

- **Board**: Seeed XIAO ESP32-C5
- **Sensor**: VL53L1X (dual ROI technique)
- **Integration**: MQTT-first (no native web UI)
- **Security**: Plain MQTT (`1883`) and TLS MQTT (`8883`) with optional CA and client cert/key

The firmware publishes presence/counting telemetry and accepts remote control/configuration via MQTT commands.

---

## 2. Project Structure

- `platformio.ini`: PlatformIO environment and build settings
- `include/app_config.h`: compile-time device/network/MQTT/default tuning config
- `src/main.cpp`: main app loop, MQTT contract, runtime config handling, counter engine logic
- `src/ppc_types.h`: shared types (ROI, threshold, ranging modes)
- `src/ppc_zone.h/.cpp`: per-zone sampling, threshold calibration, adaptive thresholding
- `src/vl53l1x_device.h/.cpp`: VL53L1X init/read wrapper
- `components/vl53l1x/*`: vendor sensor library sources
- `docs/standalone-mqtt-contract.md`: compact MQTT contract reference

---

## 3. Hardware and Runtime Model

The sensor field is split into two virtual zones (entry/exit ROI). The firmware continuously:

1. reads zone distances
2. decides whether each zone is occupied
3. tracks movement path across zones
4. emits `Entry`/`Exit` events
5. updates `people_count` and `presence`

A full state snapshot is published periodically to MQTT.

---

## 4. Compile-Time Configuration (`include/app_config.h`)

### 4.1 Identity and Topic Namespace

- `kDeviceId`
- `kCustomerId`

These create topic base:

`ppc/v1/c/{customer_id}/d/{device_id}`

### 4.2 Network and MQTT

- Wi-Fi SSID/password
- MQTT host/user/pass
- MQTT plain port (`1883`)
- MQTT TLS port (`8883`)
- TLS enable flag (`kMqttUseTls`)
- optional TLS assets:
  - `kMqttCaCert`
  - `kMqttClientCert`
  - `kMqttClientKey`

### 4.3 Sensor and Algorithm Defaults

- I2C pins/frequency/address
- sampling size
- threshold min/max percentages
- path tracking timeout
- event cooldown
- peak-time heuristic threshold
- adaptive threshold params
- clamped counter mode default
- door protection default (`enabled=false`, `mm=100`)
- serial debug default and sample interval

### 4.4 Persistence Note

Values in `app_config.h` are compiled into the firmware image. In addition, the firmware now stores selected MQTT-applied settings in NVS (`Preferences`) and reapplies them on boot.

Persisted across reboot:
- runtime tuning (`set_config`, including door protection fields)
- `people_counter`
- `clamped_mode`
- latest calibration snapshot (ranging mode + thresholds + ROI)

Not persisted across reboot:
- runtime state (`presence`, `last_direction`, daily counters, uptime, seq, etc.)
- debug runtime setting (`debug_serial` always resets to disabled on boot)

---

## 5. Build and Flash

## 5.1 Build

From project root:

```powershell
& "C:\Users\StighAarstein\AppData\Local\Programs\Python\Python312\python.exe" -m platformio run
```

Produced artifacts (example env):

- `.pio\build\seeed-xiao-esp32-c5\firmware.bin`
- `.pio\build\seeed-xiao-esp32-c5\bootloader.bin`
- `.pio\build\seeed-xiao-esp32-c5\partitions.bin`

## 5.2 Upload

Recommended:

```powershell
& "C:\Users\StighAarstein\AppData\Local\Programs\Python\Python312\python.exe" -m platformio run -t upload --upload-port COM30
```

Manual flashing (esptool / flash tool) for this board layout:

- `0x2000` -> `bootloader.bin`
- `0x8000` -> `partitions.bin`
- `0xE000` -> `boot_app0.bin` (from Arduino framework package)
- `0x10000` -> `firmware.bin`

Use flash mode `dio`. If needed, use `40m` flash frequency.

---

## 6. MQTT Interface

Base topic:

`ppc/v1/c/{customer_id}/d/{device_id}`

### 6.1 Topics

- `{base}/state`: retained JSON state snapshot
- `{base}/event`: non-retained JSON events (`Entry`, `Exit`, `Timeout`)
- `{base}/cmd`: command input topic (firmware subscribes)
- `{base}/ack`: command result topic
- `{base}/availability`: retained `online`/`offline` (LWT)
- `{base}/meta`: retained static metadata/capabilities

### 6.2 QoS / Retain

- `state`: retained
- `availability`: retained
- `event`: non-retained
- `ack`: non-retained

---

## 7. State Payload

Example:

```json
{
  "ts": "2026-03-28T10:00:00Z",
  "seq": 123,
  "people_count": 2,
  "presence": true,
  "last_direction": "Entry",
  "today": { "entry": 7, "exit": 5 },
  "zones": {
    "z0_mm": 1180,
    "z1_mm": 1255,
    "min0": 0,
    "max0": 1325,
    "min1": 0,
    "max1": 1322,
    "roi0_w": 6,
    "roi0_h": 16,
    "roi1_w": 6,
    "roi1_h": 16
  },
  "health": {
    "rssi": -57,
    "uptime_s": 1032,
    "sensor_status": 0
  },
  "config": {
    "invert_direction": true,
    "sampling": 2,
    "threshold_min_percent": 0,
    "threshold_max_percent": 85,
    "path_tracking_timeout_ms": 3000,
    "event_cooldown_ms": 700,
    "peak_time_delta_ms": 120,
    "clamped_mode": true,
    "adaptive": { "enabled": true, "interval_ms": 60000, "alpha": 0.05 },
    "debug": { "serial_enabled": false, "sample_interval_ms": 200 },
    "door_protection": { "enabled": false, "mm": 100 }
  }
}
```

Notes:

- `zones.z0_mm` and `zones.z1_mm` are included only when:
  - presence is true, or
  - serial debug is enabled
- `health.rssi` and `health.uptime_s` are sampled max once per second
- `config.clamped_mode` reflects the current clamp mode (changed via `set_clamped` / `set_clamped_mode`)
- `today.entry/exit` reset at local midnight (requires NTP sync)

---

## 8. Event Payload

Published on `{base}/event`:

```json
{
  "ts": "2026-03-28T10:00:05Z",
  "event": "Entry",
  "count_after": 3
}
```

Possible `event` values:

- `Entry`
- `Exit`
- `Timeout`

---

## 9. Meta Payload

Published on `{base}/meta`, includes schema/version/capabilities.

Example fields:

- `fw_version`
- `device_id`
- `customer_id`
- `mqtt_tls`
- `schema`
- `features.commands`
- `features.daily_totals`
- `features.adaptive_threshold`
- `features.dynamic_config`
- `features.serial_debug`

---

## 10. Command API (`{base}/cmd`)

All commands support optional `request_id` that is echoed in ACK.

### 10.1 Recalibrate

Request:

```json
{ "request_id": "r1", "cmd": "recalibrate" }
```

Behavior:

- recalibrates thresholds and ROI
- resets serial debug runtime settings to defaults from `app_config.h`
- publishes detailed calibration ACK

### 10.2 Get Config

```json
{ "request_id": "g1", "cmd": "get_config" }
```

Returns ACK `config_in_state` and publishes full state/config.

### 10.3 Set Config

Use `set_config` to change runtime tuning.

Supported in either style:

- wrapped:

```json
{ "request_id": "c1", "cmd": "set_config", "config": { "sampling": 2 } }
```

- top-level fields:

```json
{ "request_id": "c2", "cmd": "set_config", "sampling": 2 }
```

Notes:

- You can send one field or many fields in the same command.
- Fields that affect thresholds/sampling trigger recalibration.
- Validated values are persisted and restored on reboot.
- `debug_serial` is applied immediately but intentionally not persisted.

#### 10.3.1 All Supported `set_config` Fields

| Field | Type | Valid values | Explanation |
|---|---|---|---|
| `sampling` | int | `1..16` | Sample window size per zone (higher = smoother, slightly slower response). |
| `threshold_min_percent` | int | `0..100` | Lower detection bound as % of idle baseline. |
| `threshold_max_percent` | int | `0..100` | Upper detection bound as % of idle baseline. Must be greater than min. |
| `invert_direction` | bool | `true`/`false` | Swaps entry/exit interpretation. |
| `path_tracking_timeout_ms` | int | `0..60000` | Resets incomplete crossing after timeout. `0` disables timeout. |
| `event_cooldown_ms` | int | `0..10000` | Minimum delay between count events (anti-double-count). |
| `peak_time_delta_ms` | int | `0..2000` | Peak-time heuristic threshold for direction fallback. |
| `debug_serial` | bool | `true`/`false` | Enables/disables serial debug logs. Resets to disabled on reboot. |
| `debug_interval_ms` | int | `20..10000` | Debug print interval in ms when debug is enabled. |
| `adaptive.enabled` | bool | `true`/`false` | Enables adaptive baseline updates when zones are empty. |
| `adaptive.interval_ms` | int | `1000..3600000` | Empty duration before adaptive update. |
| `adaptive.alpha` | float | `(0.0, 1.0]` | Adaptive smoothing factor. |
| `door_protection.enabled` | bool | `true`/`false` | Enables near-door suppression logic. |
| `door_protection.mm` | int | `0..4000` | Distances `<= mm` are discarded for presence and counting. |
| `door_protection_enabled` | bool | `true`/`false` | Top-level alias for `door_protection.enabled`. |
| `door_protection_mm` | int | `0..4000` | Top-level alias for `door_protection.mm`. |

#### 10.3.2 `set_config` Examples

Set one field:

```json
{ "request_id": "cfg-sampling", "cmd": "set_config", "config": { "sampling": 3 } }
```

Set thresholds:

```json
{
  "request_id": "cfg-th",
  "cmd": "set_config",
  "config": {
    "threshold_min_percent": 2,
    "threshold_max_percent": 84
  }
}
```

Set direction inversion:

```json
{ "request_id": "cfg-invert", "cmd": "set_config", "config": { "invert_direction": true } }
```

Set timeout/cooldown/peak detector:

```json
{
  "request_id": "cfg-timing",
  "cmd": "set_config",
  "config": {
    "path_tracking_timeout_ms": 3000,
    "event_cooldown_ms": 700,
    "peak_time_delta_ms": 120
  }
}
```

Set adaptive tuning:

```json
{
  "request_id": "cfg-adaptive",
  "cmd": "set_config",
  "config": {
    "adaptive": {
      "enabled": true,
      "interval_ms": 60000,
      "alpha": 0.05
    }
  }
}
```

Enable debug output:

```json
{
  "request_id": "cfg-debug",
  "cmd": "set_config",
  "config": {
    "debug_serial": true,
    "debug_interval_ms": 150
  }
}
```

Set door protection (nested):

```json
{
  "request_id": "cfg-door-nested",
  "cmd": "set_config",
  "config": {
    "door_protection": {
      "enabled": true,
      "mm": 100
    }
  }
}
```

Set door protection (top-level aliases):

```json
{
  "request_id": "cfg-door-top",
  "cmd": "set_config",
  "door_protection_enabled": true,
  "door_protection_mm": 120
}
```

Full combined example:

```json
{
  "request_id": "cfg-full",
  "cmd": "set_config",
  "config": {
    "sampling": 2,
    "threshold_min_percent": 0,
    "threshold_max_percent": 85,
    "invert_direction": true,
    "path_tracking_timeout_ms": 3000,
    "event_cooldown_ms": 700,
    "peak_time_delta_ms": 120,
    "adaptive": {
      "enabled": true,
      "interval_ms": 60000,
      "alpha": 0.05
    },
    "debug_serial": false,
    "debug_interval_ms": 200,
    "door_protection": {
      "enabled": false,
      "mm": 100
    }
  }
}
```

### 10.4 Set People Counter

```json
{ "request_id": "pc1", "cmd": "set_people_counter", "value": 5 }
```

### 10.5 Set Clamp Mode

Both command names are accepted:

- `set_clamped`
- `set_clamped_mode`

Accepted value keys:

- `value`
- `clamped`

Accepted bool formats:

- `true` / `false`
- `1` / `0`
- `"on"` / `"off"`

Example:

```json
{ "request_id": "cl1", "cmd": "set_clamped", "value": "on" }
```

This command updates `config.clamped_mode` in subsequent state payloads.

### 10.6 Set Door Protection (quick command)

```json
{ "request_id": "dp1", "cmd": "set_door_protection", "enabled": true, "mm": 100 }
```

Aliases:

- `enabled` or `value` for on/off
- `mm` or `distance_mm` for threshold distance

### 10.7 Restart

```json
{ "request_id": "rs1", "cmd": "restart" }
```

### 10.8 Unsupported or Invalid JSON

- invalid JSON -> ACK with `ok=false` and message `invalid_json: ...`
- unknown command -> ACK with `unsupported_command`

---

## 11. ACK Payloads

Standard ACK shape:

```json
{ "ts": "...", "request_id": "...", "ok": true, "message": "..." }
```

`recalibrate` success ACK additionally includes:

- `calibration.ranging_mode`
- `calibration.z0` and `calibration.z1`:
  - `idle_mm`, `min_mm`, `max_mm`, `roi_w`, `roi_h`, `roi_center`

---

## 12. Runtime Config Reference

| Field | Type | Range | Notes |
|---|---|---|---|
| `sampling` | int | 1..16 | Recalibration required |
| `threshold_min_percent` | int | 0..100 | Must be `< threshold_max_percent`, recalibration required |
| `threshold_max_percent` | int | 0..100 | Must be `> threshold_min_percent`, recalibration required |
| `invert_direction` | bool | - | Swaps entry/exit interpretation |
| `path_tracking_timeout_ms` | int | 0..60000 | `0` disables timeout |
| `event_cooldown_ms` | int | 0..10000 | Prevents quick double counts |
| `peak_time_delta_ms` | int | 0..2000 | Direction fallback threshold |
| `clamped_mode` | bool | - | Current clamp mode in state; set using `set_clamped` / `set_clamped_mode` (not a `set_config` field). |
| `debug_serial` | bool | - | Enables serial debug logs |
| `debug_interval_ms` | int | 20..10000 | Debug print period |
| `adaptive.enabled` | bool | - | Enable adaptive baseline |
| `adaptive.interval_ms` | int | 1000..3600000 | Empty-time required before update |
| `adaptive.alpha` | float | (0, 1] | EMA factor |
| `door_protection.enabled` | bool | - | Blocks near-sensor readings |
| `door_protection.mm` | int | 0..4000 | Distances `<= mm` are discarded |
| `door_protection_enabled` | bool | - | Top-level alias |
| `door_protection_mm` | int | 0..4000 | Top-level alias |

Persistence behavior:
- Most config changes are persisted and restored on boot.
- `debug_serial` is intentionally non-persistent and resets to disabled after reboot.

---

## 13. Door Protection

Purpose: avoid false events when very near objects (for example door leaf close to sensor) cross the ROI.

When enabled:

- if either zone reads distance `> 0` and `<= configured mm`:
  - `presence = false`
  - path tracking state is reset
  - no entry/exit count is generated from that sample

This applies only while protection is enabled.

---

## 14. Counting Logic (High-Level)

The engine combines zone occupancy into states:

- left-only
- right-only
- both
- none

Direction decision uses layered heuristics:

1. **First/last single-zone order** (preferred)
2. **Peak-time ordering** if needed
3. **Balance crossing sign** fallback

Protection and robustness:

- path timeout emits `Timeout` then resets
- event cooldown suppresses rapid re-trigger
- optional adaptive thresholding adjusts idle baseline over time

---

## 15. Calibration and ROI Behavior

Calibration sequence:

1. reset ROI defaults
2. measure idle distances
3. auto-select ranging mode based on idle distance (short/medium/long...) unless override exists in code
4. auto-calculate ROI size/center and thresholds

Threshold logic:

- `min = idle * min_percent`
- `max = idle * max_percent`
- occupied if `min_distance < max && min_distance > min`

Sampling:

- each zone keeps a sample window (`sampling`)
- occupancy uses minimum of that window to smooth detection

---

## 16. Serial Diagnostics

Enable at runtime:

```json
{ "request_id": "dbg1", "cmd": "set_config", "config": { "debug_serial": true, "debug_interval_ms": 150 } }
```

Main serial log families:

- `[PPC][DBG]` periodic snapshot
- `[PPC][TRACE]` state transitions, timeout, finalize reason, cooldown suppression, door-protect blocks
- `[PPC][EVENT]` entry/exit event logs
- `[VL53L1X] ...` driver-level sensor errors
- `[WiFi] ...`, `[MQTT] ...` connectivity status

`[PPC][DBG]` fields:

- `d0`, `d1`: latest zone distances (mm)
- `th0`, `th1`: min/max thresholds per zone
- `occ0`, `occ1`: occupied flags per zone
- `cnt`: people counter
- `pres`: presence
- `dir`: last direction string

---

## 17. Daily Totals and Time

`today.entry` and `today.exit` reset when local day changes.

Requirements:

- successful NTP sync (`configTime(...)`)
- correct GMT/DST offset in `app_config.h`

---

## 18. MQTT Security and Multi-Customer Isolation

Recommended topic ACL boundary is `customer_id` segment.

Example policy approach:

- device identity can read/write only its own base topic
- customer app user can read only `ppc/v1/c/{customer_id}/#`
- never grant cross-customer wildcards

TLS options:

- server CA validation via `kMqttCaCert`
- optional mutual TLS with client cert/key
- if TLS enabled without CA, code falls back to insecure TLS (testing only)

---

## 19. Operational Examples

### 19.1 Enable door protection at 120 mm

```json
{ "request_id": "dp120", "cmd": "set_door_protection", "enabled": true, "mm": 120 }
```

### 19.2 Disable door protection

```json
{ "request_id": "dpoff", "cmd": "set_door_protection", "enabled": false }
```

### 19.3 Tighten detection threshold and sampling

```json
{
  "request_id": "tune1",
  "cmd": "set_config",
  "config": {
    "sampling": 3,
    "threshold_min_percent": 3,
    "threshold_max_percent": 82,
    "event_cooldown_ms": 800,
    "peak_time_delta_ms": 140
  }
}
```

### 19.4 Get live config snapshot

```json
{ "request_id": "cfgdump", "cmd": "get_config" }
```

### 19.5 Force recalibration

```json
{ "request_id": "recal1", "cmd": "recalibrate" }
```

---

## 20. Known Limitations

- Runtime state is intentionally non-persistent (presence, last direction, daily totals, uptime, sequence)
- No OTA update flow included in current firmware
- Orientation is currently fixed in code (not exposed by MQTT command)
- Debug over MQTT is indirect (state payload includes live distances only when presence or debug serial is enabled)

---

## 21. Troubleshooting Quick Checklist

1. **No boot after flash**
   - verify correct board (`seeed-xiao-esp32-c5`)
   - verify flash offsets (`0x2000`, `0x8000`, `0xE000`, `0x10000`)
2. **Sensor init fails**
   - check I2C pins/wiring/power
   - inspect `[VL53L1X]` logs
3. **Wrong direction counts**
   - toggle `invert_direction`
   - review ROI/thresholds and pass height/speed
4. **Double counts**
   - increase `event_cooldown_ms`
   - tune `peak_time_delta_ms`
5. **Too much MQTT traffic**
   - keep debug serial off
   - rely on state filtering behavior (`z0_mm/z1_mm` only when active/debug)
6. **Door motion false triggers**
   - enable `door_protection` and adjust `mm`

---

## 22. Related Docs

- `docs/standalone-mqtt-contract.md`: compact API contract
- `README.md`: repository overview and legacy ESPHome context





