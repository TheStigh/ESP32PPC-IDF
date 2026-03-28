# ESP32PPC Test Checklist (MQTT + Provisioning)

Use this as a practical runbook during validation.

## Test Session Info

- [ ] Date:
- [ ] Tester:
- [ ] Board/Port:
- [ ] Firmware build path:
  - `.pio/build/seeed-xiao-esp32-c5/firmware.bin`
- [ ] Broker host/port:
- [ ] Base topic:
  - `ppc/v1/c/<customer_id>/d/<device_id>`

## 1) Flash and Boot

- [ ] Build firmware succeeds.
- [ ] Flash succeeds (bootloader/partitions/boot_app0/firmware offsets correct).
- [ ] Device boots without reset loop.
- [ ] Serial log shows setup complete.

## 2) First-Boot Provisioning Portal

- [ ] With empty/invalid provisioning in NVS, AP starts.
- [ ] AP SSID format is `ESP32PPC-Setup-XXXX`.
- [ ] AP password works (`ppcsetup`, unless changed).
- [ ] Setup page opens at `http://192.168.4.1`.
- [ ] Top-right logo is shown.
- [ ] WiFi scan button shows scanning state (spinner + disabled button).
- [ ] Scan list populates nearby SSIDs.
- [ ] Save and reboot works.
- [ ] Device exits AP mode and connects to configured WiFi.

## 3) MQTT Connectivity

- [ ] Device connects and publishes `availability = online`.
- [ ] `meta` is published and retained.
- [ ] `state` is published and retained.
- [ ] Commands on `{base}/cmd` produce ACK on `{base}/ack`.

## 4) State Payload Validation

- [ ] `state.ts`, `state.seq`, `people_count`, `presence`, `last_direction` present.
- [ ] `today.entry` and `today.exit` present.
- [ ] `zones.min0/max0/min1/max1` present.
- [ ] `config.clamped_mode` present.
- [ ] `health.rssi` and `health.uptime_s` update at ~1 Hz max.
- [ ] `zones.z0_mm/z1_mm` only sent when presence is true or debug is enabled.

## 5) Command Tests (MQTT Payloads)

Publish to: `{base}/cmd`

### 5.1 get_config

- [ ] Sent
- [ ] ACK is `config_in_state`

```json
{ "request_id": "t-getcfg-1", "cmd": "get_config" }
```

### 5.2 set_config - sampling

- [ ] Sent
- [ ] ACK is `config_updated`
- [ ] Reflected in `state.config.sampling`

```json
{ "request_id": "t-cfg-1", "cmd": "set_config", "config": { "sampling": 2 } }
```

### 5.3 set_config - thresholds + timing

- [ ] Sent
- [ ] ACK is `config_updated`

```json
{
  "request_id": "t-cfg-2",
  "cmd": "set_config",
  "config": {
    "threshold_min_percent": 0,
    "threshold_max_percent": 85,
    "path_tracking_timeout_ms": 3000,
    "event_cooldown_ms": 700,
    "peak_time_delta_ms": 120
  }
}
```

### 5.4 set_config - adaptive

- [ ] Sent
- [ ] ACK is `config_updated`

```json
{
  "request_id": "t-cfg-3",
  "cmd": "set_config",
  "config": {
    "adaptive": { "enabled": true, "interval_ms": 60000, "alpha": 0.05 }
  }
}
```

### 5.5 set_config - invert direction

- [ ] Sent
- [ ] ACK is `config_updated`
- [ ] `state.config.invert_direction` changed

```json
{ "request_id": "t-cfg-4", "cmd": "set_config", "config": { "invert_direction": true } }
```

### 5.6 set_config - debug serial

- [ ] Sent
- [ ] ACK is `config_updated`
- [ ] Debug lines appear on serial

```json
{
  "request_id": "t-cfg-5",
  "cmd": "set_config",
  "config": { "debug_serial": true, "debug_interval_ms": 150 }
}
```

### 5.7 set_people_counter

- [ ] Sent
- [ ] ACK is `people_counter_updated`
- [ ] `state.people_count` updated

```json
{ "request_id": "t-pcnt-1", "cmd": "set_people_counter", "value": 3 }
```

### 5.8 set_clamped / set_clamped_mode

- [ ] Sent (`set_clamped`)
- [ ] ACK is `clamped_mode_updated`
- [ ] `state.config.clamped_mode` updated

```json
{ "request_id": "t-clamp-1", "cmd": "set_clamped", "value": true }
```

- [ ] Sent (`set_clamped_mode`)
- [ ] ACK is `clamped_mode_updated`

```json
{ "request_id": "t-clamp-2", "cmd": "set_clamped_mode", "value": "off" }
```

### 5.9 set_door_protection

- [ ] Sent
- [ ] ACK is `door_protection_updated`
- [ ] Reflected under `state.config.door_protection`

```json
{ "request_id": "t-door-1", "cmd": "set_door_protection", "enabled": true, "mm": 100 }
```

### 5.10 recalibrate

- [ ] Sent
- [ ] ACK is `recalibrated`
- [ ] ACK includes calibration object (`ranging_mode`, `z0`, `z1`)

```json
{ "request_id": "t-recal-1", "cmd": "recalibrate" }
```

### 5.11 restart

- [ ] Sent
- [ ] ACK is `restarting`
- [ ] Device reconnects and publishes `online`

```json
{ "request_id": "t-rst-1", "cmd": "restart" }
```

### 5.12 factory_reset

- [ ] Sent
- [ ] ACK is `factory_reset_restarting`
- [ ] Device reboots to provisioning AP mode

```json
{ "request_id": "t-fr-1", "cmd": "factory_reset", "confirm": "ERASE_NVS" }
```

## 6) Persistence Tests (Reboot Survival)

Before reboot, set values:
- [ ] `invert_direction`
- [ ] `sampling`
- [ ] thresholds
- [ ] `path_tracking_timeout_ms`
- [ ] `event_cooldown_ms`
- [ ] `peak_time_delta_ms`
- [ ] adaptive fields
- [ ] `door_protection`
- [ ] `clamped_mode`
- [ ] `people_counter`

After reboot:
- [ ] All above values are restored from NVS.
- [ ] Calibration snapshot is reused/restored.
- [ ] `debug_serial` is reset to default OFF.
- [ ] Runtime state (`presence`, `last_direction`, daily counters, uptime/seq) is not persisted.

## 7) Functional Counting Tests

- [ ] Entry movement increments count.
- [ ] Exit movement decrements count.
- [ ] Direction inversion behaves as expected when `invert_direction=true`.
- [ ] Rapid repeated passes do not double count excessively (cooldown).
- [ ] Timeout behavior works for incomplete crossings.

## 8) Door Protection Behavior

- [ ] With door protection enabled, near-object movement (`<= mm`) is suppressed.
- [ ] No false presence during near-door movement.
- [ ] No false entry/exit during near-door movement.
- [ ] Disabling door protection restores normal detection.

## 9) TLS Tests

- [ ] Plain MQTT (`1883`) works when TLS disabled.
- [ ] TLS MQTT (`8883`) works with valid CA cert in provisioning.
- [ ] TLS without CA gives expected insecure TLS behavior (test-only warning).

## 10) Negative / Validation Tests

- [ ] Invalid JSON returns `invalid_json: ...` ACK.
- [ ] Unsupported command returns `unsupported_command`.
- [ ] Invalid config fields return proper `invalid_*` ACK messages.
- [ ] `factory_reset` without valid confirmation returns `factory_reset_confirmation_required`.

## 11) Final Sign-off

- [ ] All blocking tests passed.
- [ ] Remaining issues documented.
- [ ] Ready for pilot/field test.

## Issues Found

- [ ] Issue #1:
- [ ] Issue #2:
- [ ] Issue #3:
