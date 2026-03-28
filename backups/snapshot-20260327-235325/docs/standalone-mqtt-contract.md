# Standalone MQTT Contract (v1)

This firmware publishes and subscribes under:

`ppc/v1/c/{customer_id}/d/{device_id}`

## Topics

- `{base}/state` JSON snapshot, retained, QoS 1
- `{base}/event` JSON events (`Entry`, `Exit`, `Timeout`), non-retained, QoS 1
- `{base}/cmd` JSON commands to the device, subscribed by firmware
- `{base}/ack` JSON command results, non-retained
- `{base}/availability` `online|offline` (LWT), retained
- `{base}/meta` static metadata, retained

## Command payloads

- Recalibrate:
  `{ "request_id": "abc", "cmd": "recalibrate" }`
- Set counter:
  `{ "request_id": "abc", "cmd": "set_people_counter", "value": 4 }`
- Set clamp mode:
  `{ "request_id": "abc", "cmd": "set_clamped", "value": true }`
- Get runtime config (response is in `{base}/state.config`):
  `{ "request_id": "abc", "cmd": "get_config" }`
- Set runtime config (ESPHome-like tuning fields):
  `{ "request_id": "abc", "cmd": "set_config", "config": { "sampling": 2, "threshold_min_percent": 0, "threshold_max_percent": 85, "invert_direction": true, "path_tracking_timeout_ms": 3000, "event_cooldown_ms": 700, "peak_time_delta_ms": 120, "adaptive": { "enabled": true, "interval_ms": 60000, "alpha": 0.05 }, "debug_serial": true, "debug_interval_ms": 200 } }`
- Restart:
  `{ "request_id": "abc", "cmd": "restart" }`

## Runtime config notes

- `sampling`: `1..16`
- `threshold_min_percent`: `0..100`
- `threshold_max_percent`: `0..100` and must be greater than min
- `path_tracking_timeout_ms`: `0..60000` (`0` disables timeout)
- `event_cooldown_ms`: `0..10000` (anti-double-count guard)
- `peak_time_delta_ms`: `0..2000` (fallback direction detector for overlap-heavy crossings)
- `adaptive.interval_ms`: `1000..3600000`
- `adaptive.alpha`: `(0.0, 1.0]`
- `debug_serial`: enables detailed serial traces (`[PPC][DBG]`, `[PPC][TRACE]`, `[PPC][EVENT]`)

## ACL recommendation

Use the `customer_id` segment as hard isolation boundary.

- Device identity `device-{customer_id}-{device_id}`:
  - read/write: `ppc/v1/c/{customer_id}/d/{device_id}/#`
- Customer user identity `user-{customer_id}`:
  - read-only (or app-specific write to cmd): `ppc/v1/c/{customer_id}/#`

Never grant wildcard read across customers.

## TLS

- Unencrypted: port 1883
- TLS: port 8883
- Firmware supports CA pinning and optional client certificate/key.
- If CA is empty, firmware can use insecure TLS for testing only.

