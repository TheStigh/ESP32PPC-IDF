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
- Restart:
  `{ "request_id": "abc", "cmd": "restart" }`

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
