from __future__ import annotations

import json
import queue
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from tkinter import Canvas, messagebox
from typing import Any

import customtkinter as ctk
import paho.mqtt.client as mqtt


SETTINGS_PATH = Path.home() / ".esp32ppc_gui_settings.json"
MAX_WIDTH = 1920
MAX_HEIGHT = 1080
TEXT_PRIMARY = "#f3f4f6"
TEXT_MUTED = "#a1a1aa"
PRESENCE_GREEN = "#22c55e"
PRESENCE_RED = "#ef4444"


@dataclass
class MqttSettings:
    host: str = "127.0.0.1"
    port: int = 1883
    username: str = ""
    password: str = ""
    use_tls: bool = False
    tls_insecure: bool = False
    ca_cert: str = ""
    client_cert: str = ""
    client_key: str = ""
    keepalive: int = 60
    base_prefix: str = "ppc/v1"
    customer_id: str = "customer-demo"
    device_id: str = "esp32ppc-c5-01"

    def base_topic(self) -> str:
        prefix = self.base_prefix.strip().strip("/")
        customer = self.customer_id.strip()
        device = self.device_id.strip()
        return f"{prefix}/c/{customer}/d/{device}"


class Esp32PpcGui(ctk.CTk):
    def __init__(self) -> None:
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        super().__init__()

        self.title("ESP32PPC MQTT Command UI")
        self.maxsize(MAX_WIDTH, MAX_HEIGHT)
        self.minsize(1080, 700)
        self.geometry("1400x900")

        self.settings = self._load_settings()
        self.topic_state = ""
        self.topic_ack = ""
        self.topic_cmd = ""
        self.topic_availability = ""
        self._refresh_topics()

        self.client: mqtt.Client | None = None
        self.ui_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.pending_requests: dict[str, dict[str, Any]] = {}
        self.connected = False
        self.connecting = False
        self.settings_dialog: ctk.CTkToplevel | None = None
        self._suspend_switch_callbacks = False
        self._door_toggle_waiting_ack = False
        self._cancel_device_validation()
        self._set_command_controls_enabled(False)
        self.door_enabled_switch: ctk.CTkSwitch | None = None

        self._device_validation_pending = False
        self._device_seen_since_connect = False
        self._device_validation_after_id: str | None = None
        self.command_controls: list[Any] = []
        self.people_count_var = ctk.StringVar(value="0")
        self.entry_count_var = ctk.StringVar(value="0")
        self.exit_count_var = ctk.StringVar(value="0")
        self.last_command_var = ctk.StringVar(value="")
        self.last_ack_var = ctk.StringVar(value="")
        self.status_var = ctk.StringVar(value="Disconnected")

        self.people_counter_input = ctk.StringVar(value="0")
        self.clamped_var = ctk.BooleanVar(value=True)
        self.invert_direction_var = ctk.BooleanVar(value=True)
        self.adaptive_enabled_var = ctk.BooleanVar(value=True)
        self.debug_serial_var = ctk.BooleanVar(value=False)
        self.door_enabled_var = ctk.BooleanVar(value=False)
        self.door_mm_var = ctk.StringVar(value="100")

        self.config_vars: dict[str, ctk.StringVar] = {
            "sampling": ctk.StringVar(value="2"),
            "threshold_min_percent": ctk.StringVar(value="0"),
            "threshold_max_percent": ctk.StringVar(value="85"),
            "path_tracking_timeout_ms": ctk.StringVar(value="3000"),
            "event_cooldown_ms": ctk.StringVar(value="700"),
            "peak_time_delta_ms": ctk.StringVar(value="120"),
            "adaptive_interval_ms": ctk.StringVar(value="60000"),
            "adaptive_alpha": ctk.StringVar(value="0.05"),
            "debug_interval_ms": ctk.StringVar(value="200"),
        }
        self._build_ui()
        self._set_command_controls_enabled(False)
        self.after(100, self._drain_ui_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        top = ctk.CTkFrame(self, corner_radius=14)
        top.grid(row=0, column=0, padx=16, pady=(16, 10), sticky="nsew")
        top.grid_columnconfigure(0, weight=1)
        top.grid_columnconfigure(1, weight=0)

        metrics = ctk.CTkFrame(top, fg_color="transparent")
        metrics.grid(row=0, column=0, padx=(8, 0), pady=8, sticky="w")

        metric_font = ctk.CTkFont(family="Segoe UI", size=34, weight="bold")

        ctk.CTkLabel(
            metrics,
            text="People Presence:",
            width=340,
            anchor="e",
            font=metric_font,
            text_color=TEXT_PRIMARY,
        ).grid(row=0, column=0, padx=(0, 14), pady=(4, 6), sticky="e")
        ctk.CTkLabel(
            metrics,
            textvariable=self.people_count_var,
            font=metric_font,
            text_color=TEXT_PRIMARY,
            width=80,
            anchor="w",
        ).grid(row=0, column=1, pady=(4, 6), sticky="w")

        self.presence_canvas = Canvas(metrics, width=26, height=26, bg="#2b2b2b", highlightthickness=0)
        self.presence_canvas.grid(row=0, column=2, pady=(4, 6), padx=(8, 0), sticky="w")
        self.presence_circle = self.presence_canvas.create_oval(3, 3, 23, 23, fill=PRESENCE_RED, outline="")

        ctk.CTkLabel(
            metrics,
            text="Entry:",
            width=340,
            anchor="e",
            font=metric_font,
            text_color=TEXT_PRIMARY,
        ).grid(row=1, column=0, padx=(0, 14), pady=6, sticky="e")
        ctk.CTkLabel(
            metrics,
            textvariable=self.entry_count_var,
            font=metric_font,
            text_color=TEXT_PRIMARY,
            width=120,
            anchor="w",
        ).grid(row=1, column=1, columnspan=2, pady=6, sticky="w")

        ctk.CTkLabel(
            metrics,
            text="Exit:",
            width=340,
            anchor="e",
            font=metric_font,
            text_color=TEXT_PRIMARY,
        ).grid(row=2, column=0, padx=(0, 14), pady=(6, 4), sticky="e")
        ctk.CTkLabel(
            metrics,
            textvariable=self.exit_count_var,
            font=metric_font,
            text_color=TEXT_PRIMARY,
            width=120,
            anchor="w",
        ).grid(row=2, column=1, columnspan=2, pady=(6, 4), sticky="w")

        right = ctk.CTkFrame(top, fg_color="transparent")
        right.grid(row=0, column=1, padx=10, pady=10, sticky="ne")

        self.connect_button = ctk.CTkButton(
            right,
            text="Connect",
            width=130,
            height=40,
            command=self._toggle_connection,
        )
        self.connect_button.grid(row=0, column=0, padx=(0, 8), pady=(0, 6), sticky="e")

        self.settings_button = ctk.CTkButton(
            right,
            text="\u2699",
            width=40,
            height=40,
            command=self._open_settings_dialog,
            font=ctk.CTkFont(family="Segoe UI", size=20),
        )
        self.settings_button.grid(row=0, column=1, pady=(0, 6), sticky="e")

        ctk.CTkLabel(right, textvariable=self.status_var, text_color=TEXT_MUTED, anchor="e").grid(
            row=1, column=0, columnspan=2, sticky="e"
        )

        content = ctk.CTkFrame(self, corner_radius=14)
        content.grid(row=1, column=0, padx=16, pady=10, sticky="nsew")
        content.grid_rowconfigure(0, weight=1)
        content.grid_columnconfigure(0, weight=1)

        tabs = ctk.CTkTabview(content, corner_radius=12)
        tabs.grid(row=0, column=0, padx=8, pady=8, sticky="nsew")

        tab_cmd = tabs.add("Commands")
        tab_cfg = tabs.add("Runtime Config")
        tab_cmd.grid_columnconfigure(0, weight=1)
        tab_cmd.grid_columnconfigure(1, weight=1)
        tab_cfg.grid_columnconfigure(0, weight=1)

        self._build_commands_tab(tab_cmd)
        self._build_runtime_config_tab(tab_cfg)

        bottom = ctk.CTkFrame(self, corner_radius=14)
        bottom.grid(row=2, column=0, padx=16, pady=(10, 16), sticky="ew")
        bottom.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(bottom, text="Command:", width=100, anchor="e").grid(
            row=0, column=0, padx=(10, 8), pady=(10, 6), sticky="e"
        )
        ctk.CTkEntry(bottom, textvariable=self.last_command_var, state="readonly", height=32).grid(
            row=0, column=1, padx=(0, 10), pady=(10, 6), sticky="ew"
        )

        ctk.CTkLabel(bottom, text="ACK:", width=100, anchor="e").grid(
            row=1, column=0, padx=(10, 8), pady=(6, 10), sticky="e"
        )
        ctk.CTkEntry(bottom, textvariable=self.last_ack_var, state="readonly", height=32).grid(
            row=1, column=1, padx=(0, 10), pady=(6, 10), sticky="ew"
        )
    def _build_commands_tab(self, parent: ctk.CTkFrame) -> None:
        actions = ctk.CTkFrame(parent)
        actions.grid(row=0, column=0, columnspan=2, padx=12, pady=(12, 8), sticky="ew")
        actions.grid_columnconfigure((0, 1, 2, 3), weight=1)

        btn_recal = ctk.CTkButton(actions, text="Recalibrate", command=self._cmd_recalibrate)
        btn_recal.grid(row=0, column=0, padx=8, pady=10, sticky="ew")
        self._register_command_control(btn_recal)

        btn_get_cfg = ctk.CTkButton(actions, text="Get Config", command=self._cmd_get_config)
        btn_get_cfg.grid(row=0, column=1, padx=8, pady=10, sticky="ew")
        self._register_command_control(btn_get_cfg)

        btn_restart = ctk.CTkButton(actions, text="Restart", command=self._cmd_restart)
        btn_restart.grid(row=0, column=2, padx=8, pady=10, sticky="ew")
        self._register_command_control(btn_restart)

        btn_factory = ctk.CTkButton(
            actions,
            text="Factory Reset",
            fg_color="#7f1d1d",
            hover_color="#991b1b",
            command=self._cmd_factory_reset,
        )
        btn_factory.grid(row=0, column=3, padx=8, pady=10, sticky="ew")
        self._register_command_control(btn_factory)

        counter = ctk.CTkFrame(parent)
        counter.grid(row=1, column=0, padx=(12, 6), pady=8, sticky="nsew")
        counter.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(counter, text="People Counter").grid(
            row=0, column=0, columnspan=2, padx=12, pady=(10, 8), sticky="w"
        )
        ctk.CTkLabel(counter, text="Value:", width=80, anchor="e").grid(
            row=1, column=0, padx=(12, 6), pady=(0, 10), sticky="e"
        )
        ctk.CTkEntry(counter, textvariable=self.people_counter_input).grid(
            row=1, column=1, padx=(0, 12), pady=(0, 10), sticky="ew"
        )

        btn_set_counter = ctk.CTkButton(counter, text="Set People Counter", command=self._cmd_set_people_counter)
        btn_set_counter.grid(row=2, column=0, columnspan=2, padx=12, pady=(0, 12), sticky="ew")
        self._register_command_control(btn_set_counter)

        modes = ctk.CTkFrame(parent)
        modes.grid(row=1, column=1, padx=(6, 12), pady=8, sticky="nsew")

        ctk.CTkLabel(modes, text="Modes and Protection").grid(
            row=0, column=0, columnspan=2, padx=12, pady=(10, 8), sticky="w"
        )

        ctk.CTkLabel(modes, text="Clamped mode:", width=140, anchor="e").grid(
            row=1, column=0, padx=(12, 8), pady=6, sticky="e"
        )
        sw_clamped = ctk.CTkSwitch(modes, text="", variable=self.clamped_var, command=self._cmd_set_clamped_mode)
        sw_clamped.grid(row=1, column=1, padx=(0, 12), pady=6, sticky="w")
        self._register_command_control(sw_clamped)

        ctk.CTkLabel(modes, text="Invert Direction:", width=140, anchor="e").grid(
            row=2, column=0, padx=(12, 8), pady=6, sticky="e"
        )
        sw_invert = ctk.CTkSwitch(modes, text="", variable=self.invert_direction_var, command=self._cmd_set_invert_direction)
        sw_invert.grid(row=2, column=1, padx=(0, 12), pady=6, sticky="w")
        self._register_command_control(sw_invert)

        ctk.CTkLabel(modes, text="Adaptive Enabled:", width=140, anchor="e").grid(
            row=3, column=0, padx=(12, 8), pady=6, sticky="e"
        )
        sw_adaptive = ctk.CTkSwitch(modes, text="", variable=self.adaptive_enabled_var, command=self._cmd_set_adaptive_enabled)
        sw_adaptive.grid(row=3, column=1, padx=(0, 12), pady=6, sticky="w")
        self._register_command_control(sw_adaptive)

        ctk.CTkLabel(modes, text="Debug Serial:", width=140, anchor="e").grid(
            row=4, column=0, padx=(12, 8), pady=6, sticky="e"
        )
        sw_debug = ctk.CTkSwitch(modes, text="", variable=self.debug_serial_var, command=self._cmd_set_debug_serial)
        sw_debug.grid(row=4, column=1, padx=(0, 12), pady=6, sticky="w")
        self._register_command_control(sw_debug)

        ctk.CTkLabel(modes, text="Door protect:", width=140, anchor="e").grid(
            row=5, column=0, padx=(12, 8), pady=6, sticky="e"
        )
        self.door_enabled_switch = ctk.CTkSwitch(
            modes,
            text="",
            variable=self.door_enabled_var,
            command=self._cmd_set_door_enabled,
        )
        self.door_enabled_switch.grid(row=5, column=1, padx=(0, 12), pady=6, sticky="w")
        self._register_command_control(self.door_enabled_switch)

        ctk.CTkLabel(modes, text="Distance mm:", width=140, anchor="e").grid(
            row=6, column=0, padx=(12, 8), pady=6, sticky="e"
        )
        ctk.CTkEntry(modes, textvariable=self.door_mm_var).grid(
            row=6, column=1, padx=(0, 12), pady=6, sticky="w"
        )
        btn_apply_door = ctk.CTkButton(modes, text="Apply Door Protection", command=self._cmd_set_door_protection)
        btn_apply_door.grid(row=7, column=0, columnspan=2, padx=12, pady=(6, 12), sticky="ew")
        self._register_command_control(btn_apply_door)

    def _build_runtime_config_tab(self, parent: ctk.CTkFrame) -> None:
        scroll = ctk.CTkScrollableFrame(parent)
        scroll.grid(row=0, column=0, padx=12, pady=12, sticky="nsew")
        scroll.grid_columnconfigure(1, weight=1)

        rows = [
            ("sampling", "Sampling:"),
            ("threshold_min_percent", "Threshold Min %:"),
            ("threshold_max_percent", "Threshold Max %:"),
            ("path_tracking_timeout_ms", "Path Timeout ms:"),
            ("event_cooldown_ms", "Cooldown ms:"),
            ("peak_time_delta_ms", "Peak Delta ms:"),
            ("adaptive_interval_ms", "Adaptive Interval ms:"),
            ("adaptive_alpha", "Adaptive Alpha:"),
            ("debug_interval_ms", "Debug Interval ms:"),
        ]

        for i, (key, label) in enumerate(rows):
            ctk.CTkLabel(scroll, text=label, width=190, anchor="e").grid(
                row=i, column=0, padx=(6, 10), pady=6, sticky="e"
            )
            ctk.CTkEntry(scroll, textvariable=self.config_vars[key]).grid(
                row=i, column=1, padx=(0, 6), pady=6, sticky="ew"
            )

        btn_apply_cfg = ctk.CTkButton(scroll, text="Apply Runtime Config", command=self._cmd_set_config)
        btn_apply_cfg.grid(row=len(rows), column=0, columnspan=2, padx=6, pady=(14, 8), sticky="ew")
        self._register_command_control(btn_apply_cfg)

    def _register_command_control(self, widget: Any) -> None:
        self.command_controls.append(widget)

    def _set_command_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for widget in self.command_controls:
            try:
                widget.configure(state=state)
            except Exception:
                pass

        if enabled and self._door_toggle_waiting_ack:
            self._set_door_toggle_enabled(False)

    def _cancel_device_validation(self) -> None:
        if self._device_validation_after_id is not None:
            try:
                self.after_cancel(self._device_validation_after_id)
            except Exception:
                pass
        self._device_validation_after_id = None
        self._device_validation_pending = False
        self._device_seen_since_connect = False

    def _mark_device_seen(self) -> None:
        if not self._device_validation_pending:
            return

        self._device_seen_since_connect = True
        if self._device_validation_after_id is not None:
            try:
                self.after_cancel(self._device_validation_after_id)
            except Exception:
                pass
        self._device_validation_after_id = None
        self._device_validation_pending = False

    def _start_device_validation(self) -> None:
        self._cancel_device_validation()
        self._device_validation_pending = True
        self._device_seen_since_connect = False
        self._device_validation_after_id = self.after(4000, self._on_device_validation_timeout)

    def _on_device_validation_timeout(self) -> None:
        self._device_validation_after_id = None
        if not self.connected or not self._device_validation_pending:
            self._device_validation_pending = False
            return

        if self._device_seen_since_connect:
            self._device_validation_pending = False
            self._device_seen_since_connect = False
            return

        self._device_validation_pending = False
        self._device_seen_since_connect = False
        invalid_device_id = self.settings.device_id.strip() or "<empty>"
        self._disconnect_mqtt()
        self.status_var.set(f"Invalid Device ID: {invalid_device_id}")
        self._show_error_popup(
            "Invalid Device ID",
            f"Device ID '{invalid_device_id}' was not found on the broker.\nCheck Settings and try again.",
        )

    def _show_error_popup(self, title: str, message: str) -> None:
        dlg = ctk.CTkToplevel(self)
        dlg.title(title)
        dlg.transient(self)
        dlg.geometry("560x220")
        dlg.grid_columnconfigure(0, weight=1)
        dlg.grab_set()

        ctk.CTkLabel(
            dlg,
            text=title,
            text_color="#fca5a5",
            font=ctk.CTkFont(size=18, weight="bold"),
            anchor="w",
        ).grid(row=0, column=0, padx=16, pady=(16, 8), sticky="w")

        ctk.CTkLabel(
            dlg,
            text=message,
            justify="left",
            anchor="w",
            wraplength=520,
        ).grid(row=1, column=0, padx=16, pady=(0, 16), sticky="w")

        ctk.CTkButton(dlg, text="Close", command=dlg.destroy).grid(
            row=2, column=0, padx=16, pady=(0, 16), sticky="e"
        )
    def _load_settings(self) -> MqttSettings:
        if not SETTINGS_PATH.exists():
            return MqttSettings()
        try:
            payload = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        except Exception:
            return MqttSettings()

        defaults = asdict(MqttSettings())
        defaults.update(payload if isinstance(payload, dict) else {})
        return MqttSettings(**defaults)

    def _save_settings(self) -> None:
        SETTINGS_PATH.write_text(json.dumps(asdict(self.settings), indent=2), encoding="utf-8")

    def _refresh_topics(self) -> None:
        base = self.settings.base_topic()
        self.topic_state = f"{base}/state"
        self.topic_ack = f"{base}/ack"
        self.topic_cmd = f"{base}/cmd"
        self.topic_availability = f"{base}/availability"

    def _toggle_connection(self) -> None:
        if self.connected or self.connecting:
            self._disconnect_mqtt()
        else:
            self._connect_mqtt()

    def _create_client(self) -> mqtt.Client:
        client_id = f"esp32ppc-gui-{uuid.uuid4().hex[:8]}"
        if hasattr(mqtt, "CallbackAPIVersion"):
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id=client_id,
                protocol=mqtt.MQTTv311,
            )
        else:
            client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)

        if self.settings.username:
            client.username_pw_set(self.settings.username, self.settings.password)

        if self.settings.use_tls:
            ca_cert = self.settings.ca_cert.strip() or None
            certfile = self.settings.client_cert.strip() or None
            keyfile = self.settings.client_key.strip() or None
            client.tls_set(ca_certs=ca_cert, certfile=certfile, keyfile=keyfile)
            client.tls_insecure_set(self.settings.tls_insecure)

        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        return client

    def _connect_mqtt(self) -> None:
        self._cancel_device_validation()
        self._refresh_topics()
        self.status_var.set("Connecting...")
        self.connect_button.configure(text="Disconnect")
        self.connecting = True

        try:
            self.client = self._create_client()
            self.client.connect_async(
                self.settings.host,
                int(self.settings.port),
                keepalive=int(self.settings.keepalive),
            )
            self.client.loop_start()
        except Exception as exc:
            self.connected = False
            self.connecting = False
            self.connect_button.configure(text="Connect")
            self._set_command_controls_enabled(False)
            self._cancel_device_validation()
            self.status_var.set(f"Connect failed: {exc}")
    def _disconnect_mqtt(self) -> None:
        self.connected = False
        self.connecting = False
        if self.client is not None:
            try:
                self.client.disconnect()
            except Exception:
                pass
            try:
                self.client.loop_stop()
            except Exception:
                pass
            self.client = None
        self._door_toggle_waiting_ack = False
        self._cancel_device_validation()
        self._set_command_controls_enabled(False)
        self._set_door_toggle_enabled(True)
        self.connect_button.configure(text="Connect")
        self.status_var.set("Disconnected")

    @staticmethod
    def _reason_code_to_int(reason_code: Any) -> int:
        try:
            return int(reason_code)
        except Exception:
            value = getattr(reason_code, "value", None)
            if isinstance(value, int):
                return value
            return -1

    def _on_connect(
        self,
        client: mqtt.Client,
        userdata: Any,
        flags: Any,
        reason_code: Any,
        properties: Any = None,
    ) -> None:
        code = self._reason_code_to_int(reason_code)
        self.ui_queue.put(("connect", code))
        if code == 0:
            client.subscribe(self.topic_state, qos=1)
            client.subscribe(self.topic_ack, qos=1)
            client.subscribe(self.topic_availability, qos=1)

    def _on_disconnect(
        self,
        client: mqtt.Client,
        userdata: Any,
        flags: Any = None,
        reason_code: Any = None,
        properties: Any = None,
    ) -> None:
        code = self._reason_code_to_int(reason_code)
        self.ui_queue.put(("disconnect", code))

    def _on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        payload = msg.payload.decode("utf-8", errors="replace")
        self.ui_queue.put(("message", {"topic": msg.topic, "payload": payload}))

    def _drain_ui_queue(self) -> None:
        while True:
            try:
                event, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break

            if event == "connect":
                if payload == 0:
                    self.connected = True
                    self.connecting = False
                    self.connect_button.configure(text="Disconnect")
                    self._set_command_controls_enabled(True)
                    self.status_var.set(f"Connected to {self.settings.host}:{self.settings.port}")
                    self._start_device_validation()
                else:
                    self.connected = False
                    self.connecting = False
                    self.connect_button.configure(text="Connect")
                    self._set_command_controls_enabled(False)
                    self.status_var.set(f"Connection refused (code {payload})")
                    self._cancel_device_validation()

            elif event == "disconnect":
                self.connected = False
                self.connecting = False
                self.connect_button.configure(text="Connect")
                self._set_command_controls_enabled(False)
                self._cancel_device_validation()
                if payload in (0, -1):
                    self.status_var.set("Disconnected")
                else:
                    self.status_var.set(f"Disconnected (code {payload})")

            elif event == "message":
                self._handle_message(payload["topic"], payload["payload"])

        self.after(100, self._drain_ui_queue)

    def _handle_message(self, topic: str, payload: str) -> None:
        if topic == self.topic_ack:
            self._mark_device_seen()
            self.last_ack_var.set(payload)
            self._handle_ack(payload)
            return

        if topic == self.topic_state:
            self._mark_device_seen()
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                return
            self._update_state(data)
            return

        if topic == self.topic_availability:
            self._mark_device_seen()
            self.status_var.set(f"Availability: {payload}")

    def _handle_ack(self, payload: str) -> None:
        try:
            ack = json.loads(payload)
        except json.JSONDecodeError:
            return

        request_id = str(ack.get("request_id", ""))
        if not request_id:
            return

        pending = self.pending_requests.pop(request_id, None)
        if pending is None:
            return

        ok = bool(ack.get("ok"))
        kind = str(pending.get("kind", ""))
        previous = bool(pending.get("previous", False))

        if kind == "door_enabled":
            desired = bool(pending.get("desired", previous))
            self._door_toggle_waiting_ack = False
            self._set_door_toggle_enabled(True)
            self._suspend_switch_callbacks = True
            try:
                self.door_enabled_var.set(desired if ok else previous)
            finally:
                self._suspend_switch_callbacks = False
            return

        if ok:
            return

        self._suspend_switch_callbacks = True
        try:
            if kind == "clamped":
                self.clamped_var.set(previous)
            elif kind == "invert_direction":
                self.invert_direction_var.set(previous)
            elif kind == "adaptive_enabled":
                self.adaptive_enabled_var.set(previous)
            elif kind == "debug_serial":
                self.debug_serial_var.set(previous)
        finally:
            self._suspend_switch_callbacks = False
    def _update_state(self, state: dict[str, Any]) -> None:
        people = state.get("people_count", 0)
        presence = bool(state.get("presence", False))
        today = state.get("today", {})
        entry = today.get("entry", 0)
        exit_value = today.get("exit", 0)

        self.people_count_var.set(str(people))
        self.entry_count_var.set(str(entry))
        self.exit_count_var.set(str(exit_value))

        color = PRESENCE_GREEN if presence else PRESENCE_RED
        self.presence_canvas.itemconfig(self.presence_circle, fill=color)

        cfg = state.get("config")
        if isinstance(cfg, dict):
            self._apply_state_config(cfg)

    def _apply_state_config(self, cfg: dict[str, Any]) -> None:
        self._set_var("sampling", cfg.get("sampling"))
        self._set_var("threshold_min_percent", cfg.get("threshold_min_percent"))
        self._set_var("threshold_max_percent", cfg.get("threshold_max_percent"))
        self._set_var("path_tracking_timeout_ms", cfg.get("path_tracking_timeout_ms"))
        self._set_var("event_cooldown_ms", cfg.get("event_cooldown_ms"))
        self._set_var("peak_time_delta_ms", cfg.get("peak_time_delta_ms"))

        adaptive = cfg.get("adaptive")
        debug = cfg.get("debug")
        door = cfg.get("door_protection")

        self._suspend_switch_callbacks = True
        try:
            if isinstance(cfg.get("clamped_mode"), bool):
                self.clamped_var.set(bool(cfg["clamped_mode"]))
            if isinstance(cfg.get("invert_direction"), bool):
                self.invert_direction_var.set(bool(cfg["invert_direction"]))

            if isinstance(adaptive, dict):
                if isinstance(adaptive.get("enabled"), bool):
                    self.adaptive_enabled_var.set(bool(adaptive["enabled"]))
                self._set_var("adaptive_interval_ms", adaptive.get("interval_ms"))
                self._set_var("adaptive_alpha", adaptive.get("alpha"))

            if isinstance(debug, dict):
                if isinstance(debug.get("serial_enabled"), bool):
                    self.debug_serial_var.set(bool(debug["serial_enabled"]))
                self._set_var("debug_interval_ms", debug.get("sample_interval_ms"))

            if isinstance(door, dict):
                if isinstance(door.get("enabled"), bool) and not self._door_toggle_waiting_ack:
                    self.door_enabled_var.set(bool(door["enabled"]))
                if isinstance(door.get("mm"), int):
                    self.door_mm_var.set(str(door["mm"]))
        finally:
            self._suspend_switch_callbacks = False

    def _set_var(self, key: str, value: Any) -> None:
        var = self.config_vars.get(key)
        if isinstance(var, ctk.StringVar) and value is not None:
            var.set(str(value))

    def _set_door_toggle_enabled(self, enabled: bool) -> None:
        if self.door_enabled_switch is not None:
            self.door_enabled_switch.configure(state="normal" if enabled else "disabled")

    def _next_request_id(self) -> str:
        return uuid.uuid4().hex[:10]
    def _send_command(self, cmd: str, body: dict[str, Any], pending: dict[str, Any] | None = None) -> bool:
        if not self.connected or self.client is None:
            messagebox.showerror("Not connected", "Connect to the MQTT broker first.")
            return False

        request_id = self._next_request_id()
        payload = {"request_id": request_id, "cmd": cmd}
        payload.update(body)
        payload_text = json.dumps(payload, separators=(",", ":"))

        result = self.client.publish(self.topic_cmd, payload_text, qos=1, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            messagebox.showerror("Publish failed", f"MQTT publish failed with rc={result.rc}.")
            return False

        self.last_command_var.set(payload_text)
        if pending:
            self.pending_requests[request_id] = pending
        return True

    def _send_set_config(self, cfg_payload: dict[str, Any], pending: dict[str, Any] | None = None) -> bool:
        return self._send_command("set_config", cfg_payload, pending)

    def _cmd_recalibrate(self) -> None:
        self._send_command("recalibrate", {}, pending={"kind": "recalibrate"})

    def _cmd_get_config(self) -> None:
        self._send_command("get_config", {}, pending={"kind": "get_config"})

    def _cmd_restart(self) -> None:
        if messagebox.askyesno("Restart device", "Send restart command to device?"):
            self._send_command("restart", {}, pending={"kind": "restart"})

    def _cmd_factory_reset(self) -> None:
        dlg = ctk.CTkToplevel(self)
        dlg.title("Factory Reset")
        dlg.transient(self)
        dlg.geometry("560x250")
        dlg.grid_columnconfigure(0, weight=1)
        dlg.grab_set()

        ctk.CTkLabel(
            dlg,
            text="Warning: You are about to perform a factory reset.",
            text_color="#fca5a5",
            font=ctk.CTkFont(size=16, weight="bold"),
        ).grid(row=0, column=0, padx=16, pady=(16, 10), sticky="w")

        ctk.CTkLabel(
            dlg,
            text="Type RESET in capital letters to continue:",
            anchor="w",
        ).grid(row=1, column=0, padx=16, pady=(0, 8), sticky="w")

        token_var = ctk.StringVar(value="")
        token_entry = ctk.CTkEntry(dlg, textvariable=token_var)
        token_entry.grid(row=2, column=0, padx=16, pady=(0, 14), sticky="ew")
        token_entry.focus_set()

        button_row = ctk.CTkFrame(dlg, fg_color="transparent")
        button_row.grid(row=3, column=0, padx=16, pady=(0, 16), sticky="ew")
        button_row.grid_columnconfigure((0, 1), weight=1)

        def on_cancel() -> None:
            dlg.destroy()

        def on_reset() -> None:
            if token_var.get().strip() != "RESET":
                messagebox.showwarning(
                    "Invalid confirmation",
                    "Confirmation text must be exactly RESET (capital letters).",
                    parent=dlg,
                )
                return

            sent = self._send_command(
                "factory_reset",
                {"confirm": "FACTORY_RESET"},
                pending={"kind": "factory_reset"},
            )
            if sent:
                dlg.destroy()

        ctk.CTkButton(
            button_row,
            text="Cancel",
            command=on_cancel,
        ).grid(row=0, column=0, padx=(0, 8), sticky="ew")

        ctk.CTkButton(
            button_row,
            text="Reset",
            fg_color="#7f1d1d",
            hover_color="#991b1b",
            command=on_reset,
        ).grid(row=0, column=1, padx=(8, 0), sticky="ew")


    def _cmd_set_people_counter(self) -> None:
        try:
            value = int(self.people_counter_input.get().strip())
        except ValueError:
            messagebox.showerror("Invalid value", "People counter must be an integer.")
            return

        self._send_command(
            "set_people_counter",
            {"value": value},
            pending={"kind": "set_people_counter", "value": value},
        )

    def _cmd_set_clamped_mode(self) -> None:
        if self._suspend_switch_callbacks:
            return
        new_value = bool(self.clamped_var.get())
        previous = not new_value
        sent = self._send_command(
            "set_clamped",
            {"value": new_value},
            pending={"kind": "clamped", "value": new_value, "previous": previous},
        )
        if not sent:
            self._suspend_switch_callbacks = True
            self.clamped_var.set(previous)
            self._suspend_switch_callbacks = False

    def _cmd_set_invert_direction(self) -> None:
        if self._suspend_switch_callbacks:
            return
        new_value = bool(self.invert_direction_var.get())
        previous = not new_value
        sent = self._send_set_config(
            {"invert_direction": new_value},
            pending={"kind": "invert_direction", "previous": previous},
        )
        if not sent:
            self._suspend_switch_callbacks = True
            self.invert_direction_var.set(previous)
            self._suspend_switch_callbacks = False

    def _cmd_set_adaptive_enabled(self) -> None:
        if self._suspend_switch_callbacks:
            return
        new_value = bool(self.adaptive_enabled_var.get())
        previous = not new_value
        sent = self._send_set_config(
            {"adaptive": {"enabled": new_value}},
            pending={"kind": "adaptive_enabled", "previous": previous},
        )
        if not sent:
            self._suspend_switch_callbacks = True
            self.adaptive_enabled_var.set(previous)
            self._suspend_switch_callbacks = False

    def _cmd_set_debug_serial(self) -> None:
        if self._suspend_switch_callbacks:
            return
        new_value = bool(self.debug_serial_var.get())
        previous = not new_value
        sent = self._send_set_config(
            {"debug_serial": new_value},
            pending={"kind": "debug_serial", "previous": previous},
        )
        if not sent:
            self._suspend_switch_callbacks = True
            self.debug_serial_var.set(previous)
            self._suspend_switch_callbacks = False

    def _cmd_set_door_enabled(self) -> None:
        if self._suspend_switch_callbacks or self._door_toggle_waiting_ack:
            return

        desired = bool(self.door_enabled_var.get())
        previous = not desired

        self._suspend_switch_callbacks = True
        try:
            self.door_enabled_var.set(previous)
        finally:
            self._suspend_switch_callbacks = False

        self._door_toggle_waiting_ack = True
        self._set_door_toggle_enabled(False)

        sent = self._send_command(
            "set_door_protection",
            {"enabled": desired},
            pending={"kind": "door_enabled", "previous": previous, "desired": desired},
        )

        if not sent:
            self._door_toggle_waiting_ack = False
            self._set_door_toggle_enabled(True)
            self._suspend_switch_callbacks = True
            try:
                self.door_enabled_var.set(previous)
            finally:
                self._suspend_switch_callbacks = False
    def _cmd_set_door_protection(self) -> None:
        try:
            mm = int(self.door_mm_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid value", "Door protection mm must be an integer.")
            return

        if mm < 0 or mm > 4000:
            messagebox.showerror("Out of range", "Door protection mm must be in range 0..4000.")
            return

        self._send_command(
            "set_door_protection",
            {"enabled": bool(self.door_enabled_var.get()), "mm": mm},
            pending={"kind": "set_door_protection"},
        )

    def _parse_int(self, key: str, minimum: int, maximum: int) -> int:
        var = self.config_vars.get(key)
        if not isinstance(var, ctk.StringVar):
            raise ValueError(f"Missing field: {key}")

        value = int(var.get().strip())
        if value < minimum or value > maximum:
            raise ValueError(f"{key} must be in range {minimum}..{maximum}")
        return value

    def _parse_float(self, key: str, minimum: float, maximum: float, min_exclusive: bool = False) -> float:
        var = self.config_vars.get(key)
        if not isinstance(var, ctk.StringVar):
            raise ValueError(f"Missing field: {key}")

        value = float(var.get().strip())
        if min_exclusive:
            if value <= minimum or value > maximum:
                raise ValueError(f"{key} must be in range ({minimum}, {maximum}]")
        elif value < minimum or value > maximum:
            raise ValueError(f"{key} must be in range {minimum}..{maximum}")
        return value
    def _cmd_set_config(self) -> None:
        try:
            threshold_min = self._parse_int("threshold_min_percent", 0, 100)
            threshold_max = self._parse_int("threshold_max_percent", 0, 100)
            if threshold_max <= threshold_min:
                raise ValueError("threshold_max_percent must be greater than threshold_min_percent")

            door_mm = int(self.door_mm_var.get().strip())
            if door_mm < 0 or door_mm > 4000:
                raise ValueError("door_protection.mm must be in range 0..4000")

            payload = {
                "invert_direction": bool(self.invert_direction_var.get()),
                "sampling": self._parse_int("sampling", 1, 16),
                "threshold_min_percent": threshold_min,
                "threshold_max_percent": threshold_max,
                "path_tracking_timeout_ms": self._parse_int("path_tracking_timeout_ms", 0, 60000),
                "event_cooldown_ms": self._parse_int("event_cooldown_ms", 0, 10000),
                "peak_time_delta_ms": self._parse_int("peak_time_delta_ms", 0, 2000),
                "adaptive": {
                    "enabled": bool(self.adaptive_enabled_var.get()),
                    "interval_ms": self._parse_int("adaptive_interval_ms", 1000, 3600000),
                    "alpha": self._parse_float("adaptive_alpha", 0.0, 1.0, min_exclusive=True),
                },
                "debug_serial": bool(self.debug_serial_var.get()),
                "debug_interval_ms": self._parse_int("debug_interval_ms", 20, 10000),
                "door_protection_enabled": bool(self.door_enabled_var.get()),
                "door_protection_mm": door_mm,
                "door_protection": {
                    "enabled": bool(self.door_enabled_var.get()),
                    "mm": door_mm,
                },
            }
        except ValueError as exc:
            messagebox.showerror("Invalid config", str(exc))
            return

        self._send_set_config(payload, pending={"kind": "set_config"})
    def _open_settings_dialog(self) -> None:
        if self.settings_dialog is not None and self.settings_dialog.winfo_exists():
            self.settings_dialog.focus()
            return

        dlg = ctk.CTkToplevel(self)
        dlg.title("MQTT Settings")
        dlg.transient(self)
        dlg.geometry("760x620")
        dlg.grid_columnconfigure(1, weight=1)
        self.settings_dialog = dlg

        vars_map: dict[str, ctk.StringVar | ctk.BooleanVar] = {
            "host": ctk.StringVar(value=self.settings.host),
            "port": ctk.StringVar(value=str(self.settings.port)),
            "username": ctk.StringVar(value=self.settings.username),
            "password": ctk.StringVar(value=self.settings.password),
            "use_tls": ctk.BooleanVar(value=self.settings.use_tls),
            "tls_insecure": ctk.BooleanVar(value=self.settings.tls_insecure),
            "ca_cert": ctk.StringVar(value=self.settings.ca_cert),
            "client_cert": ctk.StringVar(value=self.settings.client_cert),
            "client_key": ctk.StringVar(value=self.settings.client_key),
            "keepalive": ctk.StringVar(value=str(self.settings.keepalive)),
            "base_prefix": ctk.StringVar(value=self.settings.base_prefix),
            "customer_id": ctk.StringVar(value=self.settings.customer_id),
            "device_id": ctk.StringVar(value=self.settings.device_id),
        }

        row = 0
        fields = [
            ("host", "Broker Host:"),
            ("port", "Broker Port:"),
            ("username", "Username:"),
            ("password", "Password:"),
            ("keepalive", "Keepalive (s):"),
            ("base_prefix", "Topic Prefix:"),
            ("customer_id", "Customer ID:"),
            ("device_id", "Device ID:"),
            ("ca_cert", "CA Cert Path:"),
            ("client_cert", "Client Cert Path:"),
            ("client_key", "Client Key Path:"),
        ]

        for key, label in fields:
            ctk.CTkLabel(dlg, text=label, width=140, anchor="e").grid(
                row=row, column=0, padx=(16, 10), pady=7, sticky="e"
            )
            show = "*" if key == "password" else None
            ctk.CTkEntry(dlg, textvariable=vars_map[key], show=show).grid(
                row=row, column=1, padx=(0, 16), pady=7, sticky="ew"
            )
            row += 1

        ctk.CTkLabel(dlg, text="Use TLS:", width=140, anchor="e").grid(
            row=row, column=0, padx=(16, 10), pady=7, sticky="e"
        )
        ctk.CTkSwitch(dlg, text="", variable=vars_map["use_tls"]).grid(
            row=row, column=1, padx=(0, 16), pady=7, sticky="w"
        )
        row += 1

        ctk.CTkLabel(dlg, text="TLS Insecure:", width=140, anchor="e").grid(
            row=row, column=0, padx=(16, 10), pady=7, sticky="e"
        )
        ctk.CTkSwitch(dlg, text="", variable=vars_map["tls_insecure"]).grid(
            row=row, column=1, padx=(0, 16), pady=7, sticky="w"
        )
        row += 1

        preview = ctk.StringVar(value=f"Base topic: {self.settings.base_topic()}")
        ctk.CTkLabel(dlg, textvariable=preview, text_color=TEXT_MUTED).grid(
            row=row, column=0, columnspan=2, padx=16, pady=(8, 12), sticky="w"
        )
        row += 1

        def refresh_preview(*_: Any) -> None:
            base_prefix = str(vars_map["base_prefix"].get()).strip() or "ppc/v1"
            customer_id = str(vars_map["customer_id"].get()).strip() or "customer-demo"
            device_id = str(vars_map["device_id"].get()).strip() or "esp32ppc-c5-01"
            preview.set(f"Base topic: {base_prefix}/c/{customer_id}/d/{device_id}")

        for key in ("base_prefix", "customer_id", "device_id"):
            var = vars_map[key]
            if isinstance(var, ctk.StringVar):
                var.trace_add("write", refresh_preview)

        buttons = ctk.CTkFrame(dlg, fg_color="transparent")
        buttons.grid(row=row, column=0, columnspan=2, padx=16, pady=(0, 16), sticky="ew")
        buttons.grid_columnconfigure((0, 1), weight=1)

        def on_save() -> None:
            try:
                host = str(vars_map["host"].get()).strip()
                if not host:
                    raise ValueError("Host is required.")
                port = int(str(vars_map["port"].get()).strip())
                if port < 1 or port > 65535:
                    raise ValueError("Port must be in range 1..65535.")
                keepalive = int(str(vars_map["keepalive"].get()).strip())
                if keepalive < 5 or keepalive > 3600:
                    raise ValueError("Keepalive must be in range 5..3600.")
            except ValueError as exc:
                messagebox.showerror("Invalid MQTT settings", str(exc), parent=dlg)
                return

            self.settings = MqttSettings(
                host=host,
                port=port,
                username=str(vars_map["username"].get()),
                password=str(vars_map["password"].get()),
                use_tls=bool(vars_map["use_tls"].get()),
                tls_insecure=bool(vars_map["tls_insecure"].get()),
                ca_cert=str(vars_map["ca_cert"].get()),
                client_cert=str(vars_map["client_cert"].get()),
                client_key=str(vars_map["client_key"].get()),
                keepalive=keepalive,
                base_prefix=str(vars_map["base_prefix"].get()).strip() or "ppc/v1",
                customer_id=str(vars_map["customer_id"].get()).strip() or "customer-demo",
                device_id=str(vars_map["device_id"].get()).strip() or "esp32ppc-c5-01",
            )
            self._save_settings()
            self._refresh_topics()

            if self.connected or self.connecting:
                self._disconnect_mqtt()
                self._connect_mqtt()

            self.settings_dialog = None
            dlg.destroy()

        def on_close_dialog() -> None:
            self.settings_dialog = None
            dlg.destroy()

        ctk.CTkButton(buttons, text="Save", command=on_save).grid(
            row=0, column=0, padx=(0, 8), pady=0, sticky="ew"
        )
        ctk.CTkButton(
            buttons,
            text="Cancel",
            fg_color="#3f3f46",
            hover_color="#52525b",
            command=on_close_dialog,
        ).grid(row=0, column=1, padx=(8, 0), pady=0, sticky="ew")

        dlg.protocol("WM_DELETE_WINDOW", on_close_dialog)

    def _on_close(self) -> None:
        self._disconnect_mqtt()
        self.destroy()


def main() -> None:
    app = Esp32PpcGui()
    app.mainloop()


if __name__ == "__main__":
    main()















