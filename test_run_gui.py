import csv
import datetime as dt
import json
import math
import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    import pysoem
except ImportError:
    pysoem = None


BAUD_RATE = 230400
SERIAL_WRITE_TIMEOUT = 1.0
SERIAL_COMMAND_SPACING_SECONDS = 0.08
MAX_LOG_LINES = 400
MAX_QUEUE_DRAIN_PER_TICK = 250
FLOW_DELAY_CAPTURE_MS = 500.0
SAMPLE_INTERVAL_MS = 5.0
VALVE_PULSE_DURATION_MS = 100.0
PRESSURE_SETTLE_SKIP_SAMPLES = 2
REGULATOR_MAX_PRESSURE_BAR = 6.0
TEST_PRESSURE_STEP_BAR = 0.1
MOTOR_MM_PER_STEP = 0.009985846
MOTOR_STEPS_PER_MM = 1.0 / MOTOR_MM_PER_STEP
MAX_MOTOR_STEPS_PER_SECOND = 5000
COLIBRI_BAUD_RATE = 9600
COLIBRI_SLAVE_ADDRESS = 0xFF
COLIBRI_MM_PER_STEP = 0.005
COLIBRI_STEPS_PER_MM = 1.0 / COLIBRI_MM_PER_STEP
COLIBRI_TRAVEL_MM = 75.0
COLIBRI_REFERENCE_CURRENT_PERCENT = 20
FORCE_BAUD_RATE = 38400
FORCE_READ_TIMEOUT_SECONDS = 0.005
FORCE_RATE_WINDOW_SECONDS = 2.0
FORCE_BINARY_SYNC = 0x2C
FORCE_BINARY_FRAME_LENGTH = 5
FORCE_BINARY_STATUS_MASK = 0x18
FORCE_BINARY_BIPOLAR_ZERO = 0x800000
FORCE_BINARY_POSITIVE_SPAN = 0x7FFFFF
FORCE_BINARY_FULL_SCALE_FACTOR = 1.05
FORCE_DEFAULT_SCALING = 14.3758
FORCE_DEFAULT_AVERAGE_SECONDS = 0.0
FORCE_DEFAULT_IMPULSE_THRESHOLD = 0.1
GSV_CMD_GET_VALUE = 0x3B
FORCE_LOGGER_POLL_INTERVAL_SECONDS = 0.005


class ColibriProtocolError(Exception):
    pass


class ColibriController:
    START_BLOCK = 0x04
    END_BLOCK = 0x05
    SHIFT = 0x06

    TG_REQ_STATUS = 0x01
    TG_REQ_ERROR = 0x02
    TG_REQ_POSITION = 0x03
    TG_REQ_PARAM = 0x06
    TG_MOVE_REL = 0x15
    TG_MOTOR = 0x16
    TG_MOVE_ABS = 0x1A
    TG_SET_PARAM = 0x1F

    TG_STATUS = 0x80
    TG_ERROR = 0x81
    TG_PARAM = 0x83
    TG_POSITION = 0x84
    TG_MOVING = 0x85

    MOTOR_ESTOP = 0
    MOTOR_STOP = 1
    MOTOR_REF = 2
    MOTOR_REMOTE = 5
    MOTOR_SET_REFERENCE_POINT = 8
    MOTOR_EXIT_REMOTE = 10
    MOTOR_DISABLE = 11
    MOTOR_ENABLE = 12

    def __init__(self, port, baudrate=COLIBRI_BAUD_RATE, slave_address=COLIBRI_SLAVE_ADDRESS, debug_logger=None):
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.port_name = port
        self.slave_address = slave_address
        self.debug_logger = debug_logger
        self.serial_port = serial.Serial(
            port,
            baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.08,
            write_timeout=SERIAL_WRITE_TIMEOUT,
        )
        self.lock = threading.Lock()

    def set_debug_logger(self, debug_logger):
        self.debug_logger = debug_logger

    def close(self):
        self.serial_port.close()

    def status(self):
        response = self._request(self.TG_REQ_STATUS, expected_type=self.TG_STATUS)
        if len(response) < 6:
            raise ColibriProtocolError(f"Short status response: {response.hex(' ')}")
        return self._decode_status(response[3], response[4], response[5])

    def position_steps(self):
        response = self._request(self.TG_REQ_POSITION, expected_type=self.TG_POSITION)
        if len(response) < 7:
            raise ColibriProtocolError(f"Short position response: {response.hex(' ')}")
        return int.from_bytes(response[3:7], byteorder="little", signed=True)

    def error(self):
        response = self._request(self.TG_REQ_ERROR, expected_type=self.TG_ERROR)
        if len(response) < 5:
            raise ColibriProtocolError(f"Short error response: {response.hex(' ')}")
        return {
            "last_error": response[3],
            "details": response[4],
        }

    def parameter(self, index, subindex):
        response = self._request(self.TG_REQ_PARAM, index, subindex, expected_type=self.TG_PARAM)
        if len(response) < 6:
            raise ColibriProtocolError(f"Short parameter response: {response.hex(' ')}")
        if response[3] != index or response[4] != subindex:
            raise ColibriProtocolError(f"Unexpected parameter response: {response.hex(' ')}")
        return int.from_bytes(response[5:-1], byteorder="little", signed=False)

    def set_parameter(self, index, subindex, value, byte_count):
        value_bytes = int(value).to_bytes(byte_count, byteorder="little", signed=False)
        return self._request(
            self.TG_SET_PARAM,
            index,
            subindex,
            *value_bytes,
            expected_type=None,
            retry_on_timeout=False,
        )

    def set_remote(self):
        return self.motor_command(self.MOTOR_REMOTE)

    def enable(self):
        return self.motor_command(self.MOTOR_ENABLE)

    def disable(self):
        return self.motor_command(self.MOTOR_DISABLE)

    def stop(self):
        return self.motor_command(self.MOTOR_STOP)

    def emergency_stop(self):
        return self.motor_command(self.MOTOR_ESTOP)

    def reference(self):
        return self.motor_command(self.MOTOR_REF)

    def set_current_position_as_reference(self):
        return self.motor_command(self.MOTOR_SET_REFERENCE_POINT)

    def configure_negative_reference(self):
        self.set_parameter(4, 1, 2, 1)
        self.set_parameter(5, 2, COLIBRI_REFERENCE_CURRENT_PERCENT, 1)

    def motor_command(self, command):
        return self._request(self.TG_MOTOR, command, expected_type=None, retry_on_timeout=False)

    def move_relative_steps(self, steps):
        return self._request(self.TG_MOVE_REL, *self._int32_bytes(steps), expected_type=None, retry_on_timeout=False)

    def move_absolute_steps(self, steps):
        return self._request(self.TG_MOVE_ABS, *self._int32_bytes(steps), expected_type=None, retry_on_timeout=False)

    def _request(self, *data, expected_type, retry_on_timeout=True):
        with self.lock:
            last_response = None
            last_error = None
            request_frame = self._build_frame(data)
            attempts = 2 if retry_on_timeout else 1
            for attempt in range(attempts):
                self.serial_port.reset_input_buffer()
                self._trace(f"TX {request_frame.hex(' ')}")
                self.serial_port.write(request_frame)
                self.serial_port.flush()
                deadline = time.time() + 0.8
                while time.time() < deadline:
                    response = self._read_frame(deadline)
                    if response is None:
                        continue
                    self._trace(f"RX {response.hex(' ')}")
                    try:
                        self._validate_response(response)
                    except ColibriProtocolError as exc:
                        last_error = exc
                        self._trace(f"RX ignored {exc}")
                        continue
                    last_response = response
                    if len(response) >= 3 and response[2] == self.TG_ERROR and expected_type != self.TG_ERROR:
                        continue
                    if expected_type is None or (len(response) >= 3 and response[2] == expected_type):
                        return response
                if attempt + 1 < attempts:
                    time.sleep(0.05)
            if last_response is not None:
                raise ColibriProtocolError(f"Unexpected response: {last_response.hex(' ')}")
            if last_error is not None:
                raise last_error
            raise TimeoutError("No response from Colibri")

    def _build_frame(self, data):
        payload = bytes([self.slave_address, len(data) + 1, *data])
        checksum = sum(payload) & 0xFF
        return bytes([self.START_BLOCK]) + self._escape(payload + bytes([checksum])) + bytes([self.END_BLOCK])

    def _escape(self, data):
        escaped = bytearray()
        for value in data:
            if value in (self.START_BLOCK, self.END_BLOCK, self.SHIFT):
                escaped.extend([self.SHIFT, (value + self.SHIFT) & 0xFF])
            else:
                escaped.append(value)
        return bytes(escaped)

    def _read_frame(self, deadline):
        frame = bytearray()
        in_frame = False
        shifted = False
        while time.time() < deadline:
            byte = self.serial_port.read(1)
            if not byte:
                continue
            value = byte[0]
            if value == self.START_BLOCK:
                frame.clear()
                in_frame = True
                shifted = False
                continue
            if value == self.END_BLOCK and in_frame:
                return bytes(frame)
            if not in_frame:
                continue
            if shifted:
                frame.append((value - self.SHIFT) & 0xFF)
                shifted = False
            elif value == self.SHIFT:
                shifted = True
            else:
                frame.append(value)
        return None

    def _validate_response(self, response):
        if len(response) < 4:
            raise ColibriProtocolError(f"Frame too short: {response.hex(' ')}")
        checksum = sum(response[:-1]) & 0xFF
        if checksum != response[-1]:
            raise ColibriProtocolError(
                f"Bad checksum: expected {checksum:02x}, got {response[-1]:02x}"
            )

    def _decode_status(self, status_byte, system_status_byte, error_byte):
        return {
            "moving": bool(status_byte & 0x01),
            "software_limit": bool(status_byte & 0x02),
            "ready": bool(status_byte & 0x08),
            "referenced": bool(status_byte & 0x10),
            "remote": bool(status_byte & 0x20),
            "enabled": bool(status_byte & 0x40),
            "password": bool(status_byte & 0x80),
            "system_status_byte": system_status_byte,
            "error_byte": error_byte,
            "error": bool(error_byte & 0x01),
            "watchdog_error": bool(error_byte & 0x02),
            "burnout_error": bool(error_byte & 0x04),
            "eeprom_error": bool(error_byte & 0x08),
            "motor_voltage_error": bool(error_byte & 0x10),
            "temperature_error": bool(error_byte & 0x20),
            "mark_error": bool(error_byte & 0x40),
            "bootloader_error": bool(error_byte & 0x80),
        }

    def _int32_bytes(self, value):
        return int(value).to_bytes(4, byteorder="little", signed=True)

    def _trace(self, message):
        if self.debug_logger:
            self.debug_logger(f"COLIBRI {message}")


class TestRunGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pneumatic Test Run")
        self.geometry("980x640")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.user_presets_path = Path(__file__).with_name("user_presets.json")
        self.user_presets = self._load_user_presets()
        self.serial_port = None
        self.reader_thread = None
        self.writer_thread = None
        self.reader_running = False
        self.writer_running = False
        self.force_serial_port = None
        self.force_reader_thread = None
        self.force_reader_running = False
        self.force_serial_lock = threading.Lock()
        self.force_lock = threading.Lock()
        self.latest_force_raw_n = None
        self.latest_force_n = None
        self.latest_force_time = None
        self.force_scaling = self._preset_float("force_scaling", FORCE_DEFAULT_SCALING, -1.0e12, 1.0e12)
        self.force_average_seconds = self._preset_float(
            "force_average_seconds", FORCE_DEFAULT_AVERAGE_SECONDS, 0.0, 60.0
        )
        self.force_impulse_threshold = self._preset_float(
            "force_impulse_threshold", FORCE_DEFAULT_IMPULSE_THRESHOLD, 0.0, 1.0e12
        )
        self.force_average_samples = deque(maxlen=10000)
        self.force_rate_times = deque(maxlen=2000)
        self.rows = []
        self.impulse_rows = []
        self.current_impulse = None
        self.last_valves_open = False
        self.messages = queue.Queue()
        self.commands = queue.Queue()
        self.port_devices = {}
        self.ethercat_adapters = {}
        self.ethercat_master = None
        self.ethercat_busy = False
        self.closing = False
        self.colibri = None
        self.colibri_busy = False
        self.debug_log_file = None
        self.debug_log_path = None
        self.debug_log_lock = threading.Lock()

        self.port_var = tk.StringVar()
        self.colibri_port_var = tk.StringVar()
        self.force_port_var = tk.StringVar()
        self.ethercat_adapter_var = tk.StringVar()
        self.ethercat_status_var = tk.StringVar(value="EtherCAT: disconnected")
        self.force_baud_var = tk.IntVar(value=FORCE_BAUD_RATE)
        self.force_scale_var = tk.DoubleVar(value=self.force_scaling)
        self.force_average_var = tk.DoubleVar(value=self.force_average_seconds)
        self.force_impulse_threshold_var = tk.DoubleVar(value=self.force_impulse_threshold)
        self.status_var = tk.StringVar(value="Disconnected")
        self.mode_var = tk.StringVar(value="Mode: disconnected")
        self.colibri_status_var = tk.StringVar(value="Colibri: disconnected")
        self.colibri_position_var = tk.StringVar(value="Position: --")
        self.force_status_var = tk.StringVar(value=self._force_status("Force sensor: disconnected"))
        self.force_value_var = tk.StringVar(value="Force reading: --")
        self.force_rate_var = tk.StringVar(value="Force rate: --")
        self.debug_log_var = tk.StringVar(value="Debug log: off")
        self.german_csv_format_var = tk.BooleanVar(value=True)
        self.target_pressure_var = tk.DoubleVar(value=0.50)
        self.starting_pressure_var = tk.DoubleVar(value=0.50)
        self.test_start_pressure_var = tk.DoubleVar(value=0.50)
        self.test_end_pressure_var = tk.DoubleVar(value=0.80)
        self.test_repeats_var = tk.IntVar(value=10)
        self.pressure_increment_var = tk.DoubleVar(value=0.05)
        self.increment_count_var = tk.IntVar(value=0)
        self.flow_threshold_var = tk.DoubleVar(value=2.0)
        self.stream_var = tk.BooleanVar(value=True)
        self.motor_enabled_var = tk.BooleanVar(value=False)
        self.motor_distance_var = tk.DoubleVar(value=10.0)
        self.motor_absolute_var = tk.DoubleVar(value=0.0)
        self.motor_speed_var = tk.DoubleVar(value=5.0)
        self.motor_position_var = tk.StringVar(value="Stepper position: --")
        self.last_motor_position_mm = None
        self.colibri_enabled_var = tk.BooleanVar(value=False)
        self.colibri_distance_var = tk.DoubleVar(value=1.0)
        self.colibri_absolute_var = tk.DoubleVar(value=0.0)
        self.last_colibri_position_mm = None
        self.part_pose_var = tk.StringVar()
        self.part_hole_var = tk.StringVar()
        self.part_csv_status_var = tk.StringVar(value="No part CSV loaded")
        self.use_cap_offsets_var = tk.BooleanVar(value=False)
        self.nozzle_offset_var = tk.DoubleVar(value=self._preset_float("nozzle_offset_mm", 0.0, -2000.0, 2000.0))
        self.test_stand_height_var = tk.DoubleVar(
            value=self._preset_float("test_stand_height_mm", 0.0, -2000.0, 2000.0)
        )
        self.holder_height_var = tk.DoubleVar(
            value=self._preset_float("holder_height_mm", 0.0, -2000.0, 2000.0)
        )
        self.part_y_offset_var = tk.StringVar(value="Y offset: --")
        self.part_z_offset_var = tk.StringVar(value="Z offset: --")
        self.part_stepper_position_var = tk.StringVar(value="Stepper target: --")
        self.part_colibri_position_var = tk.StringVar(value="Colibri target: --")
        self.nozzle_vars = [tk.BooleanVar(value=True) for _ in range(4)]
        self.nozzle_checkbuttons = []
        self.motor_controls = []
        self.colibri_controls = []
        self.last_motor_speed_steps_s = None
        self.pulse_in_progress = False
        self.pending_increment_direction = 0
        self.pending_flip_angle = -1
        self.pending_pulse_mask = ""
        self.pending_pulse_duration_ms = None
        self.active_test_mask = ""
        self.part_rows = {}

        self._build_ui()
        for variable in (
            self.use_cap_offsets_var,
            self.nozzle_offset_var,
            self.test_stand_height_var,
            self.holder_height_var,
        ):
            variable.trace_add("write", self._part_input_changed)
        self._refresh_ports()
        self._refresh_ethercat_adapters()
        self.after(50, self._drain_messages)

    def _load_user_presets(self):
        try:
            with open(self.user_presets_path, encoding="utf-8") as preset_file:
                presets = json.load(preset_file)
        except (OSError, json.JSONDecodeError):
            return {}
        return presets if isinstance(presets, dict) else {}

    def _preset_float(self, key, default, minimum, maximum):
        try:
            value = float(self.user_presets.get(key, default))
        except (TypeError, ValueError):
            value = default
        return min(max(value, minimum), maximum)

    def _on_close(self):
        response = self._ask_save_user_presets()
        if response is None:
            return
        if response and not self._save_user_presets():
            return
        self.destroy()

    def _ask_save_user_presets(self):
        result = {"value": None}
        dialog = tk.Toplevel(self)
        dialog.title("Save User Presets")
        dialog.transient(self)
        dialog.resizable(False, False)

        ttk.Label(
            dialog,
            text="Save the current force settings, nozzle offset, test stand height, and holder height as defaults?",
            wraplength=420,
            justify=tk.LEFT,
        ).pack(padx=18, pady=(16, 12))

        button_frame = ttk.Frame(dialog)
        button_frame.pack(padx=14, pady=(0, 14), anchor=tk.E)

        def choose(value):
            result["value"] = value
            dialog.destroy()

        ttk.Button(button_frame, text="Save", command=lambda: choose(True)).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_frame, text="Don't Save", command=lambda: choose(False)).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_frame, text="Cancel", command=lambda: choose(None)).pack(side=tk.LEFT, padx=4)

        dialog.protocol("WM_DELETE_WINDOW", lambda: choose(None))
        dialog.update_idletasks()
        x = self.winfo_rootx() + max(0, (self.winfo_width() - dialog.winfo_reqwidth()) // 2)
        y = self.winfo_rooty() + max(0, (self.winfo_height() - dialog.winfo_reqheight()) // 2)
        dialog.geometry(f"+{x}+{y}")
        dialog.lift()
        dialog.focus_force()
        dialog.grab_set()
        self.wait_window(dialog)
        return result["value"]

    def _save_user_presets(self):
        try:
            presets = {
                "force_scaling": float(self.force_scale_var.get()),
                "force_average_seconds": float(self.force_average_var.get()),
                "force_impulse_threshold": float(self.force_impulse_threshold_var.get()),
                "nozzle_offset_mm": float(self.nozzle_offset_var.get()),
                "test_stand_height_mm": float(self.test_stand_height_var.get()),
                "holder_height_mm": float(self.holder_height_var.get()),
            }
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror("Preset save failed", f"One of the preset values is not numeric: {exc}")
            return False

        try:
            with open(self.user_presets_path, "w", encoding="utf-8") as preset_file:
                json.dump(presets, preset_file, indent=2)
        except OSError as exc:
            messagebox.showerror("Preset save failed", str(exc))
            return False

        return True

    def _build_ui(self):
        root = ttk.Frame(self, padding=12)
        root.pack(fill=tk.BOTH, expand=True)

        controls = ttk.Frame(root)
        controls.pack(fill=tk.X)

        ttk.Label(controls, text="Port").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(6, 8))

        ttk.Button(controls, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(controls, text="Connect", command=self._toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(8, 0))

        self.start_button = ttk.Button(controls, text="Start test", command=self._start_test, state=tk.DISABLED)
        self.start_button.pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(controls, text="From").pack(side=tk.LEFT, padx=(8, 0))
        self.test_start_pressure_spinbox = ttk.Spinbox(
            controls,
            from_=0.0,
            to=REGULATOR_MAX_PRESSURE_BAR,
            increment=TEST_PRESSURE_STEP_BAR,
            textvariable=self.test_start_pressure_var,
            width=6,
            state=tk.DISABLED,
        )
        self.test_start_pressure_spinbox.pack(side=tk.LEFT, padx=(4, 2))
        ttk.Label(controls, text="to").pack(side=tk.LEFT)
        self.test_end_pressure_spinbox = ttk.Spinbox(
            controls,
            from_=0.0,
            to=REGULATOR_MAX_PRESSURE_BAR,
            increment=TEST_PRESSURE_STEP_BAR,
            textvariable=self.test_end_pressure_var,
            width=6,
            state=tk.DISABLED,
        )
        self.test_end_pressure_spinbox.pack(side=tk.LEFT, padx=(4, 2))
        ttk.Label(controls, text="bar").pack(side=tk.LEFT)
        ttk.Label(controls, text="Repeats").pack(side=tk.LEFT, padx=(8, 0))
        self.test_repeats_spinbox = ttk.Spinbox(
            controls,
            from_=1,
            to=100,
            increment=1,
            textvariable=self.test_repeats_var,
            width=5,
            state=tk.DISABLED,
        )
        self.test_repeats_spinbox.pack(side=tk.LEFT, padx=(4, 2))
        self.stop_button = ttk.Button(controls, text="Stop", command=self._stop_test, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Save CSV", command=self._save_impulse_csv).pack(side=tk.RIGHT, padx=(0, 8))
        ttk.Checkbutton(
            controls,
            text="German CSV format",
            variable=self.german_csv_format_var,
        ).pack(side=tk.RIGHT, padx=(0, 8))
        self.debug_log_button = ttk.Button(controls, text="Start debug log", command=self._toggle_debug_log)
        self.debug_log_button.pack(side=tk.RIGHT, padx=(0, 8))
        ttk.Button(controls, text="Clear", command=self._clear_log).pack(side=tk.RIGHT, padx=(0, 8))

        ethercat_controls = ttk.Frame(root)
        ethercat_controls.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(ethercat_controls, text="EtherCAT (EK1100)").pack(side=tk.LEFT)
        self.ethercat_adapter_combo = ttk.Combobox(
            ethercat_controls,
            textvariable=self.ethercat_adapter_var,
            width=52,
            state="readonly",
        )
        self.ethercat_adapter_combo.pack(side=tk.LEFT, padx=(6, 8))
        self.ethercat_refresh_button = ttk.Button(
            ethercat_controls,
            text="Refresh adapters",
            command=self._refresh_ethercat_adapters,
        )
        self.ethercat_refresh_button.pack(side=tk.LEFT)
        self.ethercat_connect_button = ttk.Button(
            ethercat_controls,
            text="Scan and connect",
            command=self._toggle_ethercat_connection,
        )
        self.ethercat_connect_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(ethercat_controls, textvariable=self.ethercat_status_var).pack(side=tk.LEFT, padx=(12, 0))

        pressure_controls = ttk.Frame(root)
        pressure_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(pressure_controls, text="Target pressure").pack(side=tk.LEFT)
        self.target_pressure_spinbox = ttk.Spinbox(
            pressure_controls,
            from_=0.0,
            to=REGULATOR_MAX_PRESSURE_BAR,
            increment=0.05,
            textvariable=self.target_pressure_var,
            width=8,
            state=tk.DISABLED,
        )
        self.target_pressure_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(pressure_controls, text="bar").pack(side=tk.LEFT)

        self.apply_pressure_button = ttk.Button(
            pressure_controls,
            text="Apply pressure",
            command=self._apply_pressure_settings,
            state=tk.DISABLED,
        )
        self.apply_pressure_button.pack(side=tk.LEFT, padx=(10, 0))

        self.stream_checkbutton = ttk.Checkbutton(
            pressure_controls,
            text="Live stream",
            variable=self.stream_var,
            command=self._apply_stream_setting,
            state=tk.DISABLED,
        )
        self.stream_checkbutton.pack(side=tk.LEFT, padx=(18, 0))

        ttk.Label(pressure_controls, text="Flow Detection Threshold").pack(side=tk.LEFT, padx=(18, 0))
        self.flow_threshold_spinbox = ttk.Spinbox(
            pressure_controls,
            from_=0.0,
            to=200.0,
            increment=0.5,
            textvariable=self.flow_threshold_var,
            width=8,
            state=tk.DISABLED,
        )
        self.flow_threshold_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(pressure_controls, text="l/min").pack(side=tk.LEFT)

        self.apply_flow_threshold_button = ttk.Button(
            pressure_controls,
            text="Apply threshold",
            command=self._apply_flow_threshold_setting,
            state=tk.DISABLED,
        )
        self.apply_flow_threshold_button.pack(side=tk.LEFT, padx=(10, 0))

        pulse_controls = ttk.Frame(root)
        pulse_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(pulse_controls, text="Nozzles").pack(side=tk.LEFT)
        for index, nozzle_var in enumerate(self.nozzle_vars, start=1):
            checkbutton = ttk.Checkbutton(
                pulse_controls,
                text=f"Nozzle {index}",
                variable=nozzle_var,
                state=tk.DISABLED,
            )
            checkbutton.pack(side=tk.LEFT, padx=(8, 0))
            self.nozzle_checkbuttons.append(checkbutton)

        self.manual_pulse_button = ttk.Button(
            pulse_controls,
            text="Manual pulse",
            command=self._manual_pulse,
            state=tk.DISABLED,
        )
        self.manual_pulse_button.pack(side=tk.LEFT, padx=(18, 0))

        self.increment_pulse_button = ttk.Button(
            pulse_controls,
            text="Pulse + increment",
            command=self._increment_pulse,
            state=tk.DISABLED,
        )
        self.increment_pulse_button.pack(side=tk.LEFT, padx=(8, 0))

        self.decrement_pulse_button = ttk.Button(
            pulse_controls,
            text="Pulse - increment",
            command=self._decrement_pulse,
            state=tk.DISABLED,
        )
        self.decrement_pulse_button.pack(side=tk.LEFT, padx=(8, 0))

        increment_controls = ttk.Frame(root)
        increment_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(increment_controls, text="Starting pressure").pack(side=tk.LEFT)
        self.starting_pressure_spinbox = ttk.Spinbox(
            increment_controls,
            from_=0.0,
            to=REGULATOR_MAX_PRESSURE_BAR,
            increment=0.05,
            textvariable=self.starting_pressure_var,
            width=8,
            state=tk.DISABLED,
        )
        self.starting_pressure_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(increment_controls, text="bar").pack(side=tk.LEFT)

        ttk.Label(increment_controls, text="Pressure increment").pack(side=tk.LEFT, padx=(18, 0))
        self.pressure_increment_spinbox = ttk.Spinbox(
            increment_controls,
            from_=0.0,
            to=REGULATOR_MAX_PRESSURE_BAR,
            increment=0.05,
            textvariable=self.pressure_increment_var,
            width=8,
            state=tk.DISABLED,
        )
        self.pressure_increment_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(increment_controls, text="bar").pack(side=tk.LEFT)

        ttk.Label(increment_controls, text="Increment count").pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(increment_controls, textvariable=self.increment_count_var, width=5).pack(side=tk.LEFT, padx=(6, 0))

        self.reset_increment_button = ttk.Button(
            increment_controls,
            text="Reset increment",
            command=self._reset_increment,
            state=tk.DISABLED,
        )
        self.reset_increment_button.pack(side=tk.LEFT, padx=(10, 0))

        motor_controls = ttk.Frame(root)
        motor_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(motor_controls, text="Stepper").pack(side=tk.LEFT)
        self.motor_enable_checkbutton = ttk.Checkbutton(
            motor_controls,
            text="Enable",
            variable=self.motor_enabled_var,
            command=self._apply_motor_enable,
            state=tk.DISABLED,
        )
        self.motor_enable_checkbutton.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(motor_controls, textvariable=self.motor_position_var).pack(side=tk.LEFT, padx=(12, 0))

        self.motor_home_button = ttk.Button(
            motor_controls,
            text="Home -",
            command=self._motor_home,
            state=tk.DISABLED,
        )
        self.motor_home_button.pack(side=tk.LEFT, padx=(12, 0))

        self.motor_zero_button = ttk.Button(
            motor_controls,
            text="Set zero here",
            command=self._motor_set_zero,
            state=tk.DISABLED,
        )
        self.motor_zero_button.pack(side=tk.LEFT, padx=(8, 0))

        motor_motion_controls = ttk.Frame(root)
        motor_motion_controls.pack(fill=tk.X, pady=(6, 0))

        ttk.Label(motor_motion_controls, text="Stepper motion").pack(side=tk.LEFT)

        ttk.Label(motor_motion_controls, text="Distance").pack(side=tk.LEFT, padx=(18, 0))
        self.motor_distance_spinbox = ttk.Spinbox(
            motor_motion_controls,
            from_=0.01,
            to=2000.0,
            increment=1.0,
            textvariable=self.motor_distance_var,
            width=8,
            state=tk.DISABLED,
        )
        self.motor_distance_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(motor_motion_controls, text="mm").pack(side=tk.LEFT)

        ttk.Label(motor_motion_controls, text="Speed").pack(side=tk.LEFT, padx=(18, 0))
        self.motor_speed_spinbox = ttk.Spinbox(
            motor_motion_controls,
            from_=0.01,
            to=50.0,
            increment=0.5,
            textvariable=self.motor_speed_var,
            width=8,
            state=tk.DISABLED,
        )
        self.motor_speed_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(motor_motion_controls, text="mm/s").pack(side=tk.LEFT)

        self.motor_reverse_button = ttk.Button(
            motor_motion_controls,
            text="Jog -",
            command=self._motor_jog_reverse,
            state=tk.DISABLED,
        )
        self.motor_reverse_button.pack(side=tk.LEFT, padx=(18, 0))
        self.motor_forward_button = ttk.Button(
            motor_motion_controls,
            text="Jog +",
            command=self._motor_jog_forward,
            state=tk.DISABLED,
        )
        self.motor_forward_button.pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(motor_motion_controls, text="Absolute").pack(side=tk.LEFT, padx=(18, 0))
        self.motor_absolute_spinbox = ttk.Spinbox(
            motor_motion_controls,
            from_=-2000.0,
            to=2000.0,
            increment=1.0,
            textvariable=self.motor_absolute_var,
            width=8,
            state=tk.DISABLED,
        )
        self.motor_absolute_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(motor_motion_controls, text="mm").pack(side=tk.LEFT)

        self.motor_absolute_button = ttk.Button(
            motor_motion_controls,
            text="Go",
            command=self._motor_move_absolute,
            state=tk.DISABLED,
        )
        self.motor_absolute_button.pack(side=tk.LEFT, padx=(8, 0))

        self.motor_stop_button = ttk.Button(
            motor_motion_controls,
            text="Stop motor",
            command=self._motor_stop,
            state=tk.DISABLED,
        )
        self.motor_stop_button.pack(side=tk.LEFT, padx=(8, 0))
        self.motor_controls = [
            self.motor_enable_checkbutton,
            self.motor_home_button,
            self.motor_zero_button,
            self.motor_distance_spinbox,
            self.motor_speed_spinbox,
            self.motor_reverse_button,
            self.motor_forward_button,
            self.motor_absolute_spinbox,
            self.motor_absolute_button,
            self.motor_stop_button,
        ]

        colibri_connection_controls = ttk.Frame(root)
        colibri_connection_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(colibri_connection_controls, text="Colibri").pack(side=tk.LEFT)
        self.colibri_port_combo = ttk.Combobox(
            colibri_connection_controls,
            textvariable=self.colibri_port_var,
            width=36,
            state="readonly",
        )
        self.colibri_port_combo.pack(side=tk.LEFT, padx=(6, 8))

        self.colibri_connect_button = ttk.Button(
            colibri_connection_controls,
            text="Connect",
            command=self._toggle_colibri_connection,
        )
        self.colibri_connect_button.pack(side=tk.LEFT)
        self.colibri_refresh_button = ttk.Button(
            colibri_connection_controls,
            text="Read status",
            command=self._colibri_refresh_status,
            state=tk.DISABLED,
        )
        self.colibri_refresh_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(colibri_connection_controls, textvariable=self.colibri_position_var).pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(colibri_connection_controls, textvariable=self.colibri_status_var).pack(side=tk.LEFT, padx=(18, 0))

        colibri_motion_controls = ttk.Frame(root)
        colibri_motion_controls.pack(fill=tk.X, pady=(10, 0))

        self.colibri_enable_checkbutton = ttk.Checkbutton(
            colibri_motion_controls,
            text="Endstage",
            variable=self.colibri_enabled_var,
            command=self._apply_colibri_enable,
            state=tk.DISABLED,
        )
        self.colibri_enable_checkbutton.pack(side=tk.LEFT)

        self.colibri_reference_button = ttk.Button(
            colibri_motion_controls,
            text="Reference -",
            command=self._colibri_reference,
            state=tk.DISABLED,
        )
        self.colibri_reference_button.pack(side=tk.LEFT, padx=(8, 0))

        self.colibri_set_zero_button = ttk.Button(
            colibri_motion_controls,
            text="Set zero here",
            command=self._colibri_set_zero_here,
            state=tk.DISABLED,
        )
        self.colibri_set_zero_button.pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(colibri_motion_controls, text="Relative").pack(side=tk.LEFT, padx=(18, 0))
        self.colibri_distance_spinbox = ttk.Spinbox(
            colibri_motion_controls,
            from_=0.005,
            to=COLIBRI_TRAVEL_MM,
            increment=0.5,
            textvariable=self.colibri_distance_var,
            width=8,
            state=tk.DISABLED,
        )
        self.colibri_distance_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(colibri_motion_controls, text="mm").pack(side=tk.LEFT)

        self.colibri_reverse_button = ttk.Button(
            colibri_motion_controls,
            text="Jog -",
            command=self._colibri_jog_reverse,
            state=tk.DISABLED,
        )
        self.colibri_reverse_button.pack(side=tk.LEFT, padx=(8, 0))
        self.colibri_forward_button = ttk.Button(
            colibri_motion_controls,
            text="Jog +",
            command=self._colibri_jog_forward,
            state=tk.DISABLED,
        )
        self.colibri_forward_button.pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(colibri_motion_controls, text="Absolute").pack(side=tk.LEFT, padx=(18, 0))
        self.colibri_absolute_spinbox = ttk.Spinbox(
            colibri_motion_controls,
            from_=-COLIBRI_TRAVEL_MM,
            to=COLIBRI_TRAVEL_MM,
            increment=0.5,
            textvariable=self.colibri_absolute_var,
            width=8,
            state=tk.DISABLED,
        )
        self.colibri_absolute_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(colibri_motion_controls, text="mm").pack(side=tk.LEFT)
        self.colibri_absolute_button = ttk.Button(
            colibri_motion_controls,
            text="Go",
            command=self._colibri_move_absolute,
            state=tk.DISABLED,
        )
        self.colibri_absolute_button.pack(side=tk.LEFT, padx=(8, 0))

        self.colibri_stop_button = ttk.Button(
            colibri_motion_controls,
            text="Stop Colibri",
            command=self._colibri_stop,
            state=tk.DISABLED,
        )
        self.colibri_stop_button.pack(side=tk.LEFT, padx=(18, 0))

        self.colibri_controls = [
            self.colibri_refresh_button,
            self.colibri_enable_checkbutton,
            self.colibri_reference_button,
            self.colibri_set_zero_button,
            self.colibri_distance_spinbox,
            self.colibri_reverse_button,
            self.colibri_forward_button,
            self.colibri_absolute_spinbox,
            self.colibri_absolute_button,
            self.colibri_stop_button,
        ]

        force_controls = ttk.Frame(root)
        force_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(force_controls, text="Force sensor").pack(side=tk.LEFT)
        self.force_port_combo = ttk.Combobox(
            force_controls,
            textvariable=self.force_port_var,
            width=36,
            state="readonly",
        )
        self.force_port_combo.pack(side=tk.LEFT, padx=(6, 8))

        ttk.Label(force_controls, text="Baud").pack(side=tk.LEFT)
        self.force_baud_spinbox = ttk.Spinbox(
            force_controls,
            from_=1200,
            to=921600,
            increment=1200,
            textvariable=self.force_baud_var,
            width=8,
            state=tk.NORMAL,
        )
        self.force_baud_spinbox.pack(side=tk.LEFT, padx=(6, 8))

        self.force_connect_button = ttk.Button(
            force_controls,
            text="Connect",
            command=self._toggle_force_connection,
        )
        self.force_connect_button.pack(side=tk.LEFT)

        ttk.Label(force_controls, text="Force scaling").pack(side=tk.LEFT, padx=(8, 0))
        self.force_scale_spinbox = ttk.Spinbox(
            force_controls,
            from_=-1000000.0,
            to=1000000.0,
            increment=0.0001,
            textvariable=self.force_scale_var,
            width=10,
        )
        self.force_scale_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        self.force_scale_button = ttk.Button(
            force_controls,
            text="Apply scaling",
            command=self._apply_force_scaling,
        )
        self.force_scale_button.pack(side=tk.LEFT)

        ttk.Label(force_controls, text="Force average").pack(side=tk.LEFT, padx=(8, 0))
        self.force_average_spinbox = ttk.Spinbox(
            force_controls,
            from_=0.0,
            to=60.0,
            increment=0.05,
            textvariable=self.force_average_var,
            width=7,
        )
        self.force_average_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(force_controls, text="s").pack(side=tk.LEFT)
        self.force_average_button = ttk.Button(
            force_controls,
            text="Apply average",
            command=self._apply_force_average,
        )
        self.force_average_button.pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(force_controls, text="Force impulse threshold").pack(side=tk.LEFT, padx=(8, 0))
        self.force_impulse_threshold_spinbox = ttk.Spinbox(
            force_controls,
            from_=0.0,
            to=1000000.0,
            increment=0.01,
            textvariable=self.force_impulse_threshold_var,
            width=8,
        )
        self.force_impulse_threshold_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        self.force_impulse_threshold_button = ttk.Button(
            force_controls,
            text="Apply threshold",
            command=self._apply_force_impulse_threshold,
        )
        self.force_impulse_threshold_button.pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(force_controls, textvariable=self.force_value_var).pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(force_controls, textvariable=self.force_status_var).pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(force_controls, textvariable=self.force_rate_var).pack(side=tk.LEFT, padx=(18, 0))

        part_controls = ttk.Frame(root)
        part_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(part_controls, text="Part CSV").pack(side=tk.LEFT)
        ttk.Button(part_controls, text="Load part CSV", command=self._load_part_csv).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(part_controls, textvariable=self.part_csv_status_var).pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(part_controls, text="Pose").pack(side=tk.LEFT, padx=(18, 0))
        self.part_pose_combo = ttk.Combobox(
            part_controls,
            textvariable=self.part_pose_var,
            width=8,
            state=tk.DISABLED,
        )
        self.part_pose_combo.pack(side=tk.LEFT, padx=(6, 4))
        self.part_pose_combo.bind("<<ComboboxSelected>>", self._part_pose_selected)

        ttk.Label(part_controls, text="Hole").pack(side=tk.LEFT, padx=(8, 0))
        self.part_hole_combo = ttk.Combobox(
            part_controls,
            textvariable=self.part_hole_var,
            width=8,
            state=tk.DISABLED,
        )
        self.part_hole_combo.pack(side=tk.LEFT, padx=(6, 4))
        self.part_hole_combo.bind("<<ComboboxSelected>>", self._part_hole_selected)

        ttk.Checkbutton(
            part_controls,
            text="Add cap in measurements",
            variable=self.use_cap_offsets_var,
        ).pack(side=tk.LEFT, padx=(12, 0))

        part_position_controls = ttk.Frame(root)
        part_position_controls.pack(fill=tk.X, pady=(6, 0))

        ttk.Label(part_position_controls, text="Nozzle offset").pack(side=tk.LEFT)
        self.nozzle_offset_spinbox = ttk.Spinbox(
            part_position_controls,
            from_=-2000.0,
            to=2000.0,
            increment=0.1,
            textvariable=self.nozzle_offset_var,
            width=8,
        )
        self.nozzle_offset_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(part_position_controls, text="mm").pack(side=tk.LEFT)

        ttk.Label(part_position_controls, text="Test stand height").pack(side=tk.LEFT, padx=(18, 0))
        self.test_stand_height_spinbox = ttk.Spinbox(
            part_position_controls,
            from_=-2000.0,
            to=2000.0,
            increment=0.1,
            textvariable=self.test_stand_height_var,
            width=8,
        )
        self.test_stand_height_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(part_position_controls, text="mm").pack(side=tk.LEFT)

        ttk.Label(part_position_controls, text="Holder height").pack(side=tk.LEFT, padx=(18, 0))
        self.holder_height_spinbox = ttk.Spinbox(
            part_position_controls,
            from_=-2000.0,
            to=2000.0,
            increment=0.1,
            textvariable=self.holder_height_var,
            width=8,
        )
        self.holder_height_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(part_position_controls, text="mm").pack(side=tk.LEFT)

        ttk.Label(part_position_controls, textvariable=self.part_y_offset_var).pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(part_position_controls, textvariable=self.part_z_offset_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(part_position_controls, textvariable=self.part_stepper_position_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(part_position_controls, textvariable=self.part_colibri_position_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(part_position_controls, text="Set targets", command=self._set_part_axis_targets).pack(
            side=tk.LEFT,
            padx=(12, 0),
        )

        ttk.Label(root, textvariable=self.mode_var).pack(fill=tk.X, pady=(10, 0))
        ttk.Label(root, textvariable=self.debug_log_var).pack(fill=tk.X, pady=(4, 0))
        ttk.Label(root, textvariable=self.status_var).pack(fill=tk.X, pady=(10, 8))

        columns = (
            "time",
            "target_pressure",
            "pressure_before",
            "regulator_feedback",
            "regulator_pwm",
            "valves_open",
            "flow",
            "force",
            "raw_force",
        )
        self.table = ttk.Treeview(root, columns=columns, show="headings", height=18)
        headings = {
            "time": "Time ms",
            "target_pressure": "Target pressure",
            "pressure_before": "Pressure before valve",
            "regulator_feedback": "Actual regulator pressure",
            "regulator_pwm": "Regulator PWM",
            "valves_open": "Valves open",
            "flow": "Flow",
            "force": "Force reading",
            "raw_force": "Raw force reading",
        }
        for col, heading in headings.items():
            self.table.heading(col, text=heading)
            self.table.column(col, width=110, anchor=tk.CENTER)
        self.table.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(root, height=7, wrap=tk.NONE)
        self.log.pack(fill=tk.X, pady=(8, 0))

        def _trigger_pulse_btn(event, button):
            if event.widget.winfo_class() in ("Entry", "TEntry", "TCombobox", "TSpinbox", "Text"):
                return
            button.invoke()

        self.bind("1", lambda event: _trigger_pulse_btn(event, self.manual_pulse_button))
        self.bind("2", lambda event: _trigger_pulse_btn(event, self.increment_pulse_button))
        self.bind("3", lambda event: _trigger_pulse_btn(event, self.decrement_pulse_button))

    def _refresh_ethercat_adapters(self):
        if pysoem is None:
            self.ethercat_status_var.set("EtherCAT: install pysoem and Npcap first")
            self.ethercat_adapter_combo["values"] = ()
            return
        if self.ethercat_master or self.ethercat_busy:
            return

        try:
            adapters = pysoem.find_adapters()
        except Exception as exc:
            self.ethercat_adapters = {}
            self.ethercat_adapter_combo["values"] = ()
            self.ethercat_status_var.set(f"EtherCAT adapter scan failed: {exc}")
            return

        self.ethercat_adapters = {}
        for adapter in adapters:
            name = self._ethercat_adapter_text(getattr(adapter, "name", ""))
            description = self._ethercat_adapter_text(
                getattr(adapter, "desc", "") or "Network adapter"
            )
            if not name:
                continue
            label = f"{description} — {name}"
            self.ethercat_adapters[label] = name

        labels = list(self.ethercat_adapters)
        self.ethercat_adapter_combo["values"] = labels
        if labels and self.ethercat_adapter_var.get() not in labels:
            self.ethercat_adapter_var.set(labels[0])
        if labels:
            self.ethercat_status_var.set(f"EtherCAT: {len(labels)} adapter(s) available")
        else:
            self.ethercat_adapter_var.set("")
            self.ethercat_status_var.set("EtherCAT: no Npcap-compatible adapter found")

    @staticmethod
    def _ethercat_adapter_text(value):
        if isinstance(value, bytes):
            return value.decode("utf-8", errors="replace")
        return str(value)

    def _toggle_ethercat_connection(self):
        if self.ethercat_master:
            self._disconnect_ethercat()
        else:
            self._connect_ethercat()

    def _connect_ethercat(self):
        if pysoem is None:
            messagebox.showerror(
                "Missing EtherCAT dependency",
                "Install PySOEM and Npcap first. During Npcap setup, enable\n"
                "'Install Npcap in WinPcap API-compatible Mode'.",
            )
            return
        adapter_name = self.ethercat_adapters.get(self.ethercat_adapter_var.get())
        if not adapter_name:
            messagebox.showerror("No adapter selected", "Select the Ethernet adapter connected to the EK1100.")
            return

        self.ethercat_busy = True
        self.ethercat_status_var.set("EtherCAT: scanning bus...")
        self.ethercat_connect_button.configure(state=tk.DISABLED)
        self.ethercat_refresh_button.configure(state=tk.DISABLED)
        self.ethercat_adapter_combo.configure(state=tk.DISABLED)
        threading.Thread(
            target=self._scan_ethercat_bus,
            args=(adapter_name,),
            daemon=True,
        ).start()

    def _scan_ethercat_bus(self, adapter_name):
        master = pysoem.Master()
        try:
            master.open(adapter_name)
            device_count = master.config_init()
            if device_count <= 0:
                master.close()
                self.messages.put(("ethercat_error", "No EtherCAT devices found on the selected adapter."))
                return

            devices = []
            for position, slave in enumerate(master.slaves, start=1):
                devices.append({
                    "position": position,
                    "name": str(getattr(slave, "name", "") or "Unknown EtherCAT device"),
                    "vendor_id": int(getattr(slave, "man", 0)),
                    "product_code": int(getattr(slave, "id", 0)),
                    "revision": int(getattr(slave, "rev", 0)),
                })
            if self.closing:
                master.close()
                return
            self.messages.put(("ethercat_connected", (master, adapter_name, devices)))
        except Exception as exc:
            try:
                master.close()
            except Exception:
                pass
            self.messages.put(("ethercat_error", str(exc)))

    def _handle_ethercat_connected(self, value):
        master, adapter_name, devices = value
        if self.closing:
            master.close()
            return
        self.ethercat_master = master
        self.ethercat_busy = False
        self.ethercat_connect_button.configure(text="Disconnect", state=tk.NORMAL)
        self.ethercat_status_var.set(f"EtherCAT: connected, {len(devices)} device(s) in PRE-OP")
        self.status_var.set(f"EtherCAT bus found on {adapter_name}: {len(devices)} device(s)")
        self._write_debug_log(f"ETHERCAT connected adapter={adapter_name!r} devices={len(devices)}")
        for device in devices:
            line = (
                f"EtherCAT [{device['position']}] {device['name']} | "
                f"vendor 0x{device['vendor_id']:08X}, product 0x{device['product_code']:08X}, "
                f"revision 0x{device['revision']:08X}"
            )
            self._append_log_line(line)
            self._write_debug_log(line)

    def _handle_ethercat_error(self, error):
        self.ethercat_busy = False
        self.ethercat_connect_button.configure(text="Scan and connect", state=tk.NORMAL)
        self.ethercat_refresh_button.configure(state=tk.NORMAL)
        self.ethercat_adapter_combo.configure(state="readonly")
        self.ethercat_status_var.set(f"EtherCAT: {error}")
        self.status_var.set(f"EtherCAT connection failed: {error}")
        self._write_debug_log(f"ETHERCAT error {error}")

    def _disconnect_ethercat(self):
        if self.ethercat_master:
            try:
                self.ethercat_master.close()
            except Exception as exc:
                self._write_debug_log(f"ETHERCAT close error {exc}")
        self.ethercat_master = None
        self.ethercat_busy = False
        self.ethercat_connect_button.configure(text="Scan and connect", state=tk.NORMAL)
        self.ethercat_refresh_button.configure(state=tk.NORMAL)
        self.ethercat_adapter_combo.configure(state="readonly")
        self.ethercat_status_var.set("EtherCAT: disconnected")
        self._write_debug_log("ETHERCAT disconnected")

    def _refresh_ports(self):
        if list_ports is None:
            self.status_var.set("Install pyserial first: python -m pip install pyserial")
            return

        ports = list(list_ports.comports())
        self.port_devices = {
            f"{port.device} - {port.description}": port.device
            for port in ports
        }
        labels = list(self.port_devices)
        self.port_combo["values"] = labels
        self.colibri_port_combo["values"] = labels
        self.force_port_combo["values"] = labels
        if labels and self.port_var.get() not in labels:
            self.port_var.set(self._preferred_port_label(labels, ("arduino", "ttyacm")) or labels[0])
        if labels and self.colibri_port_var.get() not in labels:
            self.colibri_port_var.set(
                self._preferred_port_label(labels, ("dedi", "ftdi", "rs485", "ttyusb")) or labels[0]
            )
        if labels and self.force_port_var.get() not in labels:
            self.force_port_var.set(
                self._preferred_port_label(labels, ("gsv", "me-", "me ", "usb serial", "usb-serial")) or labels[0]
            )
        elif not labels:
            self.port_var.set("")
            self.colibri_port_var.set("")
            self.force_port_var.set("")
            self.status_var.set("No serial ports found. Check the USB cable, driver, and Arduino IDE Serial Monitor.")
        else:
            self.status_var.set(f"Found {len(labels)} serial port(s).")

    def _preferred_port_label(self, labels, keywords):
        for keyword in keywords:
            for label in labels:
                if keyword in label.lower():
                    return label
        return None

    def _toggle_connection(self):
        if self.serial_port:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial first:\npython -m pip install pyserial")
            return

        port = self._selected_port_device()
        if not port:
            messagebox.showerror("No port selected", "Select the Arduino serial port.")
            return
        if self.force_serial_port and port == self._selected_force_port_device():
            messagebox.showerror("Port already in use", "Select a separate serial port for the force sensor.")
            return

        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1, write_timeout=SERIAL_WRITE_TIMEOUT)
            time.sleep(2.0)
        except serial.SerialException as exc:
            self.serial_port = None
            messagebox.showerror("Connection failed", str(exc))
            return

        self.reader_running = True
        self.writer_running = True
        self.reader_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.writer_thread = threading.Thread(target=self._write_serial, daemon=True)
        self.reader_thread.start()
        self.writer_thread.start()
        self.connect_button.configure(text="Disconnect")
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.NORMAL)
        self.test_start_pressure_spinbox.configure(state=tk.NORMAL)
        self.test_end_pressure_spinbox.configure(state=tk.NORMAL)
        self.test_repeats_spinbox.configure(state=tk.NORMAL)
        self.target_pressure_spinbox.configure(state=tk.NORMAL)
        self.apply_pressure_button.configure(state=tk.NORMAL)
        self.stream_checkbutton.configure(state=tk.NORMAL)
        self.flow_threshold_spinbox.configure(state=tk.NORMAL)
        self.apply_flow_threshold_button.configure(state=tk.NORMAL)
        for checkbutton in self.nozzle_checkbuttons:
            checkbutton.configure(state=tk.NORMAL)
        self.starting_pressure_spinbox.configure(state=tk.NORMAL)
        self.pressure_increment_spinbox.configure(state=tk.NORMAL)
        self.reset_increment_button.configure(state=tk.NORMAL)
        self._set_pulse_buttons_enabled(True)
        self._set_motor_controls_enabled(True)
        self.mode_var.set("Mode: connected")
        self.status_var.set(f"Connected to {port} at {BAUD_RATE} baud")
        self._write_debug_log(f"ARDUINO connected port={port} baud={BAUD_RATE}")
        self._apply_pressure_settings()
        self._apply_flow_threshold_setting()
        self._apply_stream_setting()
        self._send("MOTOR_POS")

    def _selected_port_device(self):
        selected = self.port_var.get()
        return self.port_devices.get(selected, selected)

    def _selected_colibri_port_device(self):
        selected = self.colibri_port_var.get()
        return self.port_devices.get(selected, selected)

    def _selected_force_port_device(self):
        selected = self.force_port_var.get()
        return self.port_devices.get(selected, selected)

    def _toggle_debug_log(self):
        if self.debug_log_file:
            self._stop_debug_log()
        else:
            self._start_debug_log()

    def _start_debug_log(self):
        default_name = f"biba_debug_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        path = filedialog.asksaveasfilename(
            initialfile=default_name,
            defaultextension=".log",
            filetypes=[("Log files", "*.log"), ("Text files", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            log_file = open(path, "w", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("Debug log failed", str(exc))
            return

        with self.debug_log_lock:
            self.debug_log_file = log_file
            self.debug_log_path = path
        if self.colibri:
            self.colibri.set_debug_logger(self._write_debug_log)
        self.debug_log_button.configure(text="Stop debug log")
        self.debug_log_var.set(f"Debug log: {path}")
        self._write_debug_log("LOG started")
        self._write_debug_log(f"GUI Arduino port label={self.port_var.get()!r} device={self._selected_port_device()!r}")
        self._write_debug_log(
            f"GUI Colibri port label={self.colibri_port_var.get()!r} device={self._selected_colibri_port_device()!r}"
        )
        self._write_debug_log(
            f"GUI force port label={self.force_port_var.get()!r} device={self._selected_force_port_device()!r} "
            f"baud={self.force_baud_var.get()!r} mode=binary scale={self.force_scaling:.12g}"
        )

    def _stop_debug_log(self):
        self._write_debug_log("LOG stopped")
        if self.colibri:
            self.colibri.set_debug_logger(None)
        with self.debug_log_lock:
            if self.debug_log_file:
                self.debug_log_file.close()
            stopped_path = self.debug_log_path
            self.debug_log_file = None
            self.debug_log_path = None
        self.debug_log_button.configure(text="Start debug log")
        self.debug_log_var.set(f"Debug log: stopped ({stopped_path})" if stopped_path else "Debug log: off")

    def _write_debug_log(self, message):
        timestamp = dt.datetime.now().isoformat(timespec="milliseconds")
        with self.debug_log_lock:
            if not self.debug_log_file:
                return
            self.debug_log_file.write(f"{timestamp} {message}\n")
            self.debug_log_file.flush()

    def _disconnect(self):
        self._write_debug_log("ARDUINO disconnect requested")
        self.writer_running = False
        self.commands.put(None)
        if self.writer_thread:
            self.writer_thread.join(timeout=0.5)
        self.reader_running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=0.5)
        if self.serial_port:
            self.serial_port.close()
        self.serial_port = None
        self._clear_pending_commands()
        self.connect_button.configure(text="Connect")
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.DISABLED)
        self.test_start_pressure_spinbox.configure(state=tk.DISABLED)
        self.test_end_pressure_spinbox.configure(state=tk.DISABLED)
        self.test_repeats_spinbox.configure(state=tk.DISABLED)
        self.target_pressure_spinbox.configure(state=tk.DISABLED)
        self.apply_pressure_button.configure(state=tk.DISABLED)
        self.stream_checkbutton.configure(state=tk.DISABLED)
        self.flow_threshold_spinbox.configure(state=tk.DISABLED)
        self.apply_flow_threshold_button.configure(state=tk.DISABLED)
        for checkbutton in self.nozzle_checkbuttons:
            checkbutton.configure(state=tk.DISABLED)
        self.starting_pressure_spinbox.configure(state=tk.DISABLED)
        self.pressure_increment_spinbox.configure(state=tk.DISABLED)
        self.reset_increment_button.configure(state=tk.DISABLED)
        self._set_pulse_buttons_enabled(False)
        self._set_motor_controls_enabled(False)
        self.motor_enabled_var.set(False)
        self.last_motor_speed_steps_s = None
        self.motor_position_var.set("Stepper position: --")
        self.last_motor_position_mm = None
        self.pulse_in_progress = False
        self.pending_increment_direction = 0
        self.pending_flip_angle = -1
        self.pending_pulse_mask = ""
        self.pending_pulse_duration_ms = None
        self.active_test_mask = ""
        self.current_impulse = None
        self.last_valves_open = False
        self.mode_var.set("Mode: disconnected")
        self.status_var.set("Disconnected")
        self._write_debug_log("ARDUINO disconnected")

    def _start_test(self):
        start_pressure = self._validated_pressure_step(self.test_start_pressure_var, "test start pressure")
        end_pressure = self._validated_pressure_step(self.test_end_pressure_var, "test end pressure")
        repeats = self._validated_repeats()
        if start_pressure is None or end_pressure is None or repeats is None:
            return
        if end_pressure < start_pressure:
            messagebox.showerror("Invalid test range", "Test end pressure must be greater than or equal to start pressure.")
            return
        mask = self._selected_nozzle_mask()
        if mask == 0:
            messagebox.showerror("No nozzle selected", "Select at least one nozzle for the test.")
            return

        if not self._apply_flow_threshold_setting():
            return

        self._clear_run_display()
        self.active_test_mask = str(mask)
        self.mode_var.set("Mode: test sequence")
        self._write_debug_log(
            f"GUI start test range={start_pressure:.2f}-{end_pressure:.2f} repeats={repeats} mask={mask}"
        )
        self._send(f"START:{start_pressure:.2f}:{end_pressure:.2f}:{repeats}:{mask}")

    def _stop_test(self):
        self.mode_var.set("Mode: idle")
        self._write_debug_log("GUI stop test")
        self._send("STOP")

    def _manual_pulse(self):
        self._start_pulse(increment_direction=0)

    def _increment_pulse(self):
        if self._validated_pressure(self.pressure_increment_var, "pressure increment") is None:
            return
        self._start_pulse(increment_direction=1)

    def _decrement_pulse(self):
        if self._validated_pressure(self.pressure_increment_var, "pressure increment") is None:
            return
        self._start_pulse(increment_direction=-1)

    def _start_pulse(self, increment_direction):
        mask = self._selected_nozzle_mask()
        if mask == 0:
            messagebox.showerror("No nozzle selected", "Select at least one nozzle for the pulse.")
            return

        if not self._apply_pressure_settings():
            return
        if not self._apply_flow_threshold_setting():
            return

        self.pulse_in_progress = True
        self.pending_increment_direction = increment_direction
        self.pending_flip_angle = -1
        self.pending_pulse_mask = str(mask)
        self._set_pulse_buttons_enabled(False)
        self.mode_var.set("Mode: manual pulse running")
        self._write_debug_log(f"GUI pulse mask={mask} increment_direction={increment_direction}")
        self._send(f"PULSE:{mask}")

    def _show_flip_angle_prompt(self, callback):
        dialog = tk.Toplevel(self)
        dialog.title("Flip angle")
        dialog.transient(self)
        dialog.resizable(False, False)

        ttk.Label(dialog, text="Flip angle for this impulse").pack(padx=18, pady=(14, 8))
        button_frame = ttk.Frame(dialog)
        button_frame.pack(padx=14, pady=(0, 14))

        def choose(value):
            callback(value)
            dialog.destroy()

        for angle in (90, 180, 0):
            ttk.Button(button_frame, text=str(angle), width=10, command=lambda value=angle: choose(value)).pack(
                side=tk.LEFT,
                padx=4,
            )
        ttk.Button(button_frame, text="Undefined", width=10, command=lambda: choose(-1)).pack(side=tk.LEFT, padx=4)

        # --- Key binding ---
        def handle_subwindow_key(event, value):
            choose(value)
            return "break"  # Prevents the event from passing to the main window

        dialog.bind("3", lambda e: handle_subwindow_key(e, 0))
        dialog.bind("1", lambda e: handle_subwindow_key(e, 90))
        dialog.bind("2", lambda e: handle_subwindow_key(e, 180))
        dialog.bind("4", lambda e: handle_subwindow_key(e, -1))
        # -----------------------------

        dialog.protocol("WM_DELETE_WINDOW", lambda: choose(-1))
        dialog.update_idletasks()
        x = self.winfo_rootx() + max(0, (self.winfo_width() - dialog.winfo_reqwidth()) // 2)
        y = self.winfo_rooty() + max(0, (self.winfo_height() - dialog.winfo_reqheight()) // 2)
        dialog.geometry(f"+{x}+{y}")
        dialog.lift()
        dialog.focus_force()

    def _selected_nozzle_mask(self):
        mask = 0
        for index, nozzle_var in enumerate(self.nozzle_vars):
            if nozzle_var.get():
                mask |= 1 << index
        return mask

    def _set_pulse_buttons_enabled(self, enabled):
        state = tk.NORMAL if enabled and self.serial_port and not self.pulse_in_progress else tk.DISABLED
        self.manual_pulse_button.configure(state=state)
        self.increment_pulse_button.configure(state=state)
        self.decrement_pulse_button.configure(state=state)

    def _reset_increment(self):
        starting_pressure = self._validated_pressure(self.starting_pressure_var, "starting pressure")
        if starting_pressure is None:
            return

        self.increment_count_var.set(0)
        self.target_pressure_var.set(round(starting_pressure, 3))
        self._apply_pressure_settings()

    def _apply_pressure_settings(self):
        try:
            target_pressure = float(self.target_pressure_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid pressure setting", "Enter a numeric target pressure.")
            return False

        target_pressure = min(max(target_pressure, 0.0), REGULATOR_MAX_PRESSURE_BAR)
        self.target_pressure_var.set(round(target_pressure, 3))
        self.mode_var.set("Mode: manual pressure pending")
        self._write_debug_log(f"GUI set pressure target={target_pressure:.3f}")
        self._send(f"SET_PRESSURE:{target_pressure:.3f}", flush_live_backlog=True)
        return True

    def _apply_flow_threshold_setting(self):
        try:
            flow_threshold = float(self.flow_threshold_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid Flow Detection Threshold", "Enter a numeric Flow Detection Threshold.")
            return False

        flow_threshold = min(max(flow_threshold, 0.0), 200.0)
        self.flow_threshold_var.set(round(flow_threshold, 3))
        self._write_debug_log(f"GUI set Flow Detection Threshold={flow_threshold:.3f} l/min")
        self._send(f"SET_FLOW_THRESHOLD:{flow_threshold:.3f}")
        return True

    def _validated_pressure(self, variable, label):
        try:
            value = float(variable.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid pressure setting", f"Enter a numeric {label}.")
            return None

        value = min(max(value, 0.0), REGULATOR_MAX_PRESSURE_BAR)
        variable.set(round(value, 3))
        return value

    def _validated_pressure_step(self, variable, label):
        try:
            value = float(variable.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid pressure setting", f"Enter a numeric {label}.")
            return None

        value = min(max(value, 0.0), REGULATOR_MAX_PRESSURE_BAR)
        step_count = math.ceil((value - 0.000001) / TEST_PRESSURE_STEP_BAR)
        stepped_value = step_count * TEST_PRESSURE_STEP_BAR
        stepped_value = min(max(stepped_value, 0.0), REGULATOR_MAX_PRESSURE_BAR)
        variable.set(round(stepped_value, 2))
        return stepped_value

    def _validated_repeats(self):
        try:
            repeats = int(float(self.test_repeats_var.get()))
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid repeats", "Enter a numeric repeat count.")
            return None

        repeats = min(max(repeats, 1), 100)
        self.test_repeats_var.set(repeats)
        return repeats

    def _apply_stream_setting(self):
        self._write_debug_log(f"GUI stream {'on' if self.stream_var.get() else 'off'}")
        self._send("STREAM_ON" if self.stream_var.get() else "STREAM_OFF")

    def _set_motor_controls_enabled(self, enabled):
        state = tk.NORMAL if enabled and self.serial_port else tk.DISABLED
        for control in self.motor_controls:
            control.configure(state=state)

    def _apply_motor_enable(self):
        self._write_debug_log(f"GUI stepper enable={self.motor_enabled_var.get()}")
        self._send(f"MOTOR_ENABLE:{1 if self.motor_enabled_var.get() else 0}")

    def _apply_motor_speed(self):
        speed_mm_s = self._validated_float(self.motor_speed_var, "motor speed", 0.01, 50.0)
        if speed_mm_s is None:
            return None

        speed_steps_s = self._mm_per_second_to_steps_per_second(speed_mm_s)
        if speed_steps_s == self.last_motor_speed_steps_s:
            return speed_steps_s

        if not self._send(f"MOTOR_SPEED:{speed_steps_s}"):
            return None
        return speed_steps_s

    def _motor_jog_forward(self):
        self._motor_jog(direction=1)

    def _motor_jog_reverse(self):
        self._motor_jog(direction=-1)

    def _motor_home(self):
        if not self.motor_enabled_var.get():
            messagebox.showerror("Motor disabled", "Enable the stepper output before homing.")
            return

        speed_steps_s = self._apply_motor_speed()
        if speed_steps_s is None:
            return

        self.mode_var.set("Mode: stepper homing")
        self._write_debug_log("GUI stepper home")
        self._send("MOTOR_HOME")

    def _motor_set_zero(self):
        self._write_debug_log("GUI stepper set zero")
        self._send("MOTOR_ZERO")

    def _motor_move_absolute(self):
        if not self.motor_enabled_var.get():
            messagebox.showerror("Motor disabled", "Enable the stepper output before moving.")
            return

        target_mm = self._validated_float(self.motor_absolute_var, "motor absolute position", -2000.0, 2000.0)
        speed_steps_s = self._apply_motor_speed()
        if target_mm is None or speed_steps_s is None:
            return

        target_steps = self._mm_to_steps(target_mm)
        self.mode_var.set(f"Mode: stepper absolute | {target_mm:.3f} mm")
        self._write_debug_log(f"GUI stepper absolute target_mm={target_mm:.3f} steps={target_steps}")
        self._send(f"MOTOR_ABS:{target_steps}")

    def _motor_jog(self, direction):
        if not self.motor_enabled_var.get():
            messagebox.showerror("Motor disabled", "Enable the stepper output before jogging.")
            return

        distance_mm = self._validated_float(self.motor_distance_var, "motor distance", 0.01, 2000.0)
        speed_steps_s = self._apply_motor_speed()
        if distance_mm is None or speed_steps_s is None:
            return

        steps = self._mm_to_steps(distance_mm)
        signed_steps = direction * steps
        self.mode_var.set(f"Mode: stepper jog | {direction * distance_mm:.3f} mm")
        self._write_debug_log(
            f"GUI stepper jog distance_mm={direction * distance_mm:.3f} steps={signed_steps}"
        )
        self._send(f"MOTOR_MOVE:{signed_steps}")

    def _motor_stop(self):
        self._write_debug_log("GUI stepper stop")
        self._send("MOTOR_STOP")

    def _toggle_force_connection(self):
        if self.force_serial_port:
            self._disconnect_force_sensor()
        else:
            self._connect_force_sensor()

    def _connect_force_sensor(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial first:\npython -m pip install pyserial")
            return

        port = self._selected_force_port_device()
        if not port:
            messagebox.showerror("No port selected", "Select the force sensor serial port.")
            return
        if self.serial_port and port == self._selected_port_device():
            messagebox.showerror("Port already in use", "Select a separate serial port for the Arduino.")
            return
        if self.colibri and port == self._selected_colibri_port_device():
            messagebox.showerror("Port already in use", "Select a separate serial port for the Colibri axis.")
            return

        try:
            baud_rate = int(self.force_baud_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid baud rate", "Enter a numeric baud rate for the force sensor.")
            return

        try:
            self.force_serial_port = serial.Serial(
                port,
                baud_rate,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=FORCE_READ_TIMEOUT_SECONDS,
                write_timeout=SERIAL_WRITE_TIMEOUT,
            )
            self.force_serial_port.reset_input_buffer()
        except serial.SerialException as exc:
            self.force_serial_port = None
            messagebox.showerror("Force sensor connection failed", str(exc))
            return

        self.force_reader_running = True
        self.force_reader_thread = threading.Thread(target=self._read_force_serial, daemon=True)
        self.force_reader_thread.start()
        self.force_connect_button.configure(text="Disconnect")
        self.force_port_combo.configure(state=tk.DISABLED)
        self.force_baud_spinbox.configure(state=tk.DISABLED)
        self.force_status_var.set(self._force_status(f"Force sensor: connected to {port} at {baud_rate} baud"))
        self.status_var.set(f"Connected force sensor on {port}")
        self._write_debug_log(f"FORCE connected port={port} baud={baud_rate}")

    def _disconnect_force_sensor(self):
        self._write_debug_log("FORCE disconnect requested")
        self.force_reader_running = False
        if self.force_reader_thread:
            self.force_reader_thread.join(timeout=0.5)
        if self.force_serial_port:
            self.force_serial_port.close()
        self.force_serial_port = None
        self.force_reader_thread = None
        with self.force_lock:
            self.latest_force_raw_n = None
            self.latest_force_n = None
            self.latest_force_time = None
            self.force_average_samples.clear()
            self.force_rate_times.clear()
        self.force_connect_button.configure(text="Connect")
        self.force_port_combo.configure(state="readonly")
        self.force_baud_spinbox.configure(state=tk.NORMAL)
        self.force_value_var.set("Force reading: --")
        self.force_rate_var.set("Force rate: --")
        self.force_status_var.set(self._force_status("Force sensor: disconnected"))
        self._write_debug_log("FORCE disconnected")

    def _force_status(self, prefix):
        return (
            f"{prefix} | binary 5-byte mode, scale {self.force_scaling:.6g} x{FORCE_BINARY_FULL_SCALE_FACTOR:.3g}, "
            f"avg {self.force_average_seconds:.3g} s, impulse threshold {self.force_impulse_threshold:.4g}"
        )

    def _format_force_value(self, force_value):
        return "Force reading: --" if force_value is None else f"Force reading: {force_value:.4f}"

    def _apply_force_scaling(self):
        try:
            force_scaling = float(self.force_scale_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid force scaling", "Enter a numeric force scaling value.")
            return

        if force_scaling == 0.0:
            messagebox.showerror("Invalid force scaling", "Force scaling must not be zero.")
            return

        with self.force_lock:
            self.force_scaling = force_scaling
            self.force_average_samples.clear()
            if self.latest_force_raw_n is not None:
                signed_fraction = (self.latest_force_raw_n - FORCE_BINARY_BIPOLAR_ZERO) / FORCE_BINARY_POSITIVE_SPAN
                self.latest_force_n = signed_fraction * self.force_scaling * FORCE_BINARY_FULL_SCALE_FACTOR
                latest_force = self.latest_force_n
            else:
                latest_force = None

        self.force_value_var.set(self._format_force_value(latest_force))
        self.force_status_var.set(self._force_status("Force sensor: scaling applied"))
        self.status_var.set(f"Force scaling set to {force_scaling:.6g}")
        self._write_debug_log(f"FORCE scaling={force_scaling:.12g}")

    def _apply_force_average(self):
        try:
            average_seconds = float(self.force_average_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid force average", "Enter a numeric force average window in seconds.")
            return

        if average_seconds < 0.0:
            messagebox.showerror("Invalid force average", "Force average window must be zero or greater.")
            return

        with self.force_lock:
            self.force_average_seconds = average_seconds
            self.force_average_samples.clear()
            latest_force = self.latest_force_n

        self.force_value_var.set(self._format_force_value(latest_force))
        self.force_status_var.set(self._force_status("Force sensor: average applied"))
        self.status_var.set(f"Force average window set to {average_seconds:.3g} s")
        self._write_debug_log(f"FORCE average_seconds={average_seconds:.12g}")

    def _apply_force_impulse_threshold(self):
        try:
            threshold = float(self.force_impulse_threshold_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid force threshold", "Enter a numeric force impulse threshold.")
            return

        if threshold < 0.0:
            messagebox.showerror("Invalid force threshold", "Force impulse threshold must be zero or greater.")
            return

        with self.force_lock:
            self.force_impulse_threshold = threshold

        self.force_status_var.set(self._force_status("Force sensor: impulse threshold applied"))
        self.status_var.set(f"Force impulse threshold set to {threshold:.4g}")
        self._write_debug_log(f"FORCE impulse_threshold={threshold:.12g}")

    def _write_force_command_locked(self, command, *payload):
        port = self.force_serial_port
        if not port:
            raise RuntimeError("force sensor is disconnected")
        port.write(bytes((command, *payload)))
        port.flush()

    def _read_force_serial(self):
        buffer = bytearray()
        last_ui_update = 0.0
        last_logger_poll = 0.0
        while self.force_reader_running and self.force_serial_port:
            try:
                with self.force_serial_lock:
                    if not self.force_serial_port:
                        break
                    data = self.force_serial_port.read(64)
                    if not data:
                        now = time.time()
                        if now - last_logger_poll >= FORCE_LOGGER_POLL_INTERVAL_SECONDS:
                            self._write_force_command_locked(GSV_CMD_GET_VALUE)
                            last_logger_poll = now
            except (OSError, serial.SerialException, RuntimeError) as exc:
                self._write_debug_log(f"FORCE RX error {exc}")
                self.messages.put(("force_status", f"Force sensor serial error: {exc}"))
                break

            if not data:
                continue

            buffer.extend(data)
            values = self._extract_force_values(buffer)
            for force_reading, raw_reading in values:
                force_n, rate_hz = self._store_force_value(force_reading, raw_reading)
                now = time.time()
                if now - last_ui_update >= 0.05:
                    self.messages.put(("force_value", (force_n, rate_hz)))
                    last_ui_update = now

        self.force_reader_running = False

    def _store_force_value(self, force_reading, raw_reading=None):
        now = time.time()
        with self.force_lock:
            force_n = self._filtered_force_value_locked(now, force_reading)
            self.latest_force_raw_n = force_reading if raw_reading is None else raw_reading
            self.latest_force_n = force_n
            self.latest_force_time = now
            self.force_rate_times.append(now)
            while self.force_rate_times and now - self.force_rate_times[0] > FORCE_RATE_WINDOW_SECONDS:
                self.force_rate_times.popleft()
            return force_n, self._force_rate_hz_locked()

    def _filtered_force_value_locked(self, now, force_reading):
        self.force_average_samples.append((now, force_reading))
        if self.force_average_seconds <= 0.0:
            return force_reading

        cutoff = now - self.force_average_seconds
        while self.force_average_samples and self.force_average_samples[0][0] < cutoff:
            self.force_average_samples.popleft()
        if not self.force_average_samples:
            return force_reading

        return sum(value for _, value in self.force_average_samples) / len(self.force_average_samples)

    def _force_rate_hz_locked(self):
        if len(self.force_rate_times) < 2:
            return None
        elapsed = self.force_rate_times[-1] - self.force_rate_times[0]
        if elapsed <= 0:
            return None
        return (len(self.force_rate_times) - 1) / elapsed

    def _force_value_for_live_row(self):
        with self.force_lock:
            calibrated_force = self.latest_force_n
            raw_force = self.latest_force_raw_n
        return (
            "" if calibrated_force is None else f"{calibrated_force:.4f}",
            "" if raw_force is None else f"{raw_force:.4f}",
        )

    def _extract_force_values(self, buffer):
        values = []
        sync_byte = bytes([FORCE_BINARY_SYNC])
        while buffer:
            sync_index = buffer.find(sync_byte)
            if sync_index < 0:
                del buffer[:-FORCE_BINARY_FRAME_LENGTH + 1]
                break
            if sync_index > 0:
                del buffer[:sync_index]
            if len(buffer) < FORCE_BINARY_FRAME_LENGTH:
                break

            frame = bytes(buffer[:FORCE_BINARY_FRAME_LENGTH])
            if not self._is_force_binary_frame(frame):
                del buffer[0]
                continue

            del buffer[:FORCE_BINARY_FRAME_LENGTH]
            values.append(self._parse_force_binary_frame(frame))

        return values

    def _is_force_binary_frame(self, frame):
        if len(frame) != FORCE_BINARY_FRAME_LENGTH or frame[0] != FORCE_BINARY_SYNC:
            return False
        return (frame[1] & ~FORCE_BINARY_STATUS_MASK) == 0

    def _parse_force_binary_frame(self, frame):
        raw_value = (frame[2] << 16) | (frame[3] << 8) | frame[4]
        signed_fraction = (raw_value - FORCE_BINARY_BIPOLAR_ZERO) / FORCE_BINARY_POSITIVE_SPAN
        with self.force_lock:
            force_value = signed_fraction * self.force_scaling * FORCE_BINARY_FULL_SCALE_FACTOR
        return float(force_value), float(raw_value)

    def _toggle_colibri_connection(self):
        if self.colibri:
            self._disconnect_colibri()
        else:
            self._connect_colibri()

    def _connect_colibri(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial first:\npython -m pip install pyserial")
            return

        port = self._selected_colibri_port_device()
        if not port:
            messagebox.showerror("No port selected", "Select the Colibri serial port.")
            return
        if self.serial_port and port == self._selected_port_device():
            messagebox.showerror("Port already in use", "Select a separate serial port for the Colibri axis.")
            return
        if self.force_serial_port and port == self._selected_force_port_device():
            messagebox.showerror("Port already in use", "Select a separate serial port for the force sensor.")
            return

        try:
            self.colibri = ColibriController(port, debug_logger=self._write_debug_log if self.debug_log_file else None)
            snapshot = self._read_colibri_snapshot()
        except (OSError, serial.SerialException, TimeoutError, ColibriProtocolError, RuntimeError) as exc:
            if self.colibri:
                self.colibri.close()
            self.colibri = None
            messagebox.showerror("Colibri connection failed", str(exc))
            return

        self.colibri_connect_button.configure(text="Disconnect")
        self.colibri_port_combo.configure(state=tk.DISABLED)
        self._set_colibri_controls_enabled(True)
        self._handle_colibri_snapshot(snapshot, prefix=f"Connected to {port}")
        self._write_debug_log(f"COLIBRI connected port={port}")

    def _disconnect_colibri(self):
        if self.colibri:
            self._write_debug_log("COLIBRI disconnect")
            self.colibri.close()
        self.colibri = None
        self.colibri_busy = False
        self.colibri_enabled_var.set(False)
        self.colibri_connect_button.configure(text="Connect")
        self.colibri_port_combo.configure(state="readonly")
        self._set_colibri_controls_enabled(False)
        self.colibri_position_var.set("Position: --")
        self.last_colibri_position_mm = None
        self.colibri_status_var.set("Colibri: disconnected")

    def _set_colibri_controls_enabled(self, enabled):
        state = tk.NORMAL if enabled and self.colibri and not self.colibri_busy else tk.DISABLED
        for control in self.colibri_controls:
            control.configure(state=state)
        if self.colibri:
            self.colibri_stop_button.configure(state=tk.NORMAL)

    def _colibri_refresh_status(self):
        self._run_colibri_task("Read Colibri status", self._read_colibri_snapshot)

    def _apply_colibri_enable(self):
        requested_enabled = self.colibri_enabled_var.get()

        def task():
            if requested_enabled:
                self.colibri.set_remote()
                self.colibri.enable()
            else:
                self.colibri.disable()
            time.sleep(0.05)
            return self._read_colibri_snapshot()

        self._run_colibri_task("Set Colibri endstage", task)

    def _colibri_reference(self):
        if not messagebox.askyesno(
            "Start reference run",
            "Start the negative Colibri reference run now? Make sure the axis can move toward the negative endstop.",
        ):
            return

        def task():
            self.colibri.set_remote()
            self.colibri.enable()
            self.colibri.configure_negative_reference()
            self._write_debug_log(
                "COLIBRI configured reference type=2 (Drehueberwachung negativ), "
                f"reference_current={COLIBRI_REFERENCE_CURRENT_PERCENT}%"
            )
            self.colibri.reference()
            return self._wait_for_colibri_reference()

        self._run_colibri_task("Start Colibri negative reference run", task)

    def _colibri_set_zero_here(self):
        if not messagebox.askyesno(
            "Set zero here",
            "Set the current Colibri position as 0.0 mm / reference point?",
        ):
            return

        def task():
            self.colibri.set_remote()
            self.colibri.enable()
            self.colibri.set_current_position_as_reference()
            time.sleep(0.1)
            return self._read_colibri_snapshot()

        self._run_colibri_task("Set Colibri zero here", task)

    def _colibri_jog_forward(self):
        self._colibri_jog(direction=1)

    def _colibri_jog_reverse(self):
        self._colibri_jog(direction=-1)

    def _colibri_jog(self, direction):
        distance_mm = self._validated_float(self.colibri_distance_var, "Colibri distance", 0.005, COLIBRI_TRAVEL_MM)
        if distance_mm is None:
            return
        signed_steps = self._colibri_mm_to_steps(direction * distance_mm)

        def task():
            self.colibri.set_remote()
            self.colibri.enable()
            start_steps = self.colibri.position_steps()
            target_steps = start_steps + signed_steps
            self.colibri.move_relative_steps(signed_steps)
            return self._wait_for_colibri_move(target_steps)

        self._run_colibri_task(f"Colibri jog {direction * distance_mm:.3f} mm", task)

    def _colibri_move_absolute(self):
        position_mm = self._validated_float(
            self.colibri_absolute_var,
            "Colibri absolute position",
            -COLIBRI_TRAVEL_MM,
            COLIBRI_TRAVEL_MM,
        )
        if position_mm is None:
            return
        target_steps = self._colibri_mm_to_steps(position_mm)

        def task():
            self.colibri.set_remote()
            self.colibri.enable()
            self.colibri.move_absolute_steps(target_steps)
            return self._wait_for_colibri_move(target_steps)

        self._run_colibri_task(f"Colibri absolute move {position_mm:.3f} mm", task)

    def _colibri_stop(self):
        def task():
            self.colibri.stop()
            time.sleep(0.05)
            return self._read_colibri_snapshot()

        self._run_colibri_task("Stop Colibri", task, allow_while_busy=True)

    def _run_colibri_task(self, label, task, allow_while_busy=False):
        if not self.colibri:
            messagebox.showerror("Colibri disconnected", "Connect the Colibri axis first.")
            return
        if self.colibri_busy and not allow_while_busy:
            messagebox.showinfo("Colibri busy", "The Colibri axis is still processing the previous command.")
            return

        self.colibri_busy = True
        self._set_colibri_controls_enabled(True)
        self.colibri_status_var.set(f"Colibri: {label}...")
        self._write_debug_log(f"COLIBRI TASK start {label}")

        def worker():
            try:
                snapshot = task()
            except (OSError, serial.SerialException, TimeoutError, ColibriProtocolError) as exc:
                self._write_debug_log(f"COLIBRI TASK error {label}: {exc}")
                self.messages.put(("colibri_error", f"{label} failed: {exc}"))
            else:
                self._write_debug_log(f"COLIBRI TASK done {label}: {snapshot}")
                self.messages.put(("colibri_snapshot", (label, snapshot)))
            finally:
                self.messages.put(("colibri_done", None))

        threading.Thread(target=worker, daemon=True).start()

    def _read_colibri_snapshot(self):
        status = self.colibri.status()
        position_steps = self.colibri.position_steps()
        error = self.colibri.error()
        return {
            "status": status,
            "position_steps": position_steps,
            "position_mm": self._colibri_steps_to_mm(position_steps),
            "error": error,
        }

    def _wait_for_colibri_reference(self, timeout_seconds=30.0):
        deadline = time.time() + timeout_seconds
        last_snapshot = None
        time.sleep(0.2)

        while time.time() < deadline:
            snapshot = self._read_colibri_snapshot()
            last_snapshot = snapshot
            status = snapshot["status"]
            self._write_debug_log(
                "COLIBRI reference poll "
                f"position_mm={snapshot['position_mm']:.3f} moving={status['moving']} "
                f"referenced={status['referenced']} error_byte=0x{status['error_byte']:02X}"
            )

            if status["error_byte"]:
                raise ColibriProtocolError(
                    f"Reference failed with error_byte=0x{status['error_byte']:02X}"
                )
            if status["referenced"] and not status["moving"]:
                return snapshot
            if not status["moving"] and last_snapshot is not None and time.time() > deadline - timeout_seconds + 0.7:
                break
            time.sleep(0.2)

        if last_snapshot is None:
            raise TimeoutError("No reference status received from Colibri")
        if not last_snapshot["status"]["referenced"]:
            raise ColibriProtocolError("Reference run ended without referenced status bit")
        return last_snapshot

    def _wait_for_colibri_move(self, target_steps, timeout_seconds=20.0, tolerance_steps=5):
        deadline = time.time() + timeout_seconds
        last_snapshot = None
        stable_stopped_reads = 0
        last_position_steps = None
        time.sleep(0.1)

        while time.time() < deadline:
            snapshot = self._read_colibri_snapshot()
            last_snapshot = snapshot
            status = snapshot["status"]
            position_steps = snapshot["position_steps"]
            remaining_steps = target_steps - position_steps
            self._write_debug_log(
                "COLIBRI move poll "
                f"position_mm={snapshot['position_mm']:.3f} target_mm={self._colibri_steps_to_mm(target_steps):.3f} "
                f"remaining_steps={remaining_steps} moving={status['moving']} error_byte=0x{status['error_byte']:02X}"
            )

            if status["error_byte"]:
                raise ColibriProtocolError(
                    f"Move aborted at {snapshot['position_mm']:.3f} mm, "
                    f"target {self._colibri_steps_to_mm(target_steps):.3f} mm, "
                    f"error_byte=0x{status['error_byte']:02X}"
                )
            if abs(remaining_steps) <= tolerance_steps and not status["moving"]:
                return snapshot

            if not status["moving"] and position_steps == last_position_steps:
                stable_stopped_reads += 1
            else:
                stable_stopped_reads = 0
            if stable_stopped_reads >= 2:
                raise ColibriProtocolError(
                    f"Move stopped at {snapshot['position_mm']:.3f} mm, "
                    f"target {self._colibri_steps_to_mm(target_steps):.3f} mm"
                )

            last_position_steps = position_steps
            time.sleep(0.2)

        if last_snapshot is None:
            raise TimeoutError("No move status received from Colibri")
        raise TimeoutError(
            f"Move timed out at {last_snapshot['position_mm']:.3f} mm, "
            f"target {self._colibri_steps_to_mm(target_steps):.3f} mm"
        )

    def _handle_colibri_snapshot(self, snapshot, prefix=None):
        status = snapshot["status"]
        self.colibri_enabled_var.set(status["enabled"])
        self.last_colibri_position_mm = snapshot["position_mm"]
        self.colibri_position_var.set(f"Position: {snapshot['position_mm']:.3f} mm")
        status_parts = []
        if status["ready"]:
            status_parts.append("ready")
        if status["enabled"]:
            status_parts.append("enabled")
        if status["remote"]:
            status_parts.append("remote")
        if status["referenced"]:
            status_parts.append("referenced")
        if status["moving"]:
            status_parts.append("moving")
        if status["error_byte"]:
            status_parts.append(f"error 0x{status['error_byte']:02X}")
        if not status_parts:
            status_parts.append("no status bits")
        message = ", ".join(status_parts)
        self.colibri_status_var.set(f"Colibri: {message}")
        if prefix:
            self.status_var.set(f"{prefix} | Colibri {message}")

    def _colibri_mm_to_steps(self, position_mm):
        return round(position_mm * COLIBRI_STEPS_PER_MM)

    def _colibri_steps_to_mm(self, steps):
        return steps * COLIBRI_MM_PER_STEP

    def _validated_int(self, variable, label, minimum, maximum):
        try:
            value = int(variable.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid motor setting", f"Enter a numeric {label}.")
            return None

        value = min(max(value, minimum), maximum)
        variable.set(value)
        return value

    def _validated_float(self, variable, label, minimum, maximum):
        try:
            value = float(variable.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid motor setting", f"Enter a numeric {label}.")
            return None

        value = min(max(value, minimum), maximum)
        variable.set(round(value, 3))
        return value

    def _load_part_csv(self):
        if self.german_csv_format_var.get():
            messagebox.showerror(
                "English CSV required",
                "Part CSV files are English format. Uncheck German CSV format before loading a part CSV.",
            )
            return

        path = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        required_fields = {
            "Pose",
            "Hole",
            "Y-offset",
            "Z-offset",
            "Y-CapOffset",
            "Z-CapOffset",
        }
        loaded_rows = {}
        try:
            with open(path, newline="", encoding="utf-8-sig") as csv_file:
                reader = csv.DictReader(csv_file)
                if not reader.fieldnames or not required_fields.issubset(reader.fieldnames):
                    missing = sorted(required_fields - set(reader.fieldnames or []))
                    raise ValueError(f"Missing columns: {', '.join(missing)}")

                for row in reader:
                    pose = row["Pose"].strip()
                    hole = row["Hole"].strip()
                    if not pose or not hole:
                        continue
                    loaded_rows[(pose, hole)] = {
                        "y": float(row["Y-offset"]),
                        "z": float(row["Z-offset"]),
                        "y_cap": float(row["Y-CapOffset"]),
                        "z_cap": float(row["Z-CapOffset"]),
                    }
        except (OSError, ValueError, KeyError) as exc:
            messagebox.showerror("Part CSV failed", str(exc))
            return

        if not loaded_rows:
            messagebox.showerror("Part CSV failed", "No pose/hole rows were found.")
            return

        self.part_rows = loaded_rows
        self.part_csv_status_var.set(f"Loaded {Path(path).name} ({len(loaded_rows)} rows)")
        poses = self._sorted_part_values({pose for pose, _hole in loaded_rows})
        self.part_pose_combo.configure(state="readonly", values=poses)
        self.part_pose_var.set(poses[0])
        self._refresh_part_holes()
        self.status_var.set(f"Loaded part CSV: {Path(path).name}")

    def _sorted_part_values(self, values):
        return sorted(values, key=lambda value: (float(value), value))

    def _part_pose_selected(self, _event=None):
        self._refresh_part_holes()

    def _part_hole_selected(self, _event=None):
        self._update_part_position_preview()

    def _part_input_changed(self, *_args):
        if hasattr(self, "part_y_offset_var"):
            self._update_part_position_preview()

    def _refresh_part_holes(self):
        pose = self.part_pose_var.get()
        holes = self._sorted_part_values({hole for row_pose, hole in self.part_rows if row_pose == pose})
        self.part_hole_combo.configure(state="readonly" if holes else tk.DISABLED, values=holes)
        if holes and self.part_hole_var.get() not in holes:
            self.part_hole_var.set(holes[0])
        elif not holes:
            self.part_hole_var.set("")
        self._update_part_position_preview()

    def _selected_part_offsets(self):
        row = self.part_rows.get((self.part_pose_var.get(), self.part_hole_var.get()))
        if row is None:
            return None
        if self.use_cap_offsets_var.get():
            return row["y_cap"], row["z_cap"]
        return row["y"], row["z"]

    def _part_float_value(self, variable):
        try:
            return float(variable.get())
        except (tk.TclError, ValueError):
            return None

    def _part_position_values(self):
        offsets = self._selected_part_offsets()
        nozzle_offset = self._part_float_value(self.nozzle_offset_var)
        test_stand_height = self._part_float_value(self.test_stand_height_var)
        holder_height = self._part_float_value(self.holder_height_var)
        if offsets is None or nozzle_offset is None or test_stand_height is None or holder_height is None:
            return None

        y_offset, z_offset = offsets
        stepper_position = nozzle_offset + y_offset
        colibri_position = test_stand_height - holder_height + z_offset
        return y_offset, z_offset, stepper_position, colibri_position

    def _current_pose_hole_for_csv(self):
        pose = self.part_pose_var.get().strip()
        hole = self.part_hole_var.get().strip()
        if self.part_rows and (pose, hole) in self.part_rows:
            return pose, hole
        return -1, -1

    def _current_readback_offsets_for_csv(self):
        stepper_y_offset = -1
        colibri_z_offset = -1

        nozzle_offset = self._part_float_value(self.nozzle_offset_var)
        if self.motor_enabled_var.get() and self.last_motor_position_mm is not None and nozzle_offset is not None:
            stepper_y_offset = self.last_motor_position_mm - nozzle_offset

        test_stand_height = self._part_float_value(self.test_stand_height_var)
        holder_height = self._part_float_value(self.holder_height_var)
        if (
            self.colibri_enabled_var.get()
            and self.last_colibri_position_mm is not None
            and test_stand_height is not None
            and holder_height is not None
        ):
            colibri_z_offset = self.last_colibri_position_mm - test_stand_height + holder_height

        return stepper_y_offset, colibri_z_offset

    def _update_part_position_preview(self):
        values = self._part_position_values()
        if values is None:
            self.part_y_offset_var.set("Y offset: --")
            self.part_z_offset_var.set("Z offset: --")
            self.part_stepper_position_var.set("Stepper target: --")
            self.part_colibri_position_var.set("Colibri target: --")
            return

        y_offset, z_offset, stepper_position, colibri_position = values
        self.part_y_offset_var.set(f"Y offset: {y_offset:.3f} mm")
        self.part_z_offset_var.set(f"Z offset: {z_offset:.3f} mm")
        self.part_stepper_position_var.set(f"Stepper target: {stepper_position:.3f} mm")
        self.part_colibri_position_var.set(f"Colibri target: {colibri_position:.3f} mm")

    def _set_part_axis_targets(self):
        values = self._part_position_values()
        if values is None:
            messagebox.showerror("No part target", "Load a part CSV and select a pose/hole first.")
            return

        _y_offset, _z_offset, stepper_position, colibri_position = values
        self.motor_absolute_var.set(round(stepper_position, 3))
        self.colibri_absolute_var.set(round(colibri_position, 3))
        self.status_var.set(
            f"Part targets set: stepper {stepper_position:.3f} mm, Colibri {colibri_position:.3f} mm"
        )

    def _mm_to_steps(self, distance_mm):
        return max(1, round(distance_mm * MOTOR_STEPS_PER_MM))

    def _steps_to_mm(self, steps):
        return steps * MOTOR_MM_PER_STEP

    def _mm_per_second_to_steps_per_second(self, speed_mm_s):
        return min(max(1, round(speed_mm_s * MOTOR_STEPS_PER_MM)), MAX_MOTOR_STEPS_PER_SECOND)

    def _steps_per_second_to_mm_per_second(self, speed_steps_s):
        return speed_steps_s * MOTOR_MM_PER_STEP

    def _send(self, command, flush_live_backlog=False):
        if not self.serial_port or not self.writer_running:
            return False
        if flush_live_backlog:
            self._clear_pending_live_messages()
        self.commands.put(command)
        self.status_var.set(f"Queued {command}")
        return True

    def _clear_pending_commands(self):
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break

    def _write_serial(self):
        while self.writer_running and self.serial_port:
            try:
                command = self.commands.get(timeout=0.1)
            except queue.Empty:
                continue

            if command is None:
                break

            try:
                self._write_debug_log(f"ARDUINO TX {command}")
                self.serial_port.write(f"{command}\n".encode("ascii"))
                self.serial_port.flush()
            except serial.SerialTimeoutException:
                self._write_debug_log(f"ARDUINO TX timeout {command}")
                self.messages.put(("status", f"Serial write timed out while sending {command}"))
            except serial.SerialException as exc:
                self._write_debug_log(f"ARDUINO TX error {exc}")
                self.messages.put(("status", f"Serial write failed: {exc}"))
                break
            else:
                self.messages.put(("status", f"Sent {command}"))
                time.sleep(SERIAL_COMMAND_SPACING_SECONDS)

        self.writer_running = False

    def _clear_pending_live_messages(self):
        kept_messages = []
        while True:
            try:
                message = self.messages.get_nowait()
            except queue.Empty:
                break
            kind, value = message
            if kind != "line" or not self._is_live_data_line(value):
                kept_messages.append(message)

        for message in kept_messages:
            self.messages.put(message)

        if self.serial_port:
            try:
                self.serial_port.reset_input_buffer()
            except serial.SerialException:
                pass

    def _read_serial(self):
        while self.reader_running and self.serial_port:
            try:
                line = self.serial_port.readline().decode("utf-8", errors="replace").strip()
            except serial.SerialException as exc:
                self._write_debug_log(f"ARDUINO RX error {exc}")
                self.messages.put(("status", f"Serial error: {exc}"))
                break
            if line:
                self._write_debug_log(f"ARDUINO RX {line}")
                self.messages.put(("line", line))

    def _drain_messages(self):
        drained_count = 0
        while drained_count < MAX_QUEUE_DRAIN_PER_TICK:
            try:
                kind, value = self.messages.get_nowait()
            except queue.Empty:
                break
            drained_count += 1
            if kind == "status":
                self._write_debug_log(f"GUI STATUS {value}")
                self.status_var.set(value)
            elif kind == "ethercat_connected":
                self._handle_ethercat_connected(value)
            elif kind == "ethercat_error":
                self._handle_ethercat_error(value)
            elif kind == "colibri_snapshot":
                label, snapshot = value
                self._write_debug_log(f"GUI COLIBRI snapshot {label}: {snapshot}")
                self._handle_colibri_snapshot(snapshot, prefix=label)
            elif kind == "colibri_error":
                self._write_debug_log(f"GUI COLIBRI error {value}")
                self.colibri_status_var.set(f"Colibri: {value}")
                self.status_var.set(value)
            elif kind == "colibri_done":
                self.colibri_busy = False
                self._set_colibri_controls_enabled(True)
            elif kind == "force_value":
                if isinstance(value, tuple):
                    force_n, rate_hz = value
                else:
                    force_n, rate_hz = value, None
                self.force_value_var.set(self._format_force_value(force_n))
                if rate_hz is not None:
                    self.force_rate_var.set(f"Force rate: {rate_hz:.0f} Hz")
            elif kind == "force_status":
                self.force_status_var.set(value)
                self.status_var.set(value)
            else:
                self._handle_line(value)
        if drained_count:
            self._scroll_log_to_end()
        self.after(1 if drained_count == MAX_QUEUE_DRAIN_PER_TICK else 50, self._drain_messages)

    def _handle_line(self, line):
        self._append_log_line(line)

        parts = line.split(";")
        if parts[0] == "MODE":
            self._handle_mode_line(parts)
            return

        if parts[0] == "STOPPED":
            self.mode_var.set("Mode: idle")
            self.active_test_mask = ""
            return

        if parts[0] == "PULSE":
            self._handle_pulse_line(parts)
            return

        if parts[0] == "MOTOR":
            self._handle_motor_line(parts)
            return

        if parts[0] == "FLOW_THRESHOLD":
            self._handle_flow_threshold_line(parts)
            return

        if not parts or not parts[0].isdigit():
            return

        if len(parts) == 5:
            parts = [
                parts[0],
                "",
                parts[1],
                parts[2],
                "",
                parts[3],
                parts[4],
            ]
        elif len(parts) != 7:
            return

        parts.extend(self._force_value_for_live_row())
        self._update_impulse_capture(parts)
        self.rows.append(parts)
        self.table.insert("", tk.END, values=parts)
        children = self.table.get_children()
        if len(children) > 250:
            self.table.delete(children[0])
        self._scroll_table_to_end()

    def _reset_impulse_capture(self, clear_saved=False):
        self.current_impulse = None
        self.last_valves_open = False
        self.pending_flip_angle = -1
        self.pending_pulse_mask = ""
        self.pending_pulse_duration_ms = None
        if clear_saved:
            self.impulse_rows.clear()
            self.active_test_mask = ""

    def _update_impulse_capture(self, parts):
        sample = self._parse_live_sample(parts)
        if sample is None:
            return

        valves_open = sample["valves_open"]
        if valves_open and not self.last_valves_open:
            if self.current_impulse:
                previous_close_ms = self.current_impulse.get("valve_close_time_ms")
                capture_end_ms = sample["time_ms"]
                previous_flow_end_ms = self._flow_capture_end_time_ms(self.current_impulse)
                if previous_flow_end_ms is not None:
                    capture_end_ms = min(capture_end_ms, previous_flow_end_ms)
                self._finalize_impulse(capture_end_ms)
            self._begin_impulse(sample)

        if self.current_impulse:
            if valves_open:
                self._add_pressure_sample(sample)
                self._add_flow_sample(sample)
            else:
                close_time_ms = self.current_impulse.get("valve_close_time_ms")
                if self.last_valves_open:
                    self.current_impulse["valve_close_time_ms"] = sample["time_ms"]
                    self._add_flow_sample(sample)
                elif close_time_ms is not None:
                    capture_end_ms = self._flow_capture_end_time_ms(self.current_impulse)
                    if sample["time_ms"] <= capture_end_ms:
                        self._add_flow_sample(sample)
                    else:
                        self._finalize_impulse(capture_end_ms)

        self.last_valves_open = valves_open

    def _parse_live_sample(self, parts):
        try:
            return {
                "time_ms": float(parts[0]),
                "target_pressure": self._optional_float(parts[1]),
                "pressure_before": self._optional_float(parts[2]),
                "regulator_feedback": self._optional_float(parts[3]),
                "regulator_pwm": self._optional_float(parts[4]),
                "valves_open": parts[5] not in ("0", "FALSE", "False", "false", ""),
                "flow": self._optional_float(parts[6]),
                "force": self._optional_float(parts[7]) if len(parts) > 7 else None,
                "raw_force": self._optional_float(parts[8]) if len(parts) > 8 else None,
            }
        except ValueError:
            return None

    def _optional_float(self, value):
        if value == "":
            return None
        return float(value)

    def _flow_capture_end_time_ms(self, impulse):
        arduino_duration_ms = impulse.get("arduino_valve_open_duration_ms")
        if arduino_duration_ms is not None:
            return impulse["start_time_ms"] + arduino_duration_ms + FLOW_DELAY_CAPTURE_MS

        close_time_ms = impulse.get("valve_close_time_ms")
        if close_time_ms is None:
            return None
        return close_time_ms + FLOW_DELAY_CAPTURE_MS

    def _begin_impulse(self, sample):
        valve_mask = self.pending_pulse_mask
        if not valve_mask and self.mode_var.get().startswith("Mode: test sequence"):
            valve_mask = self.active_test_mask
        pose_number, hole_number = self._current_pose_hole_for_csv()
        read_stepper_y_offset, read_colibri_z_offset = self._current_readback_offsets_for_csv()

        self.current_impulse = {
            "impulse_index": len(self.impulse_rows) + 1,
            "flip_angle": self.pending_flip_angle,
            "pose_number": pose_number,
            "hole_number": hole_number,
            "read_stepper_y_offset": read_stepper_y_offset,
            "read_colibri_z_offset": read_colibri_z_offset,
            "valve_mask": valve_mask,
            "arduino_valve_open_duration_ms": self.pending_pulse_duration_ms,
            "start_time_ms": sample["time_ms"],
            "valve_close_time_ms": None,
            "capture_end_time_ms": sample["time_ms"],
            "target_pressure": sample["target_pressure"],
            "valve_open_sample_count": 0,
            "pressure_skipped_count": 0,
            "pressure_sum": 0.0,
            "pressure_count": 0,
            "regulator_pressure_sum": 0.0,
            "regulator_pressure_count": 0,
            "force_sum": 0.0,
            "force_count": 0,
            "max_flow": None,
            "flow_sample_count": 0,
            "last_flow_time_ms": None,
            "last_flow_l_min": None,
            "volume_l": 0.0,
        }
        self.pending_flip_angle = -1
        self.pending_pulse_mask = ""
        self.pending_pulse_duration_ms = None

    def _add_pressure_sample(self, sample):
        self.current_impulse["valve_open_sample_count"] += 1
        skip_pressure_sample = self.current_impulse["valve_open_sample_count"] <= PRESSURE_SETTLE_SKIP_SAMPLES
        if skip_pressure_sample:
            self.current_impulse["pressure_skipped_count"] += 1
        else:
            pressure = sample["pressure_before"]
            if pressure is not None:
                self.current_impulse["pressure_sum"] += pressure
                self.current_impulse["pressure_count"] += 1

            regulator_pressure = sample["regulator_feedback"]
            if regulator_pressure is not None:
                self.current_impulse["regulator_pressure_sum"] += regulator_pressure
                self.current_impulse["regulator_pressure_count"] += 1

        force = sample["force"]
        if force is not None and abs(force) >= self.force_impulse_threshold:
            self.current_impulse["force_sum"] += force
            self.current_impulse["force_count"] += 1

    def _add_flow_sample(self, sample):
        flow = sample["flow"]
        time_ms = sample["time_ms"]
        if flow is None:
            return

        max_flow = self.current_impulse["max_flow"]
        self.current_impulse["max_flow"] = flow if max_flow is None else max(max_flow, flow)
        self.current_impulse["flow_sample_count"] += 1
        self.current_impulse["volume_l"] += flow * (SAMPLE_INTERVAL_MS / 60000.0)
        self.current_impulse["last_flow_time_ms"] = time_ms
        self.current_impulse["last_flow_l_min"] = flow
        self.current_impulse["capture_end_time_ms"] = time_ms

    def _finalize_impulse(self, capture_end_time_ms=None):
        if not self.current_impulse:
            return

        impulse = self.current_impulse
        if capture_end_time_ms is not None:
            impulse["capture_end_time_ms"] = capture_end_time_ms

        pressure_count = impulse["pressure_count"]
        avg_pressure = impulse["pressure_sum"] / pressure_count if pressure_count else ""
        regulator_pressure_count = impulse["regulator_pressure_count"]
        avg_regulator_pressure = (
            impulse["regulator_pressure_sum"] / regulator_pressure_count
            if regulator_pressure_count
            else ""
        )
        force_count = impulse["force_count"]
        avg_force = impulse["force_sum"] / force_count if force_count else ""
        valve_close_time_ms = impulse["valve_close_time_ms"]
        if valve_close_time_ms is None:
            valve_close_time_ms = impulse["capture_end_time_ms"]
        valve_open_duration_ms = impulse["arduino_valve_open_duration_ms"]
        if valve_open_duration_ms is None:
            valve_open_duration_ms = VALVE_PULSE_DURATION_MS

        row = [
            impulse["impulse_index"],
            impulse["flip_angle"],
            impulse["pose_number"],
            impulse["hole_number"],
            self._format_readback_offset(impulse["read_stepper_y_offset"]),
            self._format_readback_offset(impulse["read_colibri_z_offset"]),
            self._format_csv_number(impulse["start_time_ms"]),
            self._format_csv_number(valve_close_time_ms),
            self._format_csv_number(impulse["capture_end_time_ms"]),
            self._format_csv_number(valve_open_duration_ms),
            self._format_csv_number(impulse["target_pressure"]),
            self._format_csv_number(avg_regulator_pressure),
            self._format_csv_number(avg_pressure),
            self._format_csv_number(avg_force, digits=4),
            self._format_csv_number(impulse["max_flow"]),
            self._format_csv_number(impulse["volume_l"], digits=6),
            *self._nozzle_mask_flags(impulse["valve_mask"]),
            impulse["valve_open_sample_count"],
            pressure_count,
            impulse["flow_sample_count"],
        ]
        self.impulse_rows.append(row)
        self.current_impulse = None

    def _nozzle_mask_flags(self, mask):
        try:
            mask_value = int(float(mask))
        except (TypeError, ValueError):
            return ["", "", "", ""]

        return [1 if mask_value & (1 << index) else 0 for index in range(4)]

    def _format_csv_number(self, value, digits=3):
        if value is None or value == "":
            return ""
        return f"{float(value):.{digits}f}"

    def _format_readback_offset(self, value):
        if value == -1:
            return -1
        return self._format_csv_number(value)

    def _append_log_line(self, line):
        self.log.insert(tk.END, line + "\n")
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.log.delete("1.0", f"{line_count - MAX_LOG_LINES + 1}.0")
        self._scroll_log_to_end()

    def _clear_log(self):
        self._clear_run_display()

    def _clear_run_display(self):
        self.rows.clear()
        self._reset_impulse_capture(clear_saved=True)
        for item in self.table.get_children():
            self.table.delete(item)
        self.log.delete("1.0", tk.END)
        self._clear_pending_live_messages()

    def _scroll_log_to_end(self):
        self.log.mark_set(tk.INSERT, tk.END)
        self.log.see(tk.INSERT)
        self.log.yview_moveto(1.0)

    def _scroll_table_to_end(self):
        children = self.table.get_children()
        if not children:
            return
        self.table.see(children[-1])
        self.table.yview_moveto(1.0)

    def _is_live_data_line(self, line):
        parts = line.split(";")
        return bool(parts and parts[0].isdigit())

    def _handle_mode_line(self, parts):
        if len(parts) >= 2 and parts[1] == "TEST":
            self.mode_var.set("Mode: test sequence")
            return

        if len(parts) >= 6 and parts[1] == "MANUAL":
            setpoint = parts[3]
            pwm = parts[5]
            self.mode_var.set(f"Mode: manual pressure | target {setpoint} bar | PWM {pwm}")
            self.status_var.set(f"Arduino applied manual pressure: {setpoint} bar, PWM {pwm}")

    def _handle_flow_threshold_line(self, parts):
        if len(parts) >= 3 and parts[1] == "SET":
            self.status_var.set(f"Flow Detection Threshold set to {parts[2]} l/min")

    def _handle_pulse_line(self, parts):
        if len(parts) >= 3 and parts[1] == "ERROR":
            self.pulse_in_progress = False
            self.pending_increment_direction = 0
            self.pending_flip_angle = -1
            self.pending_pulse_mask = ""
            self._set_pulse_buttons_enabled(True)
            self.status_var.set(";".join(parts))
            return

        if len(parts) >= 3 and parts[1] == "START":
            self.pending_pulse_mask = parts[2]
            if self.current_impulse and not self.current_impulse.get("valve_mask"):
                self.current_impulse["valve_mask"] = parts[2]
            self.mode_var.set(f"Mode: manual pulse running | mask {parts[2]}")
            return

        if len(parts) >= 3 and parts[1] == "FLOW_DONE":
            self._handle_flow_done_line(parts)
            return

        if len(parts) >= 3 and parts[1] == "DONE":
            pulse_duration_ms = self._pulse_duration_ms(parts)
            self._set_completed_pulse_duration(pulse_duration_ms)

            if not self.pulse_in_progress:
                return

            self.pulse_in_progress = False
            completed_increment_direction = self.pending_increment_direction
            self.pending_increment_direction = 0
            self._set_pulse_buttons_enabled(True)
            self.status_var.set(f"Pulse complete, mask {parts[2]}")

            if completed_increment_direction:
                self._advance_increment_target(completed_increment_direction)
            else:
                self.mode_var.set("Mode: manual pressure")

            self.after(0, lambda valve_mask=parts[2]: self._show_completed_pulse_flip_angle_prompt(valve_mask))

    def _handle_flow_done_line(self, parts):
        flow_sample_count = self._pulse_field_float(parts, "SAMPLES")
        max_flow = self._pulse_field_float(parts, "MAX_FLOW")
        volume_l = self._pulse_field_float(parts, "VOLUME_L")

        if self.current_impulse:
            if flow_sample_count is not None:
                self.current_impulse["flow_sample_count"] = int(round(flow_sample_count))
            if max_flow is not None:
                self.current_impulse["max_flow"] = max_flow
            if volume_l is not None:
                self.current_impulse["volume_l"] = volume_l
            self._finalize_impulse(self.current_impulse["capture_end_time_ms"])
            return

        if self.impulse_rows:
            if max_flow is not None:
                self.impulse_rows[-1][14] = self._format_csv_number(max_flow)
            if volume_l is not None:
                self.impulse_rows[-1][15] = self._format_csv_number(volume_l, digits=6)
            if flow_sample_count is not None:
                self.impulse_rows[-1][22] = int(round(flow_sample_count))

    def _pulse_field_float(self, parts, field_name):
        if field_name not in parts:
            return None
        try:
            return float(parts[parts.index(field_name) + 1])
        except (ValueError, IndexError):
            return None

    def _pulse_duration_ms(self, parts):
        if "DURATION_US" in parts:
            try:
                duration_index = parts.index("DURATION_US") + 1
                return float(parts[duration_index]) / 1000.0
            except (ValueError, IndexError):
                return None

        if "DURATION_MS" in parts:
            try:
                duration_index = parts.index("DURATION_MS") + 1
                return float(parts[duration_index])
            except (ValueError, IndexError):
                return None

        return None

    def _set_completed_pulse_duration(self, duration_ms):
        if duration_ms is None:
            return

        if self.current_impulse:
            self.current_impulse["arduino_valve_open_duration_ms"] = duration_ms
            return

        if self.impulse_rows:
            self.impulse_rows[-1][9] = self._format_csv_number(duration_ms)
            return

        self.pending_pulse_duration_ms = duration_ms

    def _show_completed_pulse_flip_angle_prompt(self, valve_mask):
        self._show_flip_angle_prompt(
            lambda flip_angle: self._set_completed_pulse_flip_angle(flip_angle, valve_mask)
        )

    def _set_completed_pulse_flip_angle(self, flip_angle, valve_mask):
        if self.current_impulse:
            self.current_impulse["flip_angle"] = flip_angle
            if not self.current_impulse.get("valve_mask"):
                self.current_impulse["valve_mask"] = valve_mask
            return

        if self.impulse_rows:
            self.impulse_rows[-1][1] = flip_angle
            if self.impulse_rows[-1][16] == "":
                self.impulse_rows[-1][16:20] = self._nozzle_mask_flags(valve_mask)
            return

        self.pending_flip_angle = flip_angle
        self.pending_pulse_mask = valve_mask

    def _handle_motor_line(self, parts):
        if len(parts) >= 3 and parts[1] == "ENABLED":
            self.motor_enabled_var.set(parts[2] not in ("0", "FALSE"))
            self.status_var.set(f"Stepper enabled: {self.motor_enabled_var.get()}")
            return

        if len(parts) >= 3 and parts[1] == "SPEED":
            try:
                speed_steps_s = int(float(parts[2]))
            except ValueError:
                self.status_var.set(";".join(parts))
                return
            self.last_motor_speed_steps_s = speed_steps_s
            speed_mm_s = self._steps_per_second_to_mm_per_second(speed_steps_s)
            self.status_var.set(f"Stepper speed applied: {speed_mm_s:.3f} mm/s")
            return

        if "POS" in parts:
            self._handle_motor_position_line(parts)
            return

        if len(parts) >= 5 and parts[1] == "MOVE":
            try:
                move_steps = int(float(parts[2]))
                speed_steps_s = int(float(parts[4]))
            except ValueError:
                self.status_var.set(";".join(parts))
                return
            move_mm = self._steps_to_mm(move_steps)
            speed_mm_s = self._steps_per_second_to_mm_per_second(speed_steps_s)
            self.mode_var.set(f"Mode: stepper moving | {move_mm:.3f} mm at {speed_mm_s:.3f} mm/s")
            return

        if len(parts) >= 2 and parts[1] == "DONE":
            self.mode_var.set("Mode: stepper done")
            self.status_var.set("Stepper move complete")
            return

        if len(parts) >= 2 and parts[1] == "STOPPED":
            self.mode_var.set("Mode: stepper stopped")
            self.status_var.set("Stepper stopped")
            return

        if len(parts) >= 3 and parts[1] == "ERROR":
            self.status_var.set(";".join(parts))

    def _handle_motor_position_line(self, parts):
        try:
            pos_index = parts.index("POS")
            mm_index = parts.index("MM")
            ref_index = parts.index("REF")
            position_steps = int(float(parts[pos_index + 1]))
            position_mm = float(parts[mm_index + 1])
            referenced = parts[ref_index + 1] not in ("0", "FALSE")
        except (ValueError, IndexError):
            self.status_var.set(";".join(parts))
            return

        self.last_motor_position_mm = position_mm
        self.motor_position_var.set(
            f"Stepper position: {position_mm:.3f} mm ({position_steps} steps) {'ref' if referenced else 'unref'}"
        )

        event = parts[1] if len(parts) > 1 else "POSITION"
        if event == "ZERO":
            self.mode_var.set("Mode: stepper zero set")
            self.status_var.set("Stepper zero reference set")
        elif event == "HOME_DONE":
            self.mode_var.set("Mode: stepper homed")
            self.status_var.set("Stepper homed at limit switch")
        elif event in ("LIMIT", "LIMIT_STOP"):
            self.mode_var.set("Mode: stepper limit switch")
            self.status_var.set("Stepper limit switch active; zero reference set")
        elif event == "DONE":
            self.mode_var.set("Mode: stepper done")
            self.status_var.set(f"Stepper move complete at {position_mm:.3f} mm")
        elif event == "POSITION":
            self.status_var.set(f"Stepper position: {position_mm:.3f} mm")

    def _advance_increment_target(self, direction):
        current_pressure = self._validated_pressure(self.target_pressure_var, "target pressure")
        pressure_increment = self._validated_pressure(self.pressure_increment_var, "pressure increment")
        if current_pressure is None or pressure_increment is None:
            return

        next_count = self.increment_count_var.get() + direction
        next_target = min(
            max(current_pressure + direction * pressure_increment, 0.0),
            REGULATOR_MAX_PRESSURE_BAR,
        )
        self.increment_count_var.set(next_count)
        self.target_pressure_var.set(round(next_target, 3))
        self._apply_pressure_settings()

    def _save_impulse_csv(self):
        self._finalize_stale_impulse_before_save()
        if not self.impulse_rows:
            messagebox.showinfo("Nothing to save", "No impulses have been captured yet.")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        edited_path = Path(path)
        raw_path = self._raw_csv_path(edited_path)
        try:
            self._write_impulse_csv(edited_path)
            self._write_raw_csv(raw_path)
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))
            return

        self.status_var.set(f"Saved CSV and raw CSV: {edited_path.name}, {raw_path.name}")

    def _raw_csv_path(self, edited_path):
        if edited_path.suffix:
            return edited_path.with_name(f"{edited_path.stem}_raw{edited_path.suffix}")
        return edited_path.with_name(f"{edited_path.name}_raw.csv")

    def _write_impulse_csv(self, path):
        with open(path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file, delimiter=self._csv_delimiter())
            writer.writerow([
                "impulse index",
                "flip angle",
                "pose number",
                "hole number",
                "read stepper y offset",
                "read colibri z offset",
                "start time ms",
                "valve close time ms",
                "capture end time ms",
                "arduino valve open duration ms",
                "target regulator pressure",
                "average actual regulator pressure",
                "average pressure before valve",
                "average force reading",
                "maximum flow",
                "volume l",
                "nozzle 1 used",
                "nozzle 2 used",
                "nozzle 3 used",
                "nozzle 4 used",
                "valve open sample count",
                "pressure sample count",
                "flow sample count",
            ])
            writer.writerows(self._csv_output_row(row) for row in self.impulse_rows)

    def _finalize_stale_impulse_before_save(self):
        if not self.current_impulse:
            return

        capture_end_time_ms = self._flow_capture_end_time_ms(self.current_impulse)
        if capture_end_time_ms is not None:
            self._finalize_impulse(capture_end_time_ms)

    def _write_raw_csv(self, path):
        with open(path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file, delimiter=self._csv_delimiter())
            writer.writerow([
                "time",
                "target regulator pressure",
                "pressure before valve",
                "actual regulator pressure",
                "regulator pwm",
                "valves open",
                "flow",
                "force reading",
                "raw force reading",
            ])
            writer.writerows(self._csv_output_row(row) for row in self.rows)

    def _csv_delimiter(self):
        return ";" if self.german_csv_format_var.get() else ","

    def _csv_output_row(self, row):
        return [self._csv_output_cell(value) for value in row]

    def _csv_output_cell(self, value):
        if value is None:
            return ""
        text = str(value)
        if not self.german_csv_format_var.get():
            return text
        try:
            float(text)
        except ValueError:
            return text
        return text.replace(".", ",")

    def destroy(self):
        self.closing = True
        self._disconnect_ethercat()
        self._disconnect_force_sensor()
        self._disconnect_colibri()
        self._disconnect()
        if self.debug_log_file:
            self._stop_debug_log()
        super().destroy()


if __name__ == "__main__":
    app = TestRunGui()
    app.mainloop()
