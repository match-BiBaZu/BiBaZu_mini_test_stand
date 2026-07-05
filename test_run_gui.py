import csv
import datetime as dt
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_RATE = 230400
SERIAL_WRITE_TIMEOUT = 1.0
SERIAL_COMMAND_SPACING_SECONDS = 0.08
MAX_LOG_LINES = 400
MAX_QUEUE_DRAIN_PER_TICK = 250
MOTOR_MM_PER_STEP = 0.009985846
MOTOR_STEPS_PER_MM = 1.0 / MOTOR_MM_PER_STEP
MAX_MOTOR_STEPS_PER_SECOND = 5000
COLIBRI_BAUD_RATE = 9600
COLIBRI_SLAVE_ADDRESS = 0xFF
COLIBRI_MM_PER_STEP = 0.005
COLIBRI_STEPS_PER_MM = 1.0 / COLIBRI_MM_PER_STEP
COLIBRI_TRAVEL_MM = 75.0
COLIBRI_REFERENCE_CURRENT_PERCENT = 20


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

        self.serial_port = None
        self.reader_thread = None
        self.writer_thread = None
        self.reader_running = False
        self.writer_running = False
        self.rows = []
        self.messages = queue.Queue()
        self.commands = queue.Queue()
        self.port_devices = {}
        self.colibri = None
        self.colibri_busy = False
        self.debug_log_file = None
        self.debug_log_path = None
        self.debug_log_lock = threading.Lock()

        self.port_var = tk.StringVar()
        self.colibri_port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.mode_var = tk.StringVar(value="Mode: disconnected")
        self.colibri_status_var = tk.StringVar(value="Colibri: disconnected")
        self.colibri_position_var = tk.StringVar(value="Position: --")
        self.debug_log_var = tk.StringVar(value="Debug log: off")
        self.target_pressure_var = tk.DoubleVar(value=0.20)
        self.starting_pressure_var = tk.DoubleVar(value=0.20)
        self.pressure_increment_var = tk.DoubleVar(value=0.05)
        self.increment_count_var = tk.IntVar(value=0)
        self.stream_var = tk.BooleanVar(value=True)
        self.motor_enabled_var = tk.BooleanVar(value=False)
        self.motor_distance_var = tk.DoubleVar(value=10.0)
        self.motor_speed_var = tk.DoubleVar(value=5.0)
        self.colibri_enabled_var = tk.BooleanVar(value=False)
        self.colibri_distance_var = tk.DoubleVar(value=1.0)
        self.colibri_absolute_var = tk.DoubleVar(value=0.0)
        self.nozzle_vars = [tk.BooleanVar(value=True) for _ in range(4)]
        self.nozzle_checkbuttons = []
        self.motor_controls = []
        self.colibri_controls = []
        self.last_motor_speed_steps_s = None
        self.pulse_in_progress = False
        self.pending_increment_direction = 0

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._drain_messages)

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
        self.stop_button = ttk.Button(controls, text="Stop", command=self._stop_test, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Save CSV", command=self._save_csv).pack(side=tk.RIGHT)
        self.debug_log_button = ttk.Button(controls, text="Start debug log", command=self._toggle_debug_log)
        self.debug_log_button.pack(side=tk.RIGHT, padx=(0, 8))

        pressure_controls = ttk.Frame(root)
        pressure_controls.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(pressure_controls, text="Target pressure").pack(side=tk.LEFT)
        self.target_pressure_spinbox = ttk.Spinbox(
            pressure_controls,
            from_=0.0,
            to=5.0,
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
            to=5.0,
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
            to=5.0,
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

        ttk.Label(motor_controls, text="Distance").pack(side=tk.LEFT, padx=(18, 0))
        self.motor_distance_spinbox = ttk.Spinbox(
            motor_controls,
            from_=0.01,
            to=2000.0,
            increment=1.0,
            textvariable=self.motor_distance_var,
            width=8,
            state=tk.DISABLED,
        )
        self.motor_distance_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(motor_controls, text="mm").pack(side=tk.LEFT)

        ttk.Label(motor_controls, text="Speed").pack(side=tk.LEFT, padx=(18, 0))
        self.motor_speed_spinbox = ttk.Spinbox(
            motor_controls,
            from_=0.01,
            to=50.0,
            increment=0.5,
            textvariable=self.motor_speed_var,
            width=8,
            state=tk.DISABLED,
        )
        self.motor_speed_spinbox.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Label(motor_controls, text="mm/s").pack(side=tk.LEFT)

        self.motor_reverse_button = ttk.Button(
            motor_controls,
            text="Jog -",
            command=self._motor_jog_reverse,
            state=tk.DISABLED,
        )
        self.motor_reverse_button.pack(side=tk.LEFT, padx=(18, 0))
        self.motor_forward_button = ttk.Button(
            motor_controls,
            text="Jog +",
            command=self._motor_jog_forward,
            state=tk.DISABLED,
        )
        self.motor_forward_button.pack(side=tk.LEFT, padx=(8, 0))
        self.motor_stop_button = ttk.Button(
            motor_controls,
            text="Stop motor",
            command=self._motor_stop,
            state=tk.DISABLED,
        )
        self.motor_stop_button.pack(side=tk.LEFT, padx=(8, 0))
        self.motor_controls = [
            self.motor_enable_checkbutton,
            self.motor_distance_spinbox,
            self.motor_speed_spinbox,
            self.motor_reverse_button,
            self.motor_forward_button,
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
        )
        self.table = ttk.Treeview(root, columns=columns, show="headings", height=18)
        headings = {
            "time": "Time ms",
            "target_pressure": "Target pressure",
            "pressure_before": "Pressure before valve",
            "regulator_feedback": "Regulator feedback pressure",
            "regulator_pwm": "Regulator PWM",
            "valves_open": "Valves open",
            "flow": "Flow",
        }
        for col, heading in headings.items():
            self.table.heading(col, text=heading)
            self.table.column(col, width=125, anchor=tk.CENTER)
        self.table.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(root, height=7, wrap=tk.NONE)
        self.log.pack(fill=tk.X, pady=(8, 0))

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
        if labels and self.port_var.get() not in labels:
            self.port_var.set(self._preferred_port_label(labels, ("arduino", "ttyacm")) or labels[0])
        if labels and self.colibri_port_var.get() not in labels:
            self.colibri_port_var.set(
                self._preferred_port_label(labels, ("dedi", "ftdi", "rs485", "ttyusb")) or labels[0]
            )
        elif not labels:
            self.port_var.set("")
            self.colibri_port_var.set("")
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
        self.target_pressure_spinbox.configure(state=tk.NORMAL)
        self.apply_pressure_button.configure(state=tk.NORMAL)
        self.stream_checkbutton.configure(state=tk.NORMAL)
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
        self._apply_stream_setting()

    def _selected_port_device(self):
        selected = self.port_var.get()
        return self.port_devices.get(selected, selected)

    def _selected_colibri_port_device(self):
        selected = self.colibri_port_var.get()
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
        self.target_pressure_spinbox.configure(state=tk.DISABLED)
        self.apply_pressure_button.configure(state=tk.DISABLED)
        self.stream_checkbutton.configure(state=tk.DISABLED)
        for checkbutton in self.nozzle_checkbuttons:
            checkbutton.configure(state=tk.DISABLED)
        self.starting_pressure_spinbox.configure(state=tk.DISABLED)
        self.pressure_increment_spinbox.configure(state=tk.DISABLED)
        self.reset_increment_button.configure(state=tk.DISABLED)
        self._set_pulse_buttons_enabled(False)
        self._set_motor_controls_enabled(False)
        self.motor_enabled_var.set(False)
        self.last_motor_speed_steps_s = None
        self.pulse_in_progress = False
        self.pending_increment_direction = 0
        self.mode_var.set("Mode: disconnected")
        self.status_var.set("Disconnected")
        self._write_debug_log("ARDUINO disconnected")

    def _start_test(self):
        self.rows.clear()
        for item in self.table.get_children():
            self.table.delete(item)
        self.mode_var.set("Mode: test sequence")
        self._write_debug_log("GUI start test")
        self._send("START")

    def _stop_test(self):
        self.mode_var.set("Mode: idle")
        self._write_debug_log("GUI stop test")
        self._send("STOP")

    def _manual_pulse(self):
        self._start_pulse(increment_direction=0)

    def _increment_pulse(self):
        if self._validated_pressure(self.starting_pressure_var, "starting pressure") is None:
            return
        if self._validated_pressure(self.pressure_increment_var, "pressure increment") is None:
            return
        self._start_pulse(increment_direction=1)

    def _decrement_pulse(self):
        if self._validated_pressure(self.starting_pressure_var, "starting pressure") is None:
            return
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

        self.pulse_in_progress = True
        self.pending_increment_direction = increment_direction
        self._set_pulse_buttons_enabled(False)
        self.mode_var.set("Mode: manual pulse running")
        self._write_debug_log(f"GUI pulse mask={mask} increment_direction={increment_direction}")
        self._send(f"PULSE:{mask}")

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

        target_pressure = min(max(target_pressure, 0.0), 5.0)
        self.target_pressure_var.set(round(target_pressure, 3))
        self.mode_var.set("Mode: manual pressure pending")
        self._write_debug_log(f"GUI set pressure target={target_pressure:.3f}")
        self._send(f"SET_PRESSURE:{target_pressure:.3f}", flush_live_backlog=True)
        return True

    def _validated_pressure(self, variable, label):
        try:
            value = float(variable.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid pressure setting", f"Enter a numeric {label}.")
            return None

        value = min(max(value, 0.0), 5.0)
        variable.set(round(value, 3))
        return value

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
            else:
                self._handle_line(value)
        self.after(1 if drained_count == MAX_QUEUE_DRAIN_PER_TICK else 50, self._drain_messages)

    def _handle_line(self, line):
        self._append_log_line(line)

        parts = line.split(";")
        if parts[0] == "MODE":
            self._handle_mode_line(parts)
            return

        if parts[0] == "STOPPED":
            self.mode_var.set("Mode: idle")
            return

        if parts[0] == "PULSE":
            self._handle_pulse_line(parts)
            return

        if parts[0] == "MOTOR":
            self._handle_motor_line(parts)
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

        self.rows.append(parts)
        self.table.insert("", tk.END, values=parts)
        children = self.table.get_children()
        if len(children) > 250:
            self.table.delete(children[0])

    def _append_log_line(self, line):
        self.log.insert(tk.END, line + "\n")
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.log.delete("1.0", f"{line_count - MAX_LOG_LINES + 1}.0")
        self.log.see(tk.END)

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

    def _handle_pulse_line(self, parts):
        if len(parts) >= 3 and parts[1] == "ERROR":
            self.pulse_in_progress = False
            self.pending_increment_direction = 0
            self._set_pulse_buttons_enabled(True)
            self.status_var.set(";".join(parts))
            return

        if len(parts) >= 3 and parts[1] == "START":
            self.mode_var.set(f"Mode: manual pulse running | mask {parts[2]}")
            return

        if len(parts) >= 3 and parts[1] == "DONE":
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

    def _advance_increment_target(self, direction):
        starting_pressure = self._validated_pressure(self.starting_pressure_var, "starting pressure")
        pressure_increment = self._validated_pressure(self.pressure_increment_var, "pressure increment")
        if starting_pressure is None or pressure_increment is None:
            return

        next_count = self.increment_count_var.get() + direction
        next_target = min(max(starting_pressure + next_count * pressure_increment, 0.0), 5.0)
        self.increment_count_var.set(next_count)
        self.target_pressure_var.set(round(next_target, 3))
        self._apply_pressure_settings()

    def _save_csv(self):
        if not self.rows:
            messagebox.showinfo("Nothing to save", "No test samples have been received yet.")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        with open(path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file, delimiter=";")
            writer.writerow([
                "time",
                "target regulator pressure",
                "pressure before valve",
                "regulator feedback pressure",
                "regulator pwm",
                "valves open",
                "flow",
            ])
            writer.writerows(self.rows)

    def destroy(self):
        self._disconnect_colibri()
        self._disconnect()
        if self.debug_log_file:
            self._stop_debug_log()
        super().destroy()


if __name__ == "__main__":
    app = TestRunGui()
    app.mainloop()
