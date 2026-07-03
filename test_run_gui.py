import csv
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
MAX_LOG_LINES = 400
MAX_QUEUE_DRAIN_PER_TICK = 250


class TestRunGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pneumatic Test Run")
        self.geometry("900x560")

        self.serial_port = None
        self.reader_thread = None
        self.reader_running = False
        self.rows = []
        self.messages = queue.Queue()
        self.port_devices = {}

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.mode_var = tk.StringVar(value="Mode: disconnected")
        self.target_pressure_var = tk.DoubleVar(value=0.20)
        self.starting_pressure_var = tk.DoubleVar(value=0.20)
        self.pressure_increment_var = tk.DoubleVar(value=0.05)
        self.increment_count_var = tk.IntVar(value=0)
        self.stream_var = tk.BooleanVar(value=True)
        self.nozzle_vars = [tk.BooleanVar(value=True) for _ in range(4)]
        self.nozzle_checkbuttons = []
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

        ttk.Label(root, textvariable=self.mode_var).pack(fill=tk.X, pady=(10, 0))
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
        if labels and self.port_var.get() not in labels:
            self.port_var.set(labels[0])
        elif not labels:
            self.port_var.set("")
            self.status_var.set("No serial ports found. Check the USB cable, driver, and Arduino IDE Serial Monitor.")
        else:
            self.status_var.set(f"Found {len(labels)} serial port(s).")

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
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1)
            time.sleep(2.0)
        except serial.SerialException as exc:
            self.serial_port = None
            messagebox.showerror("Connection failed", str(exc))
            return

        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.reader_thread.start()
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
        self.mode_var.set("Mode: connected")
        self.status_var.set(f"Connected to {port} at {BAUD_RATE} baud")
        self._apply_pressure_settings()
        self._apply_stream_setting()

    def _selected_port_device(self):
        selected = self.port_var.get()
        return self.port_devices.get(selected, selected)

    def _disconnect(self):
        self.reader_running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=0.5)
        if self.serial_port:
            self.serial_port.close()
        self.serial_port = None
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
        self.pulse_in_progress = False
        self.pending_increment_direction = 0
        self.mode_var.set("Mode: disconnected")
        self.status_var.set("Disconnected")

    def _start_test(self):
        self.rows.clear()
        for item in self.table.get_children():
            self.table.delete(item)
        self.mode_var.set("Mode: test sequence")
        self._send("START")

    def _stop_test(self):
        self.mode_var.set("Mode: idle")
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
        self._send("STREAM_ON" if self.stream_var.get() else "STREAM_OFF")

    def _send(self, command, flush_live_backlog=False):
        if not self.serial_port:
            return
        if flush_live_backlog:
            self._clear_pending_live_messages()
        self.serial_port.write(f"{command}\n".encode("ascii"))
        self.status_var.set(f"Sent {command}")

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
                self.messages.put(("status", f"Serial error: {exc}"))
                break
            if line:
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
                self.status_var.set(value)
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
        self._disconnect()
        super().destroy()


if __name__ == "__main__":
    app = TestRunGui()
    app.mainloop()
