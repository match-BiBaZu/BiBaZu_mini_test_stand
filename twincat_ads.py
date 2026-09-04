"""TwinCAT ADS transport for the pneumatic test-stand GUI.

The PLC, rather than Windows, owns valve timing, flow integration, and motion
commands.  This adapter deliberately translates the legacy GUI command/event
protocol to a small ADS command/status contract exposed by ``MAIN``.  Keeping
that translation here lets the existing CSV, force capture, and plotting code
continue to work while removing the Arduino from the pneumatic control path.

No physical I/O symbol is written from this module.  Only the public command
symbols in ``MAIN`` are written.  The PLC is responsible for its watchdog and
for driving the mapped EL terminals.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Optional

try:  # Keep the GUI importable on a development PC before pyads is installed.
    import pyads
except ImportError:  # pragma: no cover - exercised on deployment machines.
    pyads = None


ADS_DEFAULT_PORT = 851
DEFAULT_POLL_INTERVAL_SECONDS = 0.005
HEARTBEAT_INTERVAL_SECONDS = 0.25
MOTOR_MM_PER_STEP = 0.009985846
COMMAND_ACK_TIMEOUT_SECONDS = 2.0
# A live trace is kept briefly after the PLC reports a completed flow capture.
# This preserves the GUI's historical post-pulse force/pressure window even
# when an operator has switched ordinary live streaming off.
POST_FLOW_CAPTURE_LIVE_SECONDS = 0.55
MAX_PULSE_AND_CAPTURE_SECONDS = 1.05


class AdsTransportError(RuntimeError):
    """An ADS connection or command error that can be shown to an operator."""


class _AdsShutdown(Exception):
    """Internal control flow for an intentional client shutdown."""


class Command:
    """Values of MAIN.nCmdType.  Payload is written before nCmdSeq changes."""

    SET_PRESSURE = 1
    PULSE = 2
    START_TEST = 3
    STOP_ALL = 4
    MOTOR_POWER = 5
    MOVE_RELATIVE = 6
    MOVE_ABSOLUTE = 7
    HOME = 8
    ZERO = 9
    MOTOR_STOP = 10
    MOTOR_RESET = 11
    APPLY_PULSE_SETTINGS = 12


@dataclass(frozen=True)
class AdsTarget:
    """Connection details for a TwinCAT PLC runtime."""

    ams_net_id: str
    ip_address: str
    port: int = ADS_DEFAULT_PORT


class TwinCatAdsClient:
    """Threaded ADS adapter that emits the GUI's established line events.

    ``on_message`` receives tuples compatible with ``TestRunGui.messages``:
    ``("line", (text, monotonic_seconds, utc_ns))``, ``("status", text)``,
    and ``("ads_lost", text)``.
    """

    _STATUS_SYMBOLS = (
        "MAIN.bStatusReady",
        "MAIN.bStatusOutputsPermitted",
        "MAIN.bStatusWatchdogExpired",
        "MAIN.fStatusTargetPressureBar",
        "MAIN.fStatusNozzlePressureBar",
        "MAIN.fStatusFlowNlMin",
        "MAIN.bStatusRegulatorFeedbackAvailable",
        "MAIN.fStatusRegulatorFeedbackBar",
        "MAIN.iStatusRegulatorOutputRaw",
        "MAIN.bStatusValvesOpen",
        "MAIN.bStatusPulseBusy",
        "MAIN.bStatusFlowCaptureActive",
        "MAIN.byStatusActiveNozzleMask",
        "MAIN.byStatusLastPulseMask",
        "MAIN.udStatusPulseStarted",
        "MAIN.udStatusPulseDone",
        "MAIN.udStatusFlowDone",
        "MAIN.udStatusPulseDurationMs",
        "MAIN.udStatusFlowSamples",
        "MAIN.udStatusFlowDurationMs",
        "MAIN.fStatusFlowMaxNlMin",
        "MAIN.fStatusFlowBaselineNlMin",
        "MAIN.fStatusFlowVolumeL",
        "MAIN.bStatusTestRunning",
        "MAIN.udStatusTestStarted",
        "MAIN.udStatusTestStopped",
        "MAIN.bStatusMotorEnabled",
        "MAIN.bStatusMotorBusy",
        "MAIN.bStatusMotorReferenced",
        "MAIN.bStatusMotorError",
        "MAIN.bStatusLimitSwitch",
        "MAIN.udStatusMotorErrorId",
        "MAIN.fStatusMotorPositionMm",
        "MAIN.udStatusMotorDone",
        "MAIN.udStatusMotorHomeDone",
        "MAIN.udStatusMotorZeroDone",
        "MAIN.udStatusMotorStopped",
        "MAIN.udCmdAckSeq",
        "MAIN.nStatusError",
        "MAIN.udStatusErrorId",
    )

    def __init__(
        self,
        target: AdsTarget,
        on_message: Callable[[tuple], None],
        debug_logger: Optional[Callable[[str], None]] = None,
        poll_interval_seconds: float = DEFAULT_POLL_INTERVAL_SECONDS,
    ):
        self.target = target
        self._on_message = on_message
        self._debug_logger = debug_logger
        self._poll_interval_seconds = max(0.005, float(poll_interval_seconds))
        self._connection = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._close_requested = threading.Event()
        self._commands: queue.Queue[Optional[str]] = queue.Queue()
        self._connected = False
        self._stream_enabled = True
        self._command_seq = 0
        self._heartbeat = 0
        self._last_snapshot: Optional[Dict[str, object]] = None
        self._last_position_emit = 0.0
        self._session_start_monotonic = 0.0
        self._post_pulse_live_until = 0.0
        self._pending_manual_pressure: Optional[float] = None
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def open(self) -> None:
        """Open the ADS route and begin status polling.

        Outputs remain inhibited when the GUI connects.  A deliberate pressure,
        pulse, test, or motor-enable command arms outputs only after its payload
        has been written, and the PLC heartbeat is then maintained by this
        client.
        """

        if pyads is None:
            raise AdsTransportError(
                "pyads is not installed. Install the project requirements and make TcAdsDll.dll available."
            )
        if not self.target.ams_net_id.strip() or not self.target.ip_address.strip():
            raise AdsTransportError("Enter both the C9020 AMS Net ID and IP address.")
        if self.connected:
            return

        port = int(self.target.port)
        if not 1 <= port <= 65535:
            raise AdsTransportError("ADS port must be between 1 and 65535.")

        self._stop_event.clear()
        self._close_requested.clear()
        try:
            plc_port = getattr(pyads, "PORT_TC3PLC1", ADS_DEFAULT_PORT)
            # A non-default runtime port is useful for test systems, otherwise
            # use TwinCAT's named PLC runtime constant.
            if port != ADS_DEFAULT_PORT:
                plc_port = port
            self._connection = pyads.Connection(self.target.ams_net_id.strip(), plc_port, self.target.ip_address.strip())
            self._connection.open()
            snapshot = self._read_snapshot()
            if not bool(snapshot.get("MAIN.bStatusReady", False)):
                raise AdsTransportError(
                    "The PLC runtime is reachable but MAIN is not ready. Build and activate the TwinCAT project first."
                )
            self._command_seq = self._as_int(snapshot.get("MAIN.udCmdSeq", 0))
            self._heartbeat = self._as_int(snapshot.get("MAIN.udCmdHeartbeat", 0))
            self._safe_initialize()
        except Exception as exc:
            self._close_connection()
            raise AdsTransportError(f"Unable to open ADS connection to {self.target.ams_net_id}: {exc}") from exc

        # Establish the post-safe-stop baseline before accepting GUI commands.
        # This also ensures a first 10 ms pulse cannot be mistaken for an
        # already-completed event because there was no previous snapshot.
        self._last_snapshot = self._read_snapshot()
        self._session_start_monotonic = time.monotonic()
        with self._lock:
            self._connected = True
        self._thread = threading.Thread(target=self._run, name="twincat-ads", daemon=True)
        self._thread.start()
        self._trace(
            f"ADS connected net_id={self.target.ams_net_id} ip={self.target.ip_address} port={port}"
        )
        # The first poll uses the snapshot above as its baseline, so explicitly
        # publish the current state.  This prevents a newly connected GUI from
        # offering a second motion command while an earlier command is still
        # completing in the PLC.
        self._emit_line(
            f"MOTOR;BUSY;{1 if bool(self._last_snapshot.get('MAIN.bStatusMotorBusy', False)) else 0}"
        )
        self._emit_status("Connected to TwinCAT PLC through ADS; outputs remain inhibited until commanded.")

    def close(self, timeout_seconds: float = 1.0) -> None:
        """Request PLC stop/inhibit then close the ADS connection."""

        if not self.connected:
            self._close_connection()
            return
        # Pre-empt queued non-safety requests.  The worker detects this while
        # waiting for an acknowledgement and its finally block writes the
        # output-inhibit command; the PLC watchdog remains a final fallback.
        self._close_requested.set()
        self._stop_event.set()
        self._commands.put(None)
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=max(0.1, timeout_seconds))
        if thread and thread.is_alive():
            # The PLC watchdog remains the final safety layer if a network call
            # cannot return promptly.
            self._trace("ADS worker did not stop within timeout; PLC watchdog will inhibit outputs.")

    def send_legacy(self, command: str) -> bool:
        """Queue one former Arduino command for translation to ADS."""

        if not self.connected:
            return False
        self._commands.put(str(command).strip())
        return True

    def _run(self) -> None:
        next_poll = 0.0
        next_heartbeat = 0.0
        shutdown_requested = False
        try:
            while not self._stop_event.is_set():
                now = time.monotonic()
                timeout = max(0.001, min(next_poll or now, next_heartbeat or now) - now)
                try:
                    command = self._commands.get(timeout=timeout)
                except queue.Empty:
                    command = ""

                if command is None:
                    shutdown_requested = True
                    self._stop_event.set()
                    continue
                if command:
                    self._handle_legacy_command(command)

                now = time.monotonic()
                if now >= next_heartbeat:
                    self._write_heartbeat()
                    next_heartbeat = now + HEARTBEAT_INTERVAL_SECONDS
                if now >= next_poll:
                    self._handle_snapshot(self._read_snapshot())
                    next_poll = now + self._poll_interval_seconds
        except _AdsShutdown:
            shutdown_requested = True
        except Exception as exc:
            self._trace(f"ADS worker error {exc}")
            self._emit(("ads_lost", f"TwinCAT ADS connection lost: {exc}"))
        finally:
            try:
                self._inhibit_outputs()
            except Exception as exc:
                self._trace(f"ADS safe shutdown write failed: {exc}")
            self._close_connection()
            with self._lock:
                self._connected = False
            self._stop_event.set()
            if shutdown_requested or self._close_requested.is_set():
                self._emit_status("TwinCAT PLC disconnected; PLC outputs were inhibited.")

    def _safe_initialize(self) -> None:
        """Clear any retained target before allowing the GUI to issue commands."""

        self._write_values(
            {
                "MAIN.bCmdOutputsEnable": False,
                "MAIN.fCmdTargetPressureBar": 0.0,
                "MAIN.bCmdMotorEnable": False,
            }
        )
        self._send_transaction(Command.STOP_ALL, {})
        self._write_heartbeat()

    def _inhibit_outputs(self) -> None:
        if self._connection is None:
            return
        try:
            self._write_values({"MAIN.bCmdOutputsEnable": False, "MAIN.bCmdMotorEnable": False})
            self._send_transaction(Command.STOP_ALL, {})
        except Exception:
            # Closing is still useful; the PLC watchdog covers an unreachable
            # target after the connection disappears.
            pass

    def _handle_legacy_command(self, command: str) -> None:
        self._trace(f"ADS TX legacy {command}")
        upper, _, payload = command.partition(":")
        upper = upper.strip().upper()

        try:
            if upper == "STREAM_ON":
                self._stream_enabled = True
                self._emit_status("TwinCAT live telemetry enabled.")
            elif upper == "STREAM_OFF":
                self._stream_enabled = False
                self._emit_status("TwinCAT live telemetry paused (PLC safety/watchdog still active).")
            elif upper == "SET_PRESSURE":
                pressure = self._bounded_float(payload, 0.0, 6.0, "pressure")
                self._write_values({"MAIN.fCmdTargetPressureBar": pressure})
                self._arm_outputs()
                self._pending_manual_pressure = pressure
                self._send_transaction(Command.SET_PRESSURE, {})
            elif upper == "SET_FLOW_THRESHOLD":
                threshold = self._bounded_float(payload, 0.0, 200.0, "flow threshold")
                self._write_values({"MAIN.fCmdFlowThresholdNlMin": threshold})
                self._send_transaction(Command.APPLY_PULSE_SETTINGS, {})
                self._emit_line(f"FLOW_THRESHOLD;SET;{threshold:.3f}")
            elif upper == "SET_PULSE_DURATION":
                duration_ms = self._bounded_int(payload, 10, 500, "pulse duration")
                self._write_values({"MAIN.udCmdPulseDurationMs": duration_ms})
                self._send_transaction(Command.APPLY_PULSE_SETTINGS, {})
                self._emit_line(f"PULSE_DURATION;SET;{duration_ms}")
            elif upper == "PULSE":
                mask = self._bounded_mask(payload)
                self._write_values({"MAIN.byCmdNozzleMask": mask})
                self._arm_outputs()
                self._send_transaction(Command.PULSE, {})
            elif upper == "START":
                values = payload.split(":")
                if len(values) != 4:
                    raise AdsTransportError("START requires start pressure, end pressure, repeats, and nozzle mask.")
                start_pressure = self._bounded_float(values[0], 0.0, 6.0, "start pressure")
                end_pressure = self._bounded_float(values[1], start_pressure, 6.0, "end pressure")
                repeats = self._bounded_int(values[2], 1, 100, "repeats")
                mask = self._bounded_mask(values[3])
                self._write_values(
                    {
                        "MAIN.fCmdTestStartPressureBar": start_pressure,
                        "MAIN.fCmdTestEndPressureBar": end_pressure,
                        "MAIN.uiCmdTestRepeats": repeats,
                        "MAIN.byCmdTestNozzleMask": mask,
                    }
                )
                self._arm_outputs()
                self._pending_manual_pressure = None
                self._send_transaction(Command.START_TEST, {})
            elif upper == "STOP":
                self._pending_manual_pressure = None
                self._inhibit_outputs()
            elif upper == "MOTOR_ENABLE":
                enable = self._bounded_int(payload, 0, 1, "motor enable") == 1
                if enable:
                    # Ticking Enable authorizes this PLC session's motion and
                    # homing gates.  It does not itself start motion.
                    self._write_values({"MAIN.bConfigMotionEnabled": True})
                    self._write_values({"MAIN.bConfigHomingEnabled": True})
                    # After a watchdog timeout, the PLC deliberately clears
                    # bCmdMotorEnable until it has first observed a fresh
                    # heartbeat and output arm.  Write in that safe order so
                    # the Enable checkbox cannot be latched back off.
                    self._arm_outputs()
                self._write_values({"MAIN.bCmdMotorEnable": enable})
                self._send_transaction(Command.MOTOR_POWER, {})
            elif upper == "MOTOR_SPEED":
                steps_per_second = self._bounded_int(payload, 1, 5000, "motor speed")
                mm_per_second = steps_per_second * MOTOR_MM_PER_STEP
                self._write_values({"MAIN.fCmdMotorVelocityMmS": mm_per_second})
                self._emit_line(f"MOTOR;SPEED;{steps_per_second}")
            elif upper == "MOTOR_MOVE":
                steps = self._integer(payload, "relative motor distance")
                self._write_values({"MAIN.fCmdMotorRelativeMm": steps * MOTOR_MM_PER_STEP})
                self._arm_outputs()
                self._send_transaction(Command.MOVE_RELATIVE, {})
            elif upper == "MOTOR_ABS":
                steps = self._integer(payload, "absolute motor position")
                self._write_values({"MAIN.fCmdMotorAbsoluteMm": steps * MOTOR_MM_PER_STEP})
                self._arm_outputs()
                self._send_transaction(Command.MOVE_ABSOLUTE, {})
            elif upper == "MOTOR_HOME":
                self._arm_outputs()
                # MAIN owns the fresh DI1 decision.  If the switch is already
                # active it sets zero without moving; otherwise it begins the
                # negative velocity jog.  Avoid deciding from a stale ADS poll.
                self._send_transaction(Command.HOME, {})
            elif upper == "MOTOR_ZERO":
                self._send_transaction(Command.ZERO, {})
            elif upper == "MOTOR_STOP":
                self._send_transaction(Command.MOTOR_STOP, {})
            elif upper == "MOTOR_RESET":
                self._send_transaction(Command.MOTOR_RESET, {})
            elif upper == "MOTOR_POS":
                self._last_position_emit = 0.0
            else:
                raise AdsTransportError(f"Unsupported legacy command: {command}")
        except (AdsTransportError, ValueError) as exc:
            if upper == "SET_PRESSURE":
                self._pending_manual_pressure = None
            if upper == "PULSE":
                # The existing GUI parser uses this event to re-enable its
                # pulse controls after an immediately rejected PLC command.
                self._emit_line(f"PULSE;ERROR;{exc}")
            elif upper == "START":
                # A rejected automatic run must release its archive/UI state.
                self._emit_line("STOPPED")
            elif upper in {"MOTOR_MOVE", "MOTOR_ABS", "MOTOR_HOME", "MOTOR_ZERO"}:
                # The GUI pessimistically locks motion controls as soon as it
                # queues a command.  Return the PLC's current Busy state on a
                # rejection so that a non-motion rejection (for example,
                # unreferenced absolute motion) does not leave the controls
                # locked, while error 22 keeps them locked.
                busy = bool((self._last_snapshot or {}).get("MAIN.bStatusMotorBusy", False))
                self._emit_line(f"MOTOR;REJECTED;{1 if busy else 0};{exc}")
            self._emit_status(f"TwinCAT command rejected: {exc}")
            self._trace(f"ADS command rejected {command}: {exc}")
        else:
            self._emit_status(f"Sent to TwinCAT: {command}")

    def _arm_outputs(self) -> None:
        # A heartbeat must precede a new output-enable request.  MAIN resets
        # retained enable bits while its watchdog is expired.
        self._write_heartbeat()
        self._write_values({"MAIN.bCmdOutputsEnable": True})

    def _send_transaction(self, command_type: int, payload: Dict[str, object]) -> int:
        if payload:
            self._write_values(payload)
        self._command_seq = (self._command_seq + 1) & 0xFFFFFFFF
        # Each field is a separate ADS write.  A SumWrite batch does not give
        # the PLC an ordering guarantee, so nCmdSeq must be the final write.
        self._write_values({"MAIN.nCmdType": int(command_type)})
        self._write_values({"MAIN.udCmdSeq": self._command_seq})
        self._wait_for_command_ack(self._command_seq)
        return self._command_seq

    def _wait_for_command_ack(self, sequence: int) -> None:
        """Serialize the mailbox until MAIN has consumed ``sequence``.

        MAIN intentionally has a single command slot rather than a FIFO.  The
        acknowledgement prevents several GUI actions written inside one PLC
        scan from overwriting each other.
        """

        deadline = time.monotonic() + COMMAND_ACK_TIMEOUT_SECONDS
        last_snapshot: Optional[Dict[str, object]] = None
        next_heartbeat = time.monotonic()
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                raise _AdsShutdown()
            now = time.monotonic()
            if now >= next_heartbeat:
                self._write_heartbeat()
                next_heartbeat = now + HEARTBEAT_INTERVAL_SECONDS
            snapshot = self._read_snapshot()
            last_snapshot = snapshot
            if self._as_int(snapshot.get("MAIN.udCmdAckSeq", -1)) == sequence:
                if self.connected:
                    self._handle_snapshot(snapshot)
                error = self._as_int(snapshot.get("MAIN.nStatusError", 0))
                if error:
                    detail = self._as_int(snapshot.get("MAIN.udStatusErrorId", 0))
                    raise AdsTransportError(
                        f"PLC rejected command {sequence}: error {error}, detail {detail}."
                    )
                return
            time.sleep(min(self._poll_interval_seconds, 0.005))

        observed_ack = self._as_int((last_snapshot or {}).get("MAIN.udCmdAckSeq", -1))
        raise AdsTransportError(
            f"PLC did not acknowledge command {sequence} within "
            f"{COMMAND_ACK_TIMEOUT_SECONDS:.1f} s (last acknowledgement {observed_ack})."
        )

    def _write_heartbeat(self) -> None:
        self._heartbeat = (self._heartbeat + 1) & 0xFFFFFFFF
        self._write_values({"MAIN.udCmdHeartbeat": self._heartbeat})

    def _read_snapshot(self) -> Dict[str, object]:
        symbols = (*self._STATUS_SYMBOLS, "MAIN.udCmdSeq", "MAIN.udCmdHeartbeat")
        try:
            return dict(self._connection.read_list_by_name(symbols, cache_symbol_info=True))
        except TypeError:  # Older compatible pyads releases lack this keyword.
            return dict(self._connection.read_list_by_name(symbols))

    def _write_values(self, values: Dict[str, object]) -> None:
        if self._connection is None:
            raise AdsTransportError("ADS connection is not open.")
        try:
            self._connection.write_list_by_name(values, cache_symbol_info=True)
        except TypeError:  # Older compatible pyads releases lack this keyword.
            self._connection.write_list_by_name(values)

    def _handle_snapshot(self, snapshot: Dict[str, object]) -> None:
        previous = self._last_snapshot
        self._last_snapshot = snapshot
        now = time.monotonic()

        if previous is None:
            self._emit_motor_position(snapshot, "POSITION")
            self._emit_live_snapshot(snapshot)
            return

        if self._changed(snapshot, previous, "MAIN.udStatusPulseStarted"):
            # A 10 ms PLC pulse can begin and finish between two ADS polls.
            # Emit one synthetic open sample at the preserved counter edge so
            # the existing GUI starts its impulse record regardless.
            self._post_pulse_live_until = max(
                self._post_pulse_live_until,
                now + MAX_PULSE_AND_CAPTURE_SECONDS,
            )
            self._emit_line(f"PULSE;START;{self._value(snapshot, 'MAIN.byStatusLastPulseMask', 0)}")
            self._emit_live_snapshot(snapshot, force=True, valves_open=True)
        if self._changed(snapshot, previous, "MAIN.udStatusPulseDone"):
            self._post_pulse_live_until = max(
                self._post_pulse_live_until,
                now + POST_FLOW_CAPTURE_LIVE_SECONDS,
            )
            self._emit_line(
                "PULSE;DONE;{mask};DURATION_MS;{duration}".format(
                    mask=self._value(snapshot, "MAIN.byStatusLastPulseMask", 0),
                    duration=self._value(snapshot, "MAIN.udStatusPulseDurationMs", 0),
                )
            )
        if self._changed(snapshot, previous, "MAIN.udStatusFlowDone"):
            self._post_pulse_live_until = max(
                self._post_pulse_live_until,
                now + POST_FLOW_CAPTURE_LIVE_SECONDS,
            )
            self._emit_line(
                "PULSE;FLOW_DONE;SAMPLES;{samples};DURATION_MS;{duration};MAX_FLOW;{maximum:.5f};"
                "BASELINE_FLOW;{baseline:.5f};VOLUME_L;{volume:.8f}".format(
                    samples=self._value(snapshot, "MAIN.udStatusFlowSamples", 0),
                    duration=self._value(snapshot, "MAIN.udStatusFlowDurationMs", 0),
                    maximum=self._as_float(snapshot.get("MAIN.fStatusFlowMaxNlMin", 0.0)),
                    baseline=self._as_float(snapshot.get("MAIN.fStatusFlowBaselineNlMin", 0.0)),
                    volume=self._as_float(snapshot.get("MAIN.fStatusFlowVolumeL", 0.0)),
                )
            )

        was_test_running = bool(previous.get("MAIN.bStatusTestRunning", False))
        is_test_running = bool(snapshot.get("MAIN.bStatusTestRunning", False))
        if is_test_running and not was_test_running:
            self._emit_line("MODE;TEST")
        test_stopped = self._changed(snapshot, previous, "MAIN.udStatusTestStopped")
        if test_stopped:
            self._emit_line("STOPPED")

        watchdog_rose = bool(snapshot.get("MAIN.bStatusWatchdogExpired", False)) and not bool(
            previous.get("MAIN.bStatusWatchdogExpired", False)
        )
        output_inhibited = watchdog_rose or (
            bool(previous.get("MAIN.bStatusOutputsPermitted", False))
            and not bool(snapshot.get("MAIN.bStatusOutputsPermitted", False))
        )
        if output_inhibited:
            reason = (
                "PLC watchdog expired; outputs were inhibited."
                if bool(snapshot.get("MAIN.bStatusWatchdogExpired", False))
                else "PLC outputs were inhibited."
            )
            if bool(previous.get("MAIN.bStatusPulseBusy", False)) or bool(
                previous.get("MAIN.bStatusFlowCaptureActive", False)
            ):
                self._emit_line(f"PULSE;ERROR;{reason}")
            if was_test_running and not test_stopped:
                self._emit_line("STOPPED")
            self._emit_status(reason)

        if self._changed(snapshot, previous, "MAIN.bStatusMotorEnabled"):
            self._emit_line(
                f"MOTOR;ENABLED;{1 if bool(snapshot.get('MAIN.bStatusMotorEnabled', False)) else 0}"
            )
        if self._changed(snapshot, previous, "MAIN.bStatusMotorBusy"):
            self._emit_line(
                f"MOTOR;BUSY;{1 if bool(snapshot.get('MAIN.bStatusMotorBusy', False)) else 0}"
            )
        if self._changed(snapshot, previous, "MAIN.udStatusMotorHomeDone"):
            self._emit_motor_position(snapshot, "HOME_DONE")
        if self._changed(snapshot, previous, "MAIN.udStatusMotorZeroDone"):
            self._emit_motor_position(snapshot, "ZERO")
        if self._changed(snapshot, previous, "MAIN.udStatusMotorDone"):
            self._emit_motor_position(snapshot, "DONE")
        if self._changed(snapshot, previous, "MAIN.udStatusMotorStopped"):
            self._emit_line("MOTOR;STOPPED")
        if self._changed(snapshot, previous, "MAIN.bStatusLimitSwitch") and bool(
            snapshot.get("MAIN.bStatusLimitSwitch", False)
        ):
            self._emit_motor_position(snapshot, "LIMIT")
        if bool(snapshot.get("MAIN.bStatusMotorError", False)) and (
            not bool(previous.get("MAIN.bStatusMotorError", False))
            or self._changed(snapshot, previous, "MAIN.udStatusMotorErrorId")
        ):
            self._emit_line(f"MOTOR;ERROR;{self._value(snapshot, 'MAIN.udStatusMotorErrorId', 0)}")

        if self._changed(snapshot, previous, "MAIN.nStatusError"):
            error = self._value(snapshot, "MAIN.nStatusError", 0)
            if error:
                self._emit_status(
                    f"TwinCAT PLC command error {error} (detail {self._value(snapshot, 'MAIN.udStatusErrorId', 0)})."
                )

        pending_manual_pressure = self._pending_manual_pressure
        if pending_manual_pressure is not None and not is_test_running:
            actual_target = self._as_float(snapshot.get("MAIN.fStatusTargetPressureBar", 0.0))
            if abs(actual_target - pending_manual_pressure) < 0.0005:
                self._emit_line(
                    "MODE;MANUAL;SETPOINT;{target:.3f};PWM;{raw}".format(
                        target=actual_target,
                        raw=self._value(snapshot, "MAIN.iStatusRegulatorOutputRaw", 0),
                    )
                )
                self._pending_manual_pressure = None

        position_changed = abs(
            self._as_float(snapshot.get("MAIN.fStatusMotorPositionMm", 0.0))
            - self._as_float(previous.get("MAIN.fStatusMotorPositionMm", 0.0))
        ) >= 0.0005
        if position_changed or now - self._last_position_emit >= 1.0:
            self._emit_motor_position(snapshot, "POSITION")

        if (
            self._stream_enabled
            or self._is_active(snapshot)
            or now < self._post_pulse_live_until
        ):
            self._emit_live_snapshot(snapshot)

    def _emit_live_snapshot(
        self,
        snapshot: Dict[str, object],
        *,
        force: bool = False,
        valves_open: Optional[bool] = None,
    ) -> None:
        if not (
            force
            or self._stream_enabled
            or self._is_active(snapshot)
            or time.monotonic() < self._post_pulse_live_until
        ):
            return
        elapsed_ms = round((time.monotonic() - self._session_start_monotonic) * 1000.0)
        feedback = ""
        if bool(snapshot.get("MAIN.bStatusRegulatorFeedbackAvailable", False)):
            feedback = f"{self._as_float(snapshot.get('MAIN.fStatusRegulatorFeedbackBar', 0.0)):.5f}"
        self._emit_line(
            "{time_ms};{target:.5f};{pressure:.5f};{feedback};{raw};{valves};{flow:.5f}".format(
                time_ms=max(0, elapsed_ms),
                target=self._as_float(snapshot.get("MAIN.fStatusTargetPressureBar", 0.0)),
                pressure=self._as_float(snapshot.get("MAIN.fStatusNozzlePressureBar", 0.0)),
                feedback=feedback,
                raw=self._value(snapshot, "MAIN.iStatusRegulatorOutputRaw", 0),
                valves=(
                    1
                    if (bool(snapshot.get("MAIN.bStatusValvesOpen", False)) if valves_open is None else valves_open)
                    else 0
                ),
                flow=self._as_float(snapshot.get("MAIN.fStatusFlowNlMin", 0.0)),
            )
        )

    def _emit_motor_position(self, snapshot: Dict[str, object], event: str) -> None:
        position_mm = self._as_float(snapshot.get("MAIN.fStatusMotorPositionMm", 0.0))
        position_steps = round(position_mm / MOTOR_MM_PER_STEP)
        referenced = 1 if bool(snapshot.get("MAIN.bStatusMotorReferenced", False)) else 0
        self._last_position_emit = time.monotonic()
        self._emit_line(
            f"MOTOR;{event};POS;{position_steps};MM;{position_mm:.5f};REF;{referenced}"
        )

    @staticmethod
    def _is_active(snapshot: Dict[str, object]) -> bool:
        return bool(
            snapshot.get("MAIN.bStatusPulseBusy", False)
            or snapshot.get("MAIN.bStatusFlowCaptureActive", False)
            or snapshot.get("MAIN.bStatusTestRunning", False)
        )

    @staticmethod
    def _changed(current: Dict[str, object], previous: Dict[str, object], name: str) -> bool:
        return current.get(name) != previous.get(name)

    @staticmethod
    def _value(snapshot: Dict[str, object], name: str, default: object) -> object:
        return snapshot.get(name, default)

    @staticmethod
    def _as_int(value: object) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _as_float(value: object) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    @classmethod
    def _integer(cls, value: str, label: str) -> int:
        try:
            return int(round(float(value)))
        except (TypeError, ValueError) as exc:
            raise AdsTransportError(f"Enter a numeric {label}.") from exc

    @classmethod
    def _bounded_int(cls, value: str, minimum: int, maximum: int, label: str) -> int:
        return min(max(cls._integer(value, label), minimum), maximum)

    @classmethod
    def _bounded_float(cls, value: str, minimum: float, maximum: float, label: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise AdsTransportError(f"Enter a numeric {label}.") from exc
        if number != number:  # NaN
            raise AdsTransportError(f"Enter a numeric {label}.")
        return min(max(number, minimum), maximum)

    @classmethod
    def _bounded_mask(cls, value: str) -> int:
        mask = cls._bounded_int(value, 0, 0x3F, "six-nozzle mask") & 0x3F
        if not mask:
            raise AdsTransportError("Select at least one nozzle.")
        return mask

    def _emit_line(self, text: str) -> None:
        self._trace(f"ADS RX {text}")
        self._emit(("line", (text, time.monotonic(), time.time_ns())))

    def _emit_status(self, text: str) -> None:
        self._emit(("status", text))

    def _emit(self, message: tuple) -> None:
        try:
            self._on_message(message)
        except Exception:
            # A GUI shutdown should not prevent the PLC watchdog/close path.
            pass

    def _trace(self, text: str) -> None:
        if self._debug_logger:
            self._debug_logger(text)

    def _close_connection(self) -> None:
        connection = self._connection
        self._connection = None
        if connection is None:
            return
        try:
            connection.close()
        except Exception:
            pass
