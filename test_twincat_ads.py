"""Hardware-free checks for the GUI-to-PLC ADS mailbox adapter."""

import time
import unittest

from test_run_gui import TestRunGui
from twincat_ads import AdsTarget, Command, TwinCatAdsClient


class _FakeAdsConnection:
    """Immediate-ack PLC model used to test the mailbox ordering contract."""

    def __init__(self, reject_command=None, reject_error=3, reject_detail=42):
        self.closed = False
        self.history = []
        self.reject_command = reject_command
        self.reject_error = reject_error
        self.reject_detail = reject_detail
        self.values = {
            symbol: 0 for symbol in TwinCatAdsClient._STATUS_SYMBOLS
        }
        self.values.update(
            {
                "MAIN.bStatusReady": True,
                "MAIN.bStatusOutputsPermitted": True,
                "MAIN.udCmdSeq": 0,
                "MAIN.udCmdHeartbeat": 0,
                "MAIN.udCmdAckSeq": 0,
                "MAIN.nStatusError": 0,
                "MAIN.udStatusErrorId": 0,
            }
        )

    def read_list_by_name(self, symbols, **_kwargs):
        return {symbol: self.values.get(symbol, 0) for symbol in symbols}

    def write_list_by_name(self, values, **_kwargs):
        values = dict(values)
        self.history.append(values)
        self.values.update(values)
        if "MAIN.udCmdSeq" not in values:
            return

        command = self.values.get("MAIN.nCmdType")
        sequence = self.values["MAIN.udCmdSeq"]
        self.values["MAIN.udCmdAckSeq"] = sequence
        if command == self.reject_command:
            self.values["MAIN.nStatusError"] = self.reject_error
            self.values["MAIN.udStatusErrorId"] = self.reject_detail
            return

        self.values["MAIN.nStatusError"] = 0
        self.values["MAIN.udStatusErrorId"] = 0
        if command == Command.SET_PRESSURE:
            self.values["MAIN.fStatusTargetPressureBar"] = self.values[
                "MAIN.fCmdTargetPressureBar"
            ]
        elif command == Command.PULSE:
            self.values["MAIN.byStatusLastPulseMask"] = self.values[
                "MAIN.byCmdNozzleMask"
            ]
            self.values["MAIN.udStatusPulseStarted"] += 1

    def close(self):
        self.closed = True


class _DelayedAckAdsConnection(_FakeAdsConnection):
    """PLC model whose motion completes before its mailbox acknowledgement."""

    def __init__(self):
        super().__init__()
        self.pending_sequence = None
        self.pending_reads = 0
        self.messages = None
        self.busy_false_seen_before_ack = False

    def read_list_by_name(self, symbols, **_kwargs):
        if self.pending_sequence is not None:
            self.pending_reads += 1
            if self.pending_reads == 1:
                self.values["MAIN.bStatusMotorBusy"] = True
            elif self.pending_reads == 2:
                self.values["MAIN.bStatusMotorBusy"] = False
                self.values["MAIN.udStatusMotorDone"] += 1
            else:
                # At this point the second snapshot must already have reached
                # the GUI, even though the acknowledgement is still pending.
                emitted_lines = [
                    message[1][0]
                    for message in (self.messages or [])
                    if message[0] == "line"
                ]
                self.busy_false_seen_before_ack = "MOTOR;BUSY;0" in emitted_lines
                self.values["MAIN.udCmdAckSeq"] = self.pending_sequence
                self.pending_sequence = None
        return super().read_list_by_name(symbols, **_kwargs)

    def write_list_by_name(self, values, **_kwargs):
        values = dict(values)
        if "MAIN.udCmdSeq" not in values:
            return super().write_list_by_name(values, **_kwargs)

        # Preserve the write history but deliberately delay the normal fake's
        # immediate acknowledgement until three reads have occurred.
        self.history.append(values)
        self.values.update(values)
        self.pending_sequence = self.values["MAIN.udCmdSeq"]
        self.pending_reads = 0


class TwinCatAdsMailboxTests(unittest.TestCase):
    def _client(self, connection):
        messages = []
        client = TwinCatAdsClient(
            AdsTarget("5.75.145.248.1.1", "192.168.1.10"), messages.append
        )
        client._connection = connection
        client._last_snapshot = connection.read_list_by_name(
            (*TwinCatAdsClient._STATUS_SYMBOLS, "MAIN.udCmdSeq", "MAIN.udCmdHeartbeat")
        )
        client._session_start_monotonic = time.monotonic()
        with client._lock:
            client._connected = True
        return client, messages

    def test_gui_commands_are_acknowledged_in_order(self):
        connection = _FakeAdsConnection()
        client, messages = self._client(connection)

        client._handle_legacy_command("SET_PRESSURE:3.0")
        client._handle_legacy_command("SET_FLOW_THRESHOLD:2.5")
        client._handle_legacy_command("SET_PULSE_DURATION:50")
        client._handle_legacy_command("PULSE:3")

        command_writes = [
            write["MAIN.nCmdType"]
            for write in connection.history
            if "MAIN.nCmdType" in write
        ]
        sequence_writes = [
            write["MAIN.udCmdSeq"]
            for write in connection.history
            if "MAIN.udCmdSeq" in write
        ]
        self.assertEqual(
            command_writes,
            [
                Command.SET_PRESSURE,
                Command.APPLY_PULSE_SETTINGS,
                Command.APPLY_PULSE_SETTINGS,
                Command.PULSE,
            ],
        )
        self.assertEqual(sequence_writes, [1, 2, 3, 4])
        self.assertEqual(connection.values["MAIN.udCmdAckSeq"], 4)
        self.assertTrue(connection.values["MAIN.bCmdOutputsEnable"])
        emitted_lines = [message[1][0] for message in messages if message[0] == "line"]
        self.assertIn("PULSE;START;3", emitted_lines)
        self.assertTrue(any(line.endswith(";1;0.00000") for line in emitted_lines))

    def test_rejected_pulse_releases_gui_pulse_state(self):
        connection = _FakeAdsConnection(reject_command=Command.PULSE)
        client, messages = self._client(connection)

        client._handle_legacy_command("PULSE:1")

        emitted_lines = [message[1][0] for message in messages if message[0] == "line"]
        self.assertTrue(any(line.startswith("PULSE;ERROR;") for line in emitted_lines))
        self.assertEqual(connection.values["MAIN.udCmdAckSeq"], 1)

    def test_motor_enable_heartbeats_before_motor_enable(self):
        connection = _FakeAdsConnection()
        client, _messages = self._client(connection)

        client._handle_legacy_command("MOTOR_ENABLE:1")

        writes = [next(iter(write)) for write in connection.history]
        self.assertEqual(
            writes[:5],
            [
                "MAIN.bConfigMotionEnabled",
                "MAIN.bConfigHomingEnabled",
                "MAIN.udCmdHeartbeat",
                "MAIN.bCmdOutputsEnable",
                "MAIN.bCmdMotorEnable",
            ],
        )
        self.assertTrue(connection.values["MAIN.bCmdMotorEnable"])

    def test_home_decision_is_always_left_to_fresh_plc_input(self):
        connection = _FakeAdsConnection()
        connection.values["MAIN.bStatusLimitSwitch"] = True
        client, _messages = self._client(connection)

        client._handle_legacy_command("MOTOR_HOME")

        command_writes = [
            write["MAIN.nCmdType"]
            for write in connection.history
            if "MAIN.nCmdType" in write
        ]
        self.assertEqual(command_writes, [Command.HOME])

    def test_motor_stop_uses_dedicated_plc_stop_command(self):
        connection = _FakeAdsConnection()
        client, _messages = self._client(connection)

        client._handle_legacy_command("MOTOR_STOP")

        command_writes = [
            write["MAIN.nCmdType"]
            for write in connection.history
            if "MAIN.nCmdType" in write
        ]
        self.assertEqual(command_writes, [Command.MOTOR_STOP])

    def test_completed_short_move_releases_gui_lock_without_busy_edge(self):
        connection = _FakeAdsConnection()
        client, messages = self._client(connection)

        # The fake PLC acknowledges the move while Busy remains FALSE.  This
        # models a short move that completes between two normal ADS polls.
        client._handle_legacy_command("MOTOR_MOVE:1")

        emitted_lines = [message[1][0] for message in messages if message[0] == "line"]
        self.assertIn("MOTOR;BUSY;0", emitted_lines)

    def test_busy_false_reenables_both_jog_buttons(self):
        class _Control:
            def __init__(self):
                self.state = None

            def configure(self, *, state):
                self.state = state

        class _Variable:
            def set(self, value):
                self.value = value

        class _Gui:
            _set_motor_controls_enabled = TestRunGui._set_motor_controls_enabled
            _set_motor_motion_busy = TestRunGui._set_motor_motion_busy
            _handle_motor_line = TestRunGui._handle_motor_line

            def __init__(self):
                self.motor_motion_busy = True
                self.motor_enable_checkbutton = _Control()
                self.motor_stop_button = _Control()
                self.jog_left = _Control()
                self.jog_right = _Control()
                self.motor_controls = [
                    self.motor_enable_checkbutton,
                    self.jog_left,
                    self.jog_right,
                    self.motor_stop_button,
                ]
                self.status_var = _Variable()

            @staticmethod
            def _plc_connected():
                return True

        gui = _Gui()
        gui._handle_motor_line(["MOTOR", "BUSY", "0"])

        self.assertFalse(gui.motor_motion_busy)
        self.assertEqual(gui.jog_left.state, "normal")
        self.assertEqual(gui.jog_right.state, "normal")

    def test_motion_completion_is_forwarded_while_ack_is_still_pending(self):
        connection = _DelayedAckAdsConnection()
        client, messages = self._client(connection)
        connection.messages = messages

        client._handle_legacy_command("MOTOR_MOVE:10")

        self.assertTrue(connection.busy_false_seen_before_ack)
        emitted_lines = [message[1][0] for message in messages if message[0] == "line"]
        self.assertIn("MOTOR;DONE;POS;0;MM;0.00000;REF;0", emitted_lines)

    def test_range_rejection_reports_position_target_and_limits(self):
        connection = _FakeAdsConnection(
            reject_command=Command.MOVE_RELATIVE,
            reject_error=23,
            reject_detail=0,
        )
        connection.values.update(
            {
                "MAIN.fStatusMotorPositionMm": -1995.0,
                "MAIN.bStatusMotorReferenced": True,
                "MAIN.fConfigMotorMinPositionMm": -2000.0,
                "MAIN.fConfigMotorMaxPositionMm": 0.0,
            }
        )
        client, messages = self._client(connection)

        client._handle_legacy_command("MOTOR_MOVE:1102")

        status_messages = [message[1] for message in messages if message[0] == "status"]
        rejection = next(
            message
            for message in status_messages
            if "command rejected" in message and "error 23" in message
        )
        self.assertIn("software travel range violation", rejection)
        self.assertIn("position -1995.00000 mm", rejection)
        self.assertIn("requested target -2006.00440 mm", rejection)
        self.assertIn("configured range -2000.00000..0.00000 mm", rejection)

    def test_debug_logger_can_be_attached_detached_and_cannot_break_ads(self):
        connection = _FakeAdsConnection()
        client, _messages = self._client(connection)
        traces = []

        client.set_debug_logger(traces.append)
        client._handle_legacy_command("MOTOR_SPEED:400")
        self.assertTrue(any("ADS debug logging enabled" in trace for trace in traces))
        self.assertTrue(any("ADS TX legacy MOTOR_SPEED:400" in trace for trace in traces))

        trace_count = len(traces)
        client.set_debug_logger(None)
        client._handle_legacy_command("MOTOR_SPEED:401")
        self.assertEqual(len(traces), trace_count)

        def failing_logger(_message):
            raise OSError("test logger failure")

        client.set_debug_logger(failing_logger)
        client._trace("must not escape")


if __name__ == "__main__":
    unittest.main()
