"""Hardware-free checks for the GUI-to-PLC ADS mailbox adapter."""

import time
import unittest

from twincat_ads import AdsTarget, Command, TwinCatAdsClient


class _FakeAdsConnection:
    """Immediate-ack PLC model used to test the mailbox ordering contract."""

    def __init__(self, reject_command=None):
        self.closed = False
        self.history = []
        self.reject_command = reject_command
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
            self.values["MAIN.nStatusError"] = 3
            self.values["MAIN.udStatusErrorId"] = 42
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


if __name__ == "__main__":
    unittest.main()
