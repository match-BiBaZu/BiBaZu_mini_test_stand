import json
import math
import queue
import tempfile
import threading
import unittest
import csv
from pathlib import Path

from force_sources import ForceSample
from platform_calibration import (
    CalibrationError,
    CalibrationMeasurement,
    PlatformCalibration,
    fit_platform_calibration,
    measurement_from_samples,
    parse_weight_grams,
    preserve_uncalibrated_sample,
    validate_weight_list,
)
from test_run_gui import TestRunGui


def measurement(weight_g, placement, force_1, force_2):
    return CalibrationMeasurement(
        weight_g=weight_g,
        placement_index=placement,
        sample_count=100,
        force_1_mean_n=force_1,
        force_2_mean_n=force_2,
        total_mean_n=force_1 + force_2,
        total_std_n=0.001,
    )


class PlatformCalibrationTests(unittest.TestCase):
    def test_weight_parser_accepts_decimal_comma(self):
        self.assertEqual(parse_weight_grams("50,15"), 50.15)
        self.assertEqual(validate_weight_list(["200,52", "50.15"]), [0.0, 50.15, 200.52])

    def test_weight_validation_requires_two_nonzero_points(self):
        with self.assertRaises(CalibrationError):
            validate_weight_list(["0", "50"])
        with self.assertRaises(CalibrationError):
            parse_weight_grams("401")

    def test_fit_apply_sum_and_json_roundtrip(self):
        tare_1 = 0.12
        tare_2 = -0.04
        source_gain = 0.97
        points = [measurement(0.0, 1, tare_1, tare_2)]
        for weight in (50.15, 200.52):
            expected = weight * 9.80665 / 1000.0
            measured = expected / source_gain
            for placement, fraction in enumerate((0.35, 0.5, 0.65), start=1):
                points.append(
                    measurement(
                        weight,
                        placement,
                        tare_1 + measured * fraction,
                        tare_2 + measured * (1.0 - fraction),
                    )
                )
        calibration = fit_platform_calibration(
            points,
            name="Test platform",
            device_ip="192.168.10.20",
            channels=(
                {"logical_name": "force_1", "mx_channel": 3},
                {"logical_name": "force_2", "mx_channel": 4},
            ),
        )
        self.assertAlmostEqual(calibration.gain, source_gain, places=12)

        sample = ForceSample(
            source="quantumx",
            sequence=1,
            timestamp_utc_ns=1,
            force_1_n=tare_1 + 0.3,
            force_2_n=tare_2 + 0.2,
            force_total_n=tare_1 + tare_2 + 0.5,
            status="ok",
            channel_1_status="ok",
            channel_2_status="ok",
            force_1_mean_20_n=tare_1 + 0.3,
            force_2_mean_20_n=tare_2 + 0.2,
            force_total_mean_20_n=tare_1 + tare_2 + 0.5,
            force_1_raw_n=tare_1 + 0.3,
            force_2_raw_n=tare_2 + 0.2,
            force_total_raw_n=tare_1 + tare_2 + 0.5,
        )
        corrected = calibration.apply(sample)
        self.assertAlmostEqual(corrected.force_1_n, source_gain * 0.3)
        self.assertAlmostEqual(corrected.force_2_n, source_gain * 0.2)
        self.assertAlmostEqual(
            corrected.force_total_n,
            corrected.force_1_n + corrected.force_2_n,
        )
        self.assertEqual(corrected.uncalibrated_force_total_n, sample.force_total_n)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            calibration.save(path)
            loaded = PlatformCalibration.load(path)
        self.assertEqual(loaded.profile_id, calibration.profile_id)
        self.assertAlmostEqual(loaded.gain, calibration.gain)
        self.assertEqual(len(loaded.measurements), len(calibration.measurements))

    def test_acquisition_uses_unique_unfiltered_samples(self):
        samples = []
        for sequence in range(12):
            sample = ForceSample(
                source="quantumx",
                sequence=sequence,
                timestamp_utc_ns=1000 + sequence,
                force_1_n=99.0,
                force_2_n=99.0,
                force_total_n=198.0,
                status="ok",
                channel_1_status="ok",
                channel_2_status="ok",
                force_1_raw_n=0.2 + sequence * 0.001,
                force_2_raw_n=0.3 + sequence * 0.001,
                force_total_raw_n=0.5 + sequence * 0.002,
            )
            samples.extend((sample, sample))
        result = measurement_from_samples(50.15, 1, samples)
        self.assertEqual(result.sample_count, 12)
        self.assertAlmostEqual(result.total_mean_n, 0.511)

    def test_tare_update_keeps_gain_and_source_values(self):
        profile = PlatformCalibration(
            profile_id="profile",
            name="Platform",
            created_utc="2026-07-30T00:00:00+00:00",
            device_ip="192.168.10.20",
            channels=(
                {"logical_name": "force_1", "mx_channel": 3},
                {"logical_name": "force_2", "mx_channel": 4},
            ),
            gravity_m_s2=9.80665,
            gain=1.01,
            tare_1_n=0.1,
            tare_2_n=-0.1,
            settle_seconds=1.0,
            acquisition_seconds=1.0,
            placement_repeats=3,
        )
        updated = profile.with_tare(0.2, -0.05, "2026-07-30T01:00:00+00:00")
        self.assertEqual(updated.gain, profile.gain)
        source = ForceSample(
            source="quantumx",
            sequence=1,
            timestamp_utc_ns=1,
            force_1_n=0.4,
            force_2_n=0.1,
            force_total_n=0.5,
            status="ok",
            channel_1_status="ok",
            channel_2_status="ok",
            force_1_raw_n=0.41,
            force_2_raw_n=0.11,
            force_total_raw_n=0.52,
        )
        preserved = preserve_uncalibrated_sample(source)
        corrected = updated.apply(preserved)
        self.assertAlmostEqual(corrected.force_total_n, corrected.force_1_n + corrected.force_2_n)
        self.assertEqual(corrected.uncalibrated_force_total_raw_n, 0.52)

    def test_sequence_timeseries_audit_columns_align(self):
        gui = object.__new__(TestRunGui)
        gui.active_sequence_archive = {"session_id": "session", "german_csv": False}
        impulse = {
            "impulse_index": 1,
            "target_pressure": 1.5,
            "valve_mask": "3",
            "pulse_start_utc_ns": 1_000_000_000,
            "pulse_start_monotonic": 10.0,
            "force_trace": [
                (
                    10.01,
                    1_010_000_000,
                    7,
                    0.5,
                    0.2,
                    0.3,
                    "ok",
                    0.49,
                    0.51,
                    0.48,
                    0.19,
                    0.29,
                    0.47,
                    0.18,
                    0.29,
                    0.50,
                    0.20,
                    0.30,
                    "profile-id",
                )
            ],
            "pressure_trace": [(10.02, 1.5, 1.49, 0.8)],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeseries.csv"
            gui._write_sequence_impulse_timeseries(impulse, "impulse-id", path)
            with open(path, newline="", encoding="utf-8") as csv_file:
                rows = list(csv.reader(csv_file))
        self.assertEqual(len(rows), 3)
        self.assertTrue(all(len(row) == len(rows[0]) for row in rows[1:]))
        self.assertEqual(rows[1][23], "profile-id")

    def test_force_touch_off_stops_and_retracts_after_hand_force(self):
        class FakeColibri:
            def __init__(self):
                self.steps = 0
                self.moves = []
                self.stop_count = 0

            def set_remote(self):
                pass

            def enable(self):
                pass

            def stop(self):
                self.stop_count += 1

            def position_steps(self):
                return self.steps

            def move_relative_steps(self, steps):
                self.steps += steps
                self.moves.append(steps)

        class DummyTouchGui:
            _colibri_mm_to_steps = TestRunGui._colibri_mm_to_steps
            _colibri_steps_to_mm = TestRunGui._colibri_steps_to_mm
            _perform_colibri_force_touch = TestRunGui._perform_colibri_force_touch

            def __init__(self):
                self.colibri = FakeColibri()
                self.colibri_touch_cancel_event = threading.Event()
                self.messages = queue.Queue()
                self.platform_calibration = None
                self.force_values = iter((0.0, 0.11, 0.0))

            def _read_colibri_snapshot(self):
                return {
                    "status": {
                        "error_byte": 0,
                        "referenced": True,
                        "moving": False,
                    },
                    "position_steps": self.colibri.steps,
                    "position_mm": self._colibri_steps_to_mm(self.colibri.steps),
                }

            def _acquire_touch_baseline(self):
                return 0.0, 0.001, 25

            def _current_touch_force_sample(self):
                return object(), next(self.force_values)

            def _wait_for_colibri_move(self, target_steps, **_kwargs):
                self.assert_target(target_steps)
                return self._read_colibri_snapshot()

            def assert_target(self, target_steps):
                if self.colibri.steps != target_steps:
                    raise AssertionError((self.colibri.steps, target_steps))

            def _write_colibri_touch_trace(self, *_args):
                return Path("touch.csv")

            def _set_colibri_motion_direction(self, _direction):
                pass

            def _clear_colibri_motion_direction(self, **_kwargs):
                pass

        gui = DummyTouchGui()
        result = gui._perform_colibri_force_touch(0.1)
        self.assertEqual(gui.colibri.moves, [2, -10])
        self.assertGreaterEqual(gui.colibri.stop_count, 1)
        self.assertAlmostEqual(result["position_mm"], -0.04)
        message_kinds = []
        while not gui.messages.empty():
            message_kinds.append(gui.messages.get_nowait()[0])
        self.assertIn("touch_off_result", message_kinds)

    def test_continuous_touch_off_uses_minimum_speed_and_restores_it(self):
        class ForceSampleStub:
            def __init__(self, sample_id, force_n):
                self.sample_id = ("quantumx", sample_id, sample_id)
                self.force_total_n = force_n

        class FakeColibri:
            def __init__(self):
                self.steps = 0
                self.moves = []
                self.stop_count = 0
                self.speed_setting = 24
                self.speed_writes = []

            def set_remote(self):
                pass

            def enable(self):
                pass

            def stop(self):
                self.stop_count += 1

            def position_steps(self):
                return self.steps

            def move_relative_steps(self, steps):
                self.steps += steps
                self.moves.append(steps)

            def parameter(self, index, subindex):
                self.assert_speed_parameter(index, subindex)
                return self.speed_setting

            def set_parameter(self, index, subindex, value, byte_count):
                self.assert_speed_parameter(index, subindex)
                self.speed_setting = value
                self.speed_writes.append((value, byte_count))

            @staticmethod
            def assert_speed_parameter(index, subindex):
                if (index, subindex) != (3, 2):
                    raise AssertionError((index, subindex))

        class DummyTouchGui:
            _colibri_mm_to_steps = TestRunGui._colibri_mm_to_steps
            _colibri_steps_to_mm = TestRunGui._colibri_steps_to_mm
            _perform_colibri_force_touch = TestRunGui._perform_colibri_force_touch
            _perform_continuous_touch_approach = (
                TestRunGui._perform_continuous_touch_approach
            )

            def __init__(self):
                self.colibri = FakeColibri()
                self.colibri_touch_cancel_event = threading.Event()
                self.messages = queue.Queue()
                self.platform_calibration = None
                self.force_index = 0

            def _read_colibri_snapshot(self):
                return {
                    "status": {
                        "error_byte": 0,
                        "referenced": True,
                        "moving": False,
                    },
                    "position_steps": self.colibri.steps,
                    "position_mm": self._colibri_steps_to_mm(self.colibri.steps),
                }

            def _acquire_touch_baseline(self):
                return 0.0, 0.001, 25

            def _current_touch_force_sample(self):
                self.force_index += 1
                force_n = 0.11 if self.force_index == 1 else 0.0
                return ForceSampleStub(self.force_index, force_n), force_n

            def _wait_for_colibri_move(self, target_steps, **_kwargs):
                if self.colibri.steps != target_steps:
                    raise AssertionError((self.colibri.steps, target_steps))
                return self._read_colibri_snapshot()

            def _write_colibri_touch_trace(self, *_args):
                return Path("touch.csv")

            def _write_debug_log(self, _message):
                pass

            def _set_colibri_motion_direction(self, _direction):
                pass

            def _clear_colibri_motion_direction(self, **_kwargs):
                pass

        gui = DummyTouchGui()
        result = gui._perform_colibri_force_touch(
            0.1,
            continuous_approach=True,
            retract_mm=0.05,
        )

        self.assertEqual(gui.colibri.moves, [20, -10])
        self.assertEqual(gui.colibri.speed_writes, [(1, 2), (24, 2)])
        self.assertEqual(gui.colibri.speed_setting, 24)
        self.assertAlmostEqual(result["position_mm"], 0.05)

        cancelled_gui = DummyTouchGui()
        cancelled_gui.colibri_touch_cancel_event.set()
        cancelled_result = cancelled_gui._perform_colibri_force_touch(
            0.1,
            continuous_approach=True,
            retract_mm=0.05,
        )

        self.assertEqual(cancelled_gui.colibri.moves, [20])
        self.assertGreaterEqual(cancelled_gui.colibri.stop_count, 2)
        self.assertEqual(
            cancelled_gui.colibri.speed_writes,
            [(1, 2), (24, 2)],
        )
        self.assertEqual(cancelled_gui.colibri.speed_setting, 24)
        self.assertAlmostEqual(cancelled_result["position_mm"], 0.1)


if __name__ == "__main__":
    unittest.main()
