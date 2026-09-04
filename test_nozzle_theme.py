import time
import unittest

from test_run_gui import (
    NOZZLE_MARCH_BEAT_MS,
    NOZZLE_MARCH_PULSE_MS,
    NOZZLE_MARCH_SEQUENCE,
    PULSE_DURATION_MAX_MS,
    PULSE_DURATION_MIN_MS,
    SAMPLE_INTERVAL_MS,
    TestRunGui,
)


class _Variable:
    def __init__(self, value=None):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class _Button:
    def __init__(self):
        self.options = {}

    def configure(self, **options):
        self.options.update(options)


class _NozzleThemeGui:
    _toggle_nozzle_theme = TestRunGui._toggle_nozzle_theme
    _start_nozzle_theme = TestRunGui._start_nozzle_theme
    _play_next_nozzle_theme_event = TestRunGui._play_next_nozzle_theme_event
    _schedule_next_nozzle_theme_event = TestRunGui._schedule_next_nozzle_theme_event
    _stop_nozzle_theme = TestRunGui._stop_nozzle_theme
    _handle_pulse_line = TestRunGui._handle_pulse_line
    _handle_flow_done_line = TestRunGui._handle_flow_done_line
    _pulse_duration_ms = TestRunGui._pulse_duration_ms

    def __init__(self):
        self.connected = True
        self.active_sequence_archive = None
        self.pulse_in_progress = False
        self.test_impulse_capture = None
        self.calibration_session = None
        self.colibri_touch_running = False
        self.current_impulse = None
        self.pending_increment_direction = 0
        self.pending_flip_angle = -1
        self.pending_pulse_mask = ""
        self.pending_pulse_duration_ms = None
        self.nozzle_theme_playing = False
        self.nozzle_theme_after_id = None
        self.nozzle_theme_note_index = 0
        self.nozzle_theme_note_started_monotonic = None
        self.nozzle_theme_note_beats = 0.0
        self.nozzle_theme_waiting_for_flow = False
        self.nozzle_theme_mask = 0
        self.nozzle_theme_saved_pulse_duration_ms = None
        self.nozzle_theme_stop_expected_pulse_error = False
        self.nozzle_theme_button = _Button()
        self.status_var = _Variable("")
        self.mode_var = _Variable("")
        self.commands = []
        self.after_calls = []
        self.cancelled_after_ids = []
        self.controls_running = None
        self.completed_durations = []

    def _plc_connected(self):
        return self.connected

    def _selected_nozzle_mask(self):
        return 0b100101

    def _validated_pulse_duration(self):
        return 50.0

    def _apply_pressure_settings(self):
        return True

    def _apply_flow_threshold_setting(self):
        return True

    def _finalize_stale_impulse_before_save(self):
        pass

    def _flow_capture_end_time_ms(self, _impulse):
        return 0.0

    def _finalize_impulse(self, _capture_end_time_ms):
        self.current_impulse = None

    def _set_nozzle_theme_controls(self, running):
        self.controls_running = running
        self.nozzle_theme_button.configure(
            text="Stop Imperial March" if running else "Play Imperial March"
        )

    def _set_pulse_buttons_enabled(self, _enabled):
        pass

    def _set_completed_pulse_duration(self, duration_ms):
        self.completed_durations.append(duration_ms)

    def _advance_increment_target(self, _direction):
        pass

    def _write_debug_log(self, _message):
        pass

    def _send(self, command):
        self.commands.append(command)
        return True

    def after(self, delay_ms, callback):
        after_id = f"after-{len(self.after_calls) + 1}"
        self.after_calls.append((after_id, delay_ms, callback))
        return after_id

    def after_cancel(self, after_id):
        self.cancelled_after_ids.append(after_id)


class NozzleThemeTests(unittest.TestCase):
    def test_note_table_fits_plc_limits_and_task_interval(self):
        notes_in_sequence = {note for note, _beats in NOZZLE_MARCH_SEQUENCE if note is not None}
        self.assertEqual(notes_in_sequence, set(NOZZLE_MARCH_PULSE_MS))
        for note, beats in NOZZLE_MARCH_SEQUENCE:
            if note is None:
                continue
            duration_ms = NOZZLE_MARCH_PULSE_MS[note]
            self.assertGreaterEqual(duration_ms, PULSE_DURATION_MIN_MS)
            self.assertLessEqual(duration_ms, PULSE_DURATION_MAX_MS)
            self.assertEqual(duration_ms % SAMPLE_INTERVAL_MS, 0)
            self.assertLessEqual(duration_ms, beats * NOZZLE_MARCH_BEAT_MS)

    def test_first_click_starts_first_note_on_selected_nozzles(self):
        gui = _NozzleThemeGui()

        gui._toggle_nozzle_theme()

        self.assertTrue(gui.nozzle_theme_playing)
        self.assertTrue(gui.nozzle_theme_waiting_for_flow)
        self.assertEqual(gui.nozzle_theme_mask, 0b100101)
        self.assertEqual(
            gui.commands,
            ["SET_PULSE_DURATION:160", "PULSE:37"],
        )
        self.assertEqual(gui.nozzle_theme_button.options["text"], "Stop Imperial March")

    def test_flow_done_schedules_next_note_without_blocking(self):
        gui = _NozzleThemeGui()
        gui._start_nozzle_theme()
        gui.nozzle_theme_note_started_monotonic = time.monotonic() - 0.05

        gui._schedule_next_nozzle_theme_event()

        self.assertFalse(gui.nozzle_theme_waiting_for_flow)
        self.assertEqual(len(gui.after_calls), 1)
        _after_id, delay_ms, callback = gui.after_calls[0]
        self.assertGreater(delay_ms, 0)
        self.assertLessEqual(delay_ms, NOZZLE_MARCH_BEAT_MS)
        self.assertEqual(callback, gui._play_next_nozzle_theme_event)

    def test_plc_done_and_flow_done_advance_theme_without_recording_manual_pulse(self):
        gui = _NozzleThemeGui()
        gui._start_nozzle_theme()

        gui._handle_pulse_line(["PULSE", "DONE", "37", "DURATION_MS", "160"])

        self.assertFalse(gui.pulse_in_progress)
        self.assertTrue(gui.nozzle_theme_waiting_for_flow)
        self.assertEqual(gui.completed_durations, [])
        self.assertEqual(gui.after_calls, [])

        gui._handle_flow_done_line(["PULSE", "FLOW_DONE", "SAMPLES", "20"])

        self.assertFalse(gui.nozzle_theme_waiting_for_flow)
        self.assertEqual(len(gui.after_calls), 1)

    def test_second_click_stops_outputs_and_restores_manual_duration(self):
        gui = _NozzleThemeGui()
        gui._toggle_nozzle_theme()

        gui._toggle_nozzle_theme()

        self.assertFalse(gui.nozzle_theme_playing)
        self.assertFalse(gui.pulse_in_progress)
        self.assertEqual(gui.commands[-2:], ["STOP", "SET_PULSE_DURATION:50.000"])
        self.assertEqual(gui.nozzle_theme_button.options["text"], "Play Imperial March")
        self.assertIn("pressure zero, motor power off", gui.status_var.get())

        stopped_status = gui.status_var.get()
        gui._handle_pulse_line(["PULSE", "ERROR", "PLC outputs were inhibited."])
        self.assertEqual(gui.status_var.get(), stopped_status)
        self.assertFalse(gui.nozzle_theme_stop_expected_pulse_error)


if __name__ == "__main__":
    unittest.main()
