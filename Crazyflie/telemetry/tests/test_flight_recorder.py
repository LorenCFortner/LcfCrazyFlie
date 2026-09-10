"""Tests for FlightRecorder.

Written test-first (TDD). Uses real file I/O against tmp_path since the
actual CSV output format is the point of this module — mocking the file
handle would hide exactly the kind of format bug this recorder exists to
catch (e.g. a wrong-order column, a missed field).
"""

import csv
from pathlib import Path

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.telemetry.flight_recorder import CSV_HEADER, FlightRecorder
from Crazyflie.telemetry.stabilizer_monitor import DroneState


def _read_rows(csv_path: Path) -> list[dict[str, str]]:
    with open(csv_path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


class TestStart:
    def test_creates_parent_directory(self, tmp_path):
        csv_path = tmp_path / "logs" / "run_telemetry.csv"
        recorder = FlightRecorder()

        recorder.start(csv_path)
        recorder.stop()

        assert csv_path.parent.exists()

    def test_writes_header_row(self, tmp_path):
        csv_path = tmp_path / "run_telemetry.csv"
        recorder = FlightRecorder()

        recorder.start(csv_path)
        recorder.stop()

        with open(csv_path, newline="", encoding="utf-8") as f:
            header = next(csv.reader(f))
        assert header == CSV_HEADER

    def test_overwrites_existing_file(self, tmp_path):
        csv_path = tmp_path / "run_telemetry.csv"
        csv_path.write_text("stale content\n", encoding="utf-8")
        recorder = FlightRecorder()

        recorder.start(csv_path)
        recorder.stop()

        rows = _read_rows(csv_path)
        assert rows == []  # only header, stale content gone


class TestRecordRanger:
    def test_writes_ranger_row_with_poll_context_by_default(self, tmp_path):
        csv_path = tmp_path / "run_telemetry.csv"
        recorder = FlightRecorder()
        recorder.start(csv_path)

        readings = MultiRangerReadings(front=0.5, back=None, left=1.0, right=2.0, up=None)
        recorder.record_ranger(readings, "forward", 0.5)
        recorder.stop()

        rows = _read_rows(csv_path)
        assert len(rows) == 1
        row = rows[0]
        assert row["source"] == "ranger"
        assert row["context"] == "poll"
        assert row["front"] == "0.5"
        assert row["back"] == ""
        assert row["left"] == "1.0"
        assert row["right"] == "2.0"
        assert row["up"] == ""
        assert row["commanded_direction"] == "forward"
        assert row["commanded_velocity"] == "0.5"
        assert row["roll_deg"] == ""
        assert row["battery_v"] == ""

    def test_writes_custom_context(self, tmp_path):
        csv_path = tmp_path / "run_telemetry.csv"
        recorder = FlightRecorder()
        recorder.start(csv_path)

        readings = MultiRangerReadings(front=0.1, back=None, left=None, right=None, up=None)
        recorder.record_ranger(readings, "forward", 0.5, context="trigger")
        recorder.stop()

        rows = _read_rows(csv_path)
        assert rows[0]["context"] == "trigger"

    def test_none_direction_and_readings_write_as_blank(self, tmp_path):
        csv_path = tmp_path / "run_telemetry.csv"
        recorder = FlightRecorder()
        recorder.start(csv_path)

        readings = MultiRangerReadings(front=None, back=None, left=None, right=None, up=None)
        recorder.record_ranger(readings, None, 0.0)
        recorder.stop()

        rows = _read_rows(csv_path)
        assert rows[0]["front"] == ""
        assert rows[0]["commanded_direction"] == ""

    def test_multiple_calls_append_multiple_rows(self, tmp_path):
        csv_path = tmp_path / "run_telemetry.csv"
        recorder = FlightRecorder()
        recorder.start(csv_path)

        readings = MultiRangerReadings(front=0.5, back=None, left=None, right=None, up=None)
        recorder.record_ranger(readings, "forward", 0.5)
        recorder.record_ranger(readings, "forward", 0.5)
        recorder.stop()

        rows = _read_rows(csv_path)
        assert len(rows) == 2


class TestRecordStabilizer:
    def test_writes_stabilizer_row(self, tmp_path):
        csv_path = tmp_path / "run_telemetry.csv"
        recorder = FlightRecorder()
        recorder.start(csv_path)

        state = DroneState(
            roll_deg=1.5, pitch_deg=-2.0, yaw_deg=10.0, height_mm=300, battery_v=3.9
        )
        recorder.record_stabilizer(state)
        recorder.stop()

        rows = _read_rows(csv_path)
        assert len(rows) == 1
        row = rows[0]
        assert row["source"] == "stabilizer"
        assert row["context"] == "poll"
        assert row["roll_deg"] == "1.5"
        assert row["pitch_deg"] == "-2.0"
        assert row["height_mm"] == "300"
        assert row["battery_v"] == "3.9"
        assert row["front"] == ""


class TestStop:
    def test_stop_is_safe_without_start(self):
        recorder = FlightRecorder()

        recorder.stop()  # should not raise

    def test_record_after_stop_is_a_silent_no_op(self, tmp_path):
        """Writes after stop() (or before start()) are silently dropped
        rather than raising — a recorder in an unusable state must never
        crash a flight.
        """
        csv_path = tmp_path / "run_telemetry.csv"
        recorder = FlightRecorder()
        recorder.start(csv_path)
        recorder.stop()

        readings = MultiRangerReadings(front=0.5, back=None, left=None, right=None, up=None)
        recorder.record_ranger(readings, "forward", 0.5)  # should not raise

        rows = _read_rows(csv_path)
        assert rows == []

    def test_record_before_start_is_a_silent_no_op(self):
        recorder = FlightRecorder()

        readings = MultiRangerReadings(front=0.5, back=None, left=None, right=None, up=None)
        recorder.record_ranger(readings, "forward", 0.5)  # should not raise


class TestTimestamp:
    def test_timestamp_column_is_populated(self, tmp_path, mocker):
        csv_path = tmp_path / "run_telemetry.csv"
        mocker.patch("Crazyflie.telemetry.flight_recorder.time.time", return_value=12345.678)
        recorder = FlightRecorder()
        recorder.start(csv_path)

        readings = MultiRangerReadings(front=0.5, back=None, left=None, right=None, up=None)
        recorder.record_ranger(readings, "forward", 0.5)
        recorder.stop()

        rows = _read_rows(csv_path)
        assert rows[0]["timestamp"] == "12345.678"
