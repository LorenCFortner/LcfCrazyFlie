"""Continuous flight telemetry recorder for Crazyflie 2.0.

Writes every sensor reading CollisionMonitor and StabilizerMonitor already
take to one CSV - not just the WARNING-level events that make it into the
text run log - so a flight can be reconstructed from data afterward instead
of guessed at from sparse event logs.

Fed by both monitors on every reading they already take; does not open its
own connection to any deck. A second, independent connection to the
Multi-ranger deck alongside CollisionMonitor's is not safe (duplicate log
configs, limited radio log bandwidth) - see Crazyflie.decks.multi_ranger.

Example:
    >>> recorder = FlightRecorder()
    >>> recorder.start(Path("scripts/logs/my_flight_telemetry.csv"))
    >>> collision_monitor = CollisionMonitor(scf, event_queue, recorder=recorder)
    >>> stabilizer_monitor = StabilizerMonitor(scf, event_queue, recorder=recorder)
    >>> # ... flight happens ...
    >>> recorder.stop()
"""

from __future__ import annotations

import csv
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, TextIO

if TYPE_CHECKING:
    from Crazyflie.decks.multi_ranger import MultiRangerReadings
    from Crazyflie.telemetry.stabilizer_monitor import DroneState

CSV_HEADER: list[str] = [
    "timestamp",
    "source",
    "context",
    "front",
    "back",
    "left",
    "right",
    "up",
    "commanded_direction",
    "commanded_velocity",
    "roll_deg",
    "pitch_deg",
    "height_mm",
    "battery_v",
]


class FlightRecorder:
    """Thread-safe CSV recorder fed by CollisionMonitor and StabilizerMonitor.

    Each call to record_ranger() / record_stabilizer() appends one row.
    Both monitors run on separate background threads, so writes are
    serialized with an internal lock. Every row is flushed immediately -
    this recorder exists specifically to diagnose crashes, so a hard
    failure must not lose the buffered tail of data.

    record_ranger() / record_stabilizer() calls before start() or after
    stop() are silent no-ops rather than errors, so a recorder left in an
    unusable state can never crash a flight.
    """

    def __init__(self) -> None:
        self._file: TextIO | None = None
        self._writer: Any = None
        self._lock = threading.Lock()

    def start(self, csv_path: Path) -> None:
        """Open csv_path for writing and write the header row.

        Creates the parent directory if missing. Overwrites (does not
        append to) any existing file at csv_path.

        Args:
            csv_path: Path to write telemetry rows to.
        """
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self._file = open(csv_path, "w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._file)
            self._writer.writerow(CSV_HEADER)
            self._file.flush()

    def record_ranger(
        self,
        readings: MultiRangerReadings,
        direction: str | None,
        velocity: float,
        context: str = "poll",
    ) -> None:
        """Append one Multi-ranger-sourced row.

        Args:
            readings: Current Multi-ranger snapshot.
            direction: What the software currently believes it's commanding
                ('forward', 'back', 'left', 'right', 'up'), or None.
            velocity: Current commanded velocity in m/s.
            context: "poll" for a routine sample (default), "trigger" for
                the reading that caused a COLLISION (taken before
                mc.stop()), or "post_stop" for the fresh re-read taken
                after mc.stop() that drives the avoidance move.
        """
        self._write_row(
            source="ranger",
            context=context,
            front=readings.front,
            back=readings.back,
            left=readings.left,
            right=readings.right,
            up=readings.up,
            commanded_direction=direction,
            commanded_velocity=velocity,
        )

    def record_stabilizer(self, state: DroneState) -> None:
        """Append one stabilizer-sourced row.

        Args:
            state: Current DroneState snapshot.
        """
        self._write_row(
            source="stabilizer",
            context="poll",
            roll_deg=state.roll_deg,
            pitch_deg=state.pitch_deg,
            height_mm=state.height_mm,
            battery_v=state.battery_v,
        )

    def stop(self) -> None:
        """Close the CSV file. Safe to call even if start() was never called."""
        with self._lock:
            if self._file is not None:
                self._file.close()
                self._file = None
                self._writer = None

    def _write_row(
        self,
        source: str,
        context: str,
        front: float | None = None,
        back: float | None = None,
        left: float | None = None,
        right: float | None = None,
        up: float | None = None,
        commanded_direction: str | None = None,
        commanded_velocity: float | None = None,
        roll_deg: float | None = None,
        pitch_deg: float | None = None,
        height_mm: int | None = None,
        battery_v: float | None = None,
    ) -> None:
        row: list[object] = [
            time.time(),
            source,
            context,
            front if front is not None else "",
            back if back is not None else "",
            left if left is not None else "",
            right if right is not None else "",
            up if up is not None else "",
            commanded_direction if commanded_direction is not None else "",
            commanded_velocity if commanded_velocity is not None else "",
            roll_deg if roll_deg is not None else "",
            pitch_deg if pitch_deg is not None else "",
            height_mm if height_mm is not None else "",
            battery_v if battery_v is not None else "",
        ]
        with self._lock:
            if self._writer is None or self._file is None:
                return
            self._writer.writerow(row)
            self._file.flush()
