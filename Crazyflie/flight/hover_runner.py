"""Hover flight runner for Crazyflie 2.0.

Provides run_hover_flight() - the full connection-to-landing lifecycle for
scripts/hover_test.py: take off, hold position for a fixed time, then land.

The connect -> clearance -> monitors -> takeoff-verify -> teardown lifecycle
itself lives in Crazyflie.flight.flight_lifecycle.run_flight_lifecycle(),
shared with the other flight runners. This module supplies only the hover
flight body.
"""

import logging
import time
from pathlib import Path

from Crazyflie.flight.flight_lifecycle import (
    EVENT_WAIT_TIMEOUT_S,
    FlightContext,
    handle_safety_events,
    run_flight_lifecycle,
)
from Crazyflie.hardware.drone_connection import ConnectFn

logger = logging.getLogger(__name__)

# Sleep slice between should_abort() checks while hovering.
_POLL_INTERVAL_S: float = 0.1

# A status line is logged every this many seconds of hover.
_STATUS_INTERVAL_S: float = 2.0


def run_hover_flight(
    uri: str,
    hover_duration_s: float,
    telemetry_file: Path | None = None,
    connect_fn: ConnectFn | None = None,
) -> None:
    """Execute a take-off, hover and landing with the full safety architecture.

    Connects to the drone, checks clearance, starts safety monitors, verifies
    takeoff, hovers for hover_duration_s, then lands and cleans up. A safety
    event during the hover ends it early and is handled by
    handle_safety_events() (a COLLISION lands in place).

    Args:
        uri: Radio or USB URI to connect to.
        hover_duration_s: How long to hover, in seconds. A value of zero or
            less hovers for no time at all.
        telemetry_file: Optional path to write continuous sensor telemetry
            to as CSV. When None, no telemetry is recorded.
        connect_fn: Optional factory returning a context manager that yields the
            DroneConnection to fly. Defaults to connecting over cflib when None.
    """

    def _flight_body(ctx: FlightContext) -> None:
        def _handle_abort() -> None:
            # should_abort() just became True, but the monitor's event might
            # not be queued yet - wait for it (see handle_safety_events).
            handle_safety_events(
                ctx.event_queue,
                ctx.mc,
                ctx.stabilizer_monitor,
                block_timeout_s=EVENT_WAIT_TIMEOUT_S,
            )

        logger.info(f"Hovering for {hover_duration_s:.0f} seconds...")

        start_s = time.monotonic()
        last_status_s = -1
        elapsed_s = 0.0

        while elapsed_s < hover_duration_s:
            if ctx.should_abort():
                _handle_abort()
                return

            elapsed_int = int(elapsed_s)
            if elapsed_int % _STATUS_INTERVAL_S == 0 and elapsed_int != last_status_s:
                last_status_s = elapsed_int
                battery_v = ctx.stabilizer_monitor.state.battery_v
                logger.info(
                    f"  Hover: {elapsed_int}s / {hover_duration_s:.0f}s  "
                    f"| battery: {battery_v:.2f} V"
                )

            time.sleep(_POLL_INTERVAL_S)
            elapsed_s = time.monotonic() - start_s

        # A monitor may have triggered during the final sleep slice.
        if ctx.should_abort():
            _handle_abort()
            return

        logger.info("Hover complete - landing.")

    run_flight_lifecycle(uri, _flight_body, telemetry_file=telemetry_file, connect_fn=connect_fn)
