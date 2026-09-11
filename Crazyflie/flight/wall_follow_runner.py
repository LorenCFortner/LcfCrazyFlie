"""Right-wall-following flight runner for Crazyflie 2.0.

Provides run_wall_follow_flight() - the full connection-to-landing lifecycle
for scripts/right_wall_follow.py, flying WallFollower's search/align/follow
sequence instead of a pre-planned FlightStep path.

The connect -> clearance -> monitors -> takeoff-verify -> teardown lifecycle
itself lives in Crazyflie.flight.flight_lifecycle.run_flight_lifecycle(),
shared with Crazyflie.flight.out_and_back_runner. This module supplies only
the wall-follow-specific flight body: waiting for the first Multi-ranger
reading, then running the search -> align -> follow sequence.

CollisionMonitor is the sole owner of the Multi-ranger connection for the
whole flight (see its get_latest_readings() docstring) - WallFollower reads
sensor data through the _CollisionMonitorRangerAdapter below rather than
opening a second, unsafe connection of its own.
"""

import logging
import time
from collections.abc import Callable
from pathlib import Path

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.flight.flight_lifecycle import (
    EVENT_WAIT_TIMEOUT_S,
    FlightContext,
    FlightLifecycleHooks,
    handle_safety_events,
    run_flight_lifecycle,
)
from Crazyflie.flight.wall_follower import WallFollowConfig, WallFollower
from Crazyflie.safety.collision_monitor import CollisionMonitor

logger = logging.getLogger(__name__)

# How long get_latest_readings() may return None (no poll completed yet)
# before the search leg gives up waiting for CollisionMonitor's first
# reading. Generous relative to the 10 Hz poll rate to absorb radio jitter.
_FIRST_READING_TIMEOUT_S: float = 2.0

# How stale a reading from CollisionMonitor's poll thread may be before this
# adapter treats it as unreadable rather than steering from it. WallFollower
# actively commands motion from this reading every cycle, unlike
# CollisionMonitor's own passive detection role - a stalled poll thread (an
# exception inside the MultiRangerDeck context, a log config error) must
# degrade to "no reading" rather than leave the follower flying forever on
# one frozen snapshot. Generous relative to the 10 Hz poll rate (5 periods)
# to absorb ordinary radio jitter without false-triggering wall-lost.
_MAX_READING_AGE_S: float = 0.5


class _CollisionMonitorRangerAdapter:
    """Duck-typed stand-in for a MultiRangerDeck, backed by CollisionMonitor.

    Exposes the only method WallFollower calls on its "ranger" argument -
    get_readings() - by reading CollisionMonitor's shared latest-reading
    snapshot instead of opening a second, unsafe Multi-ranger connection.

    Before CollisionMonitor's background thread has completed its first
    poll cycle, or once a reading is older than _MAX_READING_AGE_S (the poll
    thread has stalled), get_latest_readings() returns None; this adapter
    reports that as an all-None MultiRangerReadings (the project-wide
    "nothing detected yet" sentinel), so WallFollower's readings-is-None
    handling (treating it as an unreadable/not-yet-visible sensor) applies
    naturally in both cases.
    """

    _EMPTY_READINGS = MultiRangerReadings(front=None, back=None, left=None, right=None, up=None)

    def __init__(self, collision_monitor: CollisionMonitor) -> None:
        """Initialize the adapter.

        Args:
            collision_monitor: The running CollisionMonitor whose shared
                readings this adapter exposes.
        """
        self._collision_monitor = collision_monitor

    def get_readings(self) -> MultiRangerReadings:
        """Return CollisionMonitor's most recent Multi-ranger reading.

        Returns:
            The latest MultiRangerReadings, or an all-None reading if no
            poll cycle has completed yet or the most recent one is older
            than _MAX_READING_AGE_S.
        """
        readings = self._collision_monitor.get_latest_readings(max_age_s=_MAX_READING_AGE_S)
        return readings if readings is not None else self._EMPTY_READINGS


def _wait_for_first_ranger_reading(
    collision_monitor: CollisionMonitor,
    timeout_s: float = _FIRST_READING_TIMEOUT_S,
) -> bool:
    """Block until CollisionMonitor's background thread has polled once.

    Args:
        collision_monitor: The running CollisionMonitor to poll.
        timeout_s: Maximum time to wait in seconds.

    Returns:
        True once a reading is available, False on timeout.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if collision_monitor.get_latest_readings() is not None:
            return True
        time.sleep(0.01)
    return False


def run_wall_follow_flight(
    uri: str,
    config: WallFollowConfig | None = None,
    telemetry_file: Path | None = None,
    pre_flight_fn: Callable[[SyncCrazyflie], None] | None = None,
    post_flight_fn: Callable[[SyncCrazyflie], None] | None = None,
) -> None:
    """Execute a right-wall-following flight with full safety architecture.

    Connects to the drone, checks clearance, starts safety monitors, verifies
    takeoff, flies forward to the first obstacle, aligns to 45 degrees, then
    follows the wall with WallFollower - then lands and cleans up. Blocks
    until the drone has disconnected and the post-disconnect sleep has
    elapsed.

    Args:
        uri: Radio or USB URI to connect to.
        config: Tuning parameters for WallFollower. Defaults to
            WallFollowConfig() when None.
        telemetry_file: Optional path to write continuous sensor telemetry
            to as CSV. When None, no telemetry is recorded.
        pre_flight_fn: Called after clearance check and before takeoff to
            configure the drone (e.g. LED effects). Defaults to headlights-on
            + effect 7 + brightness 31 when None.
        post_flight_fn: Called after landing to clean up (e.g. LED off).
            Defaults to turning off the LED ring when None.
    """
    cfg = config if config is not None else WallFollowConfig()

    def _flight_body(ctx: FlightContext) -> None:
        follower = WallFollower(cfg, flight_state=ctx.flight_state)
        ranger_adapter = _CollisionMonitorRangerAdapter(ctx.collision_monitor)

        def _handle_abort() -> None:
            # should_abort() just became True - a monitor may have triggered
            # mid-phase, but its event might not be queued yet (see
            # handle_safety_events' block_timeout_s docstring). Wait rather
            # than check once and miss it.
            handle_safety_events(
                ctx.event_queue,
                ctx.mc,
                ctx.stabilizer_monitor,
                block_timeout_s=EVENT_WAIT_TIMEOUT_S,
            )

        if not _wait_for_first_ranger_reading(ctx.collision_monitor):
            logger.error("No Multi-ranger reading received - aborting.")
            return

        logger.info("Searching for the first obstacle...")
        found = follower.fly_to_first_obstacle(
            ctx.mc, ranger_adapter, should_abort=ctx.should_abort
        )
        if ctx.should_abort():
            _handle_abort()
            return
        if not found:
            logger.warning(
                f"No obstacle found within {cfg.max_search_distance_m:.1f} m - landing."
            )
            return

        logger.info("Aligning to the wall...")
        aligned = follower.align_to_wall(ctx.mc, ranger_adapter)
        if ctx.should_abort():
            _handle_abort()
            return
        if not aligned:
            logger.warning("Could not align to the wall - landing.")
            return

        logger.info("Following the wall...")
        follower.follow(ctx.mc, ranger_adapter, should_abort=ctx.should_abort)

        if ctx.should_abort():
            _handle_abort()
            return

        logger.info("Wall follow complete - landing.")

    hooks = FlightLifecycleHooks(pre_flight_fn=pre_flight_fn, post_flight_fn=post_flight_fn)

    run_flight_lifecycle(uri, _flight_body, hooks=hooks, telemetry_file=telemetry_file)
