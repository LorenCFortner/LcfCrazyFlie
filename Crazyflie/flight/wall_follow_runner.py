"""Right-wall-following flight runner for Crazyflie 2.0.

Provides run_wall_follow_flight() — the full connection-to-landing lifecycle
for scripts/right_wall_follow.py, mirroring
Crazyflie.flight.out_and_back_runner.run_out_and_back_flight()'s lifecycle
(connect, arm, clearance check, safety monitors, takeoff verification,
teardown) but flying WallFollower's search/align/follow sequence instead of
a pre-planned FlightStep path.

CollisionMonitor is the sole owner of the Multi-ranger connection for the
whole flight (see its get_latest_readings() docstring) — WallFollower reads
sensor data through the _CollisionMonitorRangerAdapter below rather than
opening a second, unsafe connection of its own.
"""

import logging
import queue
import time
from collections.abc import Callable
from pathlib import Path

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.led_ring import LedRingDeck
from Crazyflie.decks.multi_ranger import MultiRangerReadings
from Crazyflie.flight.wall_follower import WallFollowConfig, WallFollower
from Crazyflie.safety.clearance_check import check_preflight_clearance
from Crazyflie.safety.collision_monitor import CollisionMonitor
from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery
from Crazyflie.safety.takeoff_verifier import verify_takeoff
from Crazyflie.state.flight_state import FlightState
from Crazyflie.telemetry.flight_recorder import FlightRecorder
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

logger = logging.getLogger(__name__)

_POST_DISCONNECT_SLEEP_S: float = 5.0  # Allow drone radio to reset before next run.
_STABILIZE_STEPS: int = 3  # One-second hover/log steps before the search leg.

# How long get_latest_readings() may return None (no poll completed yet)
# before the search leg gives up waiting for CollisionMonitor's first
# reading. Generous relative to the 10 Hz poll rate to absorb radio jitter.
_FIRST_READING_TIMEOUT_S: float = 2.0

# CollisionMonitor._trigger() sets is_triggered()=True before its blocking
# avoidance move finishes and queues "COLLISION" — worst case ~0.3-0.7 s at
# MAX_SAFE_VELOCITY_M_S. This is how long a post-phase event check blocks for
# that event to actually arrive, so should_abort()==True is never followed by
# a landing sequence that races the avoidance move still driving mc from
# CollisionMonitor's own thread.
_EVENT_WAIT_TIMEOUT_S: float = 1.5


def _default_pre_flight(scf: SyncCrazyflie) -> None:
    """Arm the LED ring before takeoff.

    Args:
        scf: Connected SyncCrazyflie instance.
    """
    LedRingDeck.headlights_on(scf)
    LedRingDeck.set_effect(scf, 7)
    LedRingDeck.set_brightness(scf, 31)


def _default_post_flight(scf: SyncCrazyflie) -> None:
    """Turn off LEDs after landing.

    Args:
        scf: Connected SyncCrazyflie instance.
    """
    LedRingDeck.turn_off(scf)


# How stale a reading from CollisionMonitor's poll thread may be before this
# adapter treats it as unreadable rather than steering from it. WallFollower
# actively commands motion from this reading every cycle, unlike
# CollisionMonitor's own passive detection role — a stalled poll thread (an
# exception inside the MultiRangerDeck context, a log config error) must
# degrade to "no reading" rather than leave the follower flying forever on
# one frozen snapshot. Generous relative to the 10 Hz poll rate (5 periods)
# to absorb ordinary radio jitter without false-triggering wall-lost.
_MAX_READING_AGE_S: float = 0.5


class _CollisionMonitorRangerAdapter:
    """Duck-typed stand-in for a MultiRangerDeck, backed by CollisionMonitor.

    Exposes the only method WallFollower calls on its "ranger" argument —
    get_readings() — by reading CollisionMonitor's shared latest-reading
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
        """Initialise the adapter.

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


def _handle_safety_events(
    event_queue: queue.Queue[str],
    mc: MotionCommander,
    stabilizer_monitor: StabilizerMonitor,
    block_timeout_s: float = 0.0,
) -> bool:
    """Drain the event queue and react to any CRASH/BATLOW/COLLISION event.

    Unlike run_out_and_back_flight, a COLLISION here always lands in place:
    a wall-follow flight has no recorded path to retrace, and a side
    collision during following is a stop-and-land case, not a retrace-home
    case.

    Args:
        event_queue: Queue receiving CRASH / BATLOW / COLLISION messages.
        mc: Active MotionCommander instance.
        stabilizer_monitor: Running StabilizerMonitor (stopped on event).
        block_timeout_s: When > 0, wait up to this many seconds for an event
            to appear instead of checking once. A caller that already knows
            a monitor triggered (should_abort() is True) should block rather
            than check once and miss it — CollisionMonitor sets
            is_triggered() True before its blocking avoidance move finishes
            and queues "COLLISION". Defaults to 0.0 (non-blocking).

    Returns:
        True if a safety event was handled, False if the queue was empty.
    """
    try:
        if block_timeout_s > 0.0:
            event = event_queue.get(timeout=block_timeout_s)
        else:
            event = event_queue.get_nowait()
    except queue.Empty:
        return False

    logger.info(f"Safety event received: {event}")
    stabilizer_monitor.stop()

    if event == "CRASH":
        logger.warning("CRASH detected — emergency landing.")
        land_immediately(mc)
    elif event == "BATLOW":
        logger.warning("Low battery — landing now.")
        land_on_low_battery(mc)
    elif event == "COLLISION":
        logger.warning("Obstacle detected — avoidance complete, landing now.")
        mc.land()
    else:
        logger.warning(f"Unknown event '{event}' — landing immediately as precaution.")
        land_immediately(mc)

    return True


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
    follows the wall with WallFollower — then lands and cleans up. Blocks
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
    _pre = pre_flight_fn if pre_flight_fn is not None else _default_pre_flight
    _post = post_flight_fn if post_flight_fn is not None else _default_post_flight

    cflib.crtp.init_drivers(enable_debug_driver=False)

    event_queue: queue.Queue[str] = queue.Queue()
    flight_state = FlightState()

    logger.info(f"Connecting to {uri}...")

    with SyncCrazyflie(uri) as scf:
        logger.info("Connected.")
        scf.cf.commander.send_stop_setpoint()
        scf.cf.commander.send_notify_setpoint_stop()
        scf.cf.platform.send_arming_request(True)
        time.sleep(0.1)

        logger.info("Checking pre-flight clearance...")
        if not check_preflight_clearance(scf):
            logger.error("Pre-flight clearance check FAILED — too close to an obstacle. Aborting.")
            return
        logger.info("Clearance OK.")

        _pre(scf)

        recorder: FlightRecorder | None = None
        if telemetry_file is not None:
            recorder = FlightRecorder()
            try:
                recorder.start(telemetry_file)
            except OSError as exc:
                # Telemetry is a diagnostic nice-to-have, not a safety
                # feature — a failure here must never abort a flight that's
                # already armed, or skip the try/finally cleanup below by
                # propagating out of this `with SyncCrazyflie` block.
                logger.error(f"Failed to start telemetry recording to {telemetry_file}: {exc}")
                recorder = None

        stabilizer_monitor = StabilizerMonitor(scf, event_queue, recorder=recorder)
        stabilizer_monitor.start()

        collision_monitor = CollisionMonitor(
            scf, event_queue, flight_state=flight_state, recorder=recorder
        )
        follower = WallFollower(cfg, flight_state=flight_state)
        ranger_adapter = _CollisionMonitorRangerAdapter(collision_monitor)

        stabilizer_monitor.wait_for_first_reading()
        initial_battery_v = stabilizer_monitor.state.battery_v
        initial_height_mm = stabilizer_monitor.state.height_mm
        logger.info(f"Battery: {initial_battery_v:.2f} V")

        flight_start = time.time()

        def should_abort() -> bool:
            return collision_monitor.is_triggered() or stabilizer_monitor.is_triggered()

        try:
            with MotionCommander(scf) as mc:
                # Start collision monitoring only once airborne — clearance
                # check already guards pre-takeoff proximity, and
                # ground-level sensor readings fluctuate and can spuriously
                # trigger a COLLISION event.
                collision_monitor.start()
                collision_monitor.attach_motion_commander(mc)
                logger.info(f"Airborne — stabilizing for {_STABILIZE_STEPS} seconds...")
                for i in range(_STABILIZE_STEPS):
                    time.sleep(1.0)
                    height_cm = stabilizer_monitor.state.height_mm / 10.0
                    batt = stabilizer_monitor.state.battery_v
                    logger.info(
                        f"  Stabilizing: {i + 1}s | height: {height_cm:.1f} cm"
                        f" | battery: {batt:.2f} V"
                    )

                if not verify_takeoff(
                    height_mm=stabilizer_monitor.state.height_mm,
                    battery_v=stabilizer_monitor.state.battery_v,
                    initial_height_mm=initial_height_mm,
                    initial_battery_v=initial_battery_v,
                ):
                    return

                if not _wait_for_first_ranger_reading(collision_monitor):
                    logger.error("No Multi-ranger reading received — aborting.")
                    return

                logger.info("Searching for the first obstacle...")
                found = follower.fly_to_first_obstacle(
                    mc, ranger_adapter, should_abort=should_abort
                )
                if should_abort():
                    # should_abort() just became True — a monitor may have
                    # triggered mid-search, but its event might not be
                    # queued yet (see _handle_safety_events' block_timeout_s
                    # docstring). Wait rather than check once and miss it.
                    _handle_safety_events(
                        event_queue, mc, stabilizer_monitor, block_timeout_s=_EVENT_WAIT_TIMEOUT_S
                    )
                    return
                if not found:
                    logger.warning(
                        f"No obstacle found within {cfg.max_search_distance_m:.1f} m — landing."
                    )
                    return

                logger.info("Aligning to the wall...")
                aligned = follower.align_to_wall(mc, ranger_adapter)
                if should_abort():
                    _handle_safety_events(
                        event_queue, mc, stabilizer_monitor, block_timeout_s=_EVENT_WAIT_TIMEOUT_S
                    )
                    return
                if not aligned:
                    logger.warning("Could not align to the wall — landing.")
                    return

                logger.info("Following the wall...")
                follower.follow(mc, ranger_adapter, should_abort=should_abort)

                if should_abort():
                    _handle_safety_events(
                        event_queue, mc, stabilizer_monitor, block_timeout_s=_EVENT_WAIT_TIMEOUT_S
                    )
                    return

                logger.info("Wall follow complete — landing.")

        except Exception as exc:
            logger.error(f"Flight error: {exc}")
        finally:
            collision_monitor.detach_motion_commander()
            collision_monitor.stop()
            collision_monitor.join()
            flight_time = time.time() - flight_start
            stabilizer_monitor.stop()
            stabilizer_monitor.join()
            if recorder is not None:
                recorder.stop()
            try:
                _post(scf)
            except Exception:
                pass
            try:
                scf.cf.commander.send_stop_setpoint()
                scf.cf.commander.send_notify_setpoint_stop()
            except Exception:
                pass
            logger.info(f"Total flight time: {flight_time:.1f} s")

    time.sleep(_POST_DISCONNECT_SLEEP_S)
