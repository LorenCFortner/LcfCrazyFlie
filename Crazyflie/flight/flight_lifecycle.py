"""Shared connect-to-landing flight lifecycle for Crazyflie 2.0 runners.

Extracted from out_and_back_runner.py and wall_follow_runner.py, which had
independently grown near-identical connect -> arm -> clearance-check ->
telemetry -> monitors -> takeoff-verify -> teardown boilerplate around two
very different flight bodies (a pre-planned FlightStep path vs. reactive
wall-following). run_flight_lifecycle() owns everything both runners need
identically; each runner supplies only its flight-specific middle section as
a flight_body_fn, plus any FlightLifecycleHooks it needs.

This is a pure extraction — no behavioural change to either existing runner.
"""

from __future__ import annotations

import logging
import queue
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.led_ring import LedRingDeck
from Crazyflie.flight.collision_return import CollisionContext, OnCollisionFn
from Crazyflie.flight.safe_flight_controller import SafeFlightController
from Crazyflie.safety.adaptive_path_corrector import AdaptivePathCorrector
from Crazyflie.safety.clearance_check import check_preflight_clearance
from Crazyflie.safety.collision_monitor import CollisionMonitor
from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery
from Crazyflie.safety.takeoff_verifier import verify_takeoff
from Crazyflie.state.flight_state import FlightState
from Crazyflie.telemetry.flight_recorder import FlightRecorder
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

logger = logging.getLogger(__name__)

_POST_DISCONNECT_SLEEP_S: float = 5.0  # Allow drone radio to reset before next run.
_STABILIZE_STEPS: int = 3  # One-second hover/log steps before the flight body runs.

# CollisionMonitor._trigger() sets is_triggered()=True before its blocking
# avoidance move finishes and queues "COLLISION" — worst case ~0.3-0.7 s at
# MAX_SAFE_VELOCITY_M_S. This is how long a post-body event check should block
# for that event to actually arrive, so should_abort()==True is never followed
# by a landing sequence that races the avoidance move still driving mc from
# CollisionMonitor's own thread. Exported for flight_body_fn implementations
# to pass as handle_safety_events()'s block_timeout_s.
EVENT_WAIT_TIMEOUT_S: float = 1.5


def _never_abort() -> bool:
    """Fallback should_abort passed to on_collision_fn when no monitor is available."""
    return False


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


@dataclass
class FlightLifecycleHooks:
    """Optional per-runner hooks for run_flight_lifecycle().

    Attributes:
        pre_flight_fn: Called after clearance check and before takeoff to
            configure the drone (e.g. LED effects). Defaults to headlights-on
            + effect 7 + brightness 31 when None.
        post_flight_fn: Called after landing to clean up (e.g. LED off).
            Defaults to turning off the LED ring when None.
        make_adaptive_corrector: Optional factory invoked with the connected
            SyncCrazyflie and the shared FlightState once both exist, to
            construct an AdaptivePathCorrector for runners that use one (path
            flights, for straight-line drift correction). Left None for
            flight modes that don't need it (e.g. wall-following, whose own
            closed loop already handles heading/standoff correction) — no
            AdaptivePathCorrector is constructed or started/stopped in that
            case.
    """

    pre_flight_fn: Callable[[SyncCrazyflie], None] | None = None
    post_flight_fn: Callable[[SyncCrazyflie], None] | None = None
    make_adaptive_corrector: (
        Callable[[SyncCrazyflie, FlightState], AdaptivePathCorrector] | None
    ) = None


@dataclass
class FlightContext:
    """Everything a flight_body_fn needs to fly and to react to safety events.

    Passed to run_flight_lifecycle()'s flight_body_fn once takeoff has been
    verified. Bundled into one object rather than passed as several separate
    positional parameters, since every flight body needs the full set: mc,
    flight_state, collision_monitor and adaptive_corrector to fly, plus
    event_queue and stabilizer_monitor so its own post-body
    `if should_abort(): handle_safety_events(...)` check (each flight body
    owns that check itself — see handle_safety_events()'s docstring) has
    everything it needs.

    Attributes:
        mc: Active MotionCommander instance.
        flight_state: Shared FlightState written by whichever component is
            actively commanding motion (SafeFlightController, WallFollower).
        event_queue: Queue receiving CRASH / BATLOW / COLLISION messages from
            the running monitors.
        stabilizer_monitor: Running StabilizerMonitor (stopped by
            handle_safety_events on an event).
        collision_monitor: Running, attached CollisionMonitor.
        adaptive_corrector: The AdaptivePathCorrector built from
            FlightLifecycleHooks.make_adaptive_corrector, or None when no
            hook was given.
        should_abort: True if collision_monitor or stabilizer_monitor has
            triggered.
    """

    mc: MotionCommander
    flight_state: FlightState
    event_queue: queue.Queue[str]
    stabilizer_monitor: StabilizerMonitor
    collision_monitor: CollisionMonitor
    adaptive_corrector: AdaptivePathCorrector | None
    should_abort: Callable[[], bool]


FlightBodyFn = Callable[[FlightContext], None]


def handle_safety_events(
    event_queue: queue.Queue[str],
    mc: MotionCommander,
    stabilizer_monitor: StabilizerMonitor,
    controller: SafeFlightController | None = None,
    collision_monitor: CollisionMonitor | None = None,
    on_collision_fn: OnCollisionFn | None = None,
    flight_state: FlightState | None = None,
    adaptive_corrector: AdaptivePathCorrector | None = None,
    block_timeout_s: float = 0.0,
) -> bool:
    """Drain the event queue and react to any CRASH/BATLOW/COLLISION event.

    Shared by every flight_body_fn — each one calls this itself right after
    its own `if should_abort():` check, rather than run_flight_lifecycle()
    calling it automatically after the body returns. The body is what
    constructs any SafeFlightController (and therefore owns flight_log) and
    knows its own on_collision_fn / retrace semantics, so it is the natural
    owner of this call.

    On a COLLISION event: if on_collision_fn is given, it is invoked with a
    CollisionContext built from controller.flight_log (or an empty log when
    controller is None); collision_monitor is reset first (if given) so a
    second collision during the response is detectable via its own
    is_triggered method, which is passed as on_collision_fn's should_abort.
    Without on_collision_fn, a COLLISION always just lands in place
    (mc.land()) — the behaviour every flight mode had before any of them
    grew a collision-response hook, and still the only behaviour
    wall-following uses (a wall follow has no recorded path to retrace).

    Args:
        event_queue: Queue receiving CRASH / BATLOW / COLLISION messages.
        mc: Active MotionCommander instance.
        stabilizer_monitor: Running StabilizerMonitor (stopped on event).
        controller: SafeFlightController whose flight_log describes progress
            so far. Required (alongside on_collision_fn) to build a
            CollisionContext with a non-empty flight_log; the context carries
            an empty flight_log when omitted.
        collision_monitor: CollisionMonitor to re-arm (reset()) before
            invoking on_collision_fn, so a second collision during the
            response is detectable via its is_triggered method. Without it,
            on_collision_fn receives an always-False should_abort.
        on_collision_fn: Optional callable invoked on COLLISION instead of
            mc.land(). Receives the MotionCommander, a CollisionContext, a
            should_abort callable, the shared FlightState, and the shared
            AdaptivePathCorrector.
        flight_state: Shared FlightState passed through to on_collision_fn
            so any further movement keeps CollisionMonitor's directional
            threshold accurate.
        adaptive_corrector: Shared AdaptivePathCorrector passed through to
            on_collision_fn so a retrace response can keep the same drift
            correction the outbound leg had.
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
        if on_collision_fn is not None:
            flight_log = controller.flight_log if controller is not None else []
            context = CollisionContext(flight_log=flight_log)
            if collision_monitor is not None:
                collision_monitor.reset()
                should_abort = collision_monitor.is_triggered
            else:
                should_abort = _never_abort
            logger.warning(
                f"Obstacle detected — avoidance complete, executing collision response"
                f" ({len(context.flight_log)} logged step(s))."
            )
            on_collision_fn(
                mc, context, should_abort, flight_state or FlightState(), adaptive_corrector
            )
        else:
            logger.warning("Obstacle detected — avoidance complete, landing now.")
            mc.land()
    else:
        logger.warning(f"Unknown event '{event}' — landing immediately as precaution.")
        land_immediately(mc)

    return True


def run_flight_lifecycle(
    uri: str,
    flight_body_fn: FlightBodyFn,
    hooks: FlightLifecycleHooks | None = None,
    telemetry_file: Path | None = None,
) -> None:
    """Execute a flight's full connect-to-landing lifecycle.

    Connects to the drone, checks clearance, starts safety monitors, verifies
    takeoff, then calls flight_body_fn to fly whatever this flight mode does
    — then tears everything down. Blocks until the drone has disconnected and
    the post-disconnect sleep has elapsed.

    Args:
        uri: Radio or USB URI to connect to.
        flight_body_fn: Called once takeoff is verified, with a FlightContext
            bundling the MotionCommander, shared FlightState, event queue,
            and running monitors. Responsible for flying, for its own
            should_abort() check afterward (calling handle_safety_events()
            itself if it fires), and for any flight-specific logging.
        hooks: Optional FlightLifecycleHooks. Defaults to
            FlightLifecycleHooks() (LED pre/post-flight defaults, no
            AdaptivePathCorrector) when None.
        telemetry_file: Optional path to write continuous sensor telemetry
            (every Multi-ranger reading and stabilizer sample, not just
            WARNING-level events) to as CSV. When None, no telemetry is
            recorded.
    """
    _hooks = hooks if hooks is not None else FlightLifecycleHooks()
    _pre = _hooks.pre_flight_fn if _hooks.pre_flight_fn is not None else _default_pre_flight
    _post = _hooks.post_flight_fn if _hooks.post_flight_fn is not None else _default_post_flight

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

        adaptive_corrector: AdaptivePathCorrector | None = None
        if _hooks.make_adaptive_corrector is not None:
            adaptive_corrector = _hooks.make_adaptive_corrector(scf, flight_state)

        collision_monitor = CollisionMonitor(
            scf,
            event_queue,
            flight_state=flight_state,
            adaptive_corrector=adaptive_corrector,
            recorder=recorder,
        )

        stabilizer_monitor.wait_for_first_reading()
        initial_battery_v = stabilizer_monitor.state.battery_v
        initial_height_mm = stabilizer_monitor.state.height_mm
        logger.info(f"Battery: {initial_battery_v:.2f} V")

        flight_start = time.time()

        def should_abort() -> bool:
            return collision_monitor.is_triggered() or stabilizer_monitor.is_triggered()

        try:
            with MotionCommander(scf) as mc:
                # Start collision monitoring (and any adaptive corrector)
                # only once airborne — clearance check already guards
                # pre-takeoff proximity, and ground-level sensor readings
                # fluctuate and can spuriously trigger a COLLISION event.
                if adaptive_corrector is not None:
                    adaptive_corrector.start()
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

                flight_body_fn(
                    FlightContext(
                        mc=mc,
                        flight_state=flight_state,
                        event_queue=event_queue,
                        stabilizer_monitor=stabilizer_monitor,
                        collision_monitor=collision_monitor,
                        adaptive_corrector=adaptive_corrector,
                        should_abort=should_abort,
                    )
                )

        except Exception as exc:
            logger.error(f"Flight error: {exc}")
        finally:
            collision_monitor.detach_motion_commander()
            collision_monitor.stop()
            collision_monitor.join()
            if adaptive_corrector is not None:
                adaptive_corrector.stop()
                adaptive_corrector.join()
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
