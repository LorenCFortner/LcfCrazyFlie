"""Out-and-back path flight runner for Crazyflie 2.0.

Provides run_out_and_back_flight() — the full connection-to-landing lifecycle
for path-following scripts that use SafeFlightController.run_out_and_back().

Scripts using this runner need only define their FlightStep path and call:

    run_out_and_back_flight(MY_PATH, uri=URI, description="fly my route")
"""

import logging
import queue
import time
from collections.abc import Callable

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.led_ring import LedRingDeck
from Crazyflie.flight.collision_return import CollisionContext, OnCollisionFn
from Crazyflie.flight.path_runner import FlightStep
from Crazyflie.flight.safe_flight_controller import SafeFlightController
from Crazyflie.safety.adaptive_path_corrector import AdaptivePathCorrector
from Crazyflie.safety.clearance_check import check_preflight_clearance
from Crazyflie.safety.collision_monitor import CollisionMonitor
from Crazyflie.safety.emergency_land import land_immediately, land_on_low_battery
from Crazyflie.safety.takeoff_verifier import verify_takeoff
from Crazyflie.state.flight_state import FlightState
from Crazyflie.telemetry.stabilizer_monitor import StabilizerMonitor

logger = logging.getLogger(__name__)

_POST_DISCONNECT_SLEEP_S: float = 5.0  # Allow drone radio to reset before next run.

# CollisionMonitor._trigger() sets is_triggered()=True before its blocking
# avoidance move finishes and queues "COLLISION" — worst case ~0.3-0.5 s at
# MAX_SAFE_VELOCITY_M_S. This is how long the post-flight event wait blocks
# for that event to actually arrive, so it isn't missed by a single check.
_EVENT_WAIT_TIMEOUT_S: float = 1.5


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


def _handle_safety_events(
    event_queue: queue.Queue[str],
    mc: MotionCommander,
    scf: SyncCrazyflie,
    stabilizer_monitor: StabilizerMonitor,
    controller: SafeFlightController | None = None,
    collision_monitor: CollisionMonitor | None = None,
    on_collision_fn: OnCollisionFn | None = None,
    flight_state: FlightState | None = None,
    adaptive_corrector: AdaptivePathCorrector | None = None,
    block_timeout_s: float = 0.0,
) -> bool:
    """Drain the event queue and react to any safety events.

    Args:
        event_queue: Queue receiving CRASH / BATLOW / COLLISION messages.
        mc: Active MotionCommander instance.
        scf: Connected SyncCrazyflie instance.
        stabilizer_monitor: Running StabilizerMonitor (stopped on event).
        controller: SafeFlightController whose flight_log describes progress
            so far. Required (alongside on_collision_fn) to build a
            CollisionContext on a COLLISION event; the context carries an
            empty flight_log when omitted.
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
            to appear instead of checking once. CollisionMonitor sets
            is_triggered() True before its blocking avoidance move finishes
            and queues "COLLISION", so a caller that already knows a
            monitor triggered should wait rather than check once and miss
            it. Defaults to 0.0 (non-blocking, original behaviour).

    Returns:
        True if an emergency landing was triggered, False if queue was clear.
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


def run_out_and_back_flight(
    path: list[FlightStep],
    uri: str,
    description: str,
    pre_flight_fn: Callable[[SyncCrazyflie], None] | None = None,
    post_flight_fn: Callable[[SyncCrazyflie], None] | None = None,
    on_collision_fn: OnCollisionFn | None = None,
) -> None:
    """Execute a path out-and-back flight with full safety architecture.

    Connects to the drone, checks clearance, starts safety monitors, verifies
    takeoff, runs the path via SafeFlightController.run_out_and_back(), then
    lands and cleans up. Blocks until the drone has disconnected and the
    post-disconnect sleep has elapsed.

    Args:
        path: Sequence of FlightSteps defining the outbound leg. The return
            leg is automatically generated by reversing the steps.
        uri: Radio or USB URI to connect to.
        description: Short description logged when the path begins, e.g.
            "fly 1 metre out and back".
        pre_flight_fn: Called after clearance check and before takeoff to
            configure the drone (e.g. LED effects). Defaults to headlights-on
            + effect 7 + brightness 31 when None.
        post_flight_fn: Called after landing to clean up (e.g. LED off).
            Defaults to turning off the LED ring when None.
        on_collision_fn: Called on a COLLISION event instead of mc.land().
            Receives the MotionCommander, a CollisionContext describing
            flight progress, a should_abort callable for detecting a second
            collision during the response, the shared FlightState, and the
            shared AdaptivePathCorrector. Defaults to mc.land() when None.
    """
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

        stabilizer_monitor = StabilizerMonitor(scf, event_queue)
        stabilizer_monitor.start()

        adaptive_corrector = AdaptivePathCorrector(scf, flight_state)
        controller = SafeFlightController(
            path, flight_state=flight_state, adaptive_corrector=adaptive_corrector
        )
        collision_monitor = CollisionMonitor(
            scf,
            event_queue,
            flight_state=flight_state,
            adaptive_corrector=adaptive_corrector,
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
                # Start collision monitoring only once airborne — clearance check
                # already guards pre-takeoff proximity, and ground-level sensor
                # readings fluctuate and can spuriously trigger a COLLISION event.
                adaptive_corrector.start()
                collision_monitor.start()
                collision_monitor.attach_motion_commander(mc)
                logger.info("Airborne — stabilizing for 3 seconds...")
                for i in range(3):
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

                logger.info(f"Flying {description}...")
                controller.run_out_and_back(mc, should_abort=should_abort)

                # A monitor may have triggered mid-flight, aborting the path
                # above. Its event might not be queued yet — CollisionMonitor
                # sets is_triggered() True before its blocking avoidance move
                # finishes and calls event_queue.put() — so wait rather than
                # check once. On normal completion (should_abort() False)
                # there is nothing to wait for. The collision monitor stays
                # attached through this so on_collision_fn can detect and
                # react to a second collision during its response; it is
                # detached in the finally block below once this is done.
                if should_abort():
                    _handle_safety_events(
                        event_queue,
                        mc,
                        scf,
                        stabilizer_monitor,
                        controller=controller,
                        collision_monitor=collision_monitor,
                        on_collision_fn=on_collision_fn,
                        flight_state=flight_state,
                        adaptive_corrector=adaptive_corrector,
                        block_timeout_s=_EVENT_WAIT_TIMEOUT_S,
                    )

                # A second collision inside on_collision_fn's own response
                # (e.g. during a retrace) re-arms and re-triggers
                # collision_monitor, which queues its own "COLLISION" event.
                # That event is already fully handled synchronously by
                # on_collision_fn's should_abort check — discard it here so
                # it cannot linger and be misread by a future drain.
                while not event_queue.empty():
                    event_queue.get_nowait()

                logger.info("Route complete — landing.")

        except Exception as exc:
            logger.error(f"Flight error: {exc}")
        finally:
            collision_monitor.detach_motion_commander()
            collision_monitor.stop()
            collision_monitor.join()
            adaptive_corrector.stop()
            adaptive_corrector.join()
            flight_time = time.time() - flight_start
            stabilizer_monitor.stop()
            stabilizer_monitor.join()
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
