"""Step-aware collision return for Crazyflie 2.0.

Generalizes the "fly back home after a collision" response from a single
straight leg to any multi-step path (with turns) by retracing whatever was
actually flown, rather than assuming a straight outbound line.

SafeFlightController.flight_log records the steps actually executed so far -
including a partial entry for whichever step a collision interrupted, and
regardless of whether the interruption happened on the outbound leg, the
180° pivot, or an already-inverted return leg. Reversing that log and
inverting each command yields a path back to the start point.

The return leg is flown through SafeFlightController itself (not blocking
MotionCommander calls) so it stays interruptible: a second collision during
the retrace aborts it early. When that happens the response is not to keep
retracing - it is to back away from the new obstacle for clearance and stop,
leaving the drone to land wherever MotionCommander's context exit lands it.

Example:
    >>> context = CollisionContext(flight_log=controller.flight_log)
    >>> fly_home_after_collision(mc, context, should_abort=collision_monitor.is_triggered)
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.flight.path_runner import REVERSE_DIRECTION, FlightStep
from Crazyflie.flight.safe_flight_controller import SafeFlightController

if TYPE_CHECKING:
    from Crazyflie.safety.adaptive_path_corrector import AdaptivePathCorrector
    from Crazyflie.state.flight_state import FlightState

logger = logging.getLogger(__name__)

# Additional clearance backup on a second collision, mirroring
# _COLLISION_BACKUP_M in scripts/safe_fly_out_and_back.py. The collision
# monitor has already performed its own velocity-scaled avoidance reversal
# by the time this fires; this is a further fixed step back before landing.
_SECOND_COLLISION_BACKUP_M: float = 0.2
_SECOND_COLLISION_BACKUP_VELOCITY_M_S: float = 0.5


@dataclass
class CollisionContext:
    """Snapshot of flight progress at the moment a collision was detected.

    Attributes:
        flight_log: Steps actually flown so far, in order, as returned by
            SafeFlightController.flight_log - including a final partial step
            for the move the collision interrupted. Turn steps use degrees
            for distance_m; linear steps use meters.
    """

    flight_log: list[FlightStep]


# Callback invoked on a COLLISION event: receives the active MotionCommander,
# a CollisionContext describing flight progress, a should_abort callable
# backed by the (re-armed) CollisionMonitor so a second collision can be
# detected during the response, the shared FlightState so any further
# SafeFlightController movement keeps CollisionMonitor's directional
# threshold accurate, and the shared AdaptivePathCorrector (or None, when
# the runner was not given one) so the response can keep the same drift
# correction the outbound leg had.
OnCollisionFn = Callable[
    [
        MotionCommander,
        CollisionContext,
        Callable[[], bool],
        "FlightState",
        "AdaptivePathCorrector | None",
    ],
    None,
]


def build_return_path(flight_log: list[FlightStep]) -> list[FlightStep]:
    """Invert a flown path to build a route back to the start point.

    Reverses step order and inverts each command (forward<->back,
    left<->right, up<->down, turn_left<->turn_right) so re-executing the
    result retraces the flown path home. Each step keeps its original
    velocity and settle_s.

    Args:
        flight_log: Steps actually flown, in execution order.

    Returns:
        Steps that retrace flight_log in reverse, ready to pass to a new
        SafeFlightController.
    """
    return [
        FlightStep(
            REVERSE_DIRECTION.get(step.command, step.command),
            step.distance_m,
            step.velocity,
            step.settle_s,
        )
        for step in reversed(flight_log)
    ]


def fly_home_after_collision(
    mc: MotionCommander,
    context: CollisionContext,
    should_abort: Callable[[], bool],
    flight_state: FlightState | None = None,
    adaptive_corrector: AdaptivePathCorrector | None = None,
) -> None:
    """Retrace the flown path home; back away and stop on a second collision.

    Builds the return path from context.flight_log and flies it through a
    new interruptible SafeFlightController. If should_abort fires again
    during the retrace (a second collision), the retrace is abandoned: the
    drone backs up an additional fixed distance for clearance and the
    function returns, leaving MotionCommander's context exit to land.

    Args:
        mc: Active MotionCommander instance.
        context: Flight progress snapshot from the first collision.
        should_abort: Callable checked every poll cycle during the retrace.
            Pass the CollisionMonitor's is_triggered after calling reset()
            so a second collision is detectable.
        flight_state: Shared FlightState also used by the CollisionMonitor,
            so the retrace's velocity/direction keep the monitor's
            directional detection threshold accurate. Without it, the
            monitor would keep using stale values from before the collision.
        adaptive_corrector: Shared AdaptivePathCorrector also used by the
            outbound flight, so the retrace gets the same mid-step drift
            correction the outbound leg had rather than flying uncorrected.
    """
    return_steps = build_return_path(context.flight_log)
    if not return_steps:
        logger.warning("No flight log recorded before the collision - nothing to retrace.")
        return

    logger.warning(f"Collision: retracing {len(return_steps)} step(s) home.")
    return_controller = SafeFlightController(
        return_steps, flight_state=flight_state, adaptive_corrector=adaptive_corrector
    )
    return_controller.run(mc, should_abort=should_abort)

    if should_abort():
        logger.warning(
            "Second collision during return leg - backing away for clearance and stopping."
        )
        mc.back(_SECOND_COLLISION_BACKUP_M, velocity=_SECOND_COLLISION_BACKUP_VELOCITY_M_S)
    else:
        logger.info("Retrace complete - back at start point.")
