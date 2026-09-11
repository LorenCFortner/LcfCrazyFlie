"""Right-wall-following control loop for Crazyflie 2.0.

Flies forward until an obstacle is found, rotates counter-clockwise until
the front and right Multi-ranger sensors read equal distance (45 deg to the
wall), then flies that 45 deg diagonal (forward and left simultaneously)
along the wall - continuously yawing to hold front == right (heading) and
holding their common value at a target distance (standoff) - using a
closed-loop velocity command recomputed every poll cycle from fresh sensor
readings, rather than a pre-planned sequence of blocking steps.

Geometry (body frame: +x forward, +y left):

When front and right report an equal distance d to the same flat wall, the
drone sits at 45 deg to it. The wall's inward normal (drone -> wall) is the
bisector of +x and -y: n_hat = (1/sqrt2, -1/sqrt2). The direction *along*
the wall is perpendicular to that: (1/sqrt2, 1/sqrt2) -- forward and left in
equal measure, which is exactly the diagonal this loop flies. The
perpendicular standoff from the wall is d / sqrt2.

Error signals:
  - Heading error = front - right. Yawing further left (CCW) swings front
    away from the wall and right toward it, so a positive heading error
    means "yawed too far left" -> correct by yawing right (a negative rate).
  - Standoff error = mean(front, right) - target_wall_distance_m. Positive
    means too far from the wall -> add velocity toward it (+n_hat).

An inside corner or any obstacle closing in ahead grows the heading error
(front drops relative to right) and the loop yaws to correct -- but yaw rate
is capped (max_yaw_rate_deg_s) and cannot always complete the turn before
front reaches CollisionMonitor's own threshold if forward push continues
unabated (observed on hardware -- see scripts/logs/right_wall_follow.log,
08:30:36 run). To generalize beyond a fixed-speed continuing push, front
proximity throttles the along-wall speed term directly: as front closes
toward CollisionMonitor's own leading-sensor threshold, the forward-push
component of the velocity command (v_follow) is scaled down proportionally,
reaching zero at the threshold itself. This applies to any close-ahead
reading -- a corner, a protrusion, anything -- with no classification of
*why* front is close, mirroring AdaptivePathCorrector's velocity-scaled
buffer zone (see Crazyflie.safety.adaptive_path_corrector) applied to this
loop's own control law instead. See WallFollowConfig.front_brake_zone_m and
compute_follow_command() for the exact formula. This is an approximation,
not a guarantee -- CollisionMonitor remains the untouched, authoritative
backstop.

A single instantaneous front reading is not a reliable brake-release signal
at a corner: the front beam sweeps across the corner's edge as the yaw
correction hunts toward the true heading, so front can flicker between
"close" and "wide open" (2+ m) within consecutive 100 ms polls even though
the drone itself hasn't moved anywhere near that far (observed on hardware
-- see scripts/logs/right_wall_follow.log, 12:30 run: front swung between
0.26 m and 2.3 m repeatedly in the ~2.5 s before a COLLISION trigger). If
the brake fully releases the instant one poll reads clear, the loop
re-commands full follow_velocity_m_s right before the next flicker back to
"close" -- exactly what preceded that collision. v_follow may therefore
drop instantly (braking stays immediate -- the safety-critical direction)
but may only climb back up by a bounded step per cycle, spread over
_REACTION_S (reused from CollisionMonitor, not a new constant) -- see
compute_follow_command()'s previous_v_follow parameter. follow() tracks the
real v_follow across its loop and feeds it back in each cycle; an isolated
call (e.g. in a test) is unaffected -- previous_v_follow defaults to None,
meaning "no slew limit".

A missing front reading (None, or <= 0.0) means "nothing within
MAX_RANGE_M" per Crazyflie.decks.multi_ranger's own convention -- not a
fault, and not "the wall is lost" (observed on hardware turning this into a
false wall-lost stop -- see scripts/logs/right_wall_follow.log, 08:58:12
run: front went permanently None mid-corner while right stayed valid, and
wall_lost_timeout_s fired). right is unaffected by any of this: right is
the wall actually being followed, so a missing right reading still means
the wall is genuinely lost. This point is unaffected by everything below --
a missing front only ever substitutes MAX_RANGE_M into the heading,
standoff and front-proximity-brake formula, exactly like any other far
reading.

Two earlier attempts to handle a wall that isn't reachable on the diagonal
-- a straight-line "hold" once front reads far, then a "reacquire" once
right recedes -- both proved unreliable on hardware and have been replaced
by an explicit, bounded, four-phase corner-negotiation maneuver instead of
either passively coasting or open-endedly chasing a heading that may never
resolve. See scripts/logs/right_wall_follow.log's 16:12 run: a "hold" that
exits only on a clean single-cycle receding-delta in right can end up
holding (yaw at 0.0, no standoff correction -- flying dead straight) for
far longer than intended when right recedes gradually and noisily instead
of in one clean jump, drifting the drone far off the original wall
(observed: right growing past 3x the target following distance) before it
flew straight into the next wall it happened to be pointed at.

Outward-corner negotiation triggers on a front spike: a jump in front too
large to be explained by translation at these speeds (see is_spike() --
default spike_threshold_m=0.5 m against a ~0.03 m/cycle physical limit),
confirmed for spike_confirm_cycles consecutive polls, and only when front
was outside the front-proximity brake zone immediately beforehand
(front_threshold + front_brake_zone_m, 0.40 m at today's defaults). That
last condition matters: scripts/logs/right_wall_follow.log's 12:30 run
shows front flickering between 0.26 m and 2.3 m at an *inside* corner
(the beam sweeping across the corner's edge as yaw hunts for the true
heading) -- always starting from inside the brake zone, so it does not
qualify as an outward-corner spike and is left to the front-proximity
brake and normal heading/standoff formula, unaffected by any of this.

Once confirmed, follow() hands off to _negotiate_outer_corner(), which
runs a fixed sequence instead of an open-ended wait:

  1. _rotate_perpendicular_to_wall(): a stationary 45 deg CCW turn. At the
     normal follow attitude the wall's inward normal bisects +x and -y;
     turning 45 deg CCW puts that normal on -y, so right now points
     perpendicular at the wall and the drone flies "forward", parallel to
     the wall, continuing in the same direction of travel.
  2. _advance_past_corner_apex(): fly straight forward (front leading,
     right a side sensor) at follow_velocity_m_s, watching right for the
     same kind of spike front just showed -- once right spikes too, the
     drone has passed the corner's edge laterally, not just visually via
     front. Bounded by max_corner_advance_m.
  3. _arc_around_corner(): a constant-radius clockwise arc (forward
     velocity plus a matching negative yaw rate) with radius equal to the
     wall's target perpendicular standoff (target_wall_distance_m /
     sqrt(2)), continuing until front picks up a wall at or inside
     target_wall_distance_m -- geometrically the point on the arc where
     the drone is back at the 45 deg follow attitude relative to the new
     wall face. Bounded by max_corner_arc_deg.
  4. Control returns to the ordinary follow() loop, which resumes
     compute_follow_command()-driven flight at follow_velocity_m_s (no
     brake on the first cycle back, since the arc was already flying at
     that speed).

Each phase returns False (never calling mc.stop() itself -- follow() alone
owns that single call) on should_abort()/is_too_close() firing or its own
bound being exceeded; follow() treats a False from any phase as reason to
end the flight, the same as any other stop condition.

Blade protection: full five-sensor CollisionMonitor detection stays active
throughout (see Crazyflie.safety.collision_monitor's "forward_left" support)
as the sole general-purpose backstop; the follow() loop *additionally*
checks all five readings itself every cycle via is_too_close(), since the
followed wall on the right is deliberately held close and so is not covered
by a leading-edge threshold -- if the standoff control drifts inward, this
catches it before the blade floor does. Neither layer replaces the other.

Losing the wall (right unreadable for wall_lost_timeout_s -- a missing
front alone no longer counts, see above) or a proximity abort both stop the
flight rather than search for the wall again -- chasing a lost wall (right
itself gone, e.g. the wall ends and nothing is in range on that side
either) is out of scope.

The "ranger" argument accepted by fly_to_first_obstacle(), align_to_wall()
and follow() is any RangerSource (see below) - in production this is
Crazyflie.flight.wall_follow_runner's _CollisionMonitorRangerAdapter, which
reads CollisionMonitor's already-open Multi-ranger connection rather than
opening a second one (unsafe - see CollisionMonitor.get_latest_readings).

Example:
    >>> follower = WallFollower(WallFollowConfig(), flight_state=flight_state)
    >>> with MultiRangerDeck(scf) as ranger:  # or any other RangerSource
    ...     if follower.fly_to_first_obstacle(mc, ranger, should_abort):
    ...         follower.align_to_wall(mc, ranger)
    ...         follower.follow(mc, ranger, should_abort)
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from cflib.positioning.motion_commander import MotionCommander

from Crazyflie.decks.multi_ranger import MAX_RANGE_M, MultiRangerReadings
from Crazyflie.safety.collision_monitor import _BASE_DETECTION_M, _REACTION_S
from Crazyflie.state.flight_state import FlightState

logger = logging.getLogger(__name__)

_POLL_INTERVAL_S: float = 0.10  # 10 Hz -- matches CollisionMonitor's poll rate
_SQRT2: float = math.sqrt(2.0)

# Outward-corner negotiation's phase 1 rotation -- see the module docstring
# and WallFollower._rotate_perpendicular_to_wall().
_CORNER_ROTATION_DEG: float = 45.0


def is_spike(current: float | None, previous: float | None, threshold_m: float) -> bool:
    """Return True if current reads at least threshold_m farther than previous.

    A discontinuous jump (e.g. a sensor beam sweeping past a corner's edge
    into open space), not gradual closing/opening distance -- see the
    module docstring's corner-negotiation section for the physics
    reasoning behind a given threshold_m.

    A current reading of None/<=0.0 (nothing within MAX_RANGE_M) counts as
    a spike whenever previous was valid -- going out of range entirely is
    the extreme case of "got much farther". A previous reading of
    None/<=0.0 never counts as a spike regardless of current -- there is
    no baseline to compare against yet.

    Args:
        current: This cycle's reading in meters, or None/<=0.0 if nothing
            is within MAX_RANGE_M.
        previous: The prior reading to compare against, in meters, or
            None/<=0.0 if there is no valid baseline yet.
        threshold_m: Minimum increase (current - previous) to count as a
            spike. The comparison is inclusive (>=).

    Returns:
        True if current is at least threshold_m farther than previous (or
        current is missing while previous was valid); False otherwise,
        including whenever previous itself is missing.
    """
    if previous is None or previous <= 0.0:
        return False
    if current is None or current <= 0.0:
        return True
    return (current - previous) >= threshold_m


class RangerSource(Protocol):
    """Anything that can supply a MultiRangerReadings snapshot on demand.

    MultiRangerDeck satisfies this protocol, but so does
    Crazyflie.flight.wall_follow_runner's _CollisionMonitorRangerAdapter,
    which reads CollisionMonitor's shared latest reading instead of opening
    a second, unsafe Multi-ranger connection (CollisionMonitor is the sole
    owner of that connection for the duration of a flight). WallFollower
    only ever calls get_readings() on its "ranger" argument, so it accepts
    this narrower protocol rather than the concrete MultiRangerDeck class.
    """

    def get_readings(self) -> MultiRangerReadings:
        """Return the current Multi-ranger snapshot."""
        ...


# FlightState direction value for the 45 deg diagonal -- see
# Crazyflie.safety.collision_monitor's _FLIGHT_DIR_TO_SENSORS, which treats
# both "front" and "left" as leading sensors for this direction.
FLIGHT_DIRECTION: str = "forward_left"


@dataclass
class WallFollowConfig:
    """Tuning parameters for WallFollower.

    Attributes:
        target_wall_distance_m: The front == right distance to hold. The
            actual perpendicular standoff from the wall is this / sqrt(2).
        follow_velocity_m_s: Along-wall speed once following has started.
        max_velocity_m_s: Hard clamp on the total commanded speed
            (hypot(vx, vy)).
        standoff_gain: Proportional gain converting standoff error (meters)
            into a velocity correction toward/away from the wall (m/s).
        yaw_gain_deg_per_m: Proportional gain converting heading error
            (front - right, meters) into a yaw rate correction (deg/s).
        max_yaw_rate_deg_s: Hard clamp on the commanded yaw rate.
        align_tolerance_m: How close front and right must read to each
            other, in meters, to be considered aligned.
        align_step_deg: Rotation per incremental turn_left() during
            alignment.
        max_align_deg: Total rotation budget before alignment gives up.
        approach_velocity_m_s: Forward speed while searching for the wall.
        max_search_distance_m: Distance budget before the search gives up.
        follow_duration_s: Maximum time to spend following before stopping.
        wall_lost_timeout_s: How long right (the wall actually being
            followed) may be unreadable before the follow loop gives up
            and stops. A missing front alone does not count -- see the
            module docstring.
        abort_distance_m: WallFollower's own proximity check (is_too_close)
            -- ends the flight if any of the five sensors reads below this,
            independent of and in addition to CollisionMonitor.
        front_brake_zone_m: Width of the front-proximity brake zone above
            CollisionMonitor's own leading-sensor threshold
            (max(_BASE_DETECTION_M, follow_velocity_m_s * _REACTION_S)). The
            forward-push component of the velocity command scales linearly
            from full speed at the zone's outer edge to zero at the
            threshold itself -- see compute_follow_command() and the module
            docstring.
        spike_threshold_m: Minimum increase (is_spike()'s threshold_m) for a
            reading to count as a discontinuous jump rather than gradual
            change. Used both for follow()'s front-spike outward-corner
            trigger and _advance_past_corner_apex()'s right-spike
            edge-passed check -- one constant for both. See is_spike() and
            the module docstring.
        spike_confirm_cycles: Consecutive polls a front spike must persist
            (measured against the pre-spike baseline) before follow() starts
            outward-corner negotiation -- filters a single noisy poll.
        max_corner_advance_m: Distance budget for
            _advance_past_corner_apex() (phase 2) before giving up -- the
            corner's lateral edge is geometrically only about one
            perpendicular standoff (target_wall_distance_m / sqrt(2)) ahead
            when front spikes, so this is deliberately generous.
        max_corner_arc_deg: Rotation budget for _arc_around_corner()
            (phase 3) before giving up -- a 90 deg corner needs roughly
            90 deg of arc, so this is deliberately generous.

    Velocities are deliberately low. At follow_velocity_m_s = 0.15 m/s the
    estimated stopping distance (~4 cm, extrapolated from the repo's tuning
    data at 0.3 m/s) fits comfortably inside both the leading threshold
    (0.25 m) and the additive side threshold (~0.20 m) with margin to spare,
    and is small enough that even a 2x error in that extrapolation is
    absorbed harmlessly -- unlike faster speeds, where the margin depends
    entirely on the coast estimate being right.
    """

    target_wall_distance_m: float = 0.60
    follow_velocity_m_s: float = 0.15
    max_velocity_m_s: float = 0.20
    standoff_gain: float = 0.5
    yaw_gain_deg_per_m: float = 120.0
    max_yaw_rate_deg_s: float = 45.0
    align_tolerance_m: float = 0.05
    align_step_deg: float = 5.0
    max_align_deg: float = 120.0
    approach_velocity_m_s: float = 0.30
    max_search_distance_m: float = 4.0
    follow_duration_s: float = 45.0
    wall_lost_timeout_s: float = 1.5
    abort_distance_m: float = 0.25
    front_brake_zone_m: float = 0.15
    spike_threshold_m: float = 0.5
    spike_confirm_cycles: int = 2
    max_corner_advance_m: float = 1.5
    max_corner_arc_deg: float = 180.0


@dataclass
class FollowCommand:
    """A single cycle's velocity command from compute_follow_command.

    Attributes:
        vx: Forward velocity component in m/s (body frame).
        vy: Left velocity component in m/s (body frame).
        yaw_rate_deg_s: Commanded yaw rate in deg/s (positive = left/CCW).
        wall_visible: False when right (the wall actually being followed)
            was unreadable this cycle, in which case vx, vy and
            yaw_rate_deg_s are all 0.0. A missing front does not affect
            this -- it is treated as a very far reading (MAX_RANGE_M), not
            as the wall being lost.
        v_follow: The along-wall forward-push term actually used this
            cycle, after the front-proximity brake and slew limit -- feed
            this back in as the next call's previous_v_follow to keep the
            slew limit continuous across follow()'s loop. Always 0.0 when
            wall_visible is False.
    """

    vx: float
    vy: float
    yaw_rate_deg_s: float
    wall_visible: bool
    v_follow: float = 0.0


class WallFollower:
    """Closed-loop right-wall-following control.

    compute_follow_command() is a pure function of (front, right) plus
    config -- no MotionCommander, thread, sleep, or drone reference -- so
    the control law itself is fully unit-testable. fly_to_first_obstacle(),
    align_to_wall() and follow() are the thin I/O wrappers that drive a real
    (or mocked) MotionCommander and RangerSource from it.
    """

    def __init__(
        self,
        config: WallFollowConfig | None = None,
        flight_state: FlightState | None = None,
    ) -> None:
        """Initialize the wall follower.

        Args:
            config: Tuning parameters. Defaults to WallFollowConfig() when
                None.
            flight_state: Optional shared FlightState. When provided,
                direction and velocity are written before every movement so
                CollisionMonitor's velocity-scaled thresholds stay accurate.
        """
        self._config = config if config is not None else WallFollowConfig()
        self._flight_state = flight_state

    def _front_threshold(self) -> float:
        """Return CollisionMonitor's own leading-sensor threshold at
        follow_velocity_m_s: max(_BASE_DETECTION_M, follow_velocity_m_s *
        _REACTION_S).

        Shared by the front-proximity brake in compute_follow_command() and
        follow()'s outward-corner spike discriminator (a spike only
        qualifies as a corner if front was outside this threshold's brake
        zone beforehand -- front_threshold + front_brake_zone_m) so the two
        stay consistent by construction rather than by two separately
        maintained formulas.

        Returns:
            The dynamic front detection threshold in meters at this
            follower's configured follow_velocity_m_s.
        """
        cfg = self._config
        return max(_BASE_DETECTION_M, cfg.follow_velocity_m_s * _REACTION_S)

    def compute_follow_command(
        self,
        front: float | None,
        right: float | None,
        previous_v_follow: float | None = None,
    ) -> FollowCommand:
        """Compute the velocity command for one control cycle.

        Pure function of the two sensor readings, previous_v_follow, and
        this follower's config -- see the module docstring for the
        geometry and error-signal derivation and the forward-push slew
        limit. Outward-corner negotiation (a front spike too large to be
        gradual closing/opening distance) is handled separately by
        follow() and _negotiate_outer_corner(), not by this function -- see
        the module docstring's corner-negotiation section.

        Args:
            front: Front Multi-ranger distance in meters, or None/<=0.0 if
                nothing is within MAX_RANGE_M -- treated as a very far
                reading (MAX_RANGE_M), not as unreadable. See module
                docstring.
            right: Right Multi-ranger distance in meters, or None/<=0.0 if
                unreadable -- right is the wall actually being followed, so
                a missing reading here means the wall is genuinely lost.
            previous_v_follow: The v_follow this loop actually used last
                cycle. When given, an *increase* in the front-proximity
                brake's target v_follow is capped to one slew step this
                cycle (see the module docstring's front-proximity-brake
                paragraph) -- a *decrease* (braking) is never delayed.
                Defaults to None, meaning "no slew limit" -- an isolated
                call (e.g. in a test) reaches the target in one step, same
                as before this parameter existed. follow() is the one that
                tracks and passes the real previous cycle's value.

        Returns:
            FollowCommand with wall_visible=False (and all-zero velocity,
            including v_follow) when right is missing; otherwise the
            computed heading + standoff correction, clamped to
            max_velocity_m_s and max_yaw_rate_deg_s. A missing front is
            substituted with MAX_RANGE_M before this computation, so it
            still yields wall_visible=True and a live command -- typically
            a hard yaw back toward the wall. The forward-push term is
            additionally throttled by front proximity and slew-limited on
            the way back up -- see front_brake_zone_m and previous_v_follow
            above.
        """
        cfg = self._config
        if right is None or right <= 0.0:
            return FollowCommand(vx=0.0, vy=0.0, yaw_rate_deg_s=0.0, wall_visible=False)

        # A missing/invalid front means "nothing within MAX_RANGE_M", per
        # Crazyflie.decks.multi_ranger's own convention (None is not a
        # fault) -- substitute that distance and let the existing formula
        # below react to it like any other far reading. This is what turns
        # the sensor back toward the wall: with front this large, the yaw
        # clamp below is driven to its maximum turning toward right.
        front_effective = front if front is not None and front > 0.0 else MAX_RANGE_M

        heading_error = front_effective - right
        standoff_error = (front_effective + right) / 2.0 - cfg.target_wall_distance_m

        yaw_rate = -cfg.yaw_gain_deg_per_m * heading_error
        yaw_rate = max(-cfg.max_yaw_rate_deg_s, min(cfg.max_yaw_rate_deg_s, yaw_rate))

        # Front-proximity brake: mirrors CollisionMonitor's own leading-sensor
        # threshold formula, offset outward by front_brake_zone_m, so the
        # forward-push term tapers to zero before CollisionMonitor's threshold
        # is ever reached -- for any close-ahead reading, not just a
        # classified "corner". See module docstring.
        # Uses the nominal follow_velocity_m_s, not the actual commanded
        # speed (this function is pure -- no FlightState access). At today's
        # defaults this is exact, not approximate: max_velocity_m_s=0.20
        # still keeps CollisionMonitor's max(0.25, v*0.65) floor-dominated
        # for any velocity this config can produce. If follow_velocity_m_s
        # or max_velocity_m_s is tuned above the ~0.38 m/s crossover, this
        # threshold would under-estimate CollisionMonitor's real one --
        # re-derive this comment's numbers if either constant changes.
        front_threshold = self._front_threshold()
        front_scale = (front_effective - front_threshold) / cfg.front_brake_zone_m
        front_scale = max(0.0, min(1.0, front_scale))

        v_follow_target = cfg.follow_velocity_m_s * front_scale
        v_correct = cfg.standoff_gain * standoff_error
        # Braking (a lower target) is never delayed -- only a climb back up
        # is slew-limited, over _REACTION_S (reused from CollisionMonitor,
        # not a new constant), so a single transient "clear" reading can't
        # snap the forward push straight back to full speed before the
        # sensor has a chance to confirm it. See the module docstring's
        # front-proximity-brake paragraph for the log evidence this fixes.
        if previous_v_follow is None or v_follow_target <= previous_v_follow:
            v_follow = v_follow_target
        else:
            max_step = (cfg.follow_velocity_m_s / _REACTION_S) * _POLL_INTERVAL_S
            v_follow = min(v_follow_target, previous_v_follow + max_step)

        vx = (v_follow + v_correct) / _SQRT2
        vy = (v_follow - v_correct) / _SQRT2

        speed = math.hypot(vx, vy)
        if speed > cfg.max_velocity_m_s and speed > 0.0:
            scale = cfg.max_velocity_m_s / speed
            vx *= scale
            vy *= scale

        return FollowCommand(
            vx=vx, vy=vy, yaw_rate_deg_s=yaw_rate, wall_visible=True, v_follow=v_follow
        )

    def is_aligned(self, front: float | None, right: float | None) -> bool:
        """Return True if front and right read within align_tolerance_m.

        Args:
            front: Front Multi-ranger distance in meters, or None/<=0.0.
            right: Right Multi-ranger distance in meters, or None/<=0.0.

        Returns:
            True if both readings are valid and their difference is within
            align_tolerance_m, False otherwise.
        """
        if front is None or front <= 0.0 or right is None or right <= 0.0:
            return False
        return abs(front - right) <= self._config.align_tolerance_m

    def is_too_close(self, readings: MultiRangerReadings) -> bool:
        """Return True if any of the five sensors reads below abort_distance_m.

        None and non-positive readings are treated as clear, matching every
        other distance check in this project (a zero reading means the
        sensor has not yet produced a valid measurement, not an obstacle at
        the sensor face).

        Args:
            readings: Current MultiRangerReadings snapshot.

        Returns:
            True if any valid reading is below abort_distance_m.
        """
        threshold = self._config.abort_distance_m
        for value in (
            readings.front,
            readings.back,
            readings.left,
            readings.right,
            readings.up,
        ):
            if value is not None and 0.0 < value < threshold:
                return True
        return False

    def fly_to_first_obstacle(
        self,
        mc: MotionCommander,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None = None,
    ) -> bool:
        """Fly forward until the front sensor reaches target_wall_distance_m.

        Args:
            mc: Active MotionCommander instance.
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).
            should_abort: Optional callable checked every poll cycle.
                Returning True stops the search early (e.g. a safety event).

        Returns:
            True if an obstacle was found within max_search_distance_m,
            False if the search distance was exhausted or should_abort
            fired first.
        """
        cfg = self._config
        if self._flight_state is not None:
            self._flight_state.set_direction("forward")
            self._flight_state.set_velocity(cfg.approach_velocity_m_s)

        mc.start_forward(cfg.approach_velocity_m_s)
        distance_traveled_m = 0.0
        found = False
        while distance_traveled_m < cfg.max_search_distance_m:
            if should_abort is not None and should_abort():
                break
            readings = ranger.get_readings()
            front = readings.front
            if front is not None and 0.0 < front <= cfg.target_wall_distance_m:
                found = True
                break
            time.sleep(_POLL_INTERVAL_S)
            distance_traveled_m += cfg.approach_velocity_m_s * _POLL_INTERVAL_S

        mc.stop()
        return found

    def align_to_wall(self, mc: MotionCommander, ranger: RangerSource) -> bool:
        """Rotate left (CCW) in small steps until front and right read equal.

        Clears FlightState direction/velocity to None/0.0 for the duration
        of the turn (matching SafeFlightController's turn handling) so
        CollisionMonitor does not judge this stationary rotation against a
        stale linear velocity left over from the search leg.

        Near 45 degrees, front and right diverge quickly with heading - for
        a typical approach distance, one align_step_deg step changes
        abs(front - right) by roughly 3x align_tolerance_m's default. A pure
        "is the difference within tolerance" check can therefore straddle
        the aligned heading forever without ever landing inside a narrow
        tolerance window, spinning through the whole max_align_deg budget.
        To stay robust regardless of the actual approach distance, alignment
        also stops the instant (front - right) changes sign between two
        consecutive steps - that means the 45 degree crossing happened
        somewhere in the step just taken, so this heading is within one
        align_step_deg of true alignment. follow()'s continuous yaw
        correction removes that residual once wall-following starts; it is
        not this method's job to be exact, only close enough for follow()
        to take over safely.

        Args:
            mc: Active MotionCommander instance.
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).

        Returns:
            True once front and right are within align_tolerance_m of each
            other, or once the heading has crossed the 45 degree point
            (sign change in front - right). False if max_align_deg of
            rotation is exhausted first.
        """
        cfg = self._config
        if self._flight_state is not None:
            self._flight_state.set_direction(None)
            self._flight_state.set_velocity(0.0)

        rotated_deg = 0.0
        previous_error: float | None = None
        while rotated_deg < cfg.max_align_deg:
            readings = ranger.get_readings()
            if self.is_aligned(readings.front, readings.right):
                return True
            front, right = readings.front, readings.right
            if front is not None and front > 0.0 and right is not None and right > 0.0:
                error = front - right
                if previous_error is not None and (error > 0) != (previous_error > 0):
                    return True
                previous_error = error
            mc.turn_left(cfg.align_step_deg)
            rotated_deg += cfg.align_step_deg

        readings = ranger.get_readings()
        return self.is_aligned(readings.front, readings.right)

    def _rotate_perpendicular_to_wall(
        self,
        mc: MotionCommander,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None,
    ) -> bool:
        """Outward-corner negotiation, phase 1: a stationary CCW turn.

        At the normal follow attitude, front and right are both leading the
        wall at 45 deg. Turning an additional _CORNER_ROTATION_DEG CCW puts
        right exactly perpendicular to the wall's (former) direction and
        front along the direction of travel -- i.e. the drone now flies
        "forward", parallel to the wall, continuing in the same direction
        of travel. See the module docstring.

        Never calls mc.stop() -- follow() alone owns that single call.

        Args:
            mc: Active MotionCommander instance.
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).
            should_abort: Optional callable checked before turning.

        Returns:
            False without turning if should_abort() fires or a fresh
            reading is already too close; True after issuing the turn
            otherwise.
        """
        if should_abort is not None and should_abort():
            return False
        readings = ranger.get_readings()
        if self.is_too_close(readings):
            return False

        if self._flight_state is not None:
            self._flight_state.set_direction(None)
            self._flight_state.set_velocity(0.0)

        logger.info(
            "WallFollower: negotiating outward corner - rotating %.0f deg.",
            _CORNER_ROTATION_DEG,
        )
        mc.turn_left(_CORNER_ROTATION_DEG)
        return True

    def _corner_negotiation_poll(
        self,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None,
    ) -> MultiRangerReadings | None:
        """Pre-motion safety check for one outward-corner negotiation poll
        cycle: should_abort() then is_too_close(), shared by
        _advance_past_corner_apex() and _arc_around_corner() (phases 2 and
        3) so both check should_abort()/is_too_close() the same way, in
        the same order, as every other poll loop in this module.

        Args:
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).
            should_abort: Optional callable checked before the reading.

        Returns:
            The fresh readings, or None if should_abort() fired or the
            readings are too close -- the caller should return False
            immediately without commanding motion.
        """
        if should_abort is not None and should_abort():
            return None
        readings = ranger.get_readings()
        if self.is_too_close(readings):
            return None
        return readings

    def _corner_negotiation_move(
        self,
        mc: MotionCommander,
        should_abort: Callable[[], bool] | None,
        vx: float,
        yaw_rate_deg_s: float,
    ) -> bool:
        """Command one cycle's motion for an outward-corner negotiation
        phase, shared by _advance_past_corner_apex() and
        _arc_around_corner(): should_abort() is re-checked immediately
        before the motion command itself (matching follow()'s own safety
        pattern -- see its docstring for why), then
        start_linear_motion(vx, 0.0, 0.0, rate_yaw=yaw_rate_deg_s) is
        issued and the loop sleeps for one poll interval.

        Args:
            mc: Active MotionCommander instance.
            should_abort: Optional callable re-checked before the command.
            vx: Forward velocity in m/s to command.
            yaw_rate_deg_s: Yaw rate in deg/s to command alongside vx.

        Returns:
            True after issuing the motion command and sleeping. False (no
            motion issued) if should_abort() just fired.
        """
        if should_abort is not None and should_abort():
            return False
        mc.start_linear_motion(vx, 0.0, 0.0, rate_yaw=yaw_rate_deg_s)
        time.sleep(_POLL_INTERVAL_S)
        return True

    def _advance_past_corner_apex(
        self,
        mc: MotionCommander,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None,
        baseline_right: float | None,
    ) -> bool:
        """Outward-corner negotiation, phase 2: advance until right spikes too.

        Flies straight forward (front leading, right a side sensor) at
        follow_velocity_m_s, watching right for the same kind of spike
        front just showed -- once right spikes relative to baseline_right,
        the drone's position has passed the corner's edge laterally, not
        just visually via front. See the module docstring.

        baseline_right is the main loop's last valid right reading from
        before the maneuver, so if right is already past the edge when the
        rotation finishes, the very first cycle's is_spike() fires and this
        phase ends immediately instead of advancing until the distance cap.

        Never calls mc.stop() -- follow() alone owns that single call.

        Args:
            mc: Active MotionCommander instance.
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).
            should_abort: Optional callable checked before every motion
                command.
            baseline_right: The right reading to compare against for
                is_spike(), updated each cycle from the freshest valid
                reading.

        Returns:
            True once right spikes relative to the (continuously updated)
            baseline. False if should_abort()/is_too_close() fires, or if
            max_corner_advance_m of estimated travel is exhausted first.
        """
        cfg = self._config
        logger.info("WallFollower: negotiating outward corner - advancing past the corner apex.")
        if self._flight_state is not None:
            self._flight_state.set_direction("forward")
            self._flight_state.set_velocity(cfg.follow_velocity_m_s)

        distance_traveled_m = 0.0
        while distance_traveled_m < cfg.max_corner_advance_m:
            readings = self._corner_negotiation_poll(ranger, should_abort)
            if readings is None:
                return False
            if is_spike(readings.right, baseline_right, cfg.spike_threshold_m):
                return True
            if readings.right is not None and readings.right > 0.0:
                baseline_right = readings.right

            if not self._corner_negotiation_move(mc, should_abort, cfg.follow_velocity_m_s, 0.0):
                return False
            distance_traveled_m += cfg.follow_velocity_m_s * _POLL_INTERVAL_S

        return False

    def _arc_around_corner(
        self,
        mc: MotionCommander,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None,
    ) -> bool:
        """Outward-corner negotiation, phase 3: arc around at a fixed radius.

        Flies a constant-radius clockwise arc (forward velocity plus a
        matching negative yaw rate, clamped to max_yaw_rate_deg_s like
        every other yaw command in this module) with radius equal to the
        wall's target perpendicular standoff (target_wall_distance_m /
        sqrt(2)), so the path circles the corner apex at the following
        distance. Continues until front picks up a wall at or inside
        target_wall_distance_m -- geometrically, that is the point on the
        arc where the drone is back at the 45 deg follow attitude relative
        to the new wall face, so compute_follow_command() can take
        straight back over. See the module docstring.

        Never calls mc.stop() -- follow() alone owns that single call.

        Args:
            mc: Active MotionCommander instance.
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).
            should_abort: Optional callable checked before every motion
                command.

        Returns:
            True once front reads at or under target_wall_distance_m.
            False if should_abort()/is_too_close() fires, or if
            max_corner_arc_deg of accumulated arc is exhausted first. A
            missing or far front never ends the arc early.
        """
        cfg = self._config
        radius = cfg.target_wall_distance_m / _SQRT2
        yaw_rate_deg_s = -math.degrees(cfg.follow_velocity_m_s / radius)
        yaw_rate_deg_s = max(-cfg.max_yaw_rate_deg_s, min(cfg.max_yaw_rate_deg_s, yaw_rate_deg_s))
        logger.info("WallFollower: negotiating outward corner - arcing at radius %.2f m.", radius)
        if self._flight_state is not None:
            self._flight_state.set_direction("forward")
            self._flight_state.set_velocity(cfg.follow_velocity_m_s)

        arc_traveled_deg = 0.0
        while arc_traveled_deg < cfg.max_corner_arc_deg:
            readings = self._corner_negotiation_poll(ranger, should_abort)
            if readings is None:
                return False
            if readings.front is not None and 0.0 < readings.front <= cfg.target_wall_distance_m:
                return True

            if not self._corner_negotiation_move(
                mc, should_abort, cfg.follow_velocity_m_s, yaw_rate_deg_s
            ):
                return False
            arc_traveled_deg += abs(yaw_rate_deg_s) * _POLL_INTERVAL_S

        return False

    def _negotiate_outer_corner(
        self,
        mc: MotionCommander,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None,
        baseline_right: float | None,
    ) -> bool:
        """Run the full four-phase outward-corner maneuver (phases 1-3;
        phase 4 is simply returning control to follow()'s own loop).

        Args:
            mc: Active MotionCommander instance.
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).
            should_abort: Optional callable checked throughout every phase.
            baseline_right: The right reading from just before the maneuver
                began -- passed to _advance_past_corner_apex() as its
                initial baseline.

        Returns:
            True if all three phases succeeded (a wall has been reacquired
            at target_wall_distance_m); False if any phase failed, in
            which case the maneuver stopped wherever that phase left off.
        """
        if not self._rotate_perpendicular_to_wall(mc, ranger, should_abort):
            return False
        if not self._advance_past_corner_apex(mc, ranger, should_abort, baseline_right):
            return False
        return self._arc_around_corner(mc, ranger, should_abort)

    def follow(
        self,
        mc: MotionCommander,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None = None,
    ) -> None:
        """Run the closed-loop wall-following control at 10 Hz.

        Ends (calling mc.stop() exactly once, regardless of the reason) on
        the first of: should_abort() returning True, follow_duration_s
        elapsing, is_too_close() firing, the wall being lost for
        wall_lost_timeout_s, or a failed outward-corner negotiation (see
        _negotiate_outer_corner()). follow_duration_s is not checked while
        a corner maneuver is in progress -- the maneuver is bounded by
        max_corner_advance_m and max_corner_arc_deg instead.

        should_abort() is re-checked immediately before every
        start_linear_motion() call, not only once per loop iteration --
        CollisionMonitor may stop the drone from its own thread at any
        point, and issuing another start_linear_motion() after that would
        override the stop.

        Args:
            mc: Active MotionCommander instance.
            ranger: RangerSource to poll (MultiRangerDeck or an equivalent adapter).
            should_abort: Optional callable checked before every motion
                command. Returning True ends the loop immediately.
        """
        cfg = self._config
        start_time = time.monotonic()
        last_wall_seen_time = start_time
        previous_v_follow = 0.0  # Real velocity starts at 0.0 when follow() begins.
        previous_right: float | None = None  # Last valid right -- also the
        # maneuver's baseline_right if a corner is negotiated.
        previous_front: float | None = None  # Last valid front -- the
        # baseline for detecting a fresh spike candidate.
        spike_baseline: float | None = None  # front reading a candidate
        # spike is being measured against (see the module docstring).
        spike_cycles = 0  # Consecutive cycles the candidate has spiked.

        while True:
            if should_abort is not None and should_abort():
                break
            if time.monotonic() - start_time >= cfg.follow_duration_s:
                logger.info("WallFollower: follow_duration_s elapsed - stopping.")
                break

            readings = ranger.get_readings()
            if self.is_too_close(readings):
                logger.warning(
                    "WallFollower: a sensor is within abort_distance_m (%.2f m) - stopping.",
                    cfg.abort_distance_m,
                )
                break

            # Outward-corner detection: a front spike, confirmed for
            # spike_confirm_cycles consecutive polls, and only starting
            # from a baseline outside the front-proximity brake zone (an
            # inside-corner flicker always starts from inside it -- see
            # the module docstring). While a candidate is still being
            # confirmed, the normal command cycle below still runs -- the
            # brake is fully open anyway since front reads far.
            if spike_baseline is None:
                if (
                    is_spike(readings.front, previous_front, cfg.spike_threshold_m)
                    and previous_front is not None
                    and previous_front >= self._front_threshold() + cfg.front_brake_zone_m
                ):
                    spike_baseline = previous_front
                    spike_cycles = 1
            elif is_spike(readings.front, spike_baseline, cfg.spike_threshold_m):
                spike_cycles += 1
            else:
                spike_baseline = None
                spike_cycles = 0

            if spike_cycles >= cfg.spike_confirm_cycles:
                logger.warning(
                    "WallFollower: outward corner detected (front spike) - negotiating."
                )
                spike_baseline = None
                spike_cycles = 0
                negotiated = self._negotiate_outer_corner(mc, ranger, should_abort, previous_right)
                if not negotiated:
                    break
                # The arc phase was already flying at follow_velocity_m_s,
                # so the next cycle shouldn't brake as though starting
                # from a stop; front and the wall-lost timer are stale
                # after however long the maneuver took. previous_right is
                # seeded with target_wall_distance_m (not None) -- phase 3
                # only exits once front is at or inside that distance,
                # geometrically the point where right should read close to
                # it too, and a None baseline here would leave
                # _advance_past_corner_apex() unable to ever detect a
                # lateral-edge spike if a second corner is negotiated
                # before right is next read as valid (is_spike() never
                # fires against a missing baseline).
                previous_v_follow = cfg.follow_velocity_m_s
                previous_front = None
                previous_right = cfg.target_wall_distance_m
                last_wall_seen_time = time.monotonic()
                continue

            command = self.compute_follow_command(
                readings.front, readings.right, previous_v_follow
            )
            previous_v_follow = command.v_follow
            if readings.front is not None and readings.front > 0.0:
                previous_front = readings.front
            if readings.right is not None and readings.right > 0.0:
                previous_right = readings.right
            now = time.monotonic()
            if command.wall_visible:
                last_wall_seen_time = now
            elif now - last_wall_seen_time >= cfg.wall_lost_timeout_s:
                logger.warning(
                    "WallFollower: wall lost for %.1f s - stopping.", cfg.wall_lost_timeout_s
                )
                break

            if self._flight_state is not None:
                self._flight_state.set_direction(FLIGHT_DIRECTION)
                self._flight_state.set_velocity(math.hypot(command.vx, command.vy))

            # Re-check immediately before the motion command itself - see
            # docstring above.
            if should_abort is not None and should_abort():
                break
            mc.start_linear_motion(command.vx, command.vy, 0.0, rate_yaw=command.yaw_rate_deg_s)

            time.sleep(_POLL_INTERVAL_S)

        mc.stop()
