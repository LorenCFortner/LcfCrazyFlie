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

A large heading error -- front far ahead of right, whether front is
literally missing (substituted with MAX_RANGE_M) or simply a real, valid,
but large reading -- pins yaw_rate at its max_yaw_rate_deg_s clamp. Chasing
that error unconditionally is wrong whenever there is no wall actually
reachable on the diagonal: it yaws hard toward a heading that keeps not
resolving *and* pushes hard laterally (the standoff term blows up, since
(front_effective + right) / 2 is far above target_wall_distance_m), for as
long as the error persists. Observed on hardware twice, in two different
shapes: scripts/logs/right_wall_follow.log's 12:30 run (front missing) and
its 15:17 run (front reading a real, valid 2.2-2.3 m the entire time,
never None, against a right holding steady around 0.44-0.58 m) -- in the
15:17 run, heading_error stayed near 1.8 m continuously for ~2 seconds,
saturating yaw_rate at -45 deg/s the whole time (roughly a 90 degree
rotation at full translational speed), until the sweep brought a surface
into the front beam's narrow cone that a hardware operator identified as
having already been close the entire time, just not detectable until the
rotation exposed it -- front then read 0.201 m on the very next 100 ms
poll, and both CollisionMonitor and WallFollower's own is_too_close()
fired immediately, landing the flight (no hardware damage -- both
backstops worked as designed).

Since a missing front is only *one* way heading_error can become large
enough to saturate yaw, the trigger for backing off is sustained yaw
saturation itself, not front's validity: once the unclamped, heading-
driven yaw rate's magnitude has been >= max_yaw_rate_deg_s for at least
yaw_saturation_hold_s continuously, the loop concludes front isn't
tracking a real, reachable wall and switches to a straight-line hold --
continuing the current (front + left) diagonal at follow_velocity_m_s with
yaw held at 0.0 and the standoff correction skipped -- rather than
continuing to chase it. See compute_follow_command()'s saturated_cycles
parameter; follow() tracks consecutive saturated cycles (while not already
holding or reacquiring) and feeds the count back in each cycle, the same
way it already does for previous_v_follow.

Holding persists (see compute_follow_command()'s previously_holding
parameter and FollowCommand.holding) until right itself increases by more
than right_increasing_hysteresis_m versus the previous cycle's reading --
that increase means the followed wall is now receding (e.g. the drone has
passed an outside corner's outer apex), and only then does the loop
transition to reacquiring: the normal substitution-based formula resumes,
yawing hard toward front_effective to search for and reacquire a wall.
Like the slew limit's previous_v_follow, this latches once triggered (see
previously_reacquiring and FollowCommand.reacquiring) -- a real-world
receding trend can be gradual or noisy relative to the 10 Hz poll rate, so
comparing only the single most recent cycle's delta could otherwise flip
the loop back to holding mid-turn the instant one cycle's increase fell
short, chattering yaw between 0.0 and -max_yaw_rate_deg_s while genuinely
passing a corner.

Reacquiring itself exits once heading has genuinely converged -- the
unclamped yaw rate is no longer saturated, meaning front and right are
back in a normal following relationship -- at which point the loop resumes
ordinary following outright. This convergence-based exit, rather than
"front is no longer None", is precisely what the 15:17 incident's fix
needed: front reading a real, valid-but-unhelpful value never made the old
front-is-None gate release at all, so the same runaway rotation could
recur indefinitely under the previous mechanism. See
compute_follow_command()'s previously_holding/previously_reacquiring
parameters and FollowCommand.holding/reacquiring; follow() tracks and
feeds these back in each cycle the same way it does previous_v_follow and
previous_right.

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
        right_increasing_hysteresis_m: Minimum increase in right between
            consecutive readings, while holding or reacquiring, required to
            conclude the followed wall is receding (the drone has passed an
            outside corner's outer apex) rather than sensor noise -- see
            compute_follow_command()'s previous_right parameter and the
            module docstring.
        yaw_saturation_hold_s: Duration yaw must remain clamped at
            max_yaw_rate_deg_s continuously before concluding front isn't
            tracking a real, reachable wall ahead and switching to the
            straight-line hold -- see compute_follow_command()'s
            saturated_cycles parameter and the module docstring. Deliberately
            short (2 poll cycles at the default _POLL_INTERVAL_S): holding is
            a safe, conservative state on its own (yaw simply stops chasing;
            CollisionMonitor and this loop's own is_too_close() stay fully
            active throughout), so there is no safety reason to wait
            longer -- doing so would only reintroduce, for however long the
            wait lasts, the exact aggressive yaw-and-push-toward-a-phantom-
            wall behavior the 08:58:12 fix eliminated for a genuinely
            missing front. The short duration still filters a single noisy
            poll from triggering a hold on its own.

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
    right_increasing_hysteresis_m: float = 0.05
    yaw_saturation_hold_s: float = 0.20


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
        holding: True while yaw has been saturated long enough to trigger
            the straight-line hold and right has not yet been detected
            receding -- feed this back in as the next call's
            previously_holding to latch the hold across cycles. See
            compute_follow_command()'s previously_holding parameter and the
            module docstring.
        reacquiring: True while holding or reacquiring and right has been
            detected receding (an outside corner's outer apex has been
            passed) -- feed this back in as the next call's
            previously_reacquiring to latch reacquire mode across cycles
            even if a later cycle's right delta dips back under
            right_increasing_hysteresis_m. False once heading has converged
            (ordinary following resumed) or while still holding straight.
            See compute_follow_command()'s previously_reacquiring parameter
            and the module docstring.
        saturated_cycles: Consecutive prior cycles, while not holding or
            reacquiring, where yaw was clamped at max_yaw_rate_deg_s -- feed
            this back in as the next call's saturated_cycles to detect
            sustained saturation across follow()'s loop. Always 0 while
            holding or reacquiring (irrelevant once already handling a
            large heading error) and reset to 0 whenever yaw is not
            saturated.
    """

    vx: float
    vy: float
    yaw_rate_deg_s: float
    wall_visible: bool
    v_follow: float = 0.0
    holding: bool = False
    reacquiring: bool = False
    saturated_cycles: int = 0


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

    def compute_follow_command(
        self,
        front: float | None,
        right: float | None,
        previous_v_follow: float | None = None,
        previous_right: float | None = None,
        previously_holding: bool = False,
        previously_reacquiring: bool = False,
        saturated_cycles: int = 0,
    ) -> FollowCommand:
        """Compute the velocity command for one control cycle.

        Pure function of the two sensor readings, previous_v_follow,
        previous_right, previously_holding, previously_reacquiring,
        saturated_cycles, and this follower's config -- see the module
        docstring for the geometry and error-signal derivation, the
        forward-push slew limit, and the sustained-yaw-saturation
        straight-line-hold behavior.

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
            previous_right: The right reading from the last cycle that had
                a valid one. Only consulted while holding or reacquiring --
                used to detect whether right is now increasing (the
                followed wall receding, i.e. an outside corner's outer apex
                has been passed) by more than right_increasing_hysteresis_m.
                Defaults to None, meaning "no trend data yet", which --
                like a stable or decreasing right -- keeps holding rather
                than transitioning to reacquiring. See the module
                docstring. follow() tracks and passes the real previous
                cycle's last-valid right reading.
            previously_holding: Whether the *previous* cycle was already
                holding (see FollowCommand.holding). When True, this cycle
                stays in hold or reacquire handling regardless of this
                cycle's own saturation state -- holding forces yaw_rate to
                0.0, which is never itself saturated, so re-evaluating
                saturation from scratch every cycle would immediately (and
                incorrectly) look "resolved". Defaults to False. follow()
                tracks and passes the real previous cycle's value.
            previously_reacquiring: Whether the *previous* cycle was
                already reacquiring (see FollowCommand.reacquiring). When
                True, this cycle stays in reacquire mode regardless of
                whether the current cycle's right delta alone would cross
                right_increasing_hysteresis_m -- a real receding trend can
                be gradual or noisy relative to the poll rate, and without
                this latch a single slow/negative-delta cycle would flip
                the loop back to holding mid-turn. Defaults to False. Reset
                automatically once heading has converged (see
                FollowCommand.reacquiring). follow() tracks and passes the
                real previous cycle's value.
            saturated_cycles: Consecutive prior cycles, while not holding
                or reacquiring, where yaw was clamped at max_yaw_rate_deg_s.
                Only relevant when not already holding or reacquiring --
                used to detect sustained saturation
                (saturated_cycles * _POLL_INTERVAL_S >= yaw_saturation_hold_s)
                and enter the straight-line hold. Defaults to 0. follow()
                tracks and passes the real previous cycle's value (see
                FollowCommand.saturated_cycles).

        Returns:
            FollowCommand with wall_visible=False (and all-zero velocity,
            including v_follow) when right is missing; otherwise a live
            command, clamped to max_velocity_m_s and max_yaw_rate_deg_s.
            Ordinarily (not holding, not reacquiring) this is the normal
            heading + standoff correction. Once yaw has been saturated for
            yaw_saturation_hold_s continuously (see saturated_cycles above),
            this is instead a straight-line hold: yaw_rate_deg_s=0.0 and no
            standoff correction, so vx equals vy exactly
            (holding=True). Holding transitions to reacquiring once right
            is observed receding (see previous_right above): the normal
            formula resumes using front substituted with MAX_RANGE_M when
            missing -- typically a hard yaw back toward where the wall was,
            to search for the next wall face (reacquiring=True). Reacquiring
            exits back to ordinary following once yaw is no longer
            saturated (heading has converged). The forward-push term is
            throttled by front proximity and slew-limited on the way back
            up in every case -- see front_brake_zone_m and previous_v_follow
            above.
        """
        cfg = self._config
        if right is None or right <= 0.0:
            return FollowCommand(vx=0.0, vy=0.0, yaw_rate_deg_s=0.0, wall_visible=False)

        # A missing/invalid front means "nothing within MAX_RANGE_M", per
        # Crazyflie.decks.multi_ranger's own convention (None is not a
        # fault) -- substitute that distance so the heading/standoff
        # formula and the front-proximity brake below react to it like any
        # other far reading. See the module docstring's front-missing
        # paragraph.
        front_effective = front if front is not None and front > 0.0 else MAX_RANGE_M

        heading_error = front_effective - right
        standoff_error = (front_effective + right) / 2.0 - cfg.target_wall_distance_m
        unclamped_yaw_rate = -cfg.yaw_gain_deg_per_m * heading_error
        is_saturated = abs(unclamped_yaw_rate) >= cfg.max_yaw_rate_deg_s

        # right increasing means the followed wall is now receding (an
        # outside corner's outer apex has been passed), which is when it
        # becomes correct to resume aggressively searching for the next
        # wall face -- see the module docstring. previously_reacquiring
        # latches this True once detected, regardless of this cycle's own
        # delta, so a gradual or noisy real-world increase doesn't chatter
        # the loop between holding and reacquiring cycle to cycle. Without
        # trend data (previous_right is None) this defaults to "not
        # receding", the same conservative hold as a stable or decreasing
        # right.
        right_receding = previously_reacquiring or (
            previous_right is not None
            and (right - previous_right) > cfg.right_increasing_hysteresis_m
        )

        was_handling = previously_holding or previously_reacquiring
        if was_handling:
            if right_receding and not is_saturated:
                # Converged while reacquiring: heading has genuinely
                # resolved -- exit hold/reacquire handling entirely and
                # resume ordinary following. This is the fix for the 15:17
                # incident: exit is keyed off real convergence, not merely
                # "front is no longer missing" (which a real, valid, but
                # still-unhelpful front reading could satisfy without
                # anything having actually resolved).
                holding = False
                reacquiring = False
            elif right_receding:
                holding = False
                reacquiring = True
            else:
                holding = True
                reacquiring = False
            saturated_cycles_out = 0
        else:
            new_saturated_cycles = saturated_cycles + 1 if is_saturated else 0
            sustained = (new_saturated_cycles * _POLL_INTERVAL_S) >= cfg.yaw_saturation_hold_s
            if sustained:
                holding = True
                reacquiring = False
                saturated_cycles_out = 0
            else:
                holding = False
                reacquiring = False
                saturated_cycles_out = new_saturated_cycles

        if holding:
            yaw_rate = 0.0
            v_correct = 0.0
        else:
            yaw_rate = max(
                -cfg.max_yaw_rate_deg_s, min(cfg.max_yaw_rate_deg_s, unclamped_yaw_rate)
            )
            v_correct = cfg.standoff_gain * standoff_error

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
        front_threshold = max(_BASE_DETECTION_M, cfg.follow_velocity_m_s * _REACTION_S)
        front_scale = (front_effective - front_threshold) / cfg.front_brake_zone_m
        front_scale = max(0.0, min(1.0, front_scale))

        v_follow_target = cfg.follow_velocity_m_s * front_scale
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
            vx=vx,
            vy=vy,
            yaw_rate_deg_s=yaw_rate,
            wall_visible=True,
            v_follow=v_follow,
            holding=holding,
            reacquiring=reacquiring,
            saturated_cycles=saturated_cycles_out,
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

    def follow(
        self,
        mc: MotionCommander,
        ranger: RangerSource,
        should_abort: Callable[[], bool] | None = None,
    ) -> None:
        """Run the closed-loop wall-following control at 10 Hz.

        Ends (calling mc.stop() exactly once, regardless of the reason) on
        the first of: should_abort() returning True, follow_duration_s
        elapsing, is_too_close() firing, or the wall being lost for
        wall_lost_timeout_s.

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
        previous_right: float | None = None  # No trend data yet -- see module docstring.
        previously_holding = False  # See FollowCommand.holding.
        previously_reacquiring = False  # See FollowCommand.reacquiring.
        saturated_cycles = 0  # See FollowCommand.saturated_cycles.

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

            command = self.compute_follow_command(
                readings.front,
                readings.right,
                previous_v_follow,
                previous_right,
                previously_holding,
                previously_reacquiring,
                saturated_cycles,
            )
            previous_v_follow = command.v_follow
            previously_holding = command.holding
            previously_reacquiring = command.reacquiring
            saturated_cycles = command.saturated_cycles
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
