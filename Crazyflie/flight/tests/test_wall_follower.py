"""Tests for WallFollower.

Written test-first following the TDD rules for this project.
"""

import math

import pytest

from Crazyflie.decks.multi_ranger import MAX_RANGE_M, MultiRangerReadings
from Crazyflie.flight.wall_follower import (
    _POLL_INTERVAL_S,
    FLIGHT_DIRECTION,
    FollowCommand,
    WallFollowConfig,
    WallFollower,
)
from Crazyflie.safety.collision_monitor import _BASE_DETECTION_M, _REACTION_S
from Crazyflie.state.flight_state import FlightState

_SQRT2 = math.sqrt(2.0)


def _readings(front=None, back=None, left=None, right=None, up=None) -> MultiRangerReadings:
    return MultiRangerReadings(front=front, back=back, left=left, right=right, up=up)


# ---------------------------------------------------------------------------
# compute_follow_command - pure function, the bulk of the control law
# ---------------------------------------------------------------------------


class TestComputeFollowCommand:
    def test_on_setpoint_flies_pure_along_wall(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.60, right=0.60)

        assert command.wall_visible is True
        assert command.vx == pytest.approx(command.vy)
        assert command.vx > 0.0
        assert command.yaw_rate_deg_s == pytest.approx(0.0)

    def test_front_greater_than_right_yaws_right_negative(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.65, right=0.55)

        assert command.yaw_rate_deg_s < 0.0

    def test_front_less_than_right_yaws_left_positive(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.55, right=0.65)

        assert command.yaw_rate_deg_s > 0.0

    def test_too_far_from_wall_adds_component_toward_wall(self):
        # mean(0.80, 0.80) = 0.80 > target 0.60 -> move toward wall (+n_hat)
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.80, right=0.80)

        assert command.vx > command.vy

    def test_too_close_to_wall_adds_component_away_from_wall(self):
        # mean(0.40, 0.40) = 0.40 < target 0.60 -> move away from wall
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.40, right=0.40)

        assert command.vy > command.vx

    def test_total_speed_never_exceeds_max_velocity(self):
        follower = WallFollower(WallFollowConfig())

        # Large standoff error to force clamping.
        command = follower.compute_follow_command(front=1.5, right=1.5)

        speed = math.hypot(command.vx, command.vy)
        assert speed == pytest.approx(WallFollowConfig().max_velocity_m_s, abs=1e-9)

    def test_speed_stays_at_or_below_max_across_a_range_of_errors(self):
        follower = WallFollower(WallFollowConfig())

        for front, right in [(0.1, 0.1), (0.6, 0.6), (2.0, 0.1), (0.1, 2.0), (3.0, 3.0)]:
            command = follower.compute_follow_command(front=front, right=right)
            speed = math.hypot(command.vx, command.vy)
            assert speed <= WallFollowConfig().max_velocity_m_s + 1e-9

    def test_yaw_rate_clamped_to_max(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=1.0, right=0.1)

        assert command.yaw_rate_deg_s == pytest.approx(-WallFollowConfig().max_yaw_rate_deg_s)

    def test_yaw_rate_clamped_to_negative_max_symmetrically(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.1, right=1.0)

        assert command.yaw_rate_deg_s == pytest.approx(WallFollowConfig().max_yaw_rate_deg_s)

    @pytest.mark.parametrize(
        "front,right",
        [
            (0.5, None),
            (None, None),
            (0.5, 0.0),
            (0.5, -0.1),
        ],
    )
    def test_right_missing_or_invalid_gives_zero_command(self, front, right):
        """right is the wall actually being followed -- if it's missing or
        invalid, the wall is genuinely lost regardless of front.
        """
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=front, right=right)

        assert command == FollowCommand(vx=0.0, vy=0.0, yaw_rate_deg_s=0.0, wall_visible=False)

    @pytest.mark.parametrize("front", [None, 0.0, -0.1])
    def test_front_missing_or_invalid_substitutes_max_range(self, front):
        """A missing/invalid front means "nothing within MAX_RANGE_M" per
        Crazyflie.decks.multi_ranger's own convention -- not "unreadable,
        stop". It substitutes MAX_RANGE_M for the front-proximity-brake
        pipeline (v_follow), which is identical whether front is missing or
        a real reading exactly at MAX_RANGE_M -- that part is unaffected by
        the missing-vs-valid distinction. The heading/standoff response
        (vx, vy, yaw_rate_deg_s) is NOT necessarily expected to match a real
        MAX_RANGE_M reading -- both drive the same sustained-yaw-saturation
        hold/reacquire mechanism now (see TestYawSaturationEntersHolding),
        but a bare, single isolated call never accumulates enough
        saturated_cycles to enter it either way, so this assertion is
        scoped to v_follow, which is unaffected by any of that.
        """
        follower = WallFollower(WallFollowConfig())
        right = 0.40

        command = follower.compute_follow_command(front=front, right=right)
        expected = follower.compute_follow_command(front=MAX_RANGE_M, right=right)

        assert command.v_follow == pytest.approx(expected.v_follow)
        assert command.wall_visible is True

    def test_is_a_pure_function_no_side_effects(self):
        """No MotionCommander, thread, sleep, or drone reference anywhere."""
        follower = WallFollower(WallFollowConfig())

        result1 = follower.compute_follow_command(0.6, 0.6)
        result2 = follower.compute_follow_command(0.6, 0.6)

        assert result1 == result2


# ---------------------------------------------------------------------------
# Front-proximity brake - throttles forward push as `front` closes on
# anything (corner, protrusion, person), independent of heading/standoff.
# Mirrors AdaptivePathCorrector's velocity-scaled zone, shaped after
# CollisionMonitor's own leading-sensor threshold formula.
# ---------------------------------------------------------------------------


def _front_threshold(cfg: WallFollowConfig) -> float:
    return max(_BASE_DETECTION_M, cfg.follow_velocity_m_s * _REACTION_S)


def _unbraked_vx_vy(cfg: WallFollowConfig, front: float, right: float) -> tuple[float, float]:
    """Reference vx/vy using the pre-brake formula (v_follow unscaled)."""
    standoff_error = (front + right) / 2.0 - cfg.target_wall_distance_m
    v_correct = cfg.standoff_gain * standoff_error
    vx = (cfg.follow_velocity_m_s + v_correct) / _SQRT2
    vy = (cfg.follow_velocity_m_s - v_correct) / _SQRT2
    speed = math.hypot(vx, vy)
    if speed > cfg.max_velocity_m_s and speed > 0.0:
        scale = cfg.max_velocity_m_s / speed
        vx *= scale
        vy *= scale
    return vx, vy


class TestFrontProximityBrake:
    def test_no_effect_at_or_above_zone_upper(self):
        """front_scale clamps to 1.0 (no forward-speed boost) at the zone's
        upper edge and beyond -- braking must be a no-op during ordinary
        steady-state following, where front sits near target_wall_distance_m.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        zone_upper = _front_threshold(cfg) + cfg.front_brake_zone_m

        for front in (zone_upper, zone_upper + 0.5, cfg.target_wall_distance_m):
            command = follower.compute_follow_command(
                front=front, right=cfg.target_wall_distance_m
            )
            expected_vx, expected_vy = _unbraked_vx_vy(cfg, front, cfg.target_wall_distance_m)
            assert command.vx == pytest.approx(expected_vx)
            assert command.vy == pytest.approx(expected_vy)

    def test_forward_push_is_zero_at_front_threshold(self):
        """At front == front_threshold, v_follow_effective == 0.0 -- vx/vy
        come from v_correct alone.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg)

        command = follower.compute_follow_command(front=front, right=right)

        standoff_error = (front + right) / 2.0 - cfg.target_wall_distance_m
        v_correct = cfg.standoff_gain * standoff_error
        expected_vx = v_correct / _SQRT2
        expected_vy = -v_correct / _SQRT2
        assert command.vx == pytest.approx(expected_vx)
        assert command.vy == pytest.approx(expected_vy)

    def test_front_scale_does_not_go_negative_below_threshold(self):
        """Below front_threshold, front_scale clamps to 0.0 -- it must not
        flip the forward term's sign on its own (only v_correct may).
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg) - 0.10

        command = follower.compute_follow_command(front=front, right=right)

        standoff_error = (front + right) / 2.0 - cfg.target_wall_distance_m
        v_correct = cfg.standoff_gain * standoff_error
        expected_vx = v_correct / _SQRT2
        expected_vy = -v_correct / _SQRT2
        speed = math.hypot(expected_vx, expected_vy)
        if speed > cfg.max_velocity_m_s and speed > 0.0:
            scale = cfg.max_velocity_m_s / speed
            expected_vx *= scale
            expected_vy *= scale
        assert command.vx == pytest.approx(expected_vx)
        assert command.vy == pytest.approx(expected_vy)

    def test_halfway_through_zone_scales_v_follow_by_half(self):
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg) + cfg.front_brake_zone_m / 2.0

        command = follower.compute_follow_command(front=front, right=right)

        standoff_error = (front + right) / 2.0 - cfg.target_wall_distance_m
        v_correct = cfg.standoff_gain * standoff_error
        v_follow_effective = cfg.follow_velocity_m_s * 0.5
        expected_vx = (v_follow_effective + v_correct) / _SQRT2
        expected_vy = (v_follow_effective - v_correct) / _SQRT2
        assert command.vx == pytest.approx(expected_vx)
        assert command.vy == pytest.approx(expected_vy)

    def test_reproduces_reduced_push_for_logged_corner_scenario(self):
        """Regression guard: the exact front/right pair from
        scripts/logs/right_wall_follow_telemetry.csv (08:30:36 run, row at
        t=1789129862.6209798) that preceded the COLLISION trigger. The
        braked vx must be smaller than the pre-fix formula would have
        produced -- this test fails against the old, un-braked
        compute_follow_command.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        front, right = 0.314, 0.608

        command = follower.compute_follow_command(front=front, right=right)
        unbraked_vx, _ = _unbraked_vx_vy(cfg, front, right)

        assert command.vx < unbraked_vx

    def test_yaw_rate_unaffected_by_front_brake(self):
        """yaw_rate_deg_s depends only on heading_error -- confirm the brake
        didn't get accidentally coupled into it.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        braked = follower.compute_follow_command(front=0.20, right=0.60)
        unbraked = follower.compute_follow_command(front=1.0, right=1.4)  # same heading_error

        assert braked.yaw_rate_deg_s == pytest.approx(unbraked.yaw_rate_deg_s)

    def test_default_zone_matches_documented_numbers(self):
        """Pins the worked example in task.md -- front_threshold == 0.25 and
        front_zone_upper == 0.40 at the shipped defaults. A future change to
        _BASE_DETECTION_M/_REACTION_S or the defaults should surface here.
        """
        cfg = WallFollowConfig()

        assert _front_threshold(cfg) == pytest.approx(0.25)
        assert _front_threshold(cfg) + cfg.front_brake_zone_m == pytest.approx(0.40)


# ---------------------------------------------------------------------------
# Forward-push slew limit -- v_follow may drop instantly (braking, safety
# critical) but may only climb by a bounded step per call (recovering after
# a brake). Regression for scripts/logs/right_wall_follow.log's 12:30 run:
# front flickered between "clear" (2+ m) and "close" (0.26-0.5 m) within
# single 100 ms polls as the yaw correction swept past a corner edge -- each
# "clear" flicker snapped front_scale back to 1.0 and the loop immediately
# re-commanded full follow_velocity_m_s, so the flicker right before impact
# didn't leave enough margin to brake again in time. previous_v_follow=None
# (the default) means "no slew limit" -- a single, isolated call to this
# pure function behaves exactly as before; follow() is the one that
# maintains and passes the real previous_v_follow across its 10 Hz loop.
# ---------------------------------------------------------------------------


class TestForwardPushSlewLimit:
    def _max_step(self, cfg: WallFollowConfig) -> float:
        return (cfg.follow_velocity_m_s / _REACTION_S) * _POLL_INTERVAL_S

    def test_default_previous_v_follow_is_unlimited(self):
        """A bare call (no previous_v_follow) is not slew-limited -- this is
        the existing pure-function contract every other test in this file
        relies on.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg) + cfg.front_brake_zone_m  # front_scale == 1.0

        command = follower.compute_follow_command(front=front, right=right)

        assert command.v_follow == pytest.approx(cfg.follow_velocity_m_s)

    def test_braking_is_instant_regardless_of_previous_v_follow(self):
        """Dropping to a lower target must never be delayed -- braking is
        the safety-critical direction.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg)  # front_scale == 0.0 -> target v_follow == 0.0

        command = follower.compute_follow_command(
            front=front, right=right, previous_v_follow=cfg.follow_velocity_m_s
        )

        assert command.v_follow == pytest.approx(0.0)

    def test_recovery_to_full_speed_is_capped_per_cycle(self):
        """Recovering from a full brake (previous_v_follow=0.0) to a fully
        open front_scale must not snap straight to follow_velocity_m_s in
        one call -- it climbs by at most one slew step.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg) + cfg.front_brake_zone_m  # front_scale == 1.0

        command = follower.compute_follow_command(front=front, right=right, previous_v_follow=0.0)

        assert command.v_follow == pytest.approx(self._max_step(cfg))
        assert command.v_follow < cfg.follow_velocity_m_s

    def test_recovery_does_not_overshoot_target_within_one_step(self):
        """Once previous_v_follow is already within one slew step of the
        target, the result lands exactly on the target -- it must not
        overshoot past it.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg) + cfg.front_brake_zone_m  # target == follow_velocity_m_s
        previous = cfg.follow_velocity_m_s - (self._max_step(cfg) / 2.0)

        command = follower.compute_follow_command(
            front=front, right=right, previous_v_follow=previous
        )

        assert command.v_follow == pytest.approx(cfg.follow_velocity_m_s)

    def test_repeated_full_speed_calls_stay_at_target_not_dropping(self):
        """Steady state: once v_follow has reached the target, further calls
        at the same front/right must not drift below it -- confirms the
        slew clamp only limits increases, never holds a value down.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        right = 0.60
        front = _front_threshold(cfg) + cfg.front_brake_zone_m

        command = follower.compute_follow_command(
            front=front, right=right, previous_v_follow=cfg.follow_velocity_m_s
        )

        assert command.v_follow == pytest.approx(cfg.follow_velocity_m_s)


# ---------------------------------------------------------------------------
# Sustained yaw saturation -> straight-line hold, then reacquire once right
# recedes. Generalizes the earlier "front missing" corner-hold mechanism:
# a missing front is only one way heading_error can become large enough to
# saturate yaw at max_yaw_rate_deg_s (front_effective is substituted with
# MAX_RANGE_M) -- a real, valid, but large front reading against a
# close-and-stable right produces the identical runaway rotation. Reproduces
# scripts/logs/right_wall_follow.log's 15:17 run: front read valid 2.2-2.3 m
# values (never None) for ~2 continuous seconds while right stayed at
# ~0.44-0.58 m, saturating yaw_rate at -45 deg/s the whole time (~90 degrees
# of rotation at full speed) until the sensor picked up a surface that was,
# per hardware evidence, already close the whole time but outside front's
# narrow beam until the rotation swept onto it -- front then read 0.201 m
# on the very next 100 ms poll and both safety backstops fired.
#
# The old 08:58:12-run fix's point still holds unmodified: front is None
# only ever substitutes MAX_RANGE_M for the heading/standoff/brake formula
# and never affects wall_visible -- see TestRightMissingStillMeansWallLost.
# ---------------------------------------------------------------------------


class TestYawSaturationEntersHolding:
    """Entry into the straight-line hold requires yaw to have been
    saturated at max_yaw_rate_deg_s for yaw_saturation_hold_s continuously
    -- a single saturated cycle (the previous mechanism's instant trigger
    for a missing front) is not enough on its own.
    """

    def test_stays_normal_just_below_the_sustained_threshold(self):
        """saturated_cycles=0 plus this (saturated) cycle -> 1 cycle *
        _POLL_INTERVAL_S = 0.1 s < yaw_saturation_hold_s (0.20 s) -- still
        chasing the heading normally, not yet holding. yaw_saturation_hold_s
        is deliberately short (2 poll cycles) -- holding is a safe,
        conservative state, so there is no safety reason to wait longer;
        doing so would only reintroduce the phantom-wall-chasing behavior
        the 08:58:12 fix eliminated, for however long the wait lasts. See
        the module docstring.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        command = follower.compute_follow_command(front=2.3, right=0.5, saturated_cycles=0)

        assert command.holding is False
        assert command.yaw_rate_deg_s == pytest.approx(-cfg.max_yaw_rate_deg_s)
        assert command.saturated_cycles == 1

    def test_enters_holding_once_sustained_threshold_crossed(self):
        """saturated_cycles=1 plus this cycle -> 2 * 0.10 s = 0.2 s >=
        yaw_saturation_hold_s (0.20 s) -- now holds straight.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        command = follower.compute_follow_command(front=2.3, right=0.5, saturated_cycles=1)

        assert command.holding is True
        assert command.yaw_rate_deg_s == pytest.approx(0.0)
        assert command.saturated_cycles == 0

    def test_saturated_cycles_resets_when_heading_is_not_saturated(self):
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        command = follower.compute_follow_command(front=0.60, right=0.60, saturated_cycles=4)

        assert command.holding is False
        assert command.saturated_cycles == 0

    def test_missing_front_saturates_via_the_same_mechanism_not_instantly(self):
        """A missing front is no longer a special-cased instant trigger --
        it saturates yaw via the same MAX_RANGE_M-substitution formula as
        before, but still needs sustained saturation to enter holding, the
        same as a real large front reading. That said, with
        yaw_saturation_hold_s this short, the exposure window is only one
        poll cycle -- close to instant in practice, while still filtering a
        single noisy reading.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        still_normal = follower.compute_follow_command(front=None, right=0.40, saturated_cycles=0)
        now_holding = follower.compute_follow_command(front=None, right=0.40, saturated_cycles=1)

        assert still_normal.holding is False
        assert now_holding.holding is True
        assert now_holding.yaw_rate_deg_s == pytest.approx(0.0)


class TestHoldingStraightOnceEntered:
    """Once holding (previously_holding=True), yaw is held at 0.0 and the
    standoff correction is skipped, regardless of front's exact value --
    latched the same way regardless of whether the underlying cause was a
    missing front or a real large one.
    """

    def test_yaw_zero_and_no_standoff_skew_with_valid_far_front(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=2.3, right=0.5, previously_holding=True)

        assert command.yaw_rate_deg_s == pytest.approx(0.0)
        assert command.vx == pytest.approx(command.vy)
        assert command.holding is True

    def test_yaw_zero_and_no_standoff_skew_with_missing_front(self):
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=None, right=0.40, previously_holding=True)

        assert command.yaw_rate_deg_s == pytest.approx(0.0)
        assert command.vx == pytest.approx(command.vy)
        assert command.holding is True

    def test_v_follow_still_ramps_via_slew_limit(self):
        """The front-proximity-brake/slew pipeline for v_follow is
        untouched -- front_scale is naturally 1.0 (nothing detected ahead),
        so v_follow ramps toward follow_velocity_m_s at the normal slew
        rate, same as any other fully-open front reading (mirrors
        TestForwardPushSlewLimit).
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        max_step = (cfg.follow_velocity_m_s / _REACTION_S) * _POLL_INTERVAL_S

        command = follower.compute_follow_command(
            front=None, right=0.40, previous_v_follow=0.0, previously_holding=True
        )

        assert command.v_follow == pytest.approx(max_step)
        assert command.v_follow < cfg.follow_velocity_m_s

    def test_holds_when_right_is_stable_or_decreasing(self):
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        stable = follower.compute_follow_command(
            front=2.3, right=0.40, previous_right=0.40, previously_holding=True
        )
        decreasing = follower.compute_follow_command(
            front=2.3, right=0.35, previous_right=0.40, previously_holding=True
        )

        assert stable.holding is True
        assert decreasing.holding is True


class TestHoldingTransitionsToReacquiringWhenRightRecedes:
    def test_transitions_once_right_recedes_past_hysteresis(self):
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        previous_right = 0.40

        command = follower.compute_follow_command(
            front=2.3,
            right=previous_right + cfg.right_increasing_hysteresis_m + 0.01,
            previous_right=previous_right,
            previously_holding=True,
        )

        assert command.holding is False
        assert command.reacquiring is True
        assert command.yaw_rate_deg_s == pytest.approx(-cfg.max_yaw_rate_deg_s)

    def test_standoff_correction_resumes_once_reacquiring(self):
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        previous_right = 0.40
        right = previous_right + cfg.right_increasing_hysteresis_m + 0.01

        command = follower.compute_follow_command(
            front=2.3, right=right, previous_right=previous_right, previously_holding=True
        )
        unbraked_vx, unbraked_vy = _unbraked_vx_vy(cfg, 2.3, right)
        speed = math.hypot(unbraked_vx, unbraked_vy)
        if speed > cfg.max_velocity_m_s and speed > 0.0:
            scale = cfg.max_velocity_m_s / speed
            unbraked_vx *= scale
            unbraked_vy *= scale

        assert command.vx == pytest.approx(unbraked_vx)
        assert command.vy == pytest.approx(unbraked_vy)

    def test_exactly_at_hysteresis_boundary_still_holds(self):
        """The comparison is strictly greater-than -- exactly
        right_increasing_hysteresis_m above previous_right must NOT count
        as receding yet.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)
        previous_right = 0.40

        command = follower.compute_follow_command(
            front=2.3,
            right=previous_right + cfg.right_increasing_hysteresis_m,
            previous_right=previous_right,
            previously_holding=True,
        )

        assert command.holding is True
        assert command.reacquiring is False


class TestReacquiringLatchesUntilConverged:
    """right_receding must not be recomputed from a single cycle's delta
    alone once reacquiring has started (a real-world gradual/noisy right
    increase would otherwise chatter the loop between holding and yawing
    hard, cycle to cycle -- the original code-review finding this class
    guards against). Once previously_reacquiring is True, the loop stays in
    reacquire mode regardless of the current cycle's delta, exiting only
    once heading has genuinely converged (yaw no longer saturated) -- not
    merely because front happens to no longer be None, which is what let
    the 15:17 incident's valid-but-far front slip through the old gate.
    """

    def test_stays_reacquiring_even_when_this_cycles_delta_is_small(self):
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        command = follower.compute_follow_command(
            front=2.3,
            right=0.41,  # +0.01 versus previous_right -- below the 0.05 hysteresis
            previous_right=0.40,
            previously_reacquiring=True,
        )

        assert command.reacquiring is True
        assert command.yaw_rate_deg_s == pytest.approx(-cfg.max_yaw_rate_deg_s)

    def test_stays_reacquiring_even_when_right_momentarily_decreases(self):
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        command = follower.compute_follow_command(
            front=2.3,
            right=0.35,  # decreased versus previous_right -- still latched
            previous_right=0.40,
            previously_reacquiring=True,
        )

        assert command.reacquiring is True
        assert command.yaw_rate_deg_s == pytest.approx(-cfg.max_yaw_rate_deg_s)

    def test_exits_to_normal_once_heading_converges(self):
        """Once front and right are back in a normal following relationship
        (heading_error small enough that yaw is no longer saturated), the
        loop exits reacquiring entirely and resumes ordinary following --
        this is the fix for the 15:17 incident: exit is keyed off actual
        convergence, not merely "front is no longer None".
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        command = follower.compute_follow_command(
            front=0.60, right=0.55, previously_reacquiring=True
        )

        assert command.holding is False
        assert command.reacquiring is False
        # heading_error = 0.05 -> yaw_rate = -120 * 0.05 = -6, not zeroed.
        assert command.yaw_rate_deg_s == pytest.approx(-6.0)

    def test_previously_reacquiring_false_does_not_latch_on_its_own(self):
        """Sanity check: without previously_reacquiring=True, a small delta
        must not itself trigger reacquiring -- the latch only kicks in once
        actually set, not implicitly.
        """
        cfg = WallFollowConfig()
        follower = WallFollower(cfg)

        command = follower.compute_follow_command(
            front=2.3, right=0.41, previous_right=0.40, previously_holding=True
        )

        assert command.reacquiring is False
        assert command.holding is True


class TestRightMissingStillMeansWallLost:
    def test_right_missing_still_treated_as_wall_lost(self):
        """The one case that must NOT change: right missing means the
        followed wall is genuinely gone, regardless of front.
        """
        follower = WallFollower(WallFollowConfig())

        command = follower.compute_follow_command(front=0.40, right=None)

        assert command == FollowCommand(vx=0.0, vy=0.0, yaw_rate_deg_s=0.0, wall_visible=False)


# ---------------------------------------------------------------------------
# is_aligned
# ---------------------------------------------------------------------------


class TestIsAligned:
    def test_equal_readings_are_aligned(self):
        follower = WallFollower(WallFollowConfig())
        assert follower.is_aligned(0.60, 0.60) is True

    def test_within_tolerance_is_aligned(self):
        config = WallFollowConfig(align_tolerance_m=0.05)
        follower = WallFollower(config)
        assert follower.is_aligned(0.60, 0.58) is True  # diff 0.02, clearly within

    def test_beyond_tolerance_is_not_aligned(self):
        config = WallFollowConfig(align_tolerance_m=0.05)
        follower = WallFollower(config)
        assert follower.is_aligned(0.60, 0.54) is False

    @pytest.mark.parametrize("front,right", [(None, 0.5), (0.5, None), (0.0, 0.5), (0.5, 0.0)])
    def test_missing_reading_is_not_aligned(self, front, right):
        follower = WallFollower(WallFollowConfig())
        assert follower.is_aligned(front, right) is False


# ---------------------------------------------------------------------------
# is_too_close
# ---------------------------------------------------------------------------


class TestIsTooClose:
    @pytest.mark.parametrize("sensor", ["front", "back", "left", "right", "up"])
    def test_each_sensor_triggers_below_abort_distance(self, sensor):
        follower = WallFollower(WallFollowConfig(abort_distance_m=0.25))
        readings = _readings(**{sensor: 0.20})

        assert follower.is_too_close(readings) is True

    @pytest.mark.parametrize("sensor", ["front", "back", "left", "right", "up"])
    def test_each_sensor_clear_above_abort_distance(self, sensor):
        follower = WallFollower(WallFollowConfig(abort_distance_m=0.25))
        readings = _readings(**{sensor: 0.30})

        assert follower.is_too_close(readings) is False

    def test_none_readings_are_clear(self):
        follower = WallFollower(WallFollowConfig())
        assert follower.is_too_close(_readings()) is False

    def test_zero_readings_are_clear(self):
        follower = WallFollower(WallFollowConfig())
        readings = _readings(front=0.0, back=0.0, left=0.0, right=0.0, up=0.0)
        assert follower.is_too_close(readings) is False


# ---------------------------------------------------------------------------
# fly_to_first_obstacle
# ---------------------------------------------------------------------------


class TestFlyToFirstObstacle:
    def test_stops_when_front_reaches_target_distance(self, mocker):
        config = WallFollowConfig(target_wall_distance_m=0.60)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.side_effect = [
            _readings(front=1.0),
            _readings(front=0.80),
            _readings(front=0.55),
        ]
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        found = follower.fly_to_first_obstacle(mock_mc, mock_ranger)

        assert found is True
        mock_mc.start_forward.assert_called_once_with(config.approach_velocity_m_s)
        mock_mc.stop.assert_called_once()

    def test_returns_false_when_search_distance_exhausted(self, mocker):
        config = WallFollowConfig(max_search_distance_m=0.03, approach_velocity_m_s=0.15)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=5.0)  # never close enough
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        found = follower.fly_to_first_obstacle(mock_mc, mock_ranger)

        assert found is False
        mock_mc.stop.assert_called_once()

    def test_honors_should_abort(self, mocker):
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=5.0)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        found = follower.fly_to_first_obstacle(mock_mc, mock_ranger, should_abort=lambda: True)

        assert found is False
        mock_mc.stop.assert_called_once()

    def test_writes_forward_direction_and_approach_velocity_to_flight_state(self, mocker):
        state = FlightState()
        config = WallFollowConfig(approach_velocity_m_s=0.15)
        follower = WallFollower(config, flight_state=state)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.55)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.fly_to_first_obstacle(mock_mc, mock_ranger)

        assert state.get_direction() == "forward"
        assert state.get_velocity() == pytest.approx(0.15)


# ---------------------------------------------------------------------------
# align_to_wall
# ---------------------------------------------------------------------------


class TestAlignToWall:
    def test_stops_turning_once_aligned(self, mocker):
        config = WallFollowConfig(align_step_deg=5.0, align_tolerance_m=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.side_effect = [
            _readings(front=1.0, right=0.5),  # not aligned
            _readings(front=0.55, right=0.53),  # aligned (diff 0.02, clearly within tolerance)
        ]

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is True
        mock_mc.turn_left.assert_called_once_with(config.align_step_deg)

    def test_already_aligned_makes_no_turn(self, mocker):
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is True
        mock_mc.turn_left.assert_not_called()

    def test_gives_up_after_max_align_deg(self, mocker):
        config = WallFollowConfig(align_step_deg=5.0, max_align_deg=20.0)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=1.0, right=0.1)  # never aligns

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is False
        assert mock_mc.turn_left.call_count == 4  # 20.0 / 5.0

    def test_stops_on_sign_flip_when_tolerance_is_never_hit_exactly(self, mocker):
        """Regression: near 45 degrees, front/right can diverge by far more
        than align_tolerance_m per align_step_deg, so a real approach can
        straddle the aligned heading without ever landing inside a narrow
        tolerance window. Detecting the sign change of (front - right)
        between steps must still terminate alignment - follow()'s
        continuous yaw correction removes the remaining residual.
        """
        config = WallFollowConfig(align_step_deg=5.0, align_tolerance_m=0.05, max_align_deg=120.0)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.side_effect = [
            _readings(front=0.752, right=0.806),  # error = -0.054, not within tolerance
            _readings(front=0.822, right=0.740),  # error = +0.082, sign flipped -> stop
        ]

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is True
        mock_mc.turn_left.assert_called_once_with(config.align_step_deg)

    def test_does_not_stop_on_constant_sign_that_never_flips(self, mocker):
        """The never-aligns fixture in test_gives_up_after_max_align_deg has
        a constant, non-flipping sign (front > right throughout) - confirm
        the sign-flip check does not cause an early false-positive exit
        there.
        """
        config = WallFollowConfig(align_step_deg=5.0, max_align_deg=20.0)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=1.0, right=0.1)

        result = follower.align_to_wall(mock_mc, mock_ranger)

        assert result is False
        assert mock_mc.turn_left.call_count == 4

    def test_clears_flight_state_direction_and_velocity(self, mocker):
        state = FlightState(current_velocity_m_s=0.15)
        state.set_direction("forward")
        follower = WallFollower(WallFollowConfig(), flight_state=state)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)

        follower.align_to_wall(mock_mc, mock_ranger)

        assert state.get_direction() is None
        assert state.get_velocity() == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# follow - the 10 Hz closed-loop control
# ---------------------------------------------------------------------------


class TestFollow:
    def test_should_abort_checked_before_every_motion_command(self, mocker):
        """The safety-critical guarantee (decision 5): should_abort() is
        re-checked immediately before every start_linear_motion() call, so
        no motion command is ever issued after should_abort() has returned
        True - CollisionMonitor's own mc.stop() must never be overridden by
        the next control cycle.
        """
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        call_order: list[str] = []
        state = {"n": 0}

        def should_abort() -> bool:
            state["n"] += 1
            call_order.append(f"abort_check_{state['n']}")
            return state["n"] >= 5

        mock_mc.start_linear_motion.side_effect = lambda *a, **kw: call_order.append("motion")

        follower.follow(mock_mc, mock_ranger, should_abort=should_abort)

        first_true_index = call_order.index("abort_check_5")
        motion_indices = [i for i, v in enumerate(call_order) if v == "motion"]
        assert all(i < first_true_index for i in motion_indices)
        mock_mc.stop.assert_called_once()

    def test_exits_when_follow_duration_elapses(self, mocker):
        config = WallFollowConfig(follow_duration_s=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        clock = {"t": 0.0}

        def fake_monotonic() -> float:
            clock["t"] += 0.02
            return clock["t"]

        mocker.patch("Crazyflie.flight.wall_follower.time.monotonic", side_effect=fake_monotonic)

        follower.follow(mock_mc, mock_ranger, should_abort=None)

        mock_mc.stop.assert_called_once()

    def test_exits_after_wall_lost_timeout(self, mocker):
        config = WallFollowConfig(follow_duration_s=100.0, wall_lost_timeout_s=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=None, right=None)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        clock = {"t": 0.0}

        def fake_monotonic() -> float:
            clock["t"] += 0.02
            return clock["t"]

        mocker.patch("Crazyflie.flight.wall_follower.time.monotonic", side_effect=fake_monotonic)

        follower.follow(mock_mc, mock_ranger, should_abort=None)

        # Stops exactly once. Any motion commands issued while still within
        # the wall_lost_timeout_s grace period are safe zero-velocity
        # commands (wall_visible=False -> compute_follow_command returns an
        # all-zero FollowCommand), not stale nonzero ones.
        mock_mc.stop.assert_called_once()
        for call in mock_mc.start_linear_motion.call_args_list:
            assert call.args == (0.0, 0.0, 0.0)

    def test_front_missing_does_not_trigger_wall_lost(self, mocker, caplog):
        """Regression for scripts/logs/right_wall_follow.log's 08:58:12 run:
        front permanently None (right still valid) must not be treated as
        the wall being lost -- wall_lost_timeout_s is deliberately very
        short here so the bug (wall-lost firing) would reproduce within a
        couple of poll cycles if it still existed.
        """
        config = WallFollowConfig(follow_duration_s=100.0, wall_lost_timeout_s=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=None, right=0.40)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        clock = {"t": 0.0}

        def fake_monotonic() -> float:
            clock["t"] += 0.02
            return clock["t"]

        mocker.patch("Crazyflie.flight.wall_follower.time.monotonic", side_effect=fake_monotonic)

        call_count = {"n": 0}

        def should_abort() -> bool:
            call_count["n"] += 1
            return call_count["n"] > 10

        with caplog.at_level("WARNING", logger="Crazyflie.flight.wall_follower"):
            follower.follow(mock_mc, mock_ranger, should_abort=should_abort)

        assert "wall lost" not in caplog.text
        assert mock_mc.start_linear_motion.call_count > 0
        for call in mock_mc.start_linear_motion.call_args_list:
            assert call.args != (0.0, 0.0, 0.0)

    def test_right_missing_still_triggers_wall_lost(self, mocker):
        """Control case: right missing must still stop the flight via
        wall_lost_timeout_s -- the fix narrows what counts as "lost", it
        does not weaken real wall-loss detection.
        """
        config = WallFollowConfig(follow_duration_s=100.0, wall_lost_timeout_s=0.05)
        follower = WallFollower(config)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.40, right=None)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        clock = {"t": 0.0}

        def fake_monotonic() -> float:
            clock["t"] += 0.02
            return clock["t"]

        mocker.patch("Crazyflie.flight.wall_follower.time.monotonic", side_effect=fake_monotonic)

        follower.follow(mock_mc, mock_ranger, should_abort=None)

        mock_mc.stop.assert_called_once()
        for call in mock_mc.start_linear_motion.call_args_list:
            assert call.args == (0.0, 0.0, 0.0)

    def test_exits_when_too_close(self, mocker):
        follower = WallFollower(WallFollowConfig(abort_distance_m=0.25))
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, left=0.10, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.follow(mock_mc, mock_ranger, should_abort=None)

        mock_mc.stop.assert_called_once()
        mock_mc.start_linear_motion.assert_not_called()

    def test_writes_forward_left_direction_and_speed_to_flight_state(self, mocker):
        state = FlightState()
        config = WallFollowConfig(follow_duration_s=100.0)
        follower = WallFollower(config, flight_state=state)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.follow(
            mock_mc, mock_ranger, should_abort=lambda: state.get_direction() is not None
        )

        assert state.get_direction() == FLIGHT_DIRECTION
        assert state.get_velocity() > 0.0

    def test_front_flicker_does_not_snap_back_to_full_speed(self, mocker):
        """Regression for scripts/logs/right_wall_follow.log's 12:30 run:
        front flickered clear-close-clear-close within single 100 ms polls
        at a corner. The forward-push component of the very next command
        after a "clear" poll immediately following a "close" one must not
        jump straight back to follow_velocity_m_s -- follow() must carry
        the previous cycle's v_follow across iterations so the slew limit
        actually applies during a real flight, not just in isolated calls
        to compute_follow_command.
        """
        cfg = WallFollowConfig(follow_duration_s=100.0)
        follower = WallFollower(cfg)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        right = cfg.target_wall_distance_m
        clear_front = _front_threshold(cfg) + cfg.front_brake_zone_m + 1.0  # front_scale == 1.0
        close_front = _front_threshold(cfg)  # front_scale == 0.0
        readings_sequence = [
            _readings(front=clear_front, right=right),
            _readings(front=close_front, right=right),
            _readings(front=clear_front, right=right),  # the flicker back to "clear"
        ]
        mock_ranger.get_readings.side_effect = readings_sequence
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        # should_abort() is called twice per iteration (top-of-loop and
        # again immediately before the motion command) -- allow exactly
        # len(readings_sequence) full iterations before returning True.
        call_count = {"n": 0}

        def should_abort() -> bool:
            call_count["n"] += 1
            return call_count["n"] > len(readings_sequence) * 2

        follower.follow(mock_mc, mock_ranger, should_abort=should_abort)

        vx_values = [call.args[0] for call in mock_mc.start_linear_motion.call_args_list]
        assert len(vx_values) == len(readings_sequence)
        unbraked_vx, _ = _unbraked_vx_vy(cfg, clear_front, right)
        # Cycle 3's vx (post-flicker, previous_v_follow braked to 0.0 by
        # cycle 2) must be well short of the fully-open unbraked vx -- if
        # the brake had snapped straight back to full speed on the single
        # "clear" poll, it would equal unbraked_vx instead.
        assert vx_values[2] < unbraked_vx

    def test_sustained_far_front_eventually_holds_then_reacquires(self, mocker):
        """follow()-level regression for the 15:17 incident: front reading
        valid but far (2.3 m) against a close, stable right (0.5 m) is the
        actual scenario that caused the sustained ~90 degree rotation --
        must eventually hold straight once saturated_cycles/holding state
        is tracked and passed across follow()'s loop, not just in isolated
        calls to compute_follow_command. yaw_saturation_hold_s is set small
        here purely to keep the reading sequence short.
        """
        cfg = WallFollowConfig(follow_duration_s=100.0, yaw_saturation_hold_s=0.15)
        follower = WallFollower(cfg)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        stable_right = 0.50
        receding_right = stable_right + cfg.right_increasing_hysteresis_m + 0.01
        readings_sequence = [
            _readings(front=2.3, right=stable_right),  # cycle 1/2 saturated -> still normal
            _readings(front=2.3, right=stable_right),  # cycle 2/2 saturated -> now holding
            _readings(front=2.3, right=stable_right),  # right stable -> still holding
            _readings(front=2.3, right=receding_right),  # right recedes -> reacquiring
        ]
        mock_ranger.get_readings.side_effect = readings_sequence
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        call_count = {"n": 0}

        def should_abort() -> bool:
            call_count["n"] += 1
            return call_count["n"] > len(readings_sequence) * 2

        follower.follow(mock_mc, mock_ranger, should_abort=should_abort)

        yaw_values = [
            call.kwargs["rate_yaw"] for call in mock_mc.start_linear_motion.call_args_list
        ]
        assert len(yaw_values) == len(readings_sequence)
        assert yaw_values[0] == pytest.approx(-cfg.max_yaw_rate_deg_s)
        assert yaw_values[1] == pytest.approx(0.0)
        assert yaw_values[2] == pytest.approx(0.0)
        assert yaw_values[3] == pytest.approx(-cfg.max_yaw_rate_deg_s)

    def test_reacquiring_exits_to_normal_once_heading_converges(self, mocker):
        """follow()-level regression: once reacquiring and front is
        actually reacquired (heading_error small again, no longer
        saturated), the loop must resume ordinary following -- this is the
        actual fix for the 15:17 incident, where the old front-is-no-
        longer-None exit criterion let a still-runaway heading slip through
        because front had briefly read a real, valid, but unhelpful value.
        """
        cfg = WallFollowConfig(follow_duration_s=100.0, yaw_saturation_hold_s=0.15)
        follower = WallFollower(cfg)
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        stable_right = 0.50
        receding_right = stable_right + cfg.right_increasing_hysteresis_m + 0.01
        readings_sequence = [
            _readings(front=2.3, right=stable_right),  # saturated 1/2 -> still normal
            _readings(front=2.3, right=stable_right),  # saturated 2/2 -> now holding
            _readings(front=2.3, right=receding_right),  # right recedes -> reacquiring
            _readings(front=0.60, right=0.55),  # front reacquired -> converged, exit
        ]
        mock_ranger.get_readings.side_effect = readings_sequence
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        call_count = {"n": 0}

        def should_abort() -> bool:
            call_count["n"] += 1
            return call_count["n"] > len(readings_sequence) * 2

        follower.follow(mock_mc, mock_ranger, should_abort=should_abort)

        yaw_values = [
            call.kwargs["rate_yaw"] for call in mock_mc.start_linear_motion.call_args_list
        ]
        assert len(yaw_values) == len(readings_sequence)
        assert yaw_values[0] == pytest.approx(-cfg.max_yaw_rate_deg_s)
        assert yaw_values[1] == pytest.approx(0.0)
        assert yaw_values[2] == pytest.approx(-cfg.max_yaw_rate_deg_s)
        # heading_error = 0.60 - 0.55 = 0.05 -> yaw_rate = -120 * 0.05 = -6.0,
        # not clamped and not zeroed -- ordinary following has resumed.
        assert yaw_values[3] == pytest.approx(-6.0)

    def test_stop_called_exactly_once_on_normal_abort(self, mocker):
        follower = WallFollower(WallFollowConfig())
        mock_mc = mocker.MagicMock()
        mock_ranger = mocker.MagicMock()
        mock_ranger.get_readings.return_value = _readings(front=0.60, right=0.60)
        mocker.patch("Crazyflie.flight.wall_follower.time.sleep")

        follower.follow(mock_mc, mock_ranger, should_abort=lambda: True)

        mock_mc.stop.assert_called_once()
        mock_mc.start_linear_motion.assert_not_called()
