"""Tests for FlightState.

Written test-first following the TDD rules for this project.
"""

import threading

import pytest

from Crazyflie.state.flight_state import FlightState


class TestDefaults:
    def test_initial_velocity_is_zero(self):
        state = FlightState()

        assert state.get_velocity() == pytest.approx(0.0)

    def test_initial_velocity_can_be_set_via_constructor(self):
        state = FlightState(current_velocity_m_s=0.5)

        assert state.get_velocity() == pytest.approx(0.5)


class TestSetGet:
    def test_set_velocity_updates_value(self):
        state = FlightState()

        state.set_velocity(0.3)

        assert state.get_velocity() == pytest.approx(0.3)

    def test_multiple_sets_reflect_latest_value(self):
        state = FlightState()

        state.set_velocity(0.2)
        state.set_velocity(0.7)

        assert state.get_velocity() == pytest.approx(0.7)

    def test_set_velocity_to_zero(self):
        state = FlightState(current_velocity_m_s=0.5)

        state.set_velocity(0.0)

        assert state.get_velocity() == pytest.approx(0.0)

    def test_get_returns_exact_value_set(self):
        state = FlightState()

        state.set_velocity(0.123)

        assert state.get_velocity() == pytest.approx(0.123)


class TestThreadSafety:
    def test_concurrent_writes_do_not_raise(self):
        """Many threads writing concurrently must not raise or corrupt state."""
        state = FlightState()
        errors: list[str] = []

        def writer(value: float) -> None:
            try:
                for _ in range(1000):
                    state.set_velocity(value)
                    _ = state.get_velocity()
            except Exception as exc:  # noqa: BLE001
                errors.append(str(exc))

        threads = [threading.Thread(target=writer, args=(float(i),)) for i in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert not errors

    def test_final_value_after_concurrent_writes_is_one_of_the_written_values(self):
        """After all threads finish, the stored value must be one written by a thread."""
        state = FlightState()
        written = {float(i) for i in range(5)}

        def writer(value: float) -> None:
            for _ in range(200):
                state.set_velocity(value)

        threads = [threading.Thread(target=writer, args=(v,)) for v in written]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert state.get_velocity() in written
