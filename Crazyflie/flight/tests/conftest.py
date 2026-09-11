"""Shared test helpers for Crazyflie.flight runner tests.

Both out_and_back_runner and wall_follow_runner delegate their shared
connect/clearance/monitor/teardown lifecycle to
Crazyflie.flight.flight_lifecycle.run_flight_lifecycle() (see
test_flight_lifecycle.py for that machinery's own coverage). Each runner's
test file mocks run_flight_lifecycle itself and inspects/invokes the
flight_body_fn and hooks the runner built, to exercise only that runner's own
logic.
"""

from typing import Any


def capture_lifecycle_call(mocker: Any, module_path: str) -> Any:
    """Patch run_flight_lifecycle in the given runner module.

    Args:
        mocker: pytest-mock's mocker fixture.
        module_path: Dotted path to the runner module, e.g.
            "Crazyflie.flight.out_and_back_runner".

    Returns:
        The MagicMock replacing run_flight_lifecycle, so its call_args can be
        inspected to recover the flight_body_fn and hooks the runner built.
    """
    return mocker.patch(f"{module_path}.run_flight_lifecycle")
