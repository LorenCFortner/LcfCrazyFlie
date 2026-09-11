"""Fly forward, then follow a wall on the right using closed-loop control.

Uses WallFollower for continuous, sensor-driven flight rather than a
pre-planned path: flies forward until the front Multi-ranger sensor reaches
the target wall distance, rotates counter-clockwise until front and right
read equal (45 degrees to the wall), then flies that diagonal (forward and
left simultaneously) along the wall — continuously yawing to hold
front == right and holding their common value at the target distance.

Blade protection is active for the entire flight, on all five Multi-ranger
sensors, via CollisionMonitor's normal detection (extended to understand the
"forward_left" diagonal travel direction) plus WallFollower's own proximity
check. See Crazyflie.flight.wall_follower and
Crazyflie.safety.collision_monitor for the full design.

Pre-flight:
  1. Clearance check — aborts if any direction is within 0.1 m.
  2. LED headlights on.

Post-flight: LED ring off.

Safety events handled: CRASH, BATLOW, COLLISION. Any of these — or the
follower's own proximity abort, or losing sight of the wall — stops the
flight and lands in place. There is no retrace-home behaviour: a wall
follow has no recorded path to retrace.

Defaults are deliberately conservative for a first real flight: 0.15 m/s
along the wall, 45 s duration, ~0.42 m perpendicular standoff from the wall
(0.60 m front == right setpoint, divided by sqrt(2)).
"""

import logging
from pathlib import Path

from Crazyflie.flight.wall_follow_runner import run_wall_follow_flight
from Crazyflie.flight.wall_follower import WallFollowConfig
from Crazyflie.observability.run_logging import configure_run_logging

logger = logging.getLogger(__name__)

URI = "radio://0/1/250K"
_LOG_FILE: Path = Path(__file__).parent / "logs" / "right_wall_follow.log"
_TELEMETRY_FILE: Path = Path(__file__).parent / "logs" / "right_wall_follow_telemetry.csv"

WALL_FOLLOW_CONFIG: WallFollowConfig = WallFollowConfig()


def main() -> None:
    """Main entry point for the right-wall-following script.

    A full INFO+ trace of the run is written to _LOG_FILE (overwritten each
    run) as well as the console.
    """
    logging.basicConfig(level=logging.ERROR)
    configure_run_logging(__name__, _LOG_FILE)
    logger.info(f"Writing full run log to {_LOG_FILE}")

    run_wall_follow_flight(
        uri=URI,
        config=WALL_FOLLOW_CONFIG,
        telemetry_file=_TELEMETRY_FILE,
    )


if __name__ == "__main__":
    main()
