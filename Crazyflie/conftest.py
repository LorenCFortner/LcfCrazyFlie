"""Shared pytest fixtures for all Crazyflie tests."""

import pytest

from Crazyflie.hardware.drone_connection import DroneConnection, MotionControl


@pytest.fixture
def mock_drone(mocker):
    """Mock DroneConnection - the project-owned hardware boundary."""
    return mocker.MagicMock(spec=DroneConnection)


@pytest.fixture
def mock_mc(mocker):
    """Mock MotionControl with all movement methods stubbed."""
    return mocker.MagicMock(spec=MotionControl)


@pytest.fixture
def mock_queue(mocker):
    """Mock queue.Queue for testing thread communication."""
    return mocker.MagicMock()
