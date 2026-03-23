"""Shared pytest fixtures for all Crazyflie tests."""

import pytest


@pytest.fixture
def mock_scf(mocker):
    """Mock SyncCrazyflie with a fully stubbed cf layer."""
    scf = mocker.MagicMock()
    scf.cf.param.set_value = mocker.MagicMock()
    return scf


@pytest.fixture
def mock_mc(mocker):
    """Mock MotionCommander with all movement methods stubbed."""
    return mocker.MagicMock()


@pytest.fixture
def mock_queue(mocker):
    """Mock queue.Queue for testing thread communication."""
    return mocker.MagicMock()
