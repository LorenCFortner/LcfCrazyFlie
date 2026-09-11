"""Tests for Crazyflie.observability.run_logging.

Written test-first (TDD). configure_run_logging() mutates global logging
state (the real root logger, plus its own module-level handler-tracking
variable), so every logging.* call it makes is mocked here - tests assert
on the calls made, not on real logger/handler side effects, keeping the
suite hermetic and avoiding cross-test pollution. reset_active_file_handler
resets the module's own tracked-handler state around every test for the
same reason.
"""

import logging
import threading
from unittest.mock import MagicMock

import pytest

import Crazyflie.observability.run_logging as run_logging_module
from Crazyflie.observability.run_logging import configure_run_logging


@pytest.fixture(autouse=True)
def reset_active_file_handler():
    """Reset the module's tracked-handler state around every test."""
    run_logging_module._active_file_handler = None
    yield
    run_logging_module._active_file_handler = None


@pytest.fixture
def mock_logging(mocker) -> dict[str, object]:
    """Mock every logging.* call configure_run_logging makes.

    Returns a dict with the mocked root logger, console handler, the
    logging.getLogger patch (keyed by logger name), logging.basicConfig,
    the logging.FileHandler class patch, and the list of FileHandler mock
    instances created (in call order) - one per configure_run_logging call.
    """
    mock_basic_config = mocker.patch("Crazyflie.observability.run_logging.logging.basicConfig")

    console_handler = mocker.MagicMock()
    root_logger = mocker.MagicMock()
    root_logger.handlers = [console_handler]

    named_loggers: dict[str, MagicMock] = {}

    def fake_get_logger(name: str | None = None) -> MagicMock:
        if name is None:
            return root_logger
        if name not in named_loggers:
            named_loggers[name] = mocker.MagicMock()
        return named_loggers[name]

    mocker.patch(
        "Crazyflie.observability.run_logging.logging.getLogger", side_effect=fake_get_logger
    )

    created_file_handlers: list[MagicMock] = []

    def fake_file_handler(*args: object, **kwargs: object) -> MagicMock:
        handler = mocker.MagicMock()
        created_file_handlers.append(handler)
        return handler

    mock_file_handler_cls = mocker.patch(
        "Crazyflie.observability.run_logging.logging.FileHandler", side_effect=fake_file_handler
    )

    return {
        "basic_config": mock_basic_config,
        "root_logger": root_logger,
        "console_handler": console_handler,
        "named_loggers": named_loggers,
        "file_handler_cls": mock_file_handler_cls,
        "created_file_handlers": created_file_handlers,
    }


def test_does_not_call_basic_config(mock_logging, tmp_path):
    """configure_run_logging must not call basicConfig - only a script's
    own main() may (see .claude/rules/crazyflie/naming-and-structure.md).
    """
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    mock_logging["basic_config"].assert_not_called()


def test_sets_console_handler_to_info_by_default(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    mock_logging["console_handler"].setLevel.assert_called_once_with(logging.INFO)


def test_console_level_is_overridable(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log", console_level=logging.ERROR)

    mock_logging["console_handler"].setLevel.assert_called_once_with(logging.ERROR)


def test_creates_log_file_parent_directory(mock_logging, tmp_path):
    log_file = tmp_path / "logs" / "run.log"

    configure_run_logging("my.script", log_file)

    assert log_file.parent.exists()


def test_creates_file_handler_in_overwrite_mode(mock_logging, tmp_path):
    log_file = tmp_path / "logs" / "run.log"

    configure_run_logging("my.script", log_file)

    mock_logging["file_handler_cls"].assert_called_once_with(log_file, mode="w", encoding="utf-8")


def test_file_handler_set_to_info_by_default(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    file_handler = mock_logging["created_file_handlers"][0]
    file_handler.setLevel.assert_called_once_with(logging.INFO)


def test_file_level_is_overridable(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log", file_level=logging.DEBUG)

    file_handler = mock_logging["created_file_handlers"][0]
    file_handler.setLevel.assert_called_once_with(logging.DEBUG)


def test_sets_file_handler_formatter(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    file_handler = mock_logging["created_file_handlers"][0]
    file_handler.setFormatter.assert_called_once()
    formatter_arg = file_handler.setFormatter.call_args[0][0]
    assert isinstance(formatter_arg, logging.Formatter)


def test_file_handler_attached_to_root_logger(mock_logging, tmp_path):
    # Patching logging.getLogger reaches the real shared `logging` module, so
    # unrelated infrastructure (e.g. pytest's own log capture) may also call
    # addHandler on this mock during the test - assert_any_call tolerates that.
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    file_handler = mock_logging["created_file_handlers"][0]
    mock_logging["root_logger"].addHandler.assert_any_call(file_handler)


def test_suppresses_cflib_to_critical(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    mock_logging["named_loggers"]["cflib"].setLevel.assert_called_once_with(logging.CRITICAL)


def test_sets_script_logger_to_file_level(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log", file_level=logging.DEBUG)

    mock_logging["named_loggers"]["my.script"].setLevel.assert_called_once_with(logging.DEBUG)


def test_sets_crazyflie_package_logger_to_file_level(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log", file_level=logging.DEBUG)

    mock_logging["named_loggers"]["Crazyflie"].setLevel.assert_called_once_with(logging.DEBUG)


# ---------------------------------------------------------------------------
# Uncaught background-thread exceptions - must reach the log file, not just
# stderr (Python's default threading.excepthook only prints there, so a
# CollisionMonitor/StabilizerMonitor/LinkMonitor background thread crashing
# would otherwise leave no trace in the run's own log).
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def restore_threading_excepthook():
    """threading.excepthook is process-global state - restore it after
    every test so this file's own tests don't leak into others.
    """
    original = threading.excepthook
    yield
    threading.excepthook = original


def test_installs_a_threading_excepthook(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    assert threading.excepthook is run_logging_module._log_thread_exception


def test_thread_exception_hook_logs_via_crazyflie_logger_with_traceback(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    expected_exc_info: tuple[object, object, object] = (None, None, None)
    try:
        raise ValueError("boom")
    except ValueError as exc:
        expected_exc_info = (ValueError, exc, exc.__traceback__)
        args = threading.ExceptHookArgs(
            (ValueError, exc, exc.__traceback__, threading.current_thread())
        )

    threading.excepthook(args)

    crazyflie_logger = mock_logging["named_loggers"]["Crazyflie"]
    crazyflie_logger.critical.assert_called_once()
    _, kwargs = crazyflie_logger.critical.call_args
    assert kwargs["exc_info"] == expected_exc_info


def test_thread_exception_hook_includes_thread_name(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    fake_thread = threading.Thread(name="CollisionMonitor-poll")
    try:
        raise RuntimeError("boom")
    except RuntimeError as exc:
        args = threading.ExceptHookArgs((RuntimeError, exc, exc.__traceback__, fake_thread))

    threading.excepthook(args)

    crazyflie_logger = mock_logging["named_loggers"]["Crazyflie"]
    call_args = crazyflie_logger.critical.call_args
    assert "CollisionMonitor-poll" in call_args.args


def test_thread_exception_hook_handles_missing_exc_info_without_raising(mock_logging, tmp_path):
    """threading.ExceptHookArgs technically allows exc_type/exc_value to be
    None -- confirm the hook degrades to a plain log message instead of
    raising when that happens, rather than assuming real callers.
    """
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")
    args = threading.ExceptHookArgs((None, None, None, threading.current_thread()))

    threading.excepthook(args)  # should not raise

    crazyflie_logger = mock_logging["named_loggers"]["Crazyflie"]
    crazyflie_logger.critical.assert_called_once()
    _, kwargs = crazyflie_logger.critical.call_args
    assert "exc_info" not in kwargs


# ---------------------------------------------------------------------------
# Repeat calls in the same process - the previous FileHandler must be
# closed and detached, not left accumulating.
# ---------------------------------------------------------------------------


def test_first_call_has_no_previous_handler_to_remove(mock_logging, tmp_path):
    # Patching logging.getLogger reaches the real shared `logging` module, so
    # unrelated infrastructure (e.g. pytest's own log capture) may also call
    # removeHandler during the test - assert only that none of our own
    # FileHandler mocks (there are none yet) were ever passed to it.
    configure_run_logging("my.script", tmp_path / "logs" / "run.log")

    removed = [call.args[0] for call in mock_logging["root_logger"].removeHandler.call_args_list]
    assert not (set(removed) & set(mock_logging["created_file_handlers"]))


def test_removes_and_closes_previous_file_handler_on_repeat_call(mock_logging, tmp_path):
    configure_run_logging("my.script", tmp_path / "logs" / "first.log")
    first_handler = mock_logging["created_file_handlers"][0]

    configure_run_logging("my.script", tmp_path / "logs" / "second.log")
    second_handler = mock_logging["created_file_handlers"][1]

    mock_logging["root_logger"].removeHandler.assert_any_call(first_handler)
    first_handler.close.assert_called_once()
    mock_logging["root_logger"].addHandler.assert_any_call(second_handler)
