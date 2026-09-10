"""Console + file logging setup for Crazyflie scripts.

Every script wires up the same handful of lines by hand: suppress cflib's
own debug noise, show the script's own progress messages, and keep the
console quiet while a run is in flight. configure_run_logging() does all of
that plus a second handler that additionally writes a full INFO+ trace to a
log file — so a run's detail (flight progress, collision response, retrace
decisions) is always available on disk even when the console stays at
WARNING+.

configure_run_logging() only adjusts handlers/levels — it never calls
logging.basicConfig() itself, since only a script's own main() should do
that (see .claude/rules/crazyflie/naming-and-structure.md). Call
logging.basicConfig() first, then this function, both from main().

Example:
    >>> def main() -> None:
    ...     logging.basicConfig(level=logging.ERROR)
    ...     configure_run_logging(__name__, Path(__file__).parent / "logs" / "my_script.log")
"""

import logging
from pathlib import Path

_CFLIB_LOGGER_NAME: str = "cflib"
_PACKAGE_LOGGER_NAME: str = "Crazyflie"
_FILE_LOG_FORMAT: str = "%(asctime)s %(levelname)s %(name)s: %(message)s"

# Tracks the FileHandler added by the most recent configure_run_logging()
# call in this process, so a repeat call can close and detach it instead of
# accumulating handlers. Module-level by design: there is exactly one
# "current run's log file" per process.
_active_file_handler: logging.Handler | None = None


def configure_run_logging(
    script_logger_name: str,
    log_file: Path,
    console_level: int = logging.INFO,
    file_level: int = logging.INFO,
) -> None:
    """Configure console + file logging for a script's main().

    Must be called after logging.basicConfig() (this function does not call
    it) — console_level is applied to the console handler basicConfig()
    adds, assumed to be the first handler on the root logger.

    Console output is capped at console_level (default WARNING, so routine
    flight progress stays off the console). A full trace at file_level and
    above (default INFO) is additionally written to log_file, so both
    happen on every run: quiet console, detailed file. cflib's own debug
    noise is always suppressed to CRITICAL regardless of the levels above.

    log_file's parent directory is created if missing, and the file is
    overwritten (not appended to) on each call, so each run gets a clean
    trace of just that run. The FileHandler added by a prior call in this
    process (tracked in _active_file_handler) is closed and removed first,
    so repeated calls (e.g. a supervisor running several flights per
    process) don't accumulate handlers or keep writing to a previous run's
    file.

    Args:
        script_logger_name: The calling script's own logger name (pass
            __name__), set to file_level so the script's own messages are
            captured in the file alongside the Crazyflie package's.
        log_file: Path to write the full run log to.
        console_level: Minimum level shown on the console. Defaults to
            logging.WARNING.
        file_level: Minimum level captured in log_file, and the level set
            on script_logger_name and the "Crazyflie" package logger so
            their messages reach the file. Defaults to logging.INFO.
    """
    global _active_file_handler

    root_logger = logging.getLogger()
    root_logger.handlers[0].setLevel(console_level)

    if _active_file_handler is not None:
        root_logger.removeHandler(_active_file_handler)
        _active_file_handler.close()

    log_file.parent.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(log_file, mode="w")
    file_handler.setLevel(file_level)
    file_handler.setFormatter(logging.Formatter(_FILE_LOG_FORMAT))
    root_logger.addHandler(file_handler)
    _active_file_handler = file_handler

    logging.getLogger(_CFLIB_LOGGER_NAME).setLevel(logging.CRITICAL)
    logging.getLogger(script_logger_name).setLevel(file_level)
    logging.getLogger(_PACKAGE_LOGGER_NAME).setLevel(file_level)
