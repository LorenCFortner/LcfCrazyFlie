"""Console + file logging setup for Crazyflie scripts.

Every script wires up the same handful of lines by hand: suppress cflib's
own debug noise, show the script's own progress messages, and write a full
INFO+ trace to a log file - so a run's detail (flight progress, collision
response, retrace decisions) is always available on disk. configure_run_logging()
does all of that in one call.

It also installs a threading.excepthook (see _log_thread_exception) so an
uncaught exception on any background thread (CollisionMonitor's avoidance
move, StabilizerMonitor's/LinkMonitor's poll loop, ...) is logged through
the same file and console output as everything else, instead of Python's
default behavior of only printing it to stderr - which would otherwise
leave a crashed safety thread with no trace in the run's own log.

configure_run_logging() only adjusts handlers/levels - it never calls
logging.basicConfig() itself, since only a script's own main() should do
that (see .claude/rules/crazyflie/naming-and-structure.md). Call
logging.basicConfig() first, then this function, both from main().

Example:
    >>> def main() -> None:
    ...     logging.basicConfig(level=logging.ERROR)
    ...     configure_run_logging(__name__, Path(__file__).parent / "logs" / "my_script.log")
"""

import logging
import threading
from pathlib import Path

_CFLIB_LOGGER_NAME: str = "cflib"
_PACKAGE_LOGGER_NAME: str = "Crazyflie"
_FILE_LOG_FORMAT: str = "%(asctime)s %(levelname)s %(name)s: %(message)s"

# Tracks the FileHandler added by the most recent configure_run_logging()
# call in this process, so a repeat call can close and detach it instead of
# accumulating handlers. Module-level by design: there is exactly one
# "current run's log file" per process.
_active_file_handler: logging.Handler | None = None


def _log_thread_exception(args: threading.ExceptHookArgs) -> None:
    """threading.excepthook replacement: route uncaught background-thread
    exceptions through logging instead of only printing to stderr.

    Python's default threading.excepthook prints the traceback to stderr
    and nothing else - a background thread (CollisionMonitor's avoidance
    move, StabilizerMonitor's/LinkMonitor's poll loop, ...) that raises
    would otherwise leave no trace in the run's own log file, even though
    every other error path in this project logs through the same file.

    Args:
        args: threading.ExceptHookArgs (exc_type, exc_value, exc_traceback,
            thread) supplied by the threading module when a thread's target
            raises without catching it.
    """
    thread_name = args.thread.name if args.thread is not None else "unknown"
    logger = logging.getLogger(_PACKAGE_LOGGER_NAME)
    if args.exc_type is not None and args.exc_value is not None:
        logger.critical(
            "Unhandled exception in thread %r",
            thread_name,
            exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
        )
    else:
        # threading.ExceptHookArgs technically allows exc_type/exc_value to
        # be None; a real uncaught-exception callback always provides both,
        # but log something rather than silently dropping the report if
        # that ever isn't true.
        logger.critical(
            "Unhandled exception in thread %r (no exception info available)", thread_name
        )


def configure_run_logging(
    script_logger_name: str,
    log_file: Path,
    console_level: int = logging.INFO,
    file_level: int = logging.INFO,
) -> None:
    """Configure console + file logging for a script's main().

    Must be called after logging.basicConfig() (this function does not call
    it) - console_level is applied to the console handler basicConfig()
    adds, assumed to be the first handler on the root logger.

    Console output is capped at console_level (default INFO). A full trace
    at file_level and above (default INFO) is additionally written to
    log_file, so a run's full detail is always on disk even if console_level
    is raised to quiet the console down. cflib's own debug noise is always
    suppressed to CRITICAL regardless of the levels above.

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
            logging.INFO.
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
    file_handler = logging.FileHandler(log_file, mode="w", encoding="utf-8")
    file_handler.setLevel(file_level)
    file_handler.setFormatter(logging.Formatter(_FILE_LOG_FORMAT))
    root_logger.addHandler(file_handler)
    _active_file_handler = file_handler

    logging.getLogger(_CFLIB_LOGGER_NAME).setLevel(logging.CRITICAL)
    logging.getLogger(script_logger_name).setLevel(file_level)
    logging.getLogger(_PACKAGE_LOGGER_NAME).setLevel(file_level)

    threading.excepthook = _log_thread_exception
