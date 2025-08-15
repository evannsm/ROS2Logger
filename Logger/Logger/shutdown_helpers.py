# logger/shutdown_hooks.py
import atexit, signal, threading
import rclpy

def install_shutdown_logging(logger, source, *, also_shutdown=None, include_atexit=True):
    """
    Ensure logger.log(source) runs once on SIGINT/SIGTERM (and optionally on normal exit).
    Idempotent across multiple signals and atexit. Also calls also_shutdown() safely.
    """
    _lock = threading.Lock()
    _done = False

    def _safe_shutdown():
        try:
            # Only shutdown if a context is alive
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"[logger] shutdown hook error: {e}")

    def _run_once(_sig=None, _frame=None):
        nonlocal _done
        with _lock:
            if _done:
                return
            _done = True
        try:
            logger.log(source)
        except Exception as e:
            print(f"[logger] error during shutdown log: {e}")
        finally:
            if callable(also_shutdown):
                try:
                    also_shutdown()
                except Exception as e:
                    print(f"[logger] shutdown hook error: {e}")
            else:
                _safe_shutdown()

    # Register signal handlers
    signal.signal(signal.SIGINT,  _run_once)
    signal.signal(signal.SIGTERM, _run_once)

    # Optional atexit (can cause duplicates without the idempotent guard)
    if include_atexit:
        atexit.register(_run_once)
