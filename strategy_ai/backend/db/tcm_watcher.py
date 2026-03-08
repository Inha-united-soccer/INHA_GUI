"""
tcm_watcher.py — Watchdog that monitors logs_teamcomm/ for new .log files
and automatically imports them into the SQLite DB.

Usage (standalone):
    python3 -m db.tcm_watcher

Or start programmatically:
    from db.tcm_watcher import start_watcher
    start_watcher(log_dir)
"""

import os
import time
import threading
import logging

logger = logging.getLogger(__name__)

LOG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__),
                 "../../../3D modeling/bin/logs_teamcomm")
)


class _LogHandler:
    """Checks for newly created/modified .log files by polling."""

    def __init__(self, log_dir: str):
        self.log_dir = log_dir
        self._seen: set = set()

    def _scan(self):
        """Scan directory for new .log files and import them."""
        # Import here to avoid circular issues at module load time
        from db.tcm_importer import import_log_file

        try:
            files = [
                f for f in os.listdir(self.log_dir)
                if f.endswith(".log")
            ]
        except FileNotFoundError:
            return

        for fname in files:
            if fname in self._seen:
                continue
            fpath = os.path.join(self.log_dir, fname)
            try:
                fsize = os.path.getsize(fpath)
            except OSError:
                continue

            # Only process once the file looks complete (>1 KB)
            if fsize < 1024:
                continue

            result = import_log_file(fpath, force=False)
            if result in ("imported", "skipped", "empty"):
                self._seen.add(fname)
                if result == "imported":
                    logger.info(f"[watcher] Auto-imported: {fname}")

    def run_forever(self, interval: float = 5.0):
        logger.info(f"[watcher] Monitoring: {self.log_dir}  (interval={interval}s)")
        while True:
            self._scan()
            time.sleep(interval)


_watcher_thread: threading.Thread | None = None


def start_watcher(log_dir: str = LOG_DIR, interval: float = 5.0):
    """
    Start the background watcher thread (idempotent).
    Safe to call multiple times — only one thread is ever started.
    """
    global _watcher_thread
    if _watcher_thread and _watcher_thread.is_alive():
        return

    handler = _LogHandler(log_dir)
    _watcher_thread = threading.Thread(
        target=handler.run_forever,
        kwargs={"interval": interval},
        daemon=True,
        name="TCMWatcher",
    )
    _watcher_thread.start()
    logger.info("[watcher] Started background watcher thread.")


# ── CLI entry ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(message)s",
        datefmt="%H:%M:%S",
    )
    d = sys.argv[1] if len(sys.argv) > 1 else LOG_DIR
    handler = _LogHandler(d)
    handler.run_forever(interval=5.0)
