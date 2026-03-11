import json
import os
import threading
import time
from datetime import datetime

LOGS_DIR = os.path.join(os.path.dirname(__file__), "logs")
LOG_FILE = os.path.join(LOGS_DIR, "gamelog.json")

class GameLogger:
    def __init__(self, gc_monitor, udp_monitor):
        self.gc_monitor = gc_monitor
        self.udp_monitor = udp_monitor
        self.running = False
        self.thread = None
        
        # Ensure log directory exists
        if not os.path.exists(LOGS_DIR):
            os.makedirs(LOGS_DIR)
            
        # Initialize an empty array if file does not exist
        if not os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'w') as f:
                json.dump([], f)

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._log_loop, daemon=True)
            self.thread.start()
            print("[GameLogger] Started logging to", LOG_FILE)

    def stop(self):
        self.running = False

    def _log_loop(self):
        # We record data every second
        while self.running:
            try:
                self._record_snapshot()
            except Exception as e:
                print(f"[GameLogger] Error: {e}")
            time.sleep(1.0)

    def _record_snapshot(self):
        # Fetch current statuses
        gc_status = self.gc_monitor.get_status()
        robot_status = self.udp_monitor.get_status()
        
        snapshot = {
            "timestamp": datetime.now().isoformat(),
            "gc_state": gc_status.get("state", "UNKNOWN"),
            "robots": robot_status
        }
        
        # Read existing logs, append, and rewrite
        # For a truly large system, appending per line (JSONL) is better, 
        # but for demonstration we'll read, append, and rewrite.
        # Alternatively, we can just append to a list in memory and dump on exit, but let's append to file.
        
        try:
            with open(LOG_FILE, 'r+') as f:
                # If file is empty, initialize it
                f.seek(0, os.SEEK_END)
                if f.tell() == 0:
                    f.write("[]")
                    f.seek(0)
                    
                f.seek(0)
                try:
                    logs = json.load(f)
                except json.JSONDecodeError:
                    logs = []
                
                logs.append(snapshot)
                
                # Keep only the last 600 records (10 minutes of logs) to avoid large files
                if len(logs) > 600:
                    logs = logs[-600:]
                    
                f.seek(0)
                f.truncate()
                json.dump(logs, f)
        except Exception as e:
            print(f"[GameLogger] File write error: {e}")

    def get_logs(self):
        if not os.path.exists(LOG_FILE):
            return []
        try:
            with open(LOG_FILE, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError:
            return []

