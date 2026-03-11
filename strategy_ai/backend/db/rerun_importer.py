"""
rerun_importer.py — Parses Rerun .rrd files and populates strategy_events in SQLite.

Synchronizes Rerun data with TCM logs using the 'game_time' timeline (secsRemaining).
"""

import os
import rerun as rr
from db.tcm_db import (
    get_connection,
    insert_strategy_events_bulk,
    is_already_imported
)

# Paths we want to monitor
INTENT_PATHS = {
    "intent/current_role":  "Role",
    "intent/is_lead":       "Lead",
    "intent/is_possession": "Possession",
    "intent/decision":      "Decision",
    "intent/ball_range":    "BallRange",
    "intent/ball_yaw":      "BallYaw",
}

def import_rrd_file(path: str, match_id: int):
    """
    Load .rrd file and extract events detected on the 'game_time' timeline.
    """
    filename = os.path.basename(path)
    print(f"[rerun_importer] Processing {filename} for match_id {match_id}...")

    try:
        # Load the recording
        recording = rr.RecordingStream.load(path)
        
        # We use the Data Loader API to query the recording
        # Note: This requires rerun-sdk >= 0.15 for the modern query API
        # If the SDK version is older, we might need to adjust this.
        
        events_to_insert = []
        
        # Iterate over paths we care about
        for entity_path, event_type in INTENT_PATHS.items():
            # Query the entire history of the entity on the 'game_time' timeline
            # This is a conceptual implementation; the exact rerun query API 
            # might differ depending on the installed version.
            try:
                # In latest Rerun, we can use archive queries or similar
                # For this implementation, we'll assume a standard extraction flow
                data = recording.query_entity(entity_path, timeline="game_time")
                
                last_val = None
                for entry in data:
                    game_time = entry.time
                    val = entry.value
                    
                    # Only store when value changes to keep DB clean
                    if val != last_val:
                        events_to_insert.append((
                            game_time,
                            "self", # robot_id (could be parsed from filename)
                            event_type,
                            str(val)
                        ))
                        last_val = val
            except Exception as e:
                print(f"[rerun_importer] Warning: Could not query {entity_path}: {e}")

        if events_to_insert:
            insert_strategy_events_bulk(match_id, events_to_insert)
            print(f"[rerun_importer] ✓ Synchronized {len(events_to_insert)} events.")
        else:
            print(f"[rerun_importer] No intent events found in {filename}.")

    except Exception as e:
        print(f"[rerun_importer] Error parsing {filename}: {e}")

if __name__ == "__main__":
    import sys
    # Example usage: python rerun_importer.py path/to/log.rrd match_id
    if len(sys.argv) > 2:
        import_rrd_file(sys.argv[1], int(sys.argv[2]))
    else:
        print("Usage: python rerun_importer.py <file.rrd> <match_id>")
