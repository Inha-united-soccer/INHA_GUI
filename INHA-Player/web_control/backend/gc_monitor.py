import socket
import struct
import threading
import time

# Constants
GAMECONTROLLER_DATA_PORT = 3838
HL_GAMECONTROLLER_STRUCT_VERSION = 12
HL_MAX_NUM_PLAYERS = 11
SPL_COACH_MESSAGE_SIZE = 253

# Structure mapping
# HlRobotInfo (6 bytes)
# I I I I I I (all uint8)
FMT_HL_ROBOT_INFO = "6B"
SIZE_HL_ROBOT_INFO = struct.calcsize(FMT_HL_ROBOT_INFO)

# HlTeamInfo
# B B B B H B 253s 6B 11*6B
FMT_HL_TEAM_INFO = f"<4BHB{SPL_COACH_MESSAGE_SIZE}s{SIZE_HL_ROBOT_INFO}s{HL_MAX_NUM_PLAYERS * SIZE_HL_ROBOT_INFO}s" 
# Note: Struct format for nested structs is tricky. Better to parse incrementally.

class GCMonitor:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.5)
        try:
            self.sock.bind(('', GAMECONTROLLER_DATA_PORT))
            print(f"[GC] Listening on port {GAMECONTROLLER_DATA_PORT}")
        except Exception as e:
            print(f"[GC] Bind error: {e}")
            self.sock = None

        self.running = True
        self.data = {
            "state": "UNKNOWN",
            "secsRemaining": 0,
            "teams": [
                {"score": 0, "penalty": 0},
                {"score": 0, "penalty": 0}
            ],
            "secondaryState": "NONE",
            "secondaryTime": 0
        }

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        if not self.sock: return
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                if len(data) >= 4:
                    header = data[0:4]
                    if header == b'RGme':
                        self.parse_hl_packet(data)
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[GC] Error: {e}")
                time.sleep(1)

    def parse_hl_packet(self, data):
        try:
            # Struct Layout based on RoboCupGameControlData.h
            
            # 1. Header & Game State (18 bytes)
            # char header[4]
            # uint8 version
            # uint8 packetNumber
            # uint8 playersPerTeam
            # uint8 competitionPhase
            # uint8 competitionType
            # uint8 gamePhase
            # uint8 state
            # uint8 setPlay
            # uint8 firstHalf
            # uint8 kickingTeam
            # int16 secsRemaining
            # int16 secondaryTime
            
            # Format: <4s 10B 2h = 4 + 10 + 4 = 18 bytes
            if len(data) < 18:
                return

            (header, version, packet_num, players_per_team, comp_phase, comp_type, game_phase, state, set_play, first_half, kicking_team, secs_remaining, secondary_time) = \
                struct.unpack("<4sBBBBBBBBBBhh", data[0:18])
            
            if header != b'RGme':
                return

            # Map States
            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(state, "UNKNOWN")

            # Update Game Info
            self.data["state"] = state_str
            self.data["secsRemaining"] = secs_remaining
            self.data["secondaryState"] = f"Phase {game_phase}" # Simplified
            self.data["secondaryTime"] = secondary_time
            
            # 2. Teams (2 teams)
            # Offset starts at 18
            offset = 18
            parsed_teams = []
            
            MAX_NUM_PLAYERS = 20 # Defined in header
            
            for i in range(2):
                # TeamInfo Struct (10 bytes header + Players)
                # uint8 teamNumber
                # uint8 fieldPlayerColour
                # uint8 goalkeeperColour
                # uint8 goalkeeper
                # uint8 score
                # uint8 penaltyShot
                # uint16 singleShots
                # uint16 messageBudget
                # Format: <6B 2H = 6 + 4 = 10 bytes
                
                if len(data) < offset + 10:
                    break
                    
                (team_num, field_color, gk_color, gk_num, score, penalty_shot, single_shots, msg_budget) = \
                    struct.unpack("<6B2H", data[offset : offset+10])
                
                offset += 10
                
                # Players (MAX_NUM_PLAYERS * 2 bytes)
                # struct RobotInfo { uint8 penalty; uint8 secsTillUnpenalised; }
                penalty_count = 0
                
                players_size = MAX_NUM_PLAYERS * 2
                if len(data) < offset + players_size:
                    break
                    
                # Parse players to count penalties
                for p in range(MAX_NUM_PLAYERS):
                    p_penalty, p_secs = struct.unpack("BB", data[offset : offset+2])
                    if p_penalty != 0: # PENALTY_NONE = 0
                        penalty_count += 1
                    offset += 2
                
                parsed_teams.append({
                    "teamNumber": team_num,
                    "color": field_color,
                    "score": score,
                    "penaltyCount": penalty_count
                })

            self.data["teams"] = parsed_teams
            
        except Exception as e:
            print(f"[GC] Parse error: {e}")

    def get_status(self):
        return self.data

gc_monitor = GCMonitor()
