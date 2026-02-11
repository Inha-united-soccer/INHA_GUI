import socket
import struct
import threading
import time

# Constants
GAMECONTROLLER_DATA_PORT = 3838
# RoboCupGameControlData.h defines version 15
GAMECONTROLLER_STRUCT_VERSION = 15
MAX_NUM_PLAYERS = 20 # RoboCupGameControlData.h line 13

# -----------------------------------------------------------------------------------------
# [GameController 모니터]
# RoboCup 공식 GameController(v15)에서 보내는 UDP 패킷을 수신하고 파싱하는 클래스
# -----------------------------------------------------------------------------------------
class GCMonitor:
    def __init__(self):
        # UDP 소켓 생성
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
        
        # 파싱된 데이터 저장소
        self.data = {
            "state": "UNKNOWN",
            "secsRemaining": 0,
            "teams": [
                {"score": 0, "penaltyCount": 0},
                {"score": 0, "penaltyCount": 0}
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
                
                # 헤더 체크 'RGme'
                if len(data) >= 4 and data[0:4] == b'RGme':
                     self.parse_packet(data)

            except socket.timeout:
                pass 
            except Exception as e:
                print(f"[GC] Error: {e}")
                time.sleep(1)

    def parse_packet(self, data):
        try:
            # RoboCupGameControlData (v15) Structure Layout:
            # -------------------------------------------------
            # header(4s), version(B), packetNumber(B), playersPerTeam(B)
            # competitionPhase(B), competitionType(B), gamePhase(B)
            # state(B), setPlay(B), firstHalf(B), kickingTeam(B)
            # secsRemaining(h), secondaryTime(h)
            # -------------------------------------------------
            # Total Header Size: 4 + 1*10 + 2*2 = 18 bytes
            
            if len(data) < 18: return

            (header, version, packet_num, players_per_team, 
             comp_phase, comp_type, game_phase, 
             state, set_play, first_half, kicking_team, 
             secs_remaining, secondary_time) = struct.unpack("<4sBBBBBBBBBBhh", data[0:18])

            # State Mapping
            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(state, "UNKNOWN")

            # Set Play Mapping (Previous 'secondaryState' concept is split)
            SET_PLAY_MAP = {0: "NONE", 1: "GOAL_KICK", 2: "PUSHING_FREE_KICK", 
                            3: "CORNER_KICK", 4: "KICK_IN", 5: "PENALTY_KICK"}
            set_play_str = SET_PLAY_MAP.get(set_play, "NONE")

            self.data["state"] = state_str
            self.data["secsRemaining"] = secs_remaining
            self.data["secondaryState"] = set_play_str
            self.data["secondaryTime"] = secondary_time
            
            # Team Info Parsing
            # Offset 18 starts TeamInfo[2]
            offset = 18
            parsed_teams = []

            # TeamInfo Structure (v15):
            # teamNumber(B), fieldPlayerColour(B), goalkeeperColour(B), goalkeeper(B)
            # score(B), penaltyShot(B), singleShots(H), messageBudget(H)
            # players(RobotInfo[MAX_NUM_PLAYERS]) 
            #   -> RobotInfo: penalty(B), secsTillUnpenalised(B)
            
            # Header of TeamInfo = 1*6 + 2*2 = 10 bytes
            # Body of TeamInfo = MAX_NUM_PLAYERS(20) * 2 bytes = 40 bytes
            # Total TeamInfo Size = 50 bytes
            
            TEAM_INFO_SIZE = 10 + (MAX_NUM_PLAYERS * 2)

            for i in range(2):
                if len(data) < offset + TEAM_INFO_SIZE: break
                
                # Parse Team Header
                (team_num, field_color, gk_color, gk_num, score, penalty_shot, 
                 single_shots, msg_budget) = struct.unpack("<6B2H", data[offset : offset+10])

                # Parse Players
                players_offset = offset + 10
                penalty_count = 0
                
                for p in range(MAX_NUM_PLAYERS):
                    p_base = players_offset + (p * 2)
                    p_penalty, p_secs = struct.unpack("BB", data[p_base : p_base+2])
                    if p_penalty != 0: # 0 = PENALTY_NONE
                        penalty_count += 1
                
                parsed_teams.append({
                    "teamNumber": team_num,
                    "color": field_color,
                    "score": score,
                    "penaltyCount": penalty_count,
                    "messageBudget": msg_budget
                })

                offset += TEAM_INFO_SIZE

            self.data["teams"] = parsed_teams

        except Exception as e:
            print(f"[GC] Parse error: {e}")

    def get_status(self):
        return self.data

gc_monitor = GCMonitor()
