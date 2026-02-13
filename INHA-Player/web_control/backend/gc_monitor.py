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
        print(f"[GC] Received {len(data)} bytes") # Verbose debug
        try:
            # Check Header 'RGme'
            if len(data) < 4: 
                # print("[GC] Data too short")
                return
            if data[0:4] != b'RGme': 
                # print(f"[GC] Invalid header: {data[0:4]}")
                return

            # Read Version (Offset 4, 2 bytes for HL / 1 byte for SPL?)
            # HL uses uint16_t version, SPL uses uint8_t version.
            if len(data) < 6: return
            
            # Try to unpack version as uint16 (Little Endian)
            version = struct.unpack("<H", data[4:6])[0]
            
            # print(f"[GC] Version match check: {version} (HL=12)")

            # If version is 12, it is Humanoid League
            if version == 12:
                self.parse_hl_packet(data)
            # If version is 15 (SPL), likely byte 4 is 15. 
            elif data[4] == 15: 
                self.parse_spl_packet(data)
            else:
                # print(f"[GC] Unknown version: {version} or {data[4]}")
                pass

        except Exception as e:
            print(f"[GC] Parse error: {e}")

    def parse_spl_packet(self, data):
        # Existing SPL v15 parsing logic...
        try:
            if len(data) < 18: return
            (header, version, packet_num, players_per_team, 
             comp_phase, comp_type, game_phase, 
             state, set_play, first_half, kicking_team, 
             secs_remaining, secondary_time) = struct.unpack("<4sBBBBBBBBBBhh", data[0:18])

            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(state, "UNKNOWN")
            
            SET_PLAY_MAP = {0: "NONE", 1: "GOAL_KICK", 2: "PUSHING_FREE_KICK", 
                            3: "CORNER_KICK", 4: "KICK_IN", 5: "PENALTY_KICK"}
            set_play_str = SET_PLAY_MAP.get(set_play, "NONE")

            self.data["state"] = state_str
            self.data["secsRemaining"] = secs_remaining
            self.data["secondaryState"] = set_play_str
            self.data["secondaryTime"] = secondary_time
            self.data["gameType"] = "SPL"
            
            # Teams (SPL)
            offset = 18
            TEAM_INFO_SIZE = 10 + (20 * 2) # 50
            parsed_teams = []
            
            for i in range(2):
                if len(data) < offset + TEAM_INFO_SIZE: break
                (team_num, field_color, gk_color, gk_num, score, penalty_shot, 
                 single_shots, msg_budget) = struct.unpack("<6B2H", data[offset : offset+10])
                
                players_offset = offset + 10
                penalty_count = 0
                players_info = []

                for p in range(20):
                    p_base = players_offset + (p * 2)
                    p_penalty, p_secs = struct.unpack("BB", data[p_base : p_base+2])
                    if p_penalty != 0: penalty_count += 1
                    players_info.append({"penalty": p_penalty, "secs_till_unpenalised": p_secs})

                parsed_teams.append({
                    "teamNumber": team_num,
                    "color": field_color,
                    "score": score,
                    "penaltyCount": penalty_count,
                    "messageBudget": msg_budget,
                    "players": players_info
                })
                offset += TEAM_INFO_SIZE
            
            self.data["teams"] = parsed_teams
            
        except Exception as e:
            print(f"[GC] SPL Parse error: {e}")

    def parse_hl_packet(self, data):
        try:
            # HlRoboCupGameControlData (v12) Structure
            # header(4), version(2), packetNumber(1), playersPerTeam(1)
            # gameType(1), state(1), firstHalf(1), kickOffTeam(1)
            # secondaryState(1), secondaryStateInfo(4), dropInTeam(1)
            # dropInTime(2), secsRemaining(2), secondaryTime(2)
            # Total Header: 25 bytes
            
            if len(data) < 25: return
            
            (header, version, packet_num, players_per_team,
             game_type, state, first_half, kick_off_team,
             sec_state, sec_state_info_bytes, drop_in_team,
             drop_in_time, secs_remaining, secondary_time) = struct.unpack("<4sHBBBBBBB4sBHHH", data[0:24])

            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(state, "UNKNOWN")
            
            # HL Secondary States (RoboCupGameControlData.h line 48)
            SEC_STATE_MAP = {
                0: "NONE", 1: "PENALTY_SHOOT", 2: "OVERTIME", 3: "TIMEOUT",
                4: "DIRECT_FREEKICK", 5: "INDIRECT_FREEKICK", 6: "PENALTY_KICK",
                7: "CORNER_KICK", 8: "GOAL_KICK", 9: "THROW_IN"
            }
            sec_state_str = SEC_STATE_MAP.get(sec_state, "NONE")

            self.data["state"] = state_str
            self.data["secsRemaining"] = secs_remaining
            self.data["secondaryState"] = sec_state_str
            self.data["secondaryTime"] = secondary_time
            self.data["dropInTime"] = drop_in_time
            self.data["dropInTeam"] = drop_in_team
            self.data["gameType"] = "HL"

            # Teams (HL)
            # Offset 24 starts TeamInfo[2]
            # HlTeamInfo Size:
            # teamNumber(1), colour(1), score(1), penaltyShot(1), singleShots(2), coachSeq(1)
            # coachMessage(253) -> SPL_COACH_MESSAGE_SIZE
            # coach(6) -> HlRobotInfo
            # players(11 * 6) -> HlRobotInfo * 11
            # Total: 7 + 253 + 6 + 66 = 332 bytes
            
            offset = 24
            TEAM_INFO_SIZE = 332
            parsed_teams = []
            
            for i in range(2):
                if len(data) < offset + TEAM_INFO_SIZE: break
                
                # Header: 7 bytes
                # (team_num, color, score, penalty_shot, 
                #  single_shots, coach_seq) = struct.unpack("<BBBcHxB", data[offset : offset+7]) 
                
                # unpack "<BBBBHB" 
                try:
                    # print(f"[GC] Unpacking Team Header at {offset}")
                    (team_num, color, score, penalty_shot, single_shots, coach_seq) = struct.unpack("<BBBBHB", data[offset:offset+7])
                except Exception as e:
                    print(f"[GC] Team Header Unpack Failed: {e}. Offset: {offset}, slice len: {len(data[offset:offset+7])}")
                    raise e

                # Coach Message: 253 bytes (Offset + 7)
                coach_msg_bytes = data[offset+7 : offset+7+253]
                try:
                    coach_msg = coach_msg_bytes.decode('utf-8').rstrip('\x00')
                except:
                    coach_msg = ""
                
                # Players: Offset + 7 + 253 + 6 (Coach Info ignored for now)
                # Players start at Offset + 266
                players_offset = offset + 266
                penalty_count = 0
                players_info = []

                # HL_MAX_NUM_PLAYERS = 11
                for p in range(11):
                    p_base = players_offset + (p * 6)
                    # HlRobotInfo: penalty(1), secsTillUnpenalised(1), warnings(1), yellow(1), red(1), goalie(1)
                    try:
                        (p_penalty, p_secs, p_warn, p_yellow, p_red, p_goalie) = struct.unpack("BBBBBB", data[p_base : p_base+6])
                    except Exception as e:
                        print(f"[GC] Player Unpack Failed: {e}. p_base: {p_base}, slice len: {len(data[p_base : p_base+6])}")
                        raise e
                    
                    if p_penalty != 0: penalty_count += 1
                    players_info.append({
                        "penalty": p_penalty,
                        "secs_till_unpenalised": p_secs,
                        "yellow_cards": p_yellow,
                        "red_cards": p_red,
                        "is_goalie": (p_goalie > 0)
                    })

                parsed_teams.append({
                    "teamNumber": team_num,
                    "color": color,
                    "score": score,
                    "penaltyCount": penalty_count,
                    "coachMessage": coach_msg,
                    "players": players_info
                })
                
                offset += TEAM_INFO_SIZE

            self.data["teams"] = parsed_teams

        except Exception as e:
            print(f"[GC] HL Parse error: {e}")

    def get_status(self):
        return self.data

gc_monitor = GCMonitor()
