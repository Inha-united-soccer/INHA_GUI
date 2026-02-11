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


# -----------------------------------------------------------------------------------------
# [GameController 모니터]
# RoboCup 공식 GameController 프로그램에서 보내는 UDP 패킷을 수신하고 파싱하는 클래스
# 심판의 신호(경기 시작, 정지, 골 득점 등)를 GUI에 표시하기 위함
# -----------------------------------------------------------------------------------------
class GCMonitor:
    def __init__(self):
        # UDP 소켓 생성 (IPv4, Datagram)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 포트 재사용 옵션 (서버 재시작 시 바인딩 에러 방지)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.5) # 0.5초 동안 데이터가 없으면 타임아웃 예외 발생
        
        try:
            # GameController Data Standard Port: 3838
            self.sock.bind(('', GAMECONTROLLER_DATA_PORT))
            print(f"[GC] Listening on port {GAMECONTROLLER_DATA_PORT}")
        except Exception as e:
            print(f"[GC] Bind error: {e}")
            self.sock = None

        self.running = True
        
        # 파싱된 데이터를 저장할 딕셔너리 (초기값)
        self.data = {
            "state": "UNKNOWN",   # 경기 상태 (READY, PLAYING 등)
            "secsRemaining": 0,   # 남은 시간 (초)
            "teams": [            # 두 팀의 점수 정보
                {"score": 0, "penalty": 0}, # 현재 잘못받아오고 있는 부분
                {"score": 0, "penalty": 0}
            ],
            "secondaryState": "NONE", # 세부 상태 (페널티킥, 프리킥 등)
            "secondaryTime": 0
        }

        # 수신 루프를 별도 스레드에서 실행 (메인 프로그램 흐름 방해 안 함)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    # [수신 루프]
    # 계속해서 UDP 패킷을 기다리고, 들어오면 parse_hl_packet 함수로 넘김
    def loop(self):
        if not self.sock: return
        while self.running:
            try:
                # 4096 바이트만큼 버퍼를 읽음
                data, addr = self.sock.recvfrom(4096)
                
                # 최소 헤더 체크 (4바이트)
                if len(data) >= 4:
                    header = data[0:4]
                    # 'RGme' 헤더를 가진 패킷만 유효한 GameController 패킷임
                    if header == b'RGme':
                        self.parse_hl_packet(data)
            except socket.timeout:
                pass # 타임아웃은 정상이므로 무시
            except Exception as e:
                print(f"[GC] Error: {e}")
                time.sleep(1)

    # [패킷 파싱]
    # C++ 구조체(RoboCupGameControlData) 형태의 바이너리 데이터를 파이썬에서 읽을 수 있는 값으로 변환 (struct 모듈 사용)
    def parse_hl_packet(self, data):
        try:
            # 1. 헤더 및 경기 상태 정보 파싱 (앞부분 18바이트)
            # struct.unpack 포맷 문자열: "<4sBBBBBBBBBBhh" (리틀 엔디안)
            # 4s(헤더), B(uint8) 10개, h(int16) 2개
            if len(data) < 18: return
            
            (header, version, packet_num, players_per_team, comp_phase, comp_type, \
             game_phase, state, set_play, first_half, kicking_team, \
             secs_remaining, secondary_time) = \
                struct.unpack("<4sBBBBBBBBBBhh", data[0:18])
            
            if header != b'RGme': return

            # 숫자료 표현된 상태(state)를 사람이 읽기 쉬운 문자열로 변환
            STATE_MAP = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
            state_str = STATE_MAP.get(state, "UNKNOWN")

            # 파싱 데이터 업데이트
            self.data["state"] = state_str
            self.data["secsRemaining"] = secs_remaining
            self.data["secondaryState"] = f"Phase {game_phase}"
            self.data["secondaryTime"] = secondary_time
            
            # 2. 팀 정보 파싱 (두 팀)
            # 오프셋 18부터 팀 정보가 시작됨
            offset = 18
            parsed_teams = []
            
            MAX_NUM_PLAYERS = 11 # 헤더에 따라 다를 수 있으나 우선 11명분 데이터
            
            for i in range(2):
                # 각 팀의 헤더 정보 (10바이트)
                # 팀 번호, 색상, 점수, 페널티 샷 여부 등
                
                if len(data) < offset + 10: break
                    
                (team_num, field_color, gk_color, gk_num, score, penalty_shot, single_shots, msg_budget) = \
                    struct.unpack("<6B2H", data[offset : offset+10])
                
                offset += 10
                
                # 페널티 받은 선수 카운트 로직
                # 각 선수별 정보 (2바이트씩: 페널티 종류, 남은 시간)
                penalty_count = 0
                players_size = MAX_NUM_PLAYERS * 2 # 선수당 2바이트
                
                if len(data) < offset + players_size: break
                    
                # 선수들 데이터를 순회하며 페널티 상태 확인
                for p in range(MAX_NUM_PLAYERS):
                    p_penalty, p_secs = struct.unpack("BB", data[offset : offset+2])
                    if p_penalty != 0: # 0이 아니면 퇴장 혹은 페널티 상태
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

    # 파싱이 완료되어 최신화된 데이터 딕셔너리를 외부에 전달
    def get_status(self):
        return self.data

# 전역 인스턴스 생성 (app.py에서 import하여 사용)
gc_monitor = GCMonitor()
