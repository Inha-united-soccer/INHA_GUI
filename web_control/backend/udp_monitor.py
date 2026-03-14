import socket
import struct
import threading
import time
import json
import math

# # 팀 통신 패킷 가져오는 파일 (240 bytes)
# ------------------------------------------------
# 로봇(C++)과 GUI(Python)언어가 다른데 데이터를 주고받아야함
# 로봇은 데이터를 바이너리로 보내는데 Python은 바이트들을 다시 풀어내야(Unpacking) 함
# 이때 로봇이 데이터를 어떻게 채우는지 규칙알고 맞게 풀어야함
# 1. 패딩과 정렬
#    컴퓨터(특히 C++)는 8바이트 데이터(double)를 읽을 때 주소값이 8의 배수여야 빨리 읽음
#    만약 앞의 데이터가 1바이트(bool)라면, 뒤의 double을 위해 강제로 7바이트를 비워둠
#    그래서 아래 레이아웃 중간에 `4x`, `7x` 같은 '패딩'이 들어가는 것이며, 이를 맞춰주지 않으면
#    데이터가 한 칸씩 밀려서 x좌표가 y좌표에 들어가는 등의 문제가 생김
# 2. struct.unpack: 
#    `FMT` 변수는 이 레이아웃의 '설계도'입니다. 이 설계도를 바탕으로 240바이트 덩어리를 
#    정해진 위치에서 잘라내어 Python 변수로 변환합니다.

# [C++ TeamCommunicationMsg 구조체 레이아웃]
# int validation (31202)      // 0-3: 패킷 유효성 검사 키
# int communicationId        // 4-7: 통신 시퀀스 ID
# int teamId                 // 8-11: 팀 번호
# int playerId               // 12-15: 로봇(선수) 번호
# int playerRole             // 16-19: 로봇 역할
# int secsRemaining          // 20-23: GameController 남은 경기 시간
# bool isAlive              // 24: 로봇 생존 여부
# bool isLead               // 25: 공 점유(공격 주도) 여부
# bool ballDetected         // 26: 공 감지 여부
# bool ballLocationKnown    // 27: 공 위치 파악 여부
# -- PADDING (4 bytes) --    // 28-31: double 정렬을 위한 패딩 (4x)
# double ballConfidence      // 32-39: 공에 대한 확신도
# double ballRange           // 40-47: 공까지의 거리
# double cost                // 48-55: 행동 비용
# Point ballPosToField       // 56-79: 경기장 기준 공 위치 (x, y, z)
# Pose2D robotPoseToField    // 80-103: 경기장 기준 로봇 위치 (x, y, theta)
# double kickDir             // 104-111: 킥 방향
# double thetaRb             // 112-119: 로봇 기준 공 각도
# int cmdId                  // 120-123: 명령 시퀀스 ID
# int cmd                    // 124-127: 현재 로봇 명령 상태
# bool passSignal            // 128: 패스 신호 여부
# -- PADDING (7 bytes) --    // 129-135: 8바이트 정렬을 위한 패딩 (7x)
# double passTargetX         // 136-143: 패스 목표 X
# double passTargetY         // 144-151: 패스 목표 Y
# bool isRace               // 152: 레이스 모드 여부
# bool isPossession         // 153: 점유 모드 여부
# uint8 sharedRobotCount     // 154: 공유된 로봇(상대방) 수
# -- PADDING (1 byte) --     // 155: 배열 시작 정렬 패딩 (x)
# SharedRobotPacket[5]       // 156-235: 공유 로봇 데이터 (16바이트 * 5)
# -- PADDING (4 bytes) --    // 236-239: 최종 구조체 크기 정렬 (4x)
# 총 크기: 240 바이트
# ------------------------------------------------

# struct.unpack을 위한 포맷 문자열
# < : 리틀 엔디언
# 6i : 정수 6개 (val, commId, teamId, player, role, secsRemaining)
# 4? : 불리언 4개 (alive, lead, ballDet, ballLocKnown)
# 4x : 4바이트 패딩
# 3d : 더블 3개 (ballConf, ballRange, cost)
# 3d : 더블 3개 (ballPos X, Y, Z)
# 3d : 더블 3개 (robotPos X, Y, T)
# 2d : 더블 2개 (kickDir, thetaRb)
# 2i : 정수 2개 (cmdId, cmd)
# ? : 불리언 1개 (passSignal)
# 7x : 7바이트 패딩
# 2d : 더블 2개 (passTargetX, Y)
# ??B : 불리언 2개(race, possession), 부호없는 정수 1개(sharedCount)
# x : 1바이트 패딩
# 5 * (3fB3x) : SharedRobotPacket 5개 (float 3개, uint8 1개, 패딩 3바이트로 16바이트 정렬)
# 4x : 4바이트 최종 패딩
FMT = "<6i4?4x3d3d3d2d2i?7x2d??Bx" + "3fB3x" * 5 + "4x"
EXPECTED_SIZE = struct.calcsize(FMT)

class UDPMonitor:
    def __init__(self, team_id=13):
        self.team_id = team_id
        
        # 팀 통신 수신 포트: 30000 + 팀 번호
        self.port = 30000 + self.team_id
        
        # 수신용 UDP 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(('0.0.0.0', self.port))
            print(f"[UDP] 포트 {self.port}에서 수신 대기 중...")
        except Exception as e:
            print(f"[UDP] 바인딩 에러: {e}")
            self.sock = None

        self.robots = {} # 수신된 로봇 데이터 저장: { "robot_ID": { ... } }
        self.running = True
        
        # 패킷 속도(PPS) 측정용 상태값
        self.packet_stats = {} 
        self.packet_rates = {}

        # 3D GameController 연동용 송신 소켓 (표준 포트 3939)
        self.bridge_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.bridge_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # 1. 패킷 수신 루프 스레드 시작
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        
        # 2. 로봇에게 GUI 존재를 알리는 Discovery(생존 신고) 브로드캐스트 시작
        self.discovery_thread = threading.Thread(target=self.broadcast_discovery, daemon=True)
        self.discovery_thread.start()

    def loop(self):
        """UDP 패킷을 지속적으로 수신하는 루프"""
        if not self.sock: return
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                
                # 수신된 데이터가 예상 크기(240바이트)와 맞는지 확인
                if len(data) >= EXPECTED_SIZE:
                    self.parse_packet(data[:EXPECTED_SIZE], addr)
            except Exception as e:
                print(f"[UDP] 에러: {e}")
                time.sleep(1)

    def broadcast_discovery(self):
        """로봇들이 GUI 쪽으로 패킷을 보내도록 99번 멤버로 위장하여 생존 신고"""
        dest_port = 20000 + self.team_id
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # TeamDiscoveryMsg 구조 (16 바이트)
        validation = 41203
        comm_id = 0
        player_id = 99 # Mac/GUI를 99번 선수로 표시
        
        print(f"[UDP] 포트 {dest_port}로 Discovery 브로드캐스트 중...")
        
        while self.running:
            try:
                msg = struct.pack("<4i", validation, comm_id, self.team_id, player_id)
                sock.sendto(msg, ('<broadcast>', dest_port))
                comm_id += 1
                time.sleep(1.0) # 1초마다 전송
            except Exception as e:
                print(f"[UDP] Discovery 에러: {e}")
                time.sleep(1.0)

    def parse_packet(self, data, addr):
        """바이너리 패킷을 파싱하여 딕셔너리로 저장"""
        try:
            unpacked = struct.unpack(FMT, data)

            # 필드 추출 (struct.unpack 결과는 튜플)
            team_id = unpacked[2]
            player_id = unpacked[3]
            role_int = unpacked[4]
            secs_remaining = unpacked[5]
            
            is_alive = unpacked[6]
            is_lead = unpacked[7]
            ball_detected = unpacked[8]
            
            # 주요 표시 데이터
            ball_conf = unpacked[10]
            ball_range = unpacked[11]
            bx = unpacked[13] # 공 위치 x
            by = unpacked[14] # 공 위치 y
            rx = unpacked[16] # 로봇 위치 x
            ry = unpacked[17] # 로봇 위치 y
            rtheta = unpacked[18] # 로봇 각도
            cmd = unpacked[22] # 현재 수행 중인 로봇 내부 명령 코드
            
            pass_target_x = unpacked[24]
            pass_target_y = unpacked[25]
            is_race = unpacked[26]
            is_possession = unpacked[27]
            shared_count = unpacked[28] # 인식된 다른 로봇 수

            robot_id = f"robot_{player_id}"
            
            # 역할(Role) 번호를 문자열로 매핑
            role_map = {0: "Unknown", 1: "Striker", 2: "Defender", 3: "Goalkeeper", 4: "Support"}
            role_str = role_map.get(role_int, f"Role {role_int}")

            # 로봇 상태 텍스트 결정 (프론트엔드 표시용)
            if not is_alive:
                state_desc = "Fallen"
            elif ball_detected:
                state_desc = f"Ball Found ({ball_range:.2f}m)"
            else:
                state_desc = "Searching"

            # 데이터 수신 시각 및 PPS 갱신
            now = time.time()
            self._update_pps(robot_id, now)

            # 로봇 정보 최종 저장
            self.robots[robot_id] = {
                "id": robot_id,
                "role": role_str,
                "role_id": role_int,
                "state": state_desc,
                "is_alive": is_alive,
                "ball_detected": ball_detected,
                "cmd": cmd,
                "x": rx,
                "y": ry,
                "theta": rtheta,
                "ball_x": bx,
                "ball_y": by,
                "ball_confidence": ball_conf,
                "secs_remaining": secs_remaining,
                "last_seen": now,
                "ip": addr[0],
                "packet_size": len(data),
                "pps": self.packet_rates.get(robot_id, 0.0),
                "is_race": is_race,
                "is_possession": is_possession,
                "pass_target": {"x": pass_target_x, "y": pass_target_y},
                "shared_robots": self._parse_shared_robots(unpacked[29:], shared_count)
            }
            
            # GameController 3D 뷰어(포트 3939)로 데이터 전달
            self._bridge_to_3d(player_id, team_id, is_alive, ball_detected, rx, ry, rtheta, bx, by)

        except Exception as e:
            print(f"[UDP] 파싱 에러: {e}")

    def _bridge_to_3d(self, p_id, t_id, alive, ball_det, rx, ry, rt, bx, by):
        """GameControlReturnData (32바이트)를 포트 3939로 브로드캐스트하여 3D 시뮬레이터 연동"""
        try:
            header_bytes = b"RGrt"
            version = 4
            fallen = 0 if alive else 1
            bAge = 0.0 if ball_det else -1.0
            
            # 국제 표준 GameControlReturnData 포맷
            bridge_fmt = "<4sBBBBffffff"
            bridge_msg = struct.pack(bridge_fmt,
                                     header_bytes, version, p_id, t_id,
                                     fallen,
                                     float(rx), float(ry), float(rt),
                                     float(bAge),
                                     float(bx), float(by))
                                     
            self.bridge_sock.sendto(bridge_msg, ('127.0.0.1', 3939))
        except Exception:
            pass

    def _update_pps(self, robot_id, now):
        """각 로봇별 초당 수신 패킷 수(PPS) 계산"""
        if robot_id not in self.packet_stats:
            self.packet_stats[robot_id] = []
        
        self.packet_stats[robot_id].append(now)
        # 최근 1초 이내의 패킷 시간만 유지
        self.packet_stats[robot_id] = [t for t in self.packet_stats[robot_id] if now - t <= 1.0]
        self.packet_rates[robot_id] = len(self.packet_stats[robot_id])

    def _parse_shared_robots(self, unpacked_data, count):
        """인인된 다른 로봇(상대팀 등) 리스트 파싱"""
        shared = []
        # SharedRobotPacket은 개당 16바이트 (float 3개 + uint8 1개 + 패딩)
        # unpacked_data의 29번 인덱스부터 4개씩 묶어서 처리
        for i in range(min(count, 5)):
            base = i * 4
            if base + 3 < len(unpacked_data):
                shared.append({
                    "x": unpacked_data[base],
                    "y": unpacked_data[base + 1],
                    "confidence": unpacked_data[base + 2],
                    "label": "Opponent" if unpacked_data[base + 3] == 1 else "Unknown"
                })
        return shared

    def get_status(self):
        """현재 관리 중인 모든 로봇의 상태 반환 (3초 이상 소식 없으면 삭제)"""
        now = time.time()
        expired = [rid for rid, r in self.robots.items() if now - r['last_seen'] > 3.0]
        for rid in expired:
            del self.robots[rid]
            if rid in self.packet_stats:
                del self.packet_stats[rid]
                del self.packet_rates[rid]
        return self.robots

# 기본적으로 팀 13 모니터링 시작
udp_monitor = UDPMonitor(team_id=13)
