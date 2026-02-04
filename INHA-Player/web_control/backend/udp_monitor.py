
import socket
import struct
import threading
import time
import json
import math

# C++ TeamCommunicationMsg Structure Layout (Standard Alignment)
# int validation;        // 0-3
# int communicationId;   // 4-7
# int teamId;            // 8-11
# int playerId;          // 12-15
# int playerRole;        // 16-19
# bool isAlive;          // 20
# bool isLead;           // 21
# bool ballDetected;     // 22
# bool ballLocationKnown;// 23
# double ballConfidence; // 24-31 (8-byte aligned)
# double ballRange;      // 32-39
# double cost;           // 40-47
# Point ballPosToField;  // 48-71 (3 doubles)
# Pose2D robotPoseToField;// 72-95 (3 doubles)
# double kickDir;        // 96-103
# double thetaRb;        // 104-111
# int cmdId;             // 112-115
# int cmd;               // 116-119
# bool passSignal;       // 120
# -- PADDING --          // 121-127 (7 bytes) to align next double to 128
# double passTargetX;    // 128-135
# double passTargetY;    // 136-143
# Total Size: 144 bytes

# Format definition for struct.unpack
# < : Little Endian
# 5i: 5 integers
# 4?: 4 bools
# 3d: 3 doubles (ball info)
# 3d: 3 doubles (Point ballPos)
# 3d: 3 doubles (Pose2D robotPos)
# 2d: 2 doubles (kickDir, thetaRb)
# 2i: 2 integers (cmdId, cmd)
# ?: 1 bool (passSignal)
# 7x: 7 pad bytes
# 2d: 2 doubles (passTarget)
FMT = "<5i4?3d3d3d2d2i?7x2d" 
EXPECTED_SIZE = struct.calcsize(FMT) # Should be 144

class UDPMonitor:
    def __init__(self, team_id=1):
        self.team_id = team_id
        # Communication Broadcast Port: 20000 + team_id (Discovery) or 30000 + team_id (Unicast/Comm)?
        # team_communication_msg.h defines VALIDATION_COMMUNICATION = 31202
        # BrainCommunication.cpp uses _unicast_udp_port = 30000 + teamId for "Unicast"? 
        # Actually usually robots broadcast status to 30000+TeamID or 20000+TeamID.
        # Let's try to listen on the broadcast port. 
        # Based on BrainCommunication.cpp: _unicast_udp_port = 30000 + teamId.
        
        # UDP 소켓 생성 및 바인딩
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 포트 재사용 허용
        try:
            self.sock.bind(('0.0.0.0', self.port))
            print(f"[UDP] Listening on port {self.port}")
        except Exception as e:
            print(f"[UDP] Bind error: {e}")
            self.sock = None

        self.robots = {} # 수신된 로봇 상태 데이터 저장소 { "robot_ID": { ... } }
        self.running = True
        
        # 1. 수신 스레드 시작
        # 들어오는 패킷을 계속해서 받아서 파싱합니다.
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        
        # 2. Discovery(생존 신고) 방송 스레드 시작
        # 로봇은 '아는 팀원'에게만 데이터를 보냅니다.
        # 따라서 Mac이 주기적으로 "나도 팀원(99번)이야"라고 알려줘야 로봇이 데이터를 보내줍니다.
        self.discovery_thread = threading.Thread(target=self.broadcast_discovery, daemon=True)
        self.discovery_thread.start()

    # [수신 루프]
    def loop(self):
        if not self.sock: return
        while self.running:
            try:
                # 1024 버퍼 크기로 데이터 수신
                data, addr = self.sock.recvfrom(1024)
                
                # 데이터 크기가 예상된 구조체 크기와 일치하는지 확인
                if len(data) == EXPECTED_SIZE:
                    self.parse_packet(data, addr)
                else:
                    # 크기가 안 맞으면 무시 (다른 버전이거나 깨진 패킷)
                    pass
            except Exception as e:
                print(f"[UDP] Error: {e}")
                time.sleep(1)

    # [Discovery 브로드캐스트]
    # 로봇들에게 Mac의 존재를 알리는 함수
    def broadcast_discovery(self):
        # Discovery Port 규칙: 20000 + team_id (RoboCup 프로토콜)
        dest_port = 20000 + self.team_id
        
        # 송신용 소켓 생성
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # 브로드캐스트 패킷 허용
        
        # TeamDiscoveryMsg 구조체 (16 bytes)
        # int validation = 41203; (매직 넘버)
        # int communicationId = 0;
        # int teamId;
        # int playerId; (Mac을 99번 선수로 위장하여 구별)
        validation = 41203
        comm_id = 0
        player_id = 99 # Debugger ID
        
        print(f"[UDP] Broadcasting Discovery to port {dest_port}...")
        
        while self.running:
            try:
                # 데이터를 리틀 엔디안(<) 정수(i) 4개로 패킹
                msg = struct.pack("<4i", validation, comm_id, self.team_id, player_id)
                # 브로드캐스트 주소로 전송
                sock.sendto(msg, ('<broadcast>', dest_port))
                
                comm_id += 1
                time.sleep(1.0) # 1초마다 생존 신고 (너무 자주 보내면 대역폭 낭비)
            except Exception as e:
                print(f"[UDP] Discovery Broadcast Error: {e}")
                time.sleep(1.0)

    # [패킷 파싱]
    # 바이너리 데이터를 python 딕셔너리로 변환
    def parse_packet(self, data, addr):
        try:
            # struct.unpack을 사용하여 바이너리 데이터 해독
            unpacked = struct.unpack(FMT, data)

            # Update Status
            self.robots[robot_id] = {
                "id": robot_id,
                "role": role_str,
                "battery": 12.5, # UDP does not have battery info, dummy value
                "x": rx,
                "y": ry,
                "theta": rtheta,
                "ball_x": bx,
                "ball_y": by,
                "last_seen": time.time(),
                "ip": addr[0]
            }
        except Exception as e:
            print(f"[UDP] Parse error: {e}")

    def get_status(self):
        # Clean up old robots (timeout 3s)
        now = time.time()
        expired = [rid for rid, r in self.robots.items() if now - r['last_seen'] > 3.0]
        for rid in expired:
            del self.robots[rid]
        return self.robots

udp_monitor = UDPMonitor(team_id=1) # Default Team 1
