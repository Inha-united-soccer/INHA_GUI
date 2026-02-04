import threading
import json
import asyncio
import math
import random
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    # booster_interface가 없을 경우를 대비한 처리
    try:
        from booster_interface.msg import LowState, Odometer
    except ImportError:
        # 가짜 클래스 정의 (에러 방지용)
        class LowState: pass
        class Odometer: pass
        print("[WARN] booster_interface not found. Using dummy classes.")

    ROS_AVAILABLE = True
except ImportError:
    # ROS 2 환경이 아닐 경우 (예: 로컬 테스트), 가상 모드로 동작
    ROS_AVAILABLE = False
    class Node: pass # Dummy class

# 웹 서버와 ROS 2 시스템 간의 브릿지 역할을 하는 클래스
# 로봇의 상태(위치, 배터리 등)를 수신하고, 웹에서 내린 명령을 ROS 토픽으로 발행
class ROSBridge(Node):
    def __init__(self):
        self.robot_status = {} # 로봇들의 현재 상태 저장소
        
        if ROS_AVAILABLE:
            super().__init__('web_bridge_node')
            # 로봇별 데이터 수신 (구독) 설정
            for i in range(1, 4):
                rid = f"robot_{i}"
                # 역할(Role)은 로봇이 publish하지 않으므로, 배포 내역을 통해 추론하거나 초기값 설정
                self.update_status(rid, 'role', 'Unknown')
                
                # 하드웨어 상태(배터리 등) 수신 - /low_state 토픽 사용
                # 실제 로봇 세팅에 따라 네임스페이스가 적용될 수 있음 (/{rid}/low_state)
                self.create_subscription(LowState, f'/{rid}/low_state', lambda m, r=rid: self.update_battery(r, m), 10)
                
                # 위치 정보 수신 - /odometer_state 토픽 사용
                self.create_subscription(Odometer, f'/{rid}/odometer_state', lambda m, r=rid: self.update_odometer(r, m), 10)

            # [Patch] 네임스페이스 없는 단일 로봇 환경 지원 (robot_1로 매핑)
            self.create_subscription(LowState, '/low_state', lambda m: self.update_battery('robot_1', m), 10)
            self.create_subscription(Odometer, '/odometer_state', lambda m: self.update_odometer('robot_1', m), 10)
            
            # ROS 2 콜백 처리를 위한 별도 스레드 시작
            self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.spin_thread.start()
        else:
            print("[WARN] ROS 2 not detected. Running in Simulation Mode (No Dummy Data).")
            # 시뮬레이션 모드 초기 데이터 설정 (비워둠 -> 실제 연결된 로봇만 표시)
            # UDP 모니터가 있기 때문에 굳이 가짜 데이터를 만들지 않음
            self.robot_status = {}
            # 시뮬레이션 움직임 스레드 시작 (데이터가 없으므로 실제로는 아무것도 안 함)
            self.sim_thread = threading.Thread(target=self.simulate_movement, daemon=True)
            self.sim_thread.start()

    # ROS 2 이벤트 루프 실행 (비동기)
    def spin_ros(self):
        if ROS_AVAILABLE:
            rclpy.spin(self)

    # ROS 연결이 없을 때 가짜 움직임을 생성하는 함수 (테스트용)
    def simulate_movement(self):
        while True:
            for rid in self.robot_status:
                # 랜덤 워크 (Random Walk) 시뮬레이션
                if 'x' in self.robot_status[rid]:
                    self.robot_status[rid]['x'] += random.uniform(-0.1, 0.1)
                    self.robot_status[rid]['y'] += random.uniform(-0.1, 0.1)
                    # 경기장 범위 제한
                    self.robot_status[rid]['x'] = max(-5, min(5, self.robot_status[rid]['x']))
                    self.robot_status[rid]['y'] = max(-3.5, min(3.5, self.robot_status[rid]['y']))
            time.sleep(0.1)

    # 로봇 상태 정보 업데이트 헬퍼 함수
    def update_status(self, robot_id, key, value):
        if robot_id not in self.robot_status:
            self.robot_status[robot_id] = {}
        self.robot_status[robot_id][key] = value

    # 배터리 정보 콜백 처리 (LowState 메시지)
    def update_battery(self, robot_id, msg):
        # LowState 메시지 구조에 따라 battery_voltage 접근
        if hasattr(msg, 'battery_voltage'):
            self.update_status(robot_id, 'battery', msg.battery_voltage)

    # 위치 정보 콜백 처리 (Odometer 메시지)
    def update_odometer(self, robot_id, msg):
        # Odometer 메시지 구조 (x, y, theta)
        if hasattr(msg, 'x') and hasattr(msg, 'y'):
            self.update_status(robot_id, 'x', msg.x)
            self.update_status(robot_id, 'y', msg.y)

    # 현재 모든 로봇의 상태 반환
    def get_status(self):
        return self.robot_status

    # 전략 XML을 ROS 토픽으로 발행하여 로봇에게 전송
    def publish_strategy(self, robot_id, xml_content):
        # 전략 내용에서 Rold 추론 (XML 내용이나 파일명 등을 활용 가능)
        role = "Unknown"
        if "Striker" in xml_content or "Attacker" in xml_content:
            role = "Striker"
        elif "Defender" in xml_content:
            role = "Defender"
        elif "Goalkeeper" in xml_content or "Golie" in xml_content:
            role = "Goalkeeper"
        else:
            role = "Custom"
            
        self.update_status(robot_id, 'role', role)

        if ROS_AVAILABLE:
            topic = f"/{robot_id}/strategy/deploy"
            
            # 퍼블리셔가 없으면 생성 (지연 생성 패턴)
            if not hasattr(self, 'strat_pubs'):
                self.strat_pubs = {}
            
            if robot_id not in self.strat_pubs:
                self.strat_pubs[robot_id] = self.create_publisher(String, topic, 10)
            
            msg = String()
            msg.data = xml_content
            self.strat_pubs[robot_id].publish(msg)
            print(f"[ROS] Published strategy to {topic} (Infer Role: {role})")
        else:
            # 시뮬레이션 모드에서는 로그만 출력
            print(f"[SIM] Deployed strategy to {robot_id} (Infer Role: {role})")

# ROS 초기화 함수
def init_ros():
    if ROS_AVAILABLE:
        rclpy.init(args=None)
    return ROSBridge()

ros_bridge = None
