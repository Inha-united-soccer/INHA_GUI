import paramiko # ssh 연결을 위한 파이썬 라이브러리 (원격 제어 핵심)
import asyncio
import logging
import base64

# [SSH 관리자 클래스]
# 로봇들과의 SSH 연결 세션을 관리하고, 원격 명령을 실행하는 역할을 합니다.
# 싱글톤 패턴처럼 하나의 인스턴스(ssh_manager)를 전역에서 공유하여 사용합니다.
class SSHManager:
    def __init__(self):
        # 연결된 SSH 클라이언트 객체들을 저장하는 딕셔너리
        # Key: 로봇 ID (예: "robot_1"), Value: paramiko.SSHClient 객체
        self.clients = {} 
        self.logger = logging.getLogger("SSHManager")

    # [SSH 연결]
    # 사용자가 입력한 IP, ID, PW로 로봇에 접속을 시도합니다.
    def connect(self, robot_id, ip, username, password):
        try:
            client = paramiko.SSHClient()
            # "알 수 없는 호스트" 경고를 무시하고 자동으로 키를 등록합니다 (편의성)
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) 
            
            # 실제 연결 시도 (타임아웃 3초)
            client.connect(ip, username=username, password=password, timeout=3.0)
            
            # [검증] 실제 명령이 먹히는지 확인
            stdin, stdout, stderr = client.exec_command("whoami")
            user = stdout.read().decode().strip()
            if not user:
                raise Exception("SSH Connection verification failed (whoami no output)")
                
            self.logger.info(f"Connected to {robot_id} ({ip}) as {user}")
            
            # 연결 성공 시 클라이언트 목록에 저장
            self.clients[robot_id] = client
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to {robot_id}: {e}")
            return False

    # [원격 명령 실행]
    # 특정 로봇에게 리눅스 쉘 명령어를 전송하고 결과를 받아옵니다.
    def execute_command(self, robot_id, command):
        if robot_id not in self.clients:
            return None, "Not connected"
        
        try:
            client = self.clients[robot_id]
            # exec_command: 비차단(Non-blocking) 방식으로 명령 실행
            # stdin, stdout, stderr 스트림을 반환받습니다.
            print(f"[SSH Cmd] Executing on {robot_id}: {command}")
            stdin, stdout, stderr = client.exec_command(command)
            
            # 실행 결과를 문자열로 디코딩하여 반환
            out_str = stdout.read().decode()
            err_str = stderr.read().decode()
            print(f"[SSH Result] OUT: {out_str[:100]}... / ERR: {err_str[:100]}...")
            return out_str, err_str 
        except Exception as e:
            print(f"[SSH Error] {e}")
            return None, str(e)

    # [연결 해제]
    def disconnect(self, robot_id):
        if robot_id in self.clients:
            self.clients[robot_id].close()
            del self.clients[robot_id]

    # [전략 배포] (Hot-Swap + Persistence)
    def deploy_strategy(self, robot_id, xml_content):
        if robot_id not in self.clients:
            return False, "Not connected"

        try:
            # 0. XML 파싱하여 ID 확인
            import xml.etree.ElementTree as ET
            root = ET.fromstring(xml_content)
            # BehaviorTree 태그를 찾아서 ID 속성 확인
            bt_node = root.find('BehaviorTree')
            strategy_id = bt_node.get('ID') if bt_node is not None else None
            
            # 영구 저장 대상인지 확인 (Striker, Defender, Goalkeeper 등)
            # 대소문자 무시하고 비교
            persistence_target = None
            if strategy_id:
                lower_id = strategy_id.lower()
                if lower_id in ['striker', 'defender', 'goalkeeper', 'goalie']:
                    persistence_target = lower_id
                    # goalie -> goalkeeper 로 통일 (파일명 기준)
                    if persistence_target == 'goalie': persistence_target = 'goalkeeper'

            # 1. XML 내용을 Base64로 인코딩 (특수문자/따옴표 문제 원천 차단)
            xml_b64 = base64.b64encode(xml_content.encode('utf-8')).decode('utf-8')
            write_xml_cmd = f"echo '{xml_b64}' | base64 -d > /tmp/strategy_deploy.xml"

            # 영구 저장 명령 추가
            persistence_cmd = ""
            if persistence_target:
                print(f"[Deploy] Persistence target detected: {persistence_target}")
                # 소스 코드 경로 (재빌드 시 사용)
                src_path = f"/home/booster/Workspace/INHA/INHA-Player/src/brain/behavior_trees/subtrees/{persistence_target}.xml"
                # 설치된 경로 (현재 실행 중인 노드가 참조)
                install_path = f"/home/booster/Workspace/INHA/INHA-Player/install/brain/share/brain/behavior_trees/subtrees/{persistence_target}.xml"
                
                # 덮어쓰기 명령 (sudo가 필요할 수도 있으나, 보통 user 권한으로 가능)
                # 만약 권한 문제가 생기면 echo ... | sudo tee ... 로 변경 필요
                persistence_cmd = f"cp /tmp/strategy_deploy.xml {src_path}; cp /tmp/strategy_deploy.xml {install_path};"
            else:
                print(f"[Deploy] Runtime-only deployment (ID: {strategy_id})")

            # 2. ROS 2 환경 설정 명령어
            # Workspace 경로 INHA-Player로 수정
            setup_cmd = "source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/booster/Workspace/INHA/INHA-Player/configs/fastdds.xml"
            
            # 3. Python 스크립트 작성 (Base64 인코딩)
            # 토픽으로도 쏴줘서 즉시 반영되게 함 (재시작 없이)
            py_code = f"""
import rclpy
from std_msgs.msg import String
import time
import sys
import os

print("[Deploy] Starting deployment script...")
try:
    rclpy.init()
    node = rclpy.create_node('web_deployer')
    pub = node.create_publisher(String, '/{robot_id}/strategy/deploy', 10)
    msg = String()
    
    if not os.path.exists('/tmp/strategy_deploy.xml'):
        print("[Deploy Error] XML file not found!")
        sys.exit(1)
        
    with open('/tmp/strategy_deploy.xml', 'r') as f:
        msg.data = f.read()

    print(f"[Deploy] Publishing to /{robot_id}/strategy/deploy (size: {{len(msg.data)}})...")
    
    # 확실한 전송을 위해 여러 번 발행
    for i in range(5):
        pub.publish(msg)
        time.sleep(0.2)
    
    print("[Deploy] Publish complete.")
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"[Deploy Error] {{e}}")
    sys.exit(1)
"""
            py_b64 = base64.b64encode(py_code.encode('utf-8')).decode('utf-8')
            write_py_cmd = f"echo '{py_b64}' | base64 -d > /tmp/deploy.py"
            
            # 4. 최종 명령어 조합
            # 순서: XML파일생성 -> (영구저장) -> Python파일생성 -> 환경설정 및 Python실행
            full_cmd = f"bash -c \"{write_xml_cmd}; {persistence_cmd} {write_py_cmd}; {setup_cmd}; python3 /tmp/deploy.py\""
            
            # 원격 실행
            client = self.clients[robot_id]
            stdin, stdout, stderr = client.exec_command(full_cmd)
            
            # 결과 확인
            out_str = stdout.read().decode()
            err_str = stderr.read().decode()
            
            print(f"[Deploy Log] {out_str}")
            if err_str:
                print(f"[Deploy Err] {err_str}")
                
            if "Publish complete" in out_str:
                 msg = "Deploy Success"
                 if persistence_target:
                     msg += f" (Saved to {persistence_target}.xml)"
                 return True, msg
            else:
                 return False, f"Deploy Failed: {err_str if err_str else out_str}"
                 
        except Exception as e:
            self.logger.error(f"Deploy exception: {e}")
            return False, f"Deploy exception: {e}"

# 전역 인스턴스 생성
ssh_manager = SSHManager()
