import paramiko # ssh 연결을 위한 파이썬 라이브러리
import asyncio
import logging
import base64

# 로봇들과의 SSH 연결 세션을 관리하고, 원격 명령을 실행하는 역할 -> 싱글톤 패턴처럼 하나의 인스턴스(ssh_manager)를 전역에서 공유하여 사용
class SSHManager:
    def __init__(self):
        # 연결된 SSH 클라이언트 객체들을 저장하는 딕셔너리 -> Key: 로봇 ID (예: "robot_1"), Value: paramiko.SSHClient 객체
        self.clients = {} 
        self.logger = logging.getLogger("SSHManager")

    # [SSH 연결] -> 사용자가 입력한 IP, ID, PW로 로봇에 접속을 시도 - 디폴트값 지정해둘 수 있음
    def connect(self, robot_id, ip, username, password):
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) 
            
            # 실제 연결 시도 (타임아웃 3초) - 잘못된 연결임을 알기에는 충분
            client.connect(ip, username=username, password=password, timeout=3.0)
            
            # [검증] 실제 명령이 먹히는지 확인 - 단순 연결 후 whoami 명령어 실행으로 다시 검증
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

    # [원격 명령 실행] -> 특정 로봇에게 기본적인 리눅스 쉘 명령어를 전송하고 결과를 받아오기
    def execute_command(self, robot_id, command):
        if robot_id not in self.clients:
            return None, "Not connected"
        
        try:
            client = self.clients[robot_id]
            # exec_command: 비차단(Non-blocking) 방식으로 명령 실행 -> stdin, stdout, stderr 스트림을 반환받음
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


    # -----------------------------------------------------------------------------------------
    # [전략 배포 기능] (Hot-Swap)
    # 작성된 Behavior Tree XML을 로봇에게 전송하고 즉시 실행시키는 기능(재부팅 없이 실시간 교체를 위해)
    # 1. XML 내용을 Base64로 인코딩 (xml 안에는 특수문자가 많아 그냥 보내면 쉘이 깨질 수 있음 base64 문자열로 암호화해서 /tmp에 파일로 저장)
    # 2. 로봇의 임시 경로(/tmp)에 파일 생성
    # 3. 파이썬 스크립트를 생성하여 토픽 /{robot_id}/strategy/deploy로 XML 내용 발행
    # 4. 로봇 내부의 Brain 노드가 이 토픽을 구독하고 즉시 행동을 교체함
    # -----------------------------------------------------------------------------------------
    def deploy_strategy(self, robot_id, xml_content):
        if robot_id not in self.clients:
            return False, "Not connected"

        try:
            # XML 파싱하여 ID 및 역할 확인 (영구 저장 여부 결정)
            import xml.etree.ElementTree as ET
            root = ET.fromstring(xml_content)
            bt_node = root.find('BehaviorTree')
            strategy_id = bt_node.get('ID') if bt_node is not None else None
            
            # 영구 저장 대상인지 확인 (Striker, Defender, Goalkeeper 파일명과 일치하면 덮어쓰기 시도) -> 삭제 예정
            persistence_target = None
            if strategy_id:
                lower_id = strategy_id.lower()
                if lower_id in ['striker', 'defender', 'goalkeeper', 'goalie']:
                    persistence_target = lower_id
                    if persistence_target == 'goalie': persistence_target = 'goalkeeper'

            # 전송할 XML 파일 생성 명령어 (Base64 인코딩 사용)
            xml_b64 = base64.b64encode(xml_content.encode('utf-8')).decode('utf-8')
            write_xml_cmd = f"echo '{xml_b64}' | base64 -d > /tmp/strategy_deploy.xml"

            # 영구 저장 로직 -> 만약 중요한 역할(Striker 등) 배포라면 원본 XML 파일도 업데이트한다.
            persistence_cmd = ""
            if persistence_target:
                print(f"[Deploy] Persistence target detected: {persistence_target}")
                src_path = f"/home/booster/Workspace/GUI/INHA-Player/src/brain/behavior_trees/subtrees/{persistence_target}.xml"
                install_path = f"/home/booster/Workspace/GUI/INHA-Player/install/brain/share/brain/behavior_trees/subtrees/{persistence_target}.xml"
                persistence_cmd = f"cp /tmp/strategy_deploy.xml {src_path}; cp /tmp/strategy_deploy.xml {install_path};"
            else:
                print(f"[Deploy] Runtime-only deployment (ID: {strategy_id})")

            # ROS 2 환경 설정 명령어 (배포 스크립트 실행 전 환경 변수 로드)
            setup_cmd = "source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/booster/Workspace/GUI/INHA-Player/configs/fastdds.xml"
            
            # deploy.py배포용 파이썬 스크립트 동적 생성 -> 로봇 안에서 직접 'ros2 topic pub'을 하는 대신 파이썬 코드로 publish 하는 것이 더 안정적임
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
    # Brain 노드가 이 토픽을 구독하고 있음
    pub = node.create_publisher(String, '/{robot_id}/strategy/deploy', 10)
    #메시지를 보낼 주소(토픽)
    msg = String()
    
    if not os.path.exists('/tmp/strategy_deploy.xml'):
        sys.exit(1) # 파일 없으면 즉시 종료
        
    with open('/tmp/strategy_deploy.xml', 'r') as f:
        msg.data = f.read() # XML 파일의 모든 텍스트를 메시지에 담음

    print(f"[Deploy] Publishing to /{robot_id}/strategy/deploy...")
    
    # 안정적인 수신을 위해 5번 반복 전송
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
            # 역할: 
            # 1. /tmp에 저장된 Base64 텍스트를 읽어옴
            # 2. 그걸 다시 원래의 XML 파일로 디코딩(복구)
            # 3. ROS 2가 읽을 수 있도록 실제 파일을 생성
            # 파이썬 코드도 Base64로 인코딩해서 전송
            py_b64 = base64.b64encode(py_code.encode('utf-8')).decode('utf-8')
            write_py_cmd = f"echo '{py_b64}' | base64 -d > /tmp/deploy.py"
            
            # 최종 실행 명령어 조합 -> XML생성 -> [영구저장] -> 파이썬생성 -> 환경설정 -> 파이썬실행
            full_cmd = f"bash -c \"{write_xml_cmd}; {persistence_cmd} {write_py_cmd}; {setup_cmd}; python3 /tmp/deploy.py\""
            
            # 원격 실행
            client = self.clients[robot_id]
            stdin, stdout, stderr = client.exec_command(full_cmd)
            
            # 결과 처리
            out_str = stdout.read().decode()
            err_str = stderr.read().decode()
            
            print(f"[Deploy Log] {out_str}")
            if err_str: print(f"[Deploy Err] {err_str}")
                
            if "Publish complete" in out_str:
                 msg = "Deploy Success"
                 if persistence_target: msg += f" (Saved to {persistence_target}.xml)"
                 return True, msg
            else:
                 return False, f"Deploy Failed: {err_str if err_str else out_str}"
                 
        except Exception as e:
            self.logger.error(f"Deploy exception: {e}")
            return False, f"Deploy exception: {e}"

    # [시스템 로그 가져오기] -> 'tail' 명령어로 launcher.log 파일의 뒷부분만 짤라서 가져온다
    def fetch_log(self, robot_id, lines=50):
        if robot_id not in self.clients:
            return "Not connected"
            
        try:
            client = self.clients[robot_id]
            cmd = f"tail -n {lines} /home/booster/Workspace/GUI/INHA-Player/launcher.log"
            stdin, stdout, stderr = client.exec_command(cmd)
            
            log_content = stdout.read().decode()
            if not log_content:
                err = stderr.read().decode()
                return f"[Log Error] {err}" if err else "[Log] Empty"
            return log_content
        except Exception as e:
            return f"[SSH Error] {e}"

# 전역에서 하나만 존재해야 하는 인스턴스 (Singleton)
ssh_manager = SSHManager()

