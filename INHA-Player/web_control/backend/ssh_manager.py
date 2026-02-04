import paramiko # ssh 연결을 위한 파이썬 라이브러리 (원격 제어 핵심)
import asyncio
import logging

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
            
            # 연결 성공 시 클라이언트 목록에 저장
            self.clients[robot_id] = client
            self.logger.info(f"Connected to {robot_id} ({ip})")
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

    # [전략 배포] (Hot-Swap)
    # XML 문자열을 로봇에게 전송하여 동작을 즉시 거나 수정합니다.
    # 기술적으로는 'ros2 topic pub' 명령어를 SSH로 원격 실행하는 방식입니다.
    def deploy_strategy(self, robot_id, xml_content):
        if robot_id not in self.clients:
            return False, "Not connected"

        try:
            # 1. XML 내용 정제
            # 줄바꿈을 제거하여 한 줄 명령어(bash -c)로 만들기 위해 공백으로 치환
            safe_xml = xml_content.replace("\n", " ").replace("\r", " ")
            # 작은 따옴표(')가 쉘 명령어와 충돌하지 않도록 이스케이프 처리
            safe_xml = safe_xml.replace("'","'\\''")
            
            # 2. ROS 2 환경 설정 명령어 (Humble, Foxy 등 버전별 자동 탐색)
            # SSH 비로그인 세션에서는 환경변수가 로드되지 않으므로 명시적으로 source 해줍니다.
            setup_cmd = "source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null"
            
            # 3. 토픽 발행 명령어 생성
            # /robot_1/strategy/deploy 토픽으로 XML 문자열을 1회(--once) 발행합니다.
            # pub_cmd 내부의 쌍따옴표(")는 나중에 bash -c "..." 안에 들어갈 때 이스케이프(\") 처리해야 함
            pub_cmd = f"ros2 topic pub --once /{robot_id}/strategy/deploy std_msgs/msg/String \"{{data: '{safe_xml}'}}\""
            
            # 4. 전체 명령어 조합 (bash -c "...")
            # ROS 설정(setup_cmd)과 발행(pub_cmd)을 연달아 실행합니다.
            safe_pub_cmd = pub_cmd.replace('"', '\\"')
            full_cmd = f"bash -c \"{setup_cmd}; {safe_pub_cmd}\""
            
            # 원격 실행
            client = self.clients[robot_id]
            stdin, stdout, stderr = client.exec_command(full_cmd)
            
            # 결과 확인
            err = stderr.read().decode()
            out = stdout.read().decode()
            
            # Publisher 시작 메시지가 뜨거나 에러가 없으면 성공으로 간주
            if "Publisher" in out or "publishing" in out or not err:
                return True, "Strategy deployed successfully"
            else:
                return False, f"Deploy failed: {err}"
                
        except Exception as e:
            return False, f"Deploy exception: {str(e)}"

# 전역 인스턴스 생성
ssh_manager = SSHManager()
