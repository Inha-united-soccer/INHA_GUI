from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
import asyncio
import json
import os
from ssh_manager import ssh_manager
from ros_bridge import init_ros, ros_bridge

# FastAPI 앱 초기화
# 이 파일(app.py)은 웹 관제 시스템의 백엔드 서버 메인 파일입니다.
# 프론트엔드(React)와 로봇(ROS/SSH) 사이의 중계 역할을 담당합니다.
# - REST API: 로봇 연결, 전략 배포, 파일 저장/로드
# - WebSocket: 실시간 로봇 상태(위치, 배터리 등) 스트리밍
app = FastAPI()

# CORS 설정 (Cross-Origin Resource Sharing)
# 보안 정책 상, 다른 포트(3000번 React)에서 이 서버(8000번)로 요청을 보낼 때 차단되지 않도록 허용합니다.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 모든 출처(Origin)에서의 접근을 허용 (개발 편의성)
    allow_credentials=True, # 쿠키 등 인증 정보 포함 허용
    allow_methods=["*"],  # 모든 HTTP 메서드(GET, POST, PUT, DELETE 등) 허용
    allow_headers=["*"],  # 모든 HTTP 헤더 허용
)

# 프론트엔드 빌드 결과물 경로 설정
# React 프로젝트를 'npm run build'로 빌드하면 생성되는 'dist' 폴더를 연결합니다.
FRONTEND_DIST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../frontend/dist")

# 정적 파일 서빙 (CSS, JS, 이미지 등)
# HTML이 아닌 부가 리소스들을 브라우저가 가져갈 수 있도록 경로를 마운트합니다.
if os.path.exists(FRONTEND_DIST_DIR):
    app.mount("/assets", StaticFiles(directory=os.path.join(FRONTEND_DIST_DIR, "assets")), name="assets")

# 메인 페이지 라우팅 ("/")
# 브라우저로 접속했을 때 React 앱의 진입점(index.html)을 보여줍니다.
@app.get("/")
async def serve_index():
    if os.path.exists(os.path.join(FRONTEND_DIST_DIR, "index.html")):
        return FileResponse(os.path.join(FRONTEND_DIST_DIR, "index.html"))
    return {"message": "Frontend build not found. Please run 'npm run build' in frontend directory."}

# ROS 2 노드 초기화
# 서버 시작 시 ROS 2 노드를 생성하여 토픽 통신을 준비합니다.
# Mac 등 ROS가 없는 환경에서는 예외가 발생하며, 이 경우 자동으로 시뮬레이션 모드로 동작하거나 UDP 모니터를 사용합니다.
try:
    ros_bridge_node = init_ros()
except Exception as e:
    print(f"Warning: ROS 2 init failed (Sim Mode?): {e}")

# [데이터 모델] 로봇 연결 설정
# 프론트엔드에서 보낸 JSON 데이터를 파싱하기 위한 Pydantic 모델
class RobotConfig(BaseModel):
    id: str       # 로봇 식별자 (예: robot_1)
    ip: str       # 로봇의 IP 주소 (예: 192.168.0.10)
    username: str # SSH 접속 계정명 (예: booster)
    password: str # SSH 접속 비밀번호

# [데이터 모델] 쉘 명령어 실행 요청
class Command(BaseModel):
    robot_id: str # 명령을 수행할 로봇 ID
    cmd: str      # 실행할 리눅스 쉘 명령어 (예: sudo reboot)

# [API] 로봇 SSH 연결
# 사용자가 'Connect' 버튼을 눌렀을 때 호출됩니다.
@app.post("/api/connect")
def connect_robot(config: RobotConfig):
    print(f"[API] Connect request: {config.id}@{config.ip}")
    success = ssh_manager.connect(config.id, config.ip, config.username, config.password)
    if not success:
        raise HTTPException(status_code=400, detail="Connection Failed")
    return {"status": "connected", "id": config.id}

# [API] 쉘 명령어 전송
# 'Start Program', 'Reboot' 등의 버튼을 눌렀을 때 호출됩니다.
@app.post("/api/command")
def send_command(command: Command):
    print(f"[API] Command request: {command.cmd} -> {command.robot_id}")
    
    # [Log Modification] User requested brain.log at root
    if "brain_nohup.log" in command.cmd:
        command.cmd = command.cmd.replace("brain_nohup.log", "/home/booster/Workspace/GUI/INHA-Player/launcher.log")
        command.cmd = command.cmd.replace("Workspace/Soccer", "Workspace/GUI/INHA-Player")
        
        # [Fix] Force source install/setup.bash to ensure local packages are found
        if "source /opt/ros" in command.cmd:
            command.cmd = command.cmd.replace("; nohup", "; source ./install/setup.bash; nohup")
            
        print("[API] Redirected output to INHA-Player/launcher.log, switched workspace, and added local source")
    stdout, stderr = ssh_manager.execute_command(command.robot_id, command.cmd)
    if stdout is None:
        raise HTTPException(status_code=500, detail=stderr)
    return {"stdout": stdout, "stderr": stderr}

# [데이터 모델] 전략 배포 요청
class StrategyDeploy(BaseModel):
    robot_id: str = "all" # 특정 로봇 ID 또는 "all"(전체)
    strategy_xml: str     # 배포할 Behavior Tree XML 내용

# [API] 전략 배포 (Hot-Swap)
# 작성한 전략(XML)을 로봇에게 전송하여 즉시 적용시킵니다.
# 로봇 내부에서 'ros2 topic pub' 명령을 실행하는 방식으로 동작합니다.
@app.post("/api/deploy_strategy")
def deploy_strategy_endpoint(data: StrategyDeploy):
    # 1. 파일 내용(XML) 추출
    xml_content = data.strategy_xml
    print(f"[API] Deploy request to {data.robot_id}")
    
    # 2. 현재 SSH로 연결된 로봇 목록 확인
    connected_robots = list(ssh_manager.clients.keys())
    
    # 연결된 로봇이 없으면 경고 출력 (시뮬레이션 로그)
    if not connected_robots:
         print("[WARN] No robot connected via SSH. Deployment skipped (Simulation).")
         if ros_bridge_node:
             ros_bridge_node.publish_strategy("sim_robot", xml_content)
         return {"status": "skipped", "message": "No robot connected via SSH"}

    # 3. 대상 로봇들에게 전략 배포
    success_count = 0
    targets = connected_robots if data.robot_id == "all" else [data.robot_id]
    
    for robot_id in targets:
        if robot_id not in ssh_manager.clients: continue
        
        # ssh_manager를 통해 원격 명령 실행
        success, msg = ssh_manager.deploy_strategy(robot_id, xml_content)
        if success:
            success_count += 1
            print(f"[Deploy] Success to {robot_id}")
        else:
            print(f"[Deploy] Failed to {robot_id}: {msg}")

    if success_count > 0:
        return {"status": "success", "message": f"Deployed to {success_count} robots"}
    else:
        raise HTTPException(status_code=500, detail="Failed to deploy to any robot")

import os
from glob import glob

# 전략 파일들이 저장될 디렉토리
STRATEGY_DIR = "strategies"

# [API] 저장된 전략 목록 조회
# strategies 폴더 내의 모든 .xml 파일 이름을 반환합니다.
@app.get("/api/strategies")
async def list_strategies():
    files = glob(os.path.join(STRATEGY_DIR, "*.xml"))
    return {"strategies": [os.path.basename(f) for f in files]}

# [데이터 모델] 전략 저장 요청
class StrategySave(BaseModel):
    name: str # 파일명 (확장자 제외 가능)
    xml: str  # 파일 내용

# [API] 전략 저장
# Blockly로 작성한 전략을 서버에 파일로 저장합니다.
@app.post("/api/strategies")
async def save_strategy(data: StrategySave):
    filename = data.name if data.name.endswith(".xml") else f"{data.name}.xml"
    path = os.path.join(STRATEGY_DIR, filename)
    with open(path, "w") as f:
        f.write(data.xml)
    return {"status": "saved", "name": filename}

# [API] 특정 전략 불러오기
# 저장된 XML 파일의 내용을 읽어서 반환합니다.
@app.get("/api/strategies/{name}")
async def load_strategy(name: str):
    path = os.path.join(STRATEGY_DIR, name)
    if not os.path.exists(path):
        raise HTTPException(status_code=404, detail="Strategy not found")
    with open(path, "r") as f:
        return {"xml": f.read()}

from udp_monitor import udp_monitor

# [WebSocket] 실시간 상태 스트리밍 엔드포인트
# 프론트엔드가 이 주소로 웹소켓을 연결하면, 0.5초마다 로봇들의 최신 상태를 JSON으로 전송해줍니다.
@app.websocket("/ws/status")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept() # 연결 수락
    try:
        while True:
            status = {}
            
            # 1. UDP Monitor 데이터 우선 사용 (실제 로봇 데이터)
            # udp_monitor가 백그라운드에서 수신한 최신 패킷 정보를 가져옵니다.
            udp_data = udp_monitor.get_status()
            if udp_data:
                status = udp_data
            
            # 2. UDP 데이터가 없으면 ROS Bridge 데이터 사용 (시뮬레이션 또는 로컬 ROS)
            elif ros_bridge_node:
                status = ros_bridge_node.get_status()
            
            # 클라이언트에게 JSON 전송
            await websocket.send_json(status)
            await asyncio.sleep(0.5) # 업데이트 주기 (0.5초)
    except Exception as e:
        print(f"WebSocket closed: {e}")

# 서버 직접 실행 시 진입점
if __name__ == "__main__":
    import uvicorn
    # uvicorn 서버 실행 (호스트 0.0.0.0은 외부 접속 허용을 의미)
    uvicorn.run(app, host="0.0.0.0", port=8000)
