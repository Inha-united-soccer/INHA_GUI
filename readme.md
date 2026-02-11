# INHA-Player Web Control Center

직관적이고 강력한 휴머노이드 로봇 제어 GUI입니다. 로봇 상태 모니터링, 전략 배포, GameController 연동, 실시간 필드 시각화를 제공합니다.

---

## 📸 Screenshots

| Dashboard View | Field Visualization |
| :---: | :---: |
| ![Dashboard](./docs/images/dashboard_preview.png) <br> *메인 대시보드 및 로봇 상태 카드* | ![Field](./docs/images/field_preview.png) <br> *실시간 로봇 위치 및 공 감지 시각화* |

| Strategy Deployment | Game Info & Logs |
| :---: | :---: |
| ![Strategy](./docs/images/strategy_deploy.png) <br> *전략 선택 및 배포 인터페이스* | ![Logs](./docs/images/game_logs.png) <br> *GameController 정보 및 시스템 로그* |

---

## ✨ Key Features

1.  **Dashboard (대시보드)**
    -   각 로봇의 현재 상태(Ready, Set, Play 등)와 배터리, 통신 상태를 한눈에 파악할 수 있습니다.
    -   개별 로봇에 대한 제어 패널(Command Panel)을 제공합니다.

2.  **Field Visualizer (필드 시각화)**
    -   로봇들의 위치(Odometry)와 각 로봇이 감지한 공(Ball)의 위치를 2D 필드 맵에 실시간으로 그립니다.
    -   가장 신뢰도(Confidence)가 높은 공 하나를 선별하여 표시합니다.

3.  **GameController Integration**
    -   공식 RoboCup GameController(v15 프로토콜)와 연동되어 경기 시간, 점수, 페널티, 메시지 예산(Message Budget) 정보를 실시간으로 표시합니다.

4.  **SSH Control & Strategy**
    -   웹 인터페이스에서 SSH를 통해 로봇에 접속하고, 전략(Strategy) 파일을 직접 전송 및 실행할 수 있습니다.
    -   `start.sh` 스크립트 실행 및 로그 확인이 가능합니다.

5.  **State History (상태 이력)**
    -   로봇의 상태 변화(State Change)와 행동(Action) 기록을 타임스탬프와 함께 저장하여 디버깅에 활용할 수 있습니다.

---

## 🚀 Getting Started

### Prerequisites
- **Node.js**: v16+
- **Python**: v3.8+
- **Pip Packages**: `fastapi`, `uvicorn`, `paramiko`, `websockets`

### 1. Backend (Python/FastAPI)
백엔드 서버는 로봇과의 통신(UDP/SSH) 및 GameController 패킷 파싱을 담당합니다.

```bash
cd web_control/backend
python app.py
```
*Port: 8000 (API), 3838 (GameController UDP), 30001 (Robot UDP)*

### 2. Frontend (React/Vite)
사용자 인터페이스를 실행합니다.

```bash
cd web_control/frontend
npm install
npm run dev
```
*Access: http://localhost:5173*

---

## 🛠 Tech Stack

- **Frontend**: React, TypeScript, Material-UI (MUI), Vite, React-Use-Websocket
- **Backend**: Python, FastAPI, Asyncio, Paramiko (SSH), Struct (UDP Parsing)
- **Communication**: WebSocket (Real-time updates), HTTP (Commands)
