# INHA-Player Web Control Center

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white)
![React](https://img.shields.io/badge/React-18-61DAFB?logo=react&logoColor=black)
![FastAPI](https://img.shields.io/badge/FastAPI-0.95%2B-009688?logo=fastapi&logoColor=white)
![TypeScript](https://img.shields.io/badge/TypeScript-5.0-3178C6?logo=typescript&logoColor=white)

**A powerful and intuitive command interface designed for humanoid robots.**
This platform provides real-time status monitoring, dynamic strategy deployment, seamless GameController integration, and live 2D field visualization.

---

## 📸 Screenshots

| Dashboard View | Field Visualization |
| :---: | :---: |
| ![Dashboard](./docs/images/dashboard_preview.png) <br> *Main Dashboard & Robot Status Cards* | ![Field](./docs/images/field_preview.png) <br> *Live Localization & Ball Detection* |

| Strategy Deployment | Game Info & Logs |
| :---: | :---: |
| ![Strategy](./docs/images/strategy_deploy.png) <br> *Strategy Selection & Deployment Interface* | ![Logs](./docs/images/game_logs.png) <br> *GameController Data & System Logs* |

---

## ✨ Key Features

1.  **Comprehensive Dashboard**
    -   Monitor critical metrics for all robots at a glance, including current state (Ready, Set, Play), battery levels, and network connectivity.
    -   Access individual **Command Panels** for granular control over specific units.

2.  **Live Field Visualizer**
    -   Renders real-time robot odometry and ball detection data on an interactive 2D field map.
    -   Intelligently filters data to display the single most reliable ball position based on the highest confidence score among agents.

3.  **GameController Integration**
    -   Fully compatible with the official **RoboCup GameController (v15 protocol)**.
    -   Displays essential match data including game time, scores, penalties, and message budgets in real time.

4.  **Remote SSH & Strategy Management**
    -   Manage robots directly via the web interface using secure SSH connections.
    -   Upload and deploy strategy files, execute `start.sh` scripts, and stream execution logs without leaving the browser.

5.  **State History & Diagnostics**
    -   Archives state changes and action logs with precise timestamps.
    -   Provides a historical view of robot behavior, essential for post-game debugging and analysis.

---

## 🚀 Getting Started

### Prerequisites
Ensure the following are installed before running the project:
-   **Node.js**: v16 or higher
-   **Python**: v3.8 or higher
-   **Pip Packages**: `fastapi`, `uvicorn`, `paramiko`, `websockets`

### 1. Backend (Python/FastAPI)
The backend handles UDP/SSH communication with robots and parses GameController packets.

```bash
cd web_control/backend
python app.py
```
Active Ports: 8000 (API), 3838 (GameController UDP), 30001 (Robot UDP)

2. Frontend (React/Vite)
Launch the user interface.

```bash
cd web_control/frontend
npm install
npm run dev
```

## 🛠 Tech Stack

### Frontend

- React, TypeScript, Vite
- Material-UI (MUI) for styling
- React-Use-Websocket for real-time data

### Backend

- Python, FastAPI, Asyncio
- Paramiko (SSH Management)
- Struct (UDP Packet Parsing)

### Communication

- WebSocket: Real-time UI updates
- HTTP: RESTful commands and file transfers
