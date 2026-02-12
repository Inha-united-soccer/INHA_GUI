<div align="center">
    
# INHA Web Control Center

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white)
![React](https://img.shields.io/badge/React-18-61DAFB?logo=react&logoColor=black)
![FastAPI](https://img.shields.io/badge/FastAPI-0.95%2B-009688?logo=fastapi&logoColor=white)
![TypeScript](https://img.shields.io/badge/TypeScript-5.0-3178C6?logo=typescript&logoColor=white)

**A powerful and intuitive command interface designed for humanoid robot systems**
This platform provides real-time status monitoring, dynamic strategy deployment, seamless GameController integration, and live 2D field visualization.

---

</div>

## 📸 Interface Preview

| **Dashboard View** |
| :---: |
| <img src="https://github.com/user-attachments/assets/13146327-1a4f-477f-87f8-834c51ba048d" width="100%"> <br> **Main Dashboard & Robot Status Cards** |

<br>

| Connect to Robot | Strategy Deployment | Control Panel |
| :---: | :---: | :---: |
| <img width="100%" alt="connect" src="https://github.com/user-attachments/assets/2e863aa6-46da-4f7d-ab80-e69b93be912f" /> <br> **Connection & Logs** | <img src="https://github.com/user-attachments/assets/c15c8cce-4e40-4243-b195-a4118103f9b2" width="90%"> <br> **Strategy Interface** | <img src="https://github.com/user-attachments/assets/88302c3b-3c29-44e2-af77-b4e90d873b1a" width="80%"> <br> **Detailed Control** |

---

## 💡 Key Features

1.  **Comprehensive Dashboard**
    -   Monitor critical metrics for all robots at a glance, including their current state (Ready, Set, Play), battery levels, and network connectivity.
    -   Access individual **Command Panels** for granular control over specific units.

2.  **Live Field Visualizer**
    -   Renders real-time robot odometry and ball detection data on an interactive 2D field map.
    -   Intelligently filters data to display the most reliable ball position based on the highest confidence score among agents.

3.  **GameController Integration**
    -   Fully compatible with the official **RoboCup GameController (v15 protocol)**.
    -   Displays essential match data including game time, scores, penalties, and message budgets in real time.

4.  **Remote SSH & Strategy Management**
    -   Manage robots directly via the web interface using secure SSH connections.
    -   Upload and deploy strategy files, execute `start.sh` scripts, and stream execution logs directly within the browser.

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

### 2. Frontend (React/Vite)
Launch the user interface.

```bash
cd web_control/frontend
npm install
npm run dev
```

---

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

---

## 🎯 Next Milestones

We are continuously working to make the **INHA-Player Web Control Center** more robust and feature-rich. Here is what's coming next:

* **Game Lock Mechanism** To prevent accidental command inputs during critical match moments, a "Game Lock" mode will be implemented. Once activated, sensitive UI controls will be disabled to ensure system stability.

* **Control Authority Handover** Implementing a seamless transition system for joystick control. This will allow multiple operators to hand over control authority or switch between manual and autonomous modes without interruption.

* **Live Telemetry & Topic Visualization** Expanding data integration to subscribe to internal robot topics (ROS-based). This will allow for real-time visualization of sensor data, motor temperatures, and higher-level diagnostics directly in the browser.

* **Remote Configuration Editor** A built-in JSON/YAML editor to modify robot parameters and strategy configurations on-the-fly without needing to manually edit files via SSH terminal.

### 🚀 Core Vision & Philosophy
We are architecting this platform to be a truly scalable ecosystem, bridging the gap between complex robotic logic and intuitive control.</br>
**Stay tuned as we continue to redefine the boundaries of Human-Robot Interaction.**
