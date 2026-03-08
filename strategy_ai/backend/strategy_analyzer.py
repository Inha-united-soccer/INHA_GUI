"""
RoboCup AI Strategy Analyzer
- Port 3838: GameController data (game state, team info, score, penalties)
- Port 3939: Robot return data (positions of all robots on field)
- DB : TCM log files → SQLite (past performance, similar-situation SQL query)
- LLM: Gemini for real-time strategy recommendations
"""

import socket
import struct
import threading
import time
import json
import math
import os
import glob
from collections import defaultdict
from datetime import datetime

# ── Gemini ──────────────────────────────────────────────────────────────────
from google import genai
from google.genai import types as genai_types

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")

# ── Constants ────────────────────────────────────────────────────────────────
GC_PORT      = 3838
RETURN_PORT  = 3939
OUR_TEAM_NUM = 13
FIELD_LENGTH  = 9000   # mm (Middle Advanced)
FIELD_WIDTH   = 6000   # mm

# ── Robot Position Tracker ────────────────────────────────────────────────────
class RobotTracker:
    """Tracks all robot positions received via port 3939 (GameControlReturnData)."""

    def __init__(self):
        self.robots = {}        # { "team_player": {...} }
        self.heatmap = defaultdict(list)  # { team_num: [(x,y), ...] }
        self.lock = threading.Lock()
        self.sock = None
        self._start()

    def _start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                pass
            self.sock.settimeout(0.5)
            self.sock.bind(('', RETURN_PORT))
            print(f"[RobotTracker] Listening on port {RETURN_PORT}")
        except Exception as e:
            print(f"[RobotTracker] Bind error: {e}")
            self.sock = None

        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def _loop(self):
        if not self.sock:
            return
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                self._parse(data, addr[0])
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[RobotTracker] Error: {e}")
                time.sleep(1)

    def _parse(self, data, ip):
        # RoboCupGameControlReturnData: 4s B B B B 3f f 2f = 32 bytes
        if len(data) < 32:
            return
        try:
            header = data[0:4]
            if header != b'RGrt':
                return
            version, player_num, team_num, fallen = struct.unpack("BBBB", data[4:8])
            if version != 4:
                return
            x, y, theta = struct.unpack("<fff", data[8:20])
            ball_age = struct.unpack("<f", data[20:24])[0]
            ball_x, ball_y = struct.unpack("<ff", data[24:32])

            key = f"{team_num}_{player_num}"
            with self.lock:
                self.robots[key] = {
                    "team": team_num,
                    "player": player_num,
                    "x": x, "y": y, "theta": theta,
                    "fallen": bool(fallen),
                    "ball_age": ball_age,
                    "ball_x": ball_x, "ball_y": ball_y,
                    "ip": ip,
                    "ts": time.time(),
                    "is_ours": (team_num == OUR_TEAM_NUM)
                }
                self.heatmap[team_num].append((x, y))
                # keep last 500 positions per team
                if len(self.heatmap[team_num]) > 500:
                    self.heatmap[team_num] = self.heatmap[team_num][-500:]
        except Exception as e:
            print(f"[RobotTracker] Parse error: {e}")

    def get_snapshot(self):
        """Return current robot states, pruning stale ones (>5s)."""
        now = time.time()
        with self.lock:
            active = {k: v for k, v in self.robots.items() if now - v["ts"] < 5.0}
            self.robots = active
            return dict(active), dict(self.heatmap)

    def get_opponent_summary(self):
        """Analyze opponent positions to detect formation."""
        snapshot, heatmap = self.get_snapshot()
        opponents = [v for v in snapshot.values() if not v["is_ours"]]
        if not opponents:
            return {"formation": "unknown", "avg_x": 0, "avg_y": 0, "fallen_count": 0, "count": 0}

        avg_x    = sum(r["x"] for r in opponents) / len(opponents)
        avg_y    = sum(r["y"] for r in opponents) / len(opponents)
        fallen   = sum(1 for r in opponents if r["fallen"])
        # Formation heuristic: if avg_x > 0 → attacking, < −2000 → defending
        if avg_x > 1500:
            formation = "공격 집중"
        elif avg_x < -1500:
            formation = "수비 집중"
        else:
            formation = "균형"

        return {
            "formation": formation,
            "avg_x": round(avg_x),
            "avg_y": round(avg_y),
            "fallen_count": fallen,
            "count": len(opponents),
        }


# ── GC Monitor (reuse logic from gc_monitor.py) ──────────────────────────────
class MatchMonitor:
    """Listens on port 3838 for GameController SPL v18 packets."""

    def __init__(self):
        self.data = {"state": "UNKNOWN", "secsRemaining": 600, "teams": [], "half": 1}
        self.score_history = []
        self.penalty_history = []
        self.prev_score = [0, 0]
        self.lock = threading.Lock()
        self._start()

    def _start(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                pass
            sock.settimeout(0.5)
            sock.bind(('', GC_PORT))
            self.sock = sock
            print(f"[MatchMonitor] Listening on port {GC_PORT}")
        except Exception as e:
            print(f"[MatchMonitor] Bind error: {e}")
            self.sock = None

        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def _loop(self):
        if not self.sock:
            return
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                if len(data) >= 4 and data[0:4] == b'RGme' and data[4] in (15, 18):
                    self._parse_spl(data)
            except socket.timeout:
                pass
            except Exception as e:
                time.sleep(1)

    def _parse_spl(self, data):
        try:
            (_, version, pkt_num, players_per_team,
             comp_phase, comp_type, game_phase,
             state, set_play, first_half, kicking_team,
             secs_remaining, secondary_time) = struct.unpack("<4sBBBBBBBBBBhh", data[0:18])

            STATE_MAP = {0:"INITIAL",1:"READY",2:"SET",3:"PLAYING",4:"FINISHED",5:"STANDBY"}
            state_str = STATE_MAP.get(state, "UNKNOWN")

            offset = 18
            TEAM_SIZE = 10 + 20*2
            teams = []
            for i in range(2):
                if len(data) < offset + TEAM_SIZE:
                    break
                (team_num, fp_color, gk_color, gk_num, score, pen_shot,
                 single_shots, msg_budget) = struct.unpack("<6B2H", data[offset:offset+10])
                penalties = []
                for p in range(20):
                    pb = offset + 10 + p*2
                    pen, secs = struct.unpack("BB", data[pb:pb+2])
                    penalties.append({"penalty": pen, "secs": secs})
                teams.append({
                    "teamNumber": team_num,
                    "score": score,
                    "messageBudget": msg_budget,
                    "penalties": penalties,
                    "penaltyCount": sum(1 for p in penalties[:players_per_team] if p["penalty"] != 0)
                })
                offset += TEAM_SIZE

            with self.lock:
                # Track score changes
                for i, t in enumerate(teams):
                    if i < len(self.prev_score) and t["score"] != self.prev_score[i]:
                        self.score_history.append({
                            "time": secs_remaining,
                            "team": t["teamNumber"],
                            "score": t["score"],
                            "ts": time.time()
                        })
                        self.prev_score[i] = t["score"]

                self.data = {
                    "state": state_str,
                    "secsRemaining": secs_remaining,
                    "secondaryTime": secondary_time,
                    "half": 1 if first_half else 2,
                    "teams": teams,
                }
        except Exception:
            pass

    def get_status(self):
        with self.lock:
            return dict(self.data), list(self.score_history)


# ── DB Context Builder (SQL-based, replaces RAG) ─────────────────────────────
class TCMContextBuilder:
    """
    Queries the SQLite DB for past match situations similar to the current live
    state and formats a concise text block for the Gemini prompt.
    """

    def __init__(self):
        # Import here so the DB is already initialised by tcm_db
        from db.tcm_db import get_db_status, get_similar_situations
        self._get_status   = get_db_status
        self._get_similar  = get_similar_situations
        status = self._get_status()
        print(f"[DB] Connected — {status['total_matches']} matches, "
              f"{status['total_snapshots']} snapshots")

    def get_context(self, our_avg_x: float = 0.0,
                    score_diff: int = 0,
                    opponent_name: str = "") -> str:
        """
        Build a text context for the LLM from the SQLite DB.

        Args:
            our_avg_x     — average X of our robots this moment (mm)
            score_diff    — our_score - opp_score
            opponent_name — name string for header
        """
        status  = self._get_status()
        similar = self._get_similar(our_avg_x=our_avg_x, score_diff=score_diff, limit=5)

        lines = ["[우리팀 과거 경기 DB 분석]"]
        lines.append(f"총 {status['total_matches']}경기 / "
                     f"{status['total_snapshots']:,}개 로봇 위치 스냅샷 보유")

        if similar:
            lines.append(f"\n현재 상황 유사 과거 경기 (우리팀 평균X={our_avg_x:.0f}mm 기준):")
            for r in similar:
                lines.append(
                    f"  • {r.get('opponent','?')} {r.get('half','?')} │ "
                    f"포메이션={r.get('formation','?')} │ "
                    f"우리avg_x={r.get('our_avg_x','?')}mm │ "
                    f"공avg_x={r.get('ball_avg_x','?')}mm │ "
                    f"낙상비율={r.get('fallen_pct','?')}%"
                )
        else:
            lines.append("\n현재 상황과 유사한 과거 기록 없음 (DB 데이터 부족)")

        if opponent_name:
            lines.append(f"\n※ 상대팀: {opponent_name}")

        return "\n".join(lines)


# ── AI Strategy Engine ────────────────────────────────────────────────────────
class StrategyEngine:
    """Combines live data + DB context + manual memo → Gemini strategy."""

    def __init__(self, ctx: TCMContextBuilder, tracker: RobotTracker, monitor: MatchMonitor):
        self.ctx     = ctx
        self.tracker = tracker
        self.monitor = monitor
        self.client = None
        if GEMINI_API_KEY:
            self.client = genai.Client(api_key=GEMINI_API_KEY)
            print("[AI] Gemini 2.0 Flash ready")
        else:
            print("[AI] No GEMINI_API_KEY — running in demo mode")

    def analyze(self, memo: dict, language: str = "ko") -> dict:
        """
        memo = {
          "opponent_name": str,
          "defense_style": str,
          "attack_pattern": str,
          "gk_number": int,
          "gk_aggression": str,
          "weakness": str,
          "notes": str,
        }
        """
        gc_data, score_history = self.monitor.get_status()
        opp_summary            = self.tracker.get_opponent_summary()

        # Compute live our-team avg_x from robot tracker
        snapshot, _   = self.tracker.get_snapshot()
        our_robots    = [v for v in snapshot.values() if v.get("is_ours")]
        our_avg_x     = (sum(r["x"] for r in our_robots) / len(our_robots)
                         if our_robots else 0.0)

        # Score diff
        teams      = gc_data.get("teams", [])
        our_team   = next((t for t in teams if t.get("teamNumber") == OUR_TEAM_NUM), {})
        opp_team   = next((t for t in teams if t.get("teamNumber") != OUR_TEAM_NUM), {})
        score_diff = our_team.get("score", 0) - opp_team.get("score", 0)

        db_context = self.ctx.get_context(
            our_avg_x=our_avg_x,
            score_diff=score_diff,
            opponent_name=memo.get("opponent_name", "")
        )

        prompt = self._build_prompt(gc_data, opp_summary, score_history, memo, db_context)

        if not self.client:
            return self._demo_response(gc_data, opp_summary, memo)

        try:
            resp = self.client.models.generate_content(
                model="gemini-2.0-flash",
                contents=prompt
            )
            text = resp.text
        except Exception as e:
            text = f"[AI 오류] {e}"

        return {
            "analysis": text,
            "gc_snapshot": gc_data,
            "opponent_summary": opp_summary,
            "score_history": score_history,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "memo": memo,
        }

    def _build_prompt(self, gc, opp, scores, memo, rag) -> str:
        our_team  = next((t for t in gc.get("teams", []) if t.get("teamNumber") == OUR_TEAM_NUM), {})
        opp_team  = next((t for t in gc.get("teams", []) if t.get("teamNumber") != OUR_TEAM_NUM), {})
        our_score = our_team.get("score", 0)
        opp_score = opp_team.get("score", 0)
        msg_left  = our_team.get("messageBudget", 12000)

        return f"""당신은 RoboCup 휴머노이드 리그 축구 AI 전략 코치입니다.
우리팀 번호: 13번 (INHA United)
현재 경기 상황을 분석하여 구체적이고 실행 가능한 전략을 제시하세요.

=== 현재 경기 상태 ===
• 게임 상태: {gc.get('state','?')}  │  전반/후반: {gc.get('half','?')}
• 남은 시간: {gc.get('secsRemaining','?')}초
• 스코어: 우리팀 {our_score} : {opp_score} 상대팀
• 우리팀 통신 예산 잔여: {msg_left}/12000
• 우리팀 패널티 수: {our_team.get('penaltyCount', 0)}
• 상대팀 패널티 수: {opp_team.get('penaltyCount', 0)}

=== 상대 로봇 실시간 분석 ===
• 감지된 상대 로봇: {opp.get('count', 0)}명
• 포메이션 패턴: {opp.get('formation', '분석중')}
• 평균 X 포지션: {opp.get('avg_x', 0)}mm (양수=공격진영)
• fallen 로봇 수: {opp.get('fallen_count', 0)}명

=== 코치 관찰 메모 ===
• 상대팀: {memo.get('opponent_name', '미상')}
• 수비 스타일: {memo.get('defense_style', '미입력')}
• 공격 패턴: {memo.get('attack_pattern', '미입력')}
• 상대 GK: {memo.get('gk_number', '?')}번  공격성: {memo.get('gk_aggression', '미입력')}
• 관찰된 약점: {memo.get('weakness', '없음')}
• 추가 메모: {memo.get('notes', '없음')}

=== 과거 경기 DB 유사 상황 ===
{rag}

=== 분석 요청 ===
위 정보를 종합하여 다음을 작성하세요:

1. **전황 요약** (2-3줄): 현재 경기 흐름과 유리/불리 판단
2. **상대팀 취약점** (2-3가지): 데이터 기반 구체적 약점
3. **즉시 적용 전략** (3-5가지): 지금 당장 실행할 수 있는 전술적 조정
4. **파라미터 조정 추천**: 로봇 파라미터 변경 제안 (예: kickDir, 수비 거리, 포지션)
5. **후반전 핵심 전략** (하프타임인 경우): 후반전을 위한 핵심 전략 변화

답변은 명확하고 실행 가능하게, 한국어로 작성하세요."""

    def _demo_response(self, gc, opp, memo) -> dict:
        """Demo mode when no API key is set."""
        return {
            "analysis": f"""**[데모 모드 - GEMINI_API_KEY 없음]**

**전황 요약**
현재 {gc.get('state','?')} 상태, 상대 포메이션: {opp.get('formation','분석중')}
상대 로봇 {opp.get('count',0)}명 감지, fallen {opp.get('fallen_count',0)}명

**즉시 적용 전략 (예시)**
1. 상대 평균 포지션이 {opp.get('avg_x',0)}mm → {'공격 압박' if opp.get('avg_x',0) > 0 else '수비 강화'}
2. 실전에서 Gemini API 키 설정 시 상세 분석 제공

GEMINI_API_KEY 환경변수를 설정하면 실제 AI 분석이 활성화됩니다.""",
            "gc_snapshot": gc,
            "opponent_summary": opp,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "memo": memo,
        }


# ── Singleton instances ───────────────────────────────────────────────────────
TCM_LOG_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "../../3D modeling/bin/logs_teamcomm"
))

robot_tracker   = RobotTracker()
match_monitor   = MatchMonitor()
tcm_ctx         = TCMContextBuilder()
strategy_engine = StrategyEngine(tcm_ctx, robot_tracker, match_monitor)

# Start background watcher for new TCM log files
try:
    from db.tcm_watcher import start_watcher
    start_watcher(TCM_LOG_DIR)
except Exception as _e:
    print(f"[watcher] Could not start: {_e}")
