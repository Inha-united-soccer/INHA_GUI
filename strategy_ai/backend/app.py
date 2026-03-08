"""
Strategy AI - FastAPI Server
Run: uvicorn app:app --reload --port 8001
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
from typing import Optional
import os

from strategy_analyzer import strategy_engine, robot_tracker, match_monitor, TCM_LOG_DIR
from db.tcm_db import get_db_status
from db.tcm_importer import import_all

app = FastAPI(title="INHA Strategy AI")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve frontend
FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "../frontend")
if os.path.exists(FRONTEND_DIR):
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

class MemoInput(BaseModel):
    opponent_name: Optional[str] = ""
    defense_style: Optional[str] = ""
    attack_pattern: Optional[str] = ""
    gk_number: Optional[int] = 0
    gk_aggression: Optional[str] = ""
    weakness: Optional[str] = ""
    notes: Optional[str] = ""

@app.get("/")
async def index():
    return FileResponse(os.path.join(FRONTEND_DIR, "index.html"))

@app.get("/api/live")
async def get_live():
    """Real-time snapshot: game state + robot positions."""
    gc_data, score_history = match_monitor.get_status()
    robots, heatmap       = robot_tracker.get_snapshot()
    opp_summary           = robot_tracker.get_opponent_summary()
    db_status = get_db_status()
    return {
        "gc": gc_data,
        "robots": list(robots.values()),
        "opponent_summary": opp_summary,
        "score_history": score_history,
        "db_sessions": db_status["total_matches"],
        "db_snapshots": db_status["total_snapshots"],
    }

@app.post("/api/analyze")
async def analyze(memo: MemoInput):
    """Run full AI analysis with current data + memo."""
    result = strategy_engine.analyze(memo.dict())
    return result

@app.get("/api/db/status")
async def db_status():
    """Return SQLite DB statistics."""
    return get_db_status()


@app.post("/api/db/import")
async def db_import(force: bool = False):
    """
    Trigger manual import of all TCM log files.
    Set force=true to re-import already-imported files.
    """
    counts = import_all(TCM_LOG_DIR, force=force)
    after  = get_db_status()
    return {
        "result": counts,
        "db_status": after,
    }
