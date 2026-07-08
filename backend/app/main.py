import asyncio
import json
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .database import Base, engine
from .routers import auth, qc, resources, robot, users, voice
from . import ros_bridge

Base.metadata.create_all(bind=engine)


@asynccontextmanager
async def lifespan(app: FastAPI):
    loop = asyncio.get_event_loop()
    ros_bridge.start_bridge(loop)
    yield


app = FastAPI(title="cobot2 HMI API", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "http://127.0.0.1:5173",
        "http://localhost:5174",
        "http://127.0.0.1:5174",
    ],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(auth.router)
app.include_router(users.router)
app.include_router(resources.router)
app.include_router(qc.router)
app.include_router(voice.router)
app.include_router(robot.router)


@app.get("/api/health")
def health_check():
    return {"status": "ok"}


@app.get("/api/voice/pending")
def voice_pending():
    pending = ros_bridge._bridge.get_pending_payload() if ros_bridge._bridge else None
    return {"pending": pending}


@app.get("/api/voice/pending_release")
def voice_pending_release():
    pending = ros_bridge._bridge.get_pending_release_payload() if ros_bridge._bridge else None
    return {"pending": pending}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    ros_bridge._clients.add(ws)
    pending = ros_bridge._bridge.get_pending_payload() if ros_bridge._bridge else None
    if pending:
        await ws.send_text(json.dumps(pending, ensure_ascii=False))
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("cmd") == "confirm_response":
                ros_bridge._bridge.resolve_pending(bool(msg.get("confirmed", False)))
            elif msg.get("cmd") == "release_response":
                ros_bridge._bridge.resolve_pending_release(bool(msg.get("confirmed", False)))
    except WebSocketDisconnect:
        pass
    finally:
        ros_bridge._clients.discard(ws)
