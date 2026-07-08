from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import os

from .database import Base, engine
from .routers import auth, qc, resources, users

Base.metadata.create_all(bind=engine)

app = FastAPI(title="cobot2 HMI API")

# Ensure static directory exists
os.makedirs(os.path.join(os.path.dirname(__file__), "..", "static", "inspection_images"), exist_ok=True)
app.mount("/static", StaticFiles(directory=os.path.join(os.path.dirname(__file__), "..", "static")), name="static")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(auth.router)
app.include_router(users.router)
app.include_router(resources.router)
app.include_router(qc.router)


@app.get("/api/health")
def health_check():
    return {"status": "ok"}
