import json

from fastapi import APIRouter, Depends
from pydantic import BaseModel
from sqlalchemy.orm import Session

from .. import models, ros_bridge
from ..database import get_db

router = APIRouter(prefix="/api/voice", tags=["voice"])


class RespondIn(BaseModel):
    confirmed: bool


def _row_to_dict(row: models.VoiceConfirmRequest) -> dict:
    return {
        "id": row.id,
        "kind": row.kind,
        "tools": json.loads(row.tools),
        "targets": json.loads(row.targets),
        "status": row.status,
    }


@router.get("/requests/pending")
def list_pending_requests(db: Session = Depends(get_db)):
    """HMI가 계속 폴링해서 읽는 주 데이터 소스 (DB 기준, WS 유실과 무관하게 항상 정확)."""
    rows = (
        db.query(models.VoiceConfirmRequest)
        .filter(models.VoiceConfirmRequest.status == "pending")
        .order_by(models.VoiceConfirmRequest.id)
        .all()
    )
    return {"requests": [_row_to_dict(r) for r in rows]}


@router.post("/requests/{request_id}/respond")
def respond_to_request(request_id: int, body: RespondIn):
    resolved = ros_bridge._bridge.resolve_by_id(request_id, body.confirmed) if ros_bridge._bridge else False
    return {"resolved": resolved}
