from datetime import datetime, timezone

from sqlalchemy import DateTime, Integer, String
from sqlalchemy.orm import Mapped, mapped_column

from .database import Base


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class Resource(Base):
    __tablename__ = "resources"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    name: Mapped[str] = mapped_column(String(100), nullable=False)
    item_type: Mapped[str] = mapped_column(String(20), nullable=False, default="material")
    category: Mapped[str] = mapped_column(String(50), nullable=False, default="")
    quantity: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    unit: Mapped[str] = mapped_column(String(20), nullable=False, default="EA")
    min_quantity: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    location: Mapped[str] = mapped_column(String(100), nullable=False, default="")
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=_utcnow)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, onupdate=_utcnow
    )

    @property
    def status(self) -> str:
        if self.quantity <= 0:
            return "out"
        if self.quantity <= self.min_quantity:
            return "low"
        return "normal"


class InventoryLog(Base):
    __tablename__ = "inventory_logs"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    resource_name: Mapped[str] = mapped_column(String(100), nullable=False)
    action: Mapped[str] = mapped_column(String(20), nullable=False)
    detail: Mapped[str] = mapped_column(String(255), nullable=False, default="")
    username: Mapped[str] = mapped_column(String(50), nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=_utcnow)


class InspectionResult(Base):
    __tablename__ = "inspection_results"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    product: Mapped[str] = mapped_column(String(100), nullable=False)
    result: Mapped[str] = mapped_column(String(10), nullable=False)
    defect_location: Mapped[str | None] = mapped_column(String(100), nullable=True)
    map_data: Mapped[str | None] = mapped_column(String, nullable=True)  # JSON string
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=_utcnow)


class VoiceConfirmRequest(Base):
    """음성/HMI 확인 요청 기록. voice_processing의 confirm_tools(픽업 전 "이 도구
    맞아?")와 tool_pick_yolo_target.py의 confirm_release(배송 후 "받으셨나요?")가
    둘 다 이 테이블에 pending row를 남기고, HMI는 여기서 폴링으로 읽어간다."""
    __tablename__ = "voice_confirm_requests"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    kind: Mapped[str] = mapped_column(String(20), nullable=False)  # "tool_confirm" | "release_confirm"
    tools: Mapped[str] = mapped_column(String(255), nullable=False, default="[]")  # JSON 인코딩된 리스트
    targets: Mapped[str] = mapped_column(String(255), nullable=False, default="[]")  # JSON 인코딩된 리스트
    status: Mapped[str] = mapped_column(String(20), nullable=False, default="pending")  # pending|confirmed|rejected
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=_utcnow)
    resolved_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)


class User(Base):
    __tablename__ = "users"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    username: Mapped[str] = mapped_column(String(50), unique=True, nullable=False)
    password_hash: Mapped[str] = mapped_column(String(255), nullable=False)
    role: Mapped[str] = mapped_column(String(20), nullable=False, default="worker")
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=_utcnow)
