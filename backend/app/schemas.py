from datetime import datetime
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

ResourceStatus = Literal["normal", "low", "out"]
ResourceItemType = Literal["material", "product"]
UserRole = Literal["admin", "worker"]
InspectionResultValue = Literal["pass", "fail"]


class ResourceBase(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    item_type: ResourceItemType = "material"
    category: str = Field(default="", max_length=50)
    quantity: int = Field(default=0, ge=0)
    unit: str = Field(default="EA", max_length=20)
    min_quantity: int = Field(default=0, ge=0)
    location: str = Field(default="", max_length=100)


class ResourceCreate(ResourceBase):
    pass


class ResourceUpdate(BaseModel):
    name: str | None = Field(default=None, min_length=1, max_length=100)
    item_type: ResourceItemType | None = None
    category: str | None = Field(default=None, max_length=50)
    quantity: int | None = Field(default=None, ge=0)
    unit: str | None = Field(default=None, max_length=20)
    min_quantity: int | None = Field(default=None, ge=0)
    location: str | None = Field(default=None, max_length=100)


class QuantityAdjust(BaseModel):
    delta: int


class ResourceOut(ResourceBase):
    model_config = ConfigDict(from_attributes=True)

    id: int
    status: ResourceStatus
    created_at: datetime
    updated_at: datetime


class SummaryOut(BaseModel):
    total: int
    normal: int
    low: int
    out: int


class InventoryLogOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    resource_name: str
    action: str
    detail: str
    username: str
    created_at: datetime


class InspectionOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    product: str
    result: InspectionResultValue
    defect_location: str | None
    map_data: str | None = None
    created_at: datetime


class QcSummaryOut(BaseModel):
    total: int
    passed: int
    failed: int
    defect_rate: float | None


class QcTrendPoint(BaseModel):
    date: str
    passed: int
    failed: int


class UserCreate(BaseModel):
    username: str = Field(min_length=3, max_length=50)
    password: str = Field(min_length=6, max_length=100)
    role: UserRole = "worker"


class UserOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    username: str
    role: UserRole
    created_at: datetime


class LoginRequest(BaseModel):
    username: str
    password: str


class Token(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: UserOut
