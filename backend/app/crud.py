from datetime import datetime, timedelta, timezone

from sqlalchemy import or_
from sqlalchemy.orm import Session

from . import auth, models, schemas


def list_resources(
    db: Session,
    search: str | None = None,
    category: str | None = None,
    item_type: schemas.ResourceItemType | None = None,
    status: schemas.ResourceStatus | None = None,
) -> list[models.Resource]:
    query = db.query(models.Resource)
    if search:
        like = f"%{search}%"
        query = query.filter(
            or_(models.Resource.name.ilike(like), models.Resource.location.ilike(like))
        )
    if category:
        query = query.filter(models.Resource.category == category)
    if item_type:
        query = query.filter(models.Resource.item_type == item_type)
    resources = query.order_by(models.Resource.id).all()
    if status:
        resources = [r for r in resources if r.status == status]
    return resources


def get_resource(db: Session, resource_id: int) -> models.Resource | None:
    return db.get(models.Resource, resource_id)


def log_inventory_action(
    db: Session, resource_name: str, action: str, detail: str, username: str
) -> None:
    db.add(
        models.InventoryLog(
            resource_name=resource_name, action=action, detail=detail, username=username
        )
    )
    db.commit()


def list_inventory_logs(db: Session, limit: int = 100) -> list[models.InventoryLog]:
    return (
        db.query(models.InventoryLog)
        .order_by(models.InventoryLog.created_at.desc())
        .limit(limit)
        .all()
    )


def create_resource(
    db: Session, data: schemas.ResourceCreate, username: str
) -> models.Resource:
    resource = models.Resource(**data.model_dump())
    db.add(resource)
    db.commit()
    db.refresh(resource)
    log_inventory_action(db, resource.name, "create", "자원 등록", username)
    return resource


def update_resource(
    db: Session, resource: models.Resource, data: schemas.ResourceUpdate, username: str
) -> models.Resource:
    changes = data.model_dump(exclude_unset=True)
    for field, value in changes.items():
        setattr(resource, field, value)
    db.commit()
    db.refresh(resource)
    if changes:
        summary = ", ".join(f"{field}={value}" for field, value in changes.items())
        log_inventory_action(db, resource.name, "update", summary, username)
    return resource


def adjust_quantity(
    db: Session, resource: models.Resource, delta: int, username: str
) -> models.Resource:
    new_quantity = resource.quantity + delta
    if new_quantity < 0:
        raise ValueError("quantity cannot go below zero")
    before = resource.quantity
    resource.quantity = new_quantity
    db.commit()
    db.refresh(resource)
    log_inventory_action(
        db,
        resource.name,
        "adjust",
        f"수량 {before} → {new_quantity} ({'+' if delta >= 0 else ''}{delta})",
        username,
    )
    return resource


def delete_resource(db: Session, resource: models.Resource, username: str) -> None:
    name = resource.name
    db.delete(resource)
    db.commit()
    log_inventory_action(db, name, "delete", "자원 삭제", username)


def get_summary(db: Session) -> schemas.SummaryOut:
    resources = db.query(models.Resource).all()
    total = sum(r.quantity for r in resources)
    normal = sum(1 for r in resources if r.status == "normal")
    low = sum(1 for r in resources if r.status == "low")
    out = sum(1 for r in resources if r.status == "out")
    return schemas.SummaryOut(total=total, normal=normal, low=low, out=out)


def list_inspections(
    db: Session,
    result: str | None = None,
    product: str | None = None,
    limit: int = 50,
) -> list[models.InspectionResult]:
    query = db.query(models.InspectionResult)
    if result:
        query = query.filter(models.InspectionResult.result == result)
    if product:
        query = query.filter(models.InspectionResult.product.ilike(f"%{product}%"))
    return query.order_by(models.InspectionResult.created_at.desc()).limit(limit).all()


def get_qc_summary(db: Session) -> schemas.QcSummaryOut:
    inspections = db.query(models.InspectionResult).all()
    total = len(inspections)
    passed = sum(1 for i in inspections if i.result == "pass")
    failed = sum(1 for i in inspections if i.result == "fail")
    defect_rate = round(failed / total * 100, 1) if total else None
    return schemas.QcSummaryOut(total=total, passed=passed, failed=failed, defect_rate=defect_rate)


def get_qc_trend(db: Session, days: int = 7) -> list[schemas.QcTrendPoint]:
    since = datetime.now(timezone.utc) - timedelta(days=days - 1)
    inspections = (
        db.query(models.InspectionResult)
        .filter(models.InspectionResult.created_at >= since)
        .all()
    )
    buckets: dict[str, dict[str, int]] = {}
    for offset in range(days):
        day = (since + timedelta(days=offset)).date().isoformat()
        buckets[day] = {"passed": 0, "failed": 0}
    for inspection in inspections:
        day = inspection.created_at.date().isoformat()
        if day in buckets:
            buckets[day]["passed" if inspection.result == "pass" else "failed"] += 1
    return [
        schemas.QcTrendPoint(date=day, passed=counts["passed"], failed=counts["failed"])
        for day, counts in sorted(buckets.items())
    ]


def get_user_by_username(db: Session, username: str) -> models.User | None:
    return db.query(models.User).filter(models.User.username == username).first()


def list_users(db: Session) -> list[models.User]:
    return db.query(models.User).order_by(models.User.id).all()


def create_user(db: Session, data: schemas.UserCreate) -> models.User:
    user = models.User(
        username=data.username,
        password_hash=auth.hash_password(data.password),
        role=data.role,
    )
    db.add(user)
    db.commit()
    db.refresh(user)
    return user


def delete_user(db: Session, user: models.User) -> None:
    db.delete(user)
    db.commit()
