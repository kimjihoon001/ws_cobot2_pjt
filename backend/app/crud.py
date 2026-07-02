from sqlalchemy import or_
from sqlalchemy.orm import Session

from . import auth, models, schemas


def list_resources(
    db: Session,
    search: str | None = None,
    category: str | None = None,
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
    resources = query.order_by(models.Resource.id).all()
    if status:
        resources = [r for r in resources if r.status == status]
    return resources


def get_resource(db: Session, resource_id: int) -> models.Resource | None:
    return db.get(models.Resource, resource_id)


def create_resource(db: Session, data: schemas.ResourceCreate) -> models.Resource:
    resource = models.Resource(**data.model_dump())
    db.add(resource)
    db.commit()
    db.refresh(resource)
    return resource


def update_resource(
    db: Session, resource: models.Resource, data: schemas.ResourceUpdate
) -> models.Resource:
    for field, value in data.model_dump(exclude_unset=True).items():
        setattr(resource, field, value)
    db.commit()
    db.refresh(resource)
    return resource


def adjust_quantity(db: Session, resource: models.Resource, delta: int) -> models.Resource:
    new_quantity = resource.quantity + delta
    if new_quantity < 0:
        raise ValueError("quantity cannot go below zero")
    resource.quantity = new_quantity
    db.commit()
    db.refresh(resource)
    return resource


def delete_resource(db: Session, resource: models.Resource) -> None:
    db.delete(resource)
    db.commit()


def get_summary(db: Session) -> schemas.SummaryOut:
    resources = db.query(models.Resource).all()
    total = len(resources)
    normal = sum(1 for r in resources if r.status == "normal")
    low = sum(1 for r in resources if r.status == "low")
    out = sum(1 for r in resources if r.status == "out")
    return schemas.SummaryOut(total=total, normal=normal, low=low, out=out)


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
