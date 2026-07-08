from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from .. import auth, crud, models, schemas
from ..database import get_db

router = APIRouter(
    prefix="/api/resources", tags=["resources"], dependencies=[Depends(auth.get_current_user)]
)

admin_only = Depends(auth.require_role("admin"))


@router.get("/summary", response_model=schemas.SummaryOut)
def read_summary(db: Session = Depends(get_db)):
    return crud.get_summary(db)


@router.get("/logs", response_model=list[schemas.InventoryLogOut], dependencies=[admin_only])
def read_inventory_logs(limit: int = 100, db: Session = Depends(get_db)):
    return crud.list_inventory_logs(db, limit=limit)


@router.get("", response_model=list[schemas.ResourceOut])
def read_resources(
    search: str | None = None,
    category: str | None = None,
    item_type: schemas.ResourceItemType | None = None,
    status: schemas.ResourceStatus | None = None,
    db: Session = Depends(get_db),
):
    return crud.list_resources(
        db, search=search, category=category, item_type=item_type, status=status
    )


@router.post("", response_model=schemas.ResourceOut, status_code=201, dependencies=[admin_only])
def create_resource(
    data: schemas.ResourceCreate,
    db: Session = Depends(get_db),
    current_user: models.User = Depends(auth.get_current_user),
):
    return crud.create_resource(db, data, current_user.username)


@router.get("/{resource_id}", response_model=schemas.ResourceOut)
def read_resource(resource_id: int, db: Session = Depends(get_db)):
    resource = crud.get_resource(db, resource_id)
    if resource is None:
        raise HTTPException(status_code=404, detail="Resource not found")
    return resource


@router.put("/{resource_id}", response_model=schemas.ResourceOut, dependencies=[admin_only])
def update_resource(
    resource_id: int,
    data: schemas.ResourceUpdate,
    db: Session = Depends(get_db),
    current_user: models.User = Depends(auth.get_current_user),
):
    resource = crud.get_resource(db, resource_id)
    if resource is None:
        raise HTTPException(status_code=404, detail="Resource not found")
    return crud.update_resource(db, resource, data, current_user.username)


@router.patch("/{resource_id}/quantity", response_model=schemas.ResourceOut)
def adjust_quantity(
    resource_id: int,
    data: schemas.QuantityAdjust,
    db: Session = Depends(get_db),
    current_user: models.User = Depends(auth.get_current_user),
):
    resource = crud.get_resource(db, resource_id)
    if resource is None:
        raise HTTPException(status_code=404, detail="Resource not found")
    try:
        return crud.adjust_quantity(db, resource, data.delta, current_user.username)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.delete("/{resource_id}", status_code=204, dependencies=[admin_only])
def delete_resource(
    resource_id: int,
    db: Session = Depends(get_db),
    current_user: models.User = Depends(auth.get_current_user),
):
    resource = crud.get_resource(db, resource_id)
    if resource is None:
        raise HTTPException(status_code=404, detail="Resource not found")
    crud.delete_resource(db, resource, current_user.username)
