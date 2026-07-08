from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from .. import auth, crud, models, schemas
from ..database import get_db

router = APIRouter(
    prefix="/api/users", tags=["users"], dependencies=[Depends(auth.require_role("admin"))]
)


@router.get("", response_model=list[schemas.UserOut])
def read_users(db: Session = Depends(get_db)):
    return crud.list_users(db)


@router.post("", response_model=schemas.UserOut, status_code=201)
def create_user(data: schemas.UserCreate, db: Session = Depends(get_db)):
    if crud.get_user_by_username(db, data.username) is not None:
        raise HTTPException(status_code=409, detail="이미 존재하는 아이디입니다.")
    return crud.create_user(db, data)


@router.delete("/{user_id}", status_code=204)
def delete_user(
    user_id: int,
    db: Session = Depends(get_db),
    current_user: models.User = Depends(auth.get_current_user),
):
    if user_id == current_user.id:
        raise HTTPException(status_code=400, detail="자기 자신은 삭제할 수 없습니다.")
    user = db.get(models.User, user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="User not found")
    crud.delete_user(db, user)
