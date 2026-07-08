from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session

from .. import auth, crud, schemas
from ..database import get_db

router = APIRouter(prefix="/api/qc", tags=["qc"], dependencies=[Depends(auth.get_current_user)])


@router.get("/summary", response_model=schemas.QcSummaryOut)
def read_qc_summary(db: Session = Depends(get_db)):
    return crud.get_qc_summary(db)


@router.get("/trend", response_model=list[schemas.QcTrendPoint])
def read_qc_trend(days: int = 7, db: Session = Depends(get_db)):
    return crud.get_qc_trend(db, days=days)


@router.get("", response_model=list[schemas.InspectionOut])
def read_inspections(
    result: schemas.InspectionResultValue | None = None,
    product: str | None = None,
    limit: int = 50,
    db: Session = Depends(get_db),
):
    return crud.list_inspections(db, result=result, product=product, limit=limit)
