from fastapi_users.db import SQLAlchemyBaseUserTableUUID
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy import String, DateTime
from sqlalchemy.sql import func
from datetime import datetime
from app.models.sql import Base

class User(SQLAlchemyBaseUserTableUUID, Base):
    __tablename__ = "users"

    # id, email, hashed_password, is_active, is_superuser, is_verified 
    # are provided by SQLAlchemyBaseUserTableUUID

    name: Mapped[str | None] = mapped_column(String(100), nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), server_default=func.now())
