from fastapi import APIRouter, Depends, Body
from app.schemas.user import UserRead, UserUpdate
from app.core.auth import auth_backend
from fastapi_users import FastAPIUsers
from app.models.user import User
from app.models.progress import UserProgress
from app.models.bookmark import UserBookmark
from app.crud.user import get_user_manager
from app.db.session import get_db
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, delete
from datetime import datetime
import uuid

fastapi_users = FastAPIUsers[User, uuid.UUID](
    get_user_manager,
    [auth_backend],
)

router = APIRouter()

router.include_router(
    fastapi_users.get_users_router(UserRead, UserUpdate),
    tags=["users"],
)

@router.get("/me/progress")
async def get_my_progress(
    db: AsyncSession = Depends(get_db),
    user: User = Depends(fastapi_users.current_user())
):
    """Retrieve all reading progress for the current user."""
    result = await db.execute(
        select(UserProgress).where(UserProgress.user_id == user.id)
    )
    progress_list = result.scalars().all()
    return progress_list

@router.post("/me/progress")
async def update_my_progress(
    chapter_id: str = Body(..., embed=True),
    completed: bool = Body(..., embed=True),
    db: AsyncSession = Depends(get_db),
    user: User = Depends(fastapi_users.current_user())
):
    """Update progress for a specific chapter."""
    result = await db.execute(
        select(UserProgress).where(
            UserProgress.user_id == user.id,
            UserProgress.chapter_id == chapter_id
        )
    )
    db_progress = result.scalar_one_or_none()

    if db_progress:
        db_progress.completed = completed
        if completed:
            db_progress.completed_at = datetime.utcnow()
    else:
        db_progress = UserProgress(
            user_id=user.id,
            chapter_id=chapter_id,
            completed=completed,
            completed_at=datetime.utcnow() if completed else None
        )
        db.add(db_progress)

    await db.commit()
    await db.refresh(db_progress)
    return db_progress

@router.get("/me/bookmarks")
async def get_my_bookmarks(
    db: AsyncSession = Depends(get_db),
    user: User = Depends(fastapi_users.current_user())
):
    """Retrieve all bookmarks for the current user."""
    result = await db.execute(
        select(UserBookmark).where(UserBookmark.user_id == user.id)
    )
    return result.scalars().all()

@router.post("/me/bookmarks")
async def add_bookmark(
    chapter_id: str = Body(..., embed=True),
    db: AsyncSession = Depends(get_db),
    user: User = Depends(fastapi_users.current_user())
):
    """Add a new bookmark."""
    # Check if exists
    result = await db.execute(
        select(UserBookmark).where(
            UserBookmark.user_id == user.id,
            UserBookmark.chapter_id == chapter_id
        )
    )
    if result.scalar_one_or_none():
        return {"message": "Already bookmarked"}

    db_bookmark = UserBookmark(user_id=user.id, chapter_id=chapter_id)
    db.add(db_bookmark)
    await db.commit()
    await db.refresh(db_bookmark)
    return db_bookmark

@router.delete("/me/bookmarks/{chapter_id}")
async def remove_bookmark(
    chapter_id: str,
    db: AsyncSession = Depends(get_db),
    user: User = Depends(fastapi_users.current_user())
):
    """Remove a bookmark."""
    await db.execute(
        delete(UserBookmark).where(
            UserBookmark.user_id == user.id,
            UserBookmark.chapter_id == chapter_id
        )
    )
    await db.commit()
    return {"message": "Bookmark removed"}
