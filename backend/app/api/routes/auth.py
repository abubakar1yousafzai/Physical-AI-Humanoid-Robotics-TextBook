from fastapi import APIRouter
from app.core.auth import auth_backend
from app.schemas.user import UserCreate, UserRead
from app.core.config import settings

# We need to construct the FastAPIUsers object to get the router
# Ideally, this should be a shared instance or dependency, but typical usage allows this
from fastapi_users import FastAPIUsers
from app.models.user import User
from app.crud.user import get_user_manager
import uuid

# Construct FastAPIUsers (can also be done in core/auth.py if manager is available there, 
# but manager depends on DB so it's often better to do it where needed or in a dedicated dependency module)

fastapi_users = FastAPIUsers[User, uuid.UUID](
    get_user_manager,
    [auth_backend],
)

router = APIRouter()

# /auth/jwt/login
router.include_router(
    fastapi_users.get_auth_router(auth_backend),
    prefix="/jwt",
    tags=["auth"],
)

# /auth/register
router.include_router(
    fastapi_users.get_register_router(UserRead, UserCreate),
    tags=["auth"],
)
