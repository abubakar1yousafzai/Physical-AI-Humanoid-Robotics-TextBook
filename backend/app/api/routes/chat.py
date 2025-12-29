from fastapi import APIRouter, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from app.models.api import ChatRequest, ChatResponse
from app.services.rag import process_chat
from app.api.middleware import rate_limiter
from app.db.session import get_db

router = APIRouter()

@router.post("/chat", response_model=ChatResponse, dependencies=[Depends(rate_limiter)])
async def chat_endpoint(
    request: ChatRequest, 
    db: AsyncSession = Depends(get_db)
):
    return await process_chat(request, db)
