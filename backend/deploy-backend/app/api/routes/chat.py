from fastapi import APIRouter, Depends
from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from app.models.api import ChatRequest, ChatResponse
from app.services.rag import process_chat
from app.api.middleware import rate_limiter
from app.db.session import get_db
from app.api.routes.auth import fastapi_users
from app.models.user import User

router = APIRouter()

current_user_optional = fastapi_users.current_user(optional=True)

@router.post("/chat", response_model=ChatResponse, dependencies=[Depends(rate_limiter)])
async def chat_endpoint(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db),
    user: Optional[User] = Depends(current_user_optional)
):
    return await process_chat(request, db, user)

@router.get("/history")
async def chat_history_endpoint(
    db: AsyncSession = Depends(get_db),
    user: User = Depends(fastapi_users.current_user())
):
    """Retrieve chat history for the logged-in user."""
    conversations = await chat_crud.get_user_conversations(db, user.id)
    
    history_data = []
    for conv in conversations:
        messages = await chat_crud.get_messages(db, conv.id)
        first_user_msg = next((m.content for m in messages if m.role == "user"), "New Conversation")
        last_msg = messages[-1].content if messages else ""
        
        history_data.append({
            "id": conv.id,
            "title": first_user_msg[:30] + ("..." if len(first_user_msg) > 30 else ""),
            "preview": last_msg[:50] + ("..." if len(last_msg) > 50 else ""),
            "timestamp": conv.created_at.timestamp() * 1000
        })
        
    return history_data