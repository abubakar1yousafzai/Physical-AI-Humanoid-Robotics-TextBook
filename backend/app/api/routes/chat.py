from fastapi import APIRouter, Depends
from app.models.api import ChatRequest, ChatResponse
from app.services.rag import process_chat
from app.api.middleware import rate_limiter

router = APIRouter()

@router.post("/chat", response_model=ChatResponse, dependencies=[Depends(rate_limiter)])
def chat_endpoint(request: ChatRequest):
    return process_chat(request)