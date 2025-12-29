from typing import Dict, Optional, List
from uuid import UUID
from pydantic import BaseModel, Field

class HealthResponse(BaseModel):
    status: str
    services: Dict[str, str]

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, description="The user's natural language query")
    thread_id: Optional[UUID] = Field(None, description="UUID for continuing a conversation (Phase 2)")

class ChatResponse(BaseModel):
    answer: str = Field(..., description="The generated response from the LLM")
    sources: List[str] = Field(default_factory=list, description="List of source chunks used for context")
    thread_id: Optional[UUID] = Field(None, description="The conversation ID")