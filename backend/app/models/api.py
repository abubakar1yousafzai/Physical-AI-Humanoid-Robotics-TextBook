from typing import Dict, Optional, List
from uuid import UUID
from pydantic import BaseModel, Field

class HealthResponse(BaseModel):
    status: str
    services: Dict[str, str]

class ChatRequest(BaseModel):
    message: str
    thread_id: Optional[str] = None
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str = Field(..., description="The generated response from the LLM")
    sources: List[str] = Field(default_factory=list, description="List of source chunks used for context")
    thread_id: Optional[str] = Field(None, description="The conversation ID")