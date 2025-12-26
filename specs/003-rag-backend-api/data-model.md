# Data Model: RAG Chatbot Backend API

## 1. API Models (Pydantic)

### Chat Request
**Description**: The payload sent by the client to initiate or continue a chat.
```python
class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, description="The user's natural language query")
    thread_id: Optional[UUID] = Field(None, description="UUID for continuing a conversation (Phase 2)")
```

### Chat Response
**Description**: The payload returned to the client with the LLM's answer and citations.
```python
class ChatResponse(BaseModel):
    answer: str = Field(..., description="The generated response from the LLM")
    sources: List[str] = Field(default_factory=list, description="List of source chunks used for context")
    thread_id: UUID = Field(..., description="The conversation ID")
```

### Health Response
**Description**: System status.
```python
class HealthResponse(BaseModel):
    status: str = Field(..., example="healthy")
    services: Dict[str, str] = Field(..., example={"qdrant": "connected", "gemini": "connected"})
```

## 2. Database Schema (Neon Postgres - Phase 2)

### Table: `conversations`
| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the conversation thread |
| created_at | TIMESTAMP | DEFAULT NOW() | When the conversation started |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last activity timestamp |

### Table: `messages`
| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique message ID |
| conversation_id | UUID | FOREIGN KEY (conversations.id) | Link to parent thread |
| role | VARCHAR(50) | NOT NULL, CHECK (role IN ('user', 'assistant', 'system')) | Who sent the message |
| content | TEXT | NOT NULL | The message text |
| created_at | TIMESTAMP | DEFAULT NOW() | When the message was sent |

## 3. Relationships
- One **Conversation** has many **Messages**.
- **Messages** are ordered by `created_at` within a `conversation_id`.
