# RAG Chatbot Backend

## Overview
FastAPI backend with OpenAI Agents SDK + Google Gemini for RAG chatbot.

## Tech Stack
- Backend: FastAPI
- Agent: OpenAI Agents SDK
- LLM: Google Gemini 1.5 Flash (FREE)
- Embeddings: Cohere embed-english-v3.0
- Vector DB: Qdrant Cloud
- Database: Neon Postgres
- Package Manager: UV

## Setup

### 1. Install Dependencies
```bash
uv pip install fastapi uvicorn openai-agents cohere qdrant-client pydantic-settings python-dotenv httpx sqlalchemy asyncpg alembic
```

### 2. Configure Environment
Create `.env`:
```ini
GEMINI_API_KEY=your_key
COHERE_API_KEY=your_key
QDRANT_URL=your_url
QDRANT_API_KEY=your_key
DATABASE_URL=postgresql://...
```

### 3. Run Migrations
```bash
uv run alembic upgrade head
```

### 4. Start Server
```bash
uvicorn app.main:app --reload
```

Server runs at: http://localhost:8000

## API Endpoints

### Health Check
```
GET /api/health
```

### Chat
```
POST /api/chat
Body: {
  "message": "What is Physical AI?",
  "thread_id": null
}
```

## Features
- ✅ RAG pipeline (Cohere + Qdrant + Gemini)
- ✅ Conversation persistence (Neon Postgres)
- ✅ Thread continuity
- ✅ Source citations
- ✅ Rate limiting (15 RPM)