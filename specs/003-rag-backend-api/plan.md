# Implementation Plan - RAG Chatbot Backend API

## Technical Context

- **Language**: Python 3.10+
- **Framework**: FastAPI
- **Database**: Neon Serverless Postgres (Phase 2)
- **Project Type**: Backend API
- **Key Libraries**:
  - `fastapi`, `uvicorn`: Web server
  - `openai`: Client for Gemini (via compatible endpoint)
  - `qdrant-client`: Vector DB access
  - `cohere`: Embeddings
  - `pydantic`: Validation
  - `sqlalchemy`, `asyncpg`: Database ORM/Driver (Phase 2)

## Constitution Check

### Compliance
- **Principle 3 (Phased Development)**: This plan strictly follows the Phase 1 and Phase 2 split defined in the Spec and Constitution.
- **Principle 5 (Technical Accuracy)**: All code will be tested, and APIs are typed using Pydantic.
- **Principle 7 (Performance)**: Async/await will be used for all I/O operations (LLM, DB, Vector Search) to ensure high concurrency.
- **Standard (Code)**: Python code will be linted (Ruff/Black) and typed.

### Gates
- [x] **Spec Complete**: `spec.md` is detailed and validated.
- [x] **Research Complete**: `research.md` confirms tech stack and integration patterns.
- [x] **Constitution Aligned**: Phasing and tech stack match the constitution.

## Phase 1: Core RAG Implementation

### P1.1: Project Setup & Dependencies
- **Goal**: Initialize backend structure and install libraries.
- **Files**: `backend/pyproject.toml` (or `requirements.txt`), `backend/.env`
- **Steps**:
  1. Define dependencies in `pyproject.toml`.
  2. Create standard FastAPI directory structure (`app/main.py`, `app/api/`, `app/core/`).
  3. Configure `pydantic-settings` for `.env` loading.

### P1.2: External Clients & Utilities
- **Goal**: implement wrappers for external APIs.
- **Files**: `app/core/config.py`, `app/services/qdrant.py`, `app/services/gemini.py`
- **Steps**:
  1. Implement `get_settings()` for config.
  2. Create Qdrant client singleton.
  3. Create OpenAI client singleton (configured for Gemini).

### P1.3: Core RAG Logic
- **Goal**: Implement the retrieval and generation loop.
- **Files**: `app/services/rag.py`
- **Steps**:
  1. Define `search_knowledge_base(query)` function (Embed -> Search Qdrant).
  2. Define `generate_answer(query, context)` function using Gemini.
  3. Combine into `process_chat(message)` function.

### P1.4: API Endpoints
- **Goal**: Expose functionality via HTTP.
- **Files**: `app/api/routes/chat.py`, `app/api/routes/health.py`, `app/main.py`
- **Steps**:
  1. Implement `GET /api/health` checking service connectivity.
  2. Implement `POST /api/chat` invoking `process_chat`.
  3. Add error handling and rate limiting middleware.

## Phase 2: Database & Persistence

### P2.1: Database Setup
- **Goal**: Connect to Neon Postgres.
- **Files**: `app/db/session.py`, `app/db/base.py`
- **Steps**:
  1. Configure `asyncpg` and `SQLAlchemy` engine.
  2. Create declarative base.

### P2.2: Data Models & Migrations
- **Goal**: Define schema for chat history.
- **Files**: `app/models/chat.py`, `alembic/*`
- **Steps**:
  1. Define `Conversation` and `Message` models.
  2. Set up Alembic.
  3. Generate and run initial migration.

### P2.3: Integrate Persistence
- **Goal**: Save chats to DB.
- **Files**: `app/services/chat_history.py`, `app/api/routes/chat.py`
- **Steps**:
  1. Create CRUD functions for creating threads and adding messages.
  2. Update `POST /api/chat` to:
     - Load history if `thread_id` provided.
     - Save user message.
     - Save assistant response.
     - Return `thread_id`.

## Verification Plan

### Automated Tests
- **Unit Tests**: Test RAG logic with mocked Qdrant/Gemini.
- **Integration Tests**: Test API endpoints with TestClient.

### Manual Verification
- **Health Check**: `curl http://localhost:8000/api/health` -> 200 OK.
- **Chat Flow**: Send "What is Physical AI?" -> Receive valid answer with sources.
- **Persistence**: Send message with `thread_id` -> Verify history is maintained.