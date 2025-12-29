# Tasks: RAG Chatbot Backend API

**Feature**: RAG Chatbot Backend API
**Status**: Implementation
**Phase**: Final (Polish)

## Implementation Strategy

We will follow a phased approach strictly adhering to the "Core RAG First" strategy defined in the plan.
1. **Setup & Foundation**: Initialize the FastAPI project and external clients (OpenAI-Gemini, Qdrant).
2. **Phase 1 (MVP)**: Implement Health Check (US2) and RAG Chat (US1) endpoints using in-memory logic.
3. **Phase 2 (Persistence)**: Add Neon Postgres integration for Chat History (US3).

## Dependencies

- **US2 (Health Check)**: Depends on Project Setup.
- **US1 (RAG Chat)**: Depends on US2 (Foundation), OpenAI Client, Qdrant Client.
- **US3 (Persistence)**: Depends on US1 (Chat Logic), DB Setup.

## Phase 1: Setup

**Goal**: Initialize the project structure and configure dependencies.

- [x] T001 Create FastAPI project structure (app/api, app/core, app/services) in `backend/`
- [x] T002 Define dependencies in `backend/pyproject.toml` (fastapi, uvicorn, openai, qdrant-client, cohere, pydantic-settings)
- [x] T003 Create `.env` template in `backend/.env.example` with keys for GEMINI, COHERE, QDRANT
- [x] T004 Implement configuration loading with Pydantic in `backend/app/core/config.py`

## Phase 2: Foundational Components

**Goal**: Implement wrappers for external APIs to be used by User Stories.

- [x] T005 [P] Implement Qdrant client singleton in `backend/app/services/qdrant.py`
- [x] T006 [P] Implement OpenAI client configured for Gemini base URL in `backend/app/services/gemini.py`
- [x] T007 [P] Implement global exception handler in `backend/app/core/errors.py`

## Phase 3: User Story 2 - API Health Check (P1)

**Goal**: Ensure the system and its dependencies are reachable.
**Independent Test**: `GET /api/health` returns 200 OK with service status.

- [x] T008 [US2] Define HealthResponse model in `backend/app/models/api.py`
- [x] T009 [US2] Implement health check logic (ping Qdrant/Gemini) in `backend/app/services/health.py`
- [x] T010 [US2] Create health endpoint in `backend/app/api/routes/health.py`
- [x] T011 [US2] Register health router in `backend/app/main.py`

## Phase 4: User Story 1 - RAG Chat Interaction (P1)

**Goal**: Enable users to ask questions and receive RAG-augmented answers.
**Independent Test**: `POST /api/chat` returns relevant answer with sources.

- [x] T012 [P] [US1] Define ChatRequest and ChatResponse models in `backend/app/models/api.py`
- [x] T013 [P] [US1] Implement retrieval logic (Embed -> Search) in `backend/app/services/rag.py`
- [x] T014 [US1] Implement generation logic (Context + Query -> Gemini) in `backend/app/services/rag.py`
- [x] T015 [US1] Combine retrieval and generation into `process_chat` function in `backend/app/services/rag.py`
- [x] T016 [US1] Create chat endpoint in `backend/app/api/routes/chat.py`
- [x] T017 [US1] Register chat router in `backend/app/main.py`
- [x] T018 [US1] Add basic rate limiting (15 RPM for Gemini) in `backend/app/api/middleware.py` (or dependency)

## Phase 5: User Story 3 - Persistent Chat History (P2)

**Goal**: Persist conversation history in Neon Postgres.
**Independent Test**: Send message with `thread_id`, verify retrieval on subsequent requests.

- [x] T019 [US3] Add database dependencies (sqlalchemy, asyncpg, alembic) to `backend/pyproject.toml`
- [x] T020 [US3] Configure database connection/session in `backend/app/db/session.py`
- [x] T021 [P] [US3] Define Conversation and Message SQLAlchemy models in `backend/app/models/sql.py`
- [x] T022 [US3] Initialize Alembic and generate initial migration in `backend/alembic/`
- [x] T023 [US3] Implement CRUD for history (create_thread, add_message, get_history) in `backend/app/crud/chat.py`
- [x] T024 [US3] Update `process_chat` in `backend/app/services/rag.py` to save interactions
- [x] T025 [US3] Update chat endpoint to accept/return `thread_id` and load history context

## Final Phase: Polish

**Goal**: Ensure code quality and documentation.

- [x] T026 Update `backend/README.md` with setup and usage instructions
- [x] T027 Run linter (ruff) and fix any style issues
- [x] T028 Verify all endpoints with final manual test pass
