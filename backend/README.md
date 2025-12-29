# RAG Chatbot Backend API

FastAPI-based backend for the Physical AI and Humanoid Robotics Textbook assistant. It provides endpoints for RAG-augmented chat, health monitoring, and persistent conversation history.

## Features

- **RAG Integration**: Uses Cohere for embeddings and Qdrant for vector search.
- **LLM Reasoning**: Powered by Google Gemini (via OpenAI-compatible SDK).
- **Persistent History**: Stores conversation threads in Neon Postgres (SQLAlchemy).
- **Health Checks**: Real-time status monitoring of external dependencies.
- **Rate Limiting**: Integrated middleware for API stability.

## Prerequisites

- Python 3.10+
- `uv` package manager
- API Keys for Gemini, Cohere, Qdrant, and a Postgres connection string.

## Setup

1.  **Install dependencies**:
    ```bash
    cd backend
    uv sync
    ```

2.  **Configure Environment**:
    Copy `.env.example` to `.env` and fill in the required variables:
    ```bash
    GEMINI_API_KEY=...
    COHERE_API_KEY=...
    QDRANT_URL=...
    QDRANT_API_KEY=...
    DATABASE_URL=postgresql+asyncpg://...
    ```

3.  **Run Migrations**:
    ```bash
    uv run alembic upgrade head
    ```

## Usage

### Running the API Server
```bash
uv run uvicorn app.main:app --reload
```

### Endpoints
- `GET /api/health`: Check service and dependency status.
- `POST /api/chat`: Send a question and get a RAG-augmented response.
  - Optional `thread_id` to continue a conversation.

### Textbook Ingestion (CLI)
To (re)index textbook content into Qdrant:
```bash
uv run python main.py --run
```

## Architecture

- **`app/api/`**: API routes and middleware.
- **`app/services/`**: Core logic (RAG, Gemini, Qdrant).
- **`app/models/`**: Pydantic and SQLAlchemy models.
- **`app/crud/`**: Database operations.
- **`alembic/`**: Database migration scripts.