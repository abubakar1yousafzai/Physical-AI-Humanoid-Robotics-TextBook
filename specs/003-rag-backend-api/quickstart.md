# Quickstart: RAG Chatbot Backend API

## Prerequisites

- Python 3.10+
- `uv` package manager (optional, but recommended)
- `docker` (for local Qdrant/Postgres if not using cloud)
- API Keys:
  - `GEMINI_API_KEY` (Google AI Studio)
  - `COHERE_API_KEY` (Cohere Platform)
  - `QDRANT_URL` & `QDRANT_API_KEY` (Qdrant Cloud)

## Setup

1. **Navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Install dependencies**:
   ```bash
   # Using uv (recommended)
   uv venv
   .venv\Scripts\activate
   uv pip install -r requirements.txt
   
   # Or standard pip
   python -m venv .venv
   .venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. **Configure Environment**:
   Create a `.env` file in the `backend` directory:
   ```ini
   GEMINI_API_KEY=your_gemini_key
   COHERE_API_KEY=your_cohere_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   # Phase 2
   DATABASE_URL=postgresql://user:pass@host/db
   ```

## Running the Server

1. **Start FastAPI**:
   ```bash
   fastapi dev main.py
   # Or
   uvicorn main:app --reload
   ```

2. **Verify Health**:
   Open `http://localhost:8000/api/health`

3. **Test Chat**:
   Open Swagger UI at `http://localhost:8000/docs` and try the `/api/chat` endpoint.

## Docker (Optional)

```bash
docker build -t rag-backend .
docker run -p 8000:8000 --env-file .env rag-backend
```
