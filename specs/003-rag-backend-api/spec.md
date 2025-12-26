# Feature Specification: RAG Chatbot Backend API

**Feature Branch**: `003-rag-backend-api`  
**Created**: 2025-12-26  
**Status**: Draft  
**Input**: User description: "Generate spec.md for Spec-3: FastAPI Backend with OpenAI Agents SDK + Google Gemini Integration Follow the same professional structure as Spec-1 and Spec-2. Key points to include: - Feature: RAG Chatbot Backend API - Dependencies: Spec-1 and Spec-2 must be complete - Technology: OpenAI Agents SDK (framework) + Google Gemini LLM (free) - Implementation: Two phases (Phase 1: Core RAG without DB, Phase 2: Add Neon Postgres) - User stories in Given-When-Then format (BDD) - Functional requirements (FR-001 to FR-014) - Success criteria (measurable) - API endpoints: POST /api/chat, GET /api/health - Architecture: FastAPI → Agent → Qdrant Tool → Gemini → Response - Constraints: Gemini 15 RPM, Cohere 100 RPM, no auth required - Edge cases: no results, timeouts, rate limits Generate a complete, professional spec.md following Spec-Kit Plus standards."

## Dependencies

### Prerequisites
- **Spec-1 (Textbook Embedding)** - COMPLETE
- **Spec-2 (Retrieval Testing)** - COMPLETE
- Qdrant collection "physical-ai-textbook" with 93 chunks
- Existing `.env` with `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`

### New Requirements
- `GEMINI_API_KEY` (to be obtained from Google AI Studio)
- Neon Postgres account (Phase 2 only)

## Technology Stack

- **Backend**: FastAPI 0.104+
- **Agent Framework**: OpenAI Python SDK 1.12+ (configured for Gemini)
- **LLM**: Google Gemini API (gemini-1.5-flash) - FREE tier
- **Vector DB**: Qdrant Cloud (from Spec-1)
- **Embeddings**: Cohere embed-english-v3.0 (from Spec-1)
- **Database**: Neon Serverless Postgres (Phase 2)
- **Package Manager**: UV
- **Validation**: Pydantic v2

## User Scenarios & Testing *(mandatory)*

### User Story 1 - RAG Chat Interaction (Priority: P1)

As an API consumer, I want to send a natural language query to the backend so that I can receive an answer augmented by the knowledge base.

**Why this priority**: Core value proposition of the feature.

**Independent Test**: Can be tested by sending POST requests to `/api/chat` and verifying the response contains information from the knowledge base.

**Acceptance Scenarios**:

1. **Given** the RAG system is operational, **When** I send a POST request to `/api/chat` with a question about the textbook, **Then** I receive a 200 OK response containing a relevant answer based on the Qdrant knowledge base.
2. **Given** the knowledge base has relevant information, **When** I ask a specific question, **Then** the response cites or uses that information.
3. **Given** the backend is running, **When** I send a request, **Then** the system uses the OpenAI Agents SDK to orchestrate the flow between Qdrant and Gemini.

---

### User Story 2 - API Health Check (Priority: P1)

As a DevOps engineer or monitoring system, I want to check the status of the API so that I can ensure the service is up and running.

**Why this priority**: Essential for deployment and monitoring.

**Independent Test**: Call GET `/api/health`.

**Acceptance Scenarios**:

1. **Given** the API server is running, **When** I send a GET request to `/api/health`, **Then** I receive a 200 OK status code with a status message (e.g., `{"status": "ok"}`).

---

### User Story 3 - Persistent Chat History (Phase 2) (Priority: P2)

As a user, I want my chat history to be saved so that the conversation context is preserved across multiple interactions (if applicable for Phase 2).

**Why this priority**: Phase 2 requirement for "Add Neon Postgres".

**Independent Test**: Send multiple messages and verify persistence in the database (or simulate session).

**Acceptance Scenarios**:

1. **Given** Phase 2 is implemented, **When** I interact with the chat API, **Then** the interaction is logged/persisted in the Neon Postgres database.

### Edge Cases

- **No Results Found**: What happens when Qdrant returns no relevant chunks? (System should fall back to general LLM knowledge or state it doesn't know).
- **Rate Limits**: What happens when Gemini (15 RPM) or Cohere (100 RPM) limits are reached? (System should return a 429 Too Many Requests or handle internal backoff).
- **Timeouts**: What happens if Gemini takes too long to respond? (API should return a 504 Gateway Timeout or custom error).
- **Empty Input**: User sends empty string. (Validation error 400).
- **Service Down**: Qdrant or Gemini API is unreachable. (503 Service Unavailable).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST expose a `POST /api/chat` endpoint that accepts a JSON body with a `message` field.
- **FR-002**: System MUST expose a `GET /api/health` endpoint returning system status.
- **FR-003**: System MUST use **OpenAI Agents SDK** as the orchestration framework.
- **FR-004**: System MUST use **Google Gemini** (Free tier) as the LLM for generating responses.
- **FR-005**: System MUST use **Qdrant** (via tool/integration) to retrieve relevant context for RAG.
- **FR-006**: System MUST use **FastAPI** as the web server framework.
- **FR-007**: System MUST NOT require authentication for these endpoints (Public API).
- **FR-008**: System MUST respect the **Gemini Rate Limit** of 15 Requests Per Minute (RPM).
- **FR-009**: System MUST respect the **Cohere Rate Limit** of 100 RPM (if Cohere is used for reranking or embedding).
- **FR-010**: System MUST implement **Phase 1**: Core RAG functionality without persistent database (Postgres).
- **FR-011**: System MUST implement **Phase 2**: Integration with **Neon Postgres** database (likely for chat logs or history).
- **FR-012**: System MUST handle "No results" from vector search gracefully (e.g., by answering based on general knowledge or informing the user).
- **FR-013**: System MUST return appropriate HTTP error codes for client errors (400, 429) and server errors (500, 503).
- **FR-014**: System MUST be configured via environment variables (API keys, DB URLs).

### Key Entities

- **Agent**: The OpenAI Agents SDK entity that manages the conversation flow and tools.
- **Tool**: The interface to Qdrant for vector search.
- **ChatMessage**: The input structure (text).
- **ChatResponse**: The output structure (text, potentially sources).
- **Database (Phase 2)**: Neon Postgres for storage.

## Technical Architecture

### Component Flow
```
User Request 
  │
  ▼
FastAPI Endpoint (/api/chat)
  │
  ▼
OpenAI Agent (Orchestrator)
  │
  ├─► Tool: Qdrant Retrieval
  │      │
  │      ▼
  │   Cohere Embeddings
  │      │
  │      ▼
  │   Qdrant Cloud (Knowledge Base)
  │
  ▼
LLM Generation (Google Gemini)
  │
  ▼
Response Construction
```

### Component Breakdown
1. **FastAPI**: Handles HTTP requests, validation (Pydantic), and error handling.
2. **OpenAI Agents SDK**: Manages the conversation state and calls the retrieval tool when necessary.
3. **Qdrant Tool**: A specific function/tool exposed to the agent that queries the vector database.
4. **Google Gemini**: The language model that receives the user query + retrieved context to generate the answer.
5. **Neon Postgres (Phase 2)**: Persists chat logs and session history.

### Request/Response Lifecycle
1. User POSTs a JSON message to `/api/chat`.
2. FastAPI validates the schema.
3. Agent receives the message and decides if retrieval is needed.
4. If needed, Agent calls the Qdrant tool.
5. Tool embeds the query (via Cohere) and searches Qdrant.
6. Relevant chunks are returned to the Agent.
7. Agent constructs a prompt with context and sends it to Gemini.
8. Gemini generates a response.
9. FastAPI returns the response to the user.

## API Endpoints

### POST /api/chat

- **Description**: Primary endpoint for RAG-based chat.
- **Request Body**:
  ```json
  { 
    "message": "What is Physical AI?", 
    "thread_id": null 
  }
  ```
- **Response**:
  ```json
  { 
    "answer": "Physical AI refers to...", 
    "sources": ["Page 12: Chapter 1..."], 
    "thread_id": "uuid-string" 
  }
  ```
- **Status Codes**:
  - `200 OK`: Successful response.
  - `400 Bad Request`: Invalid input format.
  - `429 Too Many Requests`: Rate limit exceeded (Gemini/Cohere).
  - `503 Service Unavailable`: Upstream service failure.

### GET /api/health

- **Description**: System health check.
- **Response**:
  ```json
  { 
    "status": "healthy", 
    "services": {
      "qdrant": "connected", 
      "gemini": "configured"
    } 
  }
  ```
- **Status Codes**:
  - `200 OK`: All systems nominal.
  - `503 Service Unavailable`: Critical dependency down.

## Implementation Phases

### Phase 1: Core RAG (No Database)
- **Estimated time**: 4-5 hours
- **Deliverables**: 
  - Working `/api/chat` endpoint.
  - Integration with Qdrant (read-only) and Gemini.
  - Basic error handling and rate limiting.
  - No persistent history.

### Phase 2: Database Integration
- **Estimated time**: 2-3 hours
- **Deliverables**:
  - Integration with Neon Serverless Postgres.
  - Persistence of chat history.
  - `thread_id` support for continuing conversations.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: API responds to valid `POST /api/chat` requests with a relevant answer.
- **SC-002**: System successfully handles valid requests under the rate limit (15 RPM) without errors.
- **SC-003**: System returns `429` or handles throttling when request rate exceeds 15 RPM (Gemini) or 100 RPM (Cohere).
- **SC-004**: `GET /api/health` returns 200 OK when system is operational.
- **SC-005**: In Phase 2, chat interactions are successfully persisted to long-term storage.
- **SC-006**: Automated tests cover primary workflows with >80% pass rate.
