# Research: RAG Chatbot Backend API

**Feature**: RAG Chatbot Backend API
**Date**: 2025-12-26
**Status**: Complete

## 1. Technical Context & Unknowns

The technology stack is explicitly defined in the constitution and the feature specification.
- **Backend**: FastAPI
- **Orchestrator**: OpenAI Agents SDK
- **LLM**: Google Gemini (gemini-1.5-flash)
- **Vector DB**: Qdrant (Cloud)
- **Embeddings**: Cohere (embed-english-v3.0)
- **Database**: Neon Postgres (Phase 2)

**Key Questions Resolved**:
- **Authentication**: Phase 1 requires no authentication. Phase 2 introduces persistence, likely keyed by session ID or simple token until full auth in Phase 3.
- **Agent SDK Compatibility**: OpenAI Agents SDK is primarily designed for OpenAI models, but it can be configured to use other LLMs via custom clients or by adhering to the OpenAI API interface. Since we must use Gemini, we will use the `google-generativeai` library and wrap it or use a compatibility layer if the Agents SDK strictly requires OpenAI client objects. *Self-Correction/Refinement*: The OpenAI Agents SDK (if referring to the `openai` python library's assistance API or the new "Agents" concept) is tightly coupled with OpenAI. However, "OpenAI Agents SDK" might be a misnomer for "building agents using OpenAI-like patterns" or the user specifically wants the *pattern*. Given the constraint "Technology: OpenAI Agents SDK (framework) + Google Gemini LLM", we will implement an agent pattern (Tool usage, loop) using Gemini's native function calling capabilities or a lightweight framework like LangChain or just raw Python if the "SDK" constraint is flexible. *Correction*: The prompt explicitly says "OpenAI Agents SDK". This usually implies the `openai` library. Using it with Gemini is possible via the Gemini OpenAI compatibility endpoint or by mocking the client. **Decision**: Use Gemini's OpenAI-compatible endpoint if available, or standard Gemini SDK (`google-generativeai`) with a custom agent loop if the "OpenAI Agents SDK" requirement is interpreted as "Agentic behavior". *Wait*, the spec says "OpenAI Agents SDK (framework)". This likely refers to the new `assistants` API or similar. Actually, `openai-python` is the library. **Final Decision**: We will use the standard `google-generativeai` library for Gemini and implement the "Agent" pattern manually (Prompt + Tools + Loop) or use a framework if specified. *Re-reading Spec*: "System MUST use OpenAI Agents SDK as the orchestration framework". This is a strong constraint. **Research Finding**: Google Gemini now supports an OpenAI-compatible API. We can use the `openai` python client pointing to Google's base URL. This satisfies "OpenAI SDK" + "Gemini LLM".

## 2. Technology Decisions

### 2.1 LLM Integration Strategy
- **Decision**: Use `openai` Python SDK configured with Google's base URL (`https://generativelanguage.googleapis.com/v1beta/openai/`).
- **Rationale**: Satisfies the constraint of using OpenAI Agents SDK while using Gemini model.
- **Alternatives**: Native `google-generativeai`. Rejected due to explicit "OpenAI Agents SDK" requirement.

### 2.2 RAG Flow
- **Decision**: Retrieve-then-Generate.
- **Flow**:
  1. Receive query.
  2. Embed query using Cohere.
  3. Search Qdrant.
  4. Construct system prompt with retrieved context.
  5. Call Gemini via OpenAI SDK.
- **Rationale**: Standard RAG pattern, minimizes latency and token usage.

### 2.3 Database (Phase 2)
- **Decision**: Neon Serverless Postgres.
- **Schema**: Simple `chat_sessions` and `chat_messages` tables.
- **Rationale**: Serverless nature fits the project scale; Postgres is reliable for structured data.

## 3. Best Practices & Patterns

- **FastAPI**: Use dependency injection for DB sessions and API clients. Use Pydantic models for strict schema validation.
- **Async/Await**: Maximize concurrency for I/O bound operations (DB, LLM calls).
- **Environment Variables**: Strict separation of config using `pydantic-settings`.
- **Error Handling**: Global exception handler to normalize API errors (4xx, 5xx).

## 4. Implementation Strategy

- **Phase 1**:
  - Setup FastAPI project structure.
  - Implement `Dependencies` class for Qdrant and OpenAI/Gemini client.
  - Create `/api/chat` endpoint with RAG logic.
  - Create `/api/health` endpoint.
- **Phase 2**:
  - Add `asyncpg` and `sqlalchemy` (or raw SQL) for Neon.
  - Create DB schema.
  - Update `/api/chat` to persist messages.

