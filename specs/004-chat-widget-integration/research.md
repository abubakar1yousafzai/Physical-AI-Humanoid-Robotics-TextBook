# Research: Chat Widget Integration

**Feature**: Chat Widget Integration (Spec-4)
**Status**: Complete
**Date**: 2025-12-29

## Key Decisions

### 1. Framework & Libraries
- **Decision**: Use React (built-in to Docusaurus) and standard `fetch` API.
- **Rationale**: Minimal bundle size impact. No need for heavy external libraries like Axios or a 3rd party chat SDK since we have full control over the frontend and backend.
- **Alternatives Considered**:
    - *Third-party Chat Widgets (e.g., Intercom, Drift)*: Too expensive and don't integrate with our custom RAG backend easily.
    - *Axios*: Overkill for simple POST requests.

### 2. State Management
- **Decision**: Local component state (`useState`, `useEffect`) lifted to `ChatWidget` container.
- **Rationale**: The chat state is ephemeral and local to the widget. No global app state (Redux/Context) is needed for this isolated feature.

### 3. Persistence
- **Decision**: `localStorage` for storing `thread_id`.
- **Rationale**: Simple, effective for preserving session across reloads without requiring user login (supports anonymous chat).

### 4. Styling
- **Decision**: Plain CSS / CSS Modules.
- **Rationale**: Docusaurus standard practice. Keeps styles scoped and maintainable.

## Clarifications Resolved
- **Backend API**: Confirmed as `POST /api/chat`.
- **Input/Output**: JSON format `{ message, thread_id }` -> `{ answer, sources, thread_id }`.
- **CORS**: Assumed handled on backend (FastAPI `CORSMiddleware`).

## Outstanding Questions
- None.
