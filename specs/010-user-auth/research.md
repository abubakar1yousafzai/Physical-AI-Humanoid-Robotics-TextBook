# Research: User Authentication System

**Feature Branch**: `010-user-auth`
**Created**: 2026-01-05
**Status**: Completed

## Decisions & Rationale

### 1. Authentication Library
- **Decision**: **FastAPI-Users**
- **Rationale**: 
  - Specifically designed for FastAPI integration.
  - Built-in support for SQLAlchemy and standard user models.
  - Provides pre-built routers for registration, login, and logout.
  - Handles password hashing (Bcrypt) and token management automatically.
- **Alternatives Considered**: 
  - **Authlib**: More generic, requires more boilerplate for basic auth flows.
  - **Custom Implementation**: High security risk, reinvention of the wheel.

### 2. Session Management
- **Decision**: **JWT (JSON Web Tokens)**
- **Rationale**:
  - Stateless, scalable, and works well with separate frontend (Docusaurus/React) and backend.
  - Easy to implement with FastAPI-Users.
- **Alternatives Considered**:
  - **Server-side Sessions (Cookies)**: Harder to manage across domains if frontend/backend are separated, though slightly more secure against XSS. JWT stored in localStorage is acceptable for this educational platform scope, provided XSS protection is in place.

### 3. Frontend State Management
- **Decision**: **React Context API**
- **Rationale**:
  - Sufficient for managing global auth state (user object, isAuthenticated flag).
  - Built-in to React, no extra dependencies like Redux required.
- **Alternatives Considered**:
  - **Redux/Zustand**: Overkill for just auth state management.

### 4. Database Schema
- **Decision**: **Extend existing Neon Postgres schema**
- **Rationale**:
  - Keeps all data (content, users, chat history) in one place.
  - Relational integrity between users and their chat history/progress.
- **Schema Additions**:
  - `user` table (standard auth fields).
  - `user_progress` (tracking completed chapters).
  - `user_bookmarks` (saving favorite chapters).

## Integration Patterns

### Frontend-Backend Communication
- **Pattern**: Bearer Token in HTTP Headers.
- **Implementation**: `Authorization: Bearer <token>`
- **Storage**: `localStorage` for persistence across tab closes.

### Chat Integration
- **Flow**:
  1. Frontend checks AuthContext for token.
  2. If present, sends token with chat request.
  3. Backend validates token.
  4. If valid, associates message with `user_id`.
  5. If invalid/missing, treats as guest (session-only).

## Security Considerations
- **Password Storage**: Bcrypt hashing (standard practice).
- **Transport**: HTTPS required in production.
- **Input Validation**: Pydantic models on backend.
