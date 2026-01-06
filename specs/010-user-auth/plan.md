# Implementation Plan - User Authentication

**Feature Branch:** `010-user-auth`
**Feature Spec:** `specs/010-user-auth/spec.md`

## Technical Context

**Approach:** FastAPI-Users + JWT + SQLAlchemy

### Tech Stack

**Backend:**
- **FastAPI-Users**: Auth library for rapid, secure implementation.
- **SQLAlchemy**: Existing ORM for database interactions.
- **JWT**: Stateless session management.
- **Bcrypt**: Secure password hashing.
- **Neon Postgres**: Existing database.

**Frontend:**
- **React Context API**: Manage global auth state.
- **localStorage**: Persist JWT tokens.
- **Custom Service**: Encapsulate API calls.

## Database Schema

### New Tables

**users** (Managed by FastAPI-Users mixin + custom fields)
- `id`: UUID (Primary Key)
- `email`: String (Unique, Indexed)
- `hashed_password`: String
- `is_active`: Boolean
- `is_superuser`: Boolean
- `is_verified`: Boolean
- `name`: String (Custom field)
- `created_at`: DateTime (Custom field)

**user_progress**
- `id`: UUID (Primary Key)
- `user_id`: UUID (Foreign Key -> users.id)
- `chapter_id`: String (e.g., "module-01/chapter-01")
- `completed`: Boolean
- `completed_at`: DateTime

**user_bookmarks**
- `id`: UUID (Primary Key)
- `user_id`: UUID (Foreign Key -> users.id)
- `chapter_id`: String
- `created_at`: DateTime

### Modified Tables

**conversations**
- Add `user_id`: UUID (Nullable, Foreign Key -> users.id)

## Component Structure

### Backend
```
backend/app/
├── models/
│   ├── user.py              # User model definition
│   ├── progress.py          # Progress tracking model
│   └── bookmark.py          # Bookmarks model
├── api/
│   └── routes/
│       ├── auth.py          # Auth endpoints (login/signup)
│       └── users.py         # User profile & data endpoints
├── core/
│   └── auth.py              # FastAPI-Users configuration
└── crud/
    └── user.py              # User-related DB operations
```

### Frontend
```
docusaurus/src/
├── contexts/
│   └── AuthContext.jsx      # Auth state provider
├── services/
│   └── authService.js       # API client for auth
├── components/
│   ├── ProtectedRoute.jsx   # HOC for route protection
│   └── UserMenu.jsx         # Navbar user dropdown
└── pages/
    ├── login.js             # Login page
    ├── signup.js            # Registration page
    ├── dashboard.js         # User dashboard
    └── profile.js           # User profile settings
```

## Implementation Phases

### Phase 1: Backend Setup
- Install dependencies: `fastapi-users[sqlalchemy]`, `python-jose[cryptography]`, `passlib[bcrypt]`.
- Define `User` model inheriting from `SQLAlchemyBaseUserTableUUID`.
- Configure `FastAPIUsers` with JWT backend.
- Register auth routers.

### Phase 2: Database Migration
- Create Alembic migrations for `users`, `user_progress`, `user_bookmarks`.
- Update `conversations` table.
- Apply migrations to Neon Postgres.

### Phase 3: Frontend Logic
- Implement `authService.js` for API communication.
- Create `AuthContext.jsx` to manage `user`, `token`, `login()`, `logout()`.
- Wrap application root with `AuthProvider`.

### Phase 4: UI Implementation
- **Login/Signup Pages**: Neon-themed forms with validation.
- **UserMenu**: Dynamic navbar component (Avatar/Dropdown).
- **Dashboard**: Display user stats and progress.

### Phase 5: Protected Routes & Chat Integration
- Create `ProtectedRoute` component.
- Apply protection to Dashboard and Profile pages.
- Update Chat widget to send `Authorization` header.
- Update backend chat logic to link messages to `user_id`.

## API Endpoints

### Authentication (FastAPI-Users)
- `POST /auth/register`: Register a new user.
- `POST /auth/login`: Login and retrieve JWT.
- `POST /auth/logout`: Logout (handled client-side for JWT, but endpoint exists).

### User Data
- `GET /users/me`: Get current user profile.
- `PATCH /users/me`: Update profile (name).
- `GET /users/me/progress`: Retrieve reading progress.
- `POST /users/me/progress`: Update reading progress.
- `GET /users/me/bookmarks`: Retrieve bookmarks.
- `POST /users/me/bookmarks`: Add bookmark.
- `DELETE /users/me/bookmarks/{id}`: Remove bookmark.

### Chat
- `POST /chat`: (Updated) Accepts optional `user` context.
- `GET /chat/history`: (New) Retrieve user's past conversations.

## Verification Plan

### Automated Tests
- [ ] Backend: Unit tests for User model and auth flow.
- [ ] Backend: API tests for protected endpoints (401 without token).
- [ ] Frontend: Component tests for `AuthContext`.

### Manual Verification
1. **Registration**: Create a new account, verify DB entry.
2. **Login**: Login with valid/invalid credentials, check JWT in localStorage.
3. **Persistence**: Refresh page, ensure user remains logged in.
4. **Protection**: Try accessing `/dashboard` while logged out (should redirect).
5. **Chat**: Send message as logged-in user, verify `user_id` in DB.
6. **Logout**: Click logout, verify token removal and redirect.

## Constitution Check

- **Educational Excellence**: Auth enables personalized learning tracking.
- **Interactive Learning**: Progress tracking encourages completion.
- **Phased Development**: Adheres to Phase 3 (Better-Auth) of the constitution.
- **Security**: Uses standard, secure libraries (Bcrypt, JWT) and HTTPS.
- **Performance**: JWT is stateless and fast.