---
id: 002
title: Plan User Auth
stage: plan
date: 2026-01-05
surface: agent
model: gemini-2.0-flash-exp
feature: 010-user-auth
branch: 010-user-auth
user: user
command: /sp.plan
labels: ["plan", "user-auth", "fastapi-users", "jwt"]
links:
  spec: specs/010-user-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-user-auth/plan.md
  - specs/010-user-auth/research.md
  - specs/010-user-auth/data-model.md
  - specs/010-user-auth/contracts/openapi.yaml
  - specs/010-user-auth/quickstart.md
tests:
  - none
---

## Prompt

$ARGUMENTS
/sp.plan 

# Implementation Plan - User Authentication

**Approach:** FastAPI-Users + JWT + SQLAlchemy

---

## Tech Stack

**Backend:**
- FastAPI-Users (auth library)
- SQLAlchemy (ORM - existing)
- JWT tokens
- Bcrypt (password hashing)
- Neon Postgres (existing)

**Frontend:**
- React Context API (auth state)
- localStorage (token storage)
- Custom auth service module

---

## Database Schema

### New Tables

**users:**
- id, email, hashed_password, name
- is_active, is_verified, created_at

**user_progress:**
- id, user_id, chapter_id, completed, completed_at

**user_bookmarks:**
- id, user_id, chapter_id, created_at

### Modified Tables

**conversations:**
- Add: user_id (nullable, foreign key)

---

## Backend Structure
```
backend/app/
├── models/
│   ├── user.py              # NEW - User model
│   ├── progress.py          # NEW
│   └── bookmark.py          # NEW
├── api/routes/
│   ├── auth.py              # NEW - Auth endpoints
│   └── users.py             # NEW - User CRUD
├── core/
│   └── auth.py              # NEW - FastAPI-Users setup
└── crud/
    └── user.py              # NEW
```

---

## Frontend Structure
```
docusaurus/src/
├── contexts/
│   └── AuthContext.jsx      # NEW - Auth state
├── services/
│   └── authService.js       # NEW - API calls
├── components/
│   ├── ProtectedRoute.jsx   # NEW
│   └── UserMenu.jsx         # NEW - Navbar dropdown
└── pages/
    ├── login.js             # NEW
    ├── signup.js            # NEW
    ├── dashboard.js         # NEW
    └── profile.js           # NEW
```

---

## Implementation Phases

### Phase 1: Backend Setup

**Install:**
```bash
pip install fastapi-users[sqlalchemy] python-jose[cryptography] passlib[bcrypt]
```

**Create:**
- User model (extends SQLAlchemyBaseUserTable)
- FastAPI-Users instance
- Auth routers (/api/auth/register, /login, /logout)
- JWT authentication backend

**Test:** Signup/login with Postman

---

### Phase 2: Database Migration
**Create Alembic migrations:**
```bash
alembic revision -m "add users table"
alembic revision -m "add user_progress table"
alembic revision -m "add user_bookmarks table"
alembic revision -m "add user_id to conversations"
alembic upgrade head
```

**Verify:** Tables created in Neon Postgres

---

### Phase 3: Frontend Auth

**AuthContext.jsx:**
- State: user, isAuthenticated, loading
- Functions: login, signup, logout, checkAuth
- Store token in localStorage

**authService.js:**
```javascript
signup(email, password, name)
login(email, password)
logout()
getCurrentUser()
```

**Integration:** Wrap app with AuthProvider

---

### Phase 4: Auth UI

**Login page (/login):**
- Email + password fields
- Neon theme glassmorphism card
- Error handling
- Redirect after login

**Signup page (/signup):**
- Name + email + password
- Client-side validation
- Match login styling

**UserMenu (Navbar):**
- Show when logged in
- Avatar with dropdown
- Links: Dashboard, Profile, Logout

---

### Phase 5: Protected Routes

**ProtectedRoute component:**
```javascript
if (!isAuthenticated) redirect to /login
else render children
```

**Create pages:**
- Dashboard: Progress, recent chats, stats
- Profile: User info, edit name, logout

**Apply:** Wrap dashboard and profile routes

---

### Phase 6: Chat Integration

**Update ChatWidget:**
- Get auth token from context
- Send in API request headers
- Link conversation to user_id if logged in

**Update backend:**
- Chat endpoint accepts optional user
- Save user_id if authenticated

**Guest message:** "Sign up to save your chat history"

---

### Phase 7: Progress & Bookmarks optional

**Progress tracking:**
- Track scroll position
- Mark chapter complete at 90%
- Save to user_progress table
- Show on dashboard

**Bookmarks:**
- Add bookmark button to chapters
- Save to user_bookmarks table
- Show on dashboard

---

## API Endpoints

**Auth:**
```
POST /api/auth/register     (signup)
POST /api/auth/login        (JWT login)
POST /api/auth/logout
GET  /api/auth/users/me     (current user)
```

**Users:**
```
GET    /api/users/me/progress
POST   /api/users/me/progress
GET    /api/users/me/bookmarks
POST   /api/users/me/bookmarks
DELETE /api/users/me/bookmarks/:id
```

**Chat (updated):**
```
POST /api/chat              (now links to user if authenticated)
GET  /api/chat/history      (requires auth)
```

---

## Key Code Snippets

### Backend Auth Setup
```python
from fastapi_users import FastAPIUsers
from fastapi_users.authentication import JWTAuthentication

jwt_auth = JWTAuthentication(
    secret=settings.SECRET_KEY,
    lifetime_seconds=604800  # 7 days
)

fastapi_users = FastAPIUsers(
    get_user_db,
    [jwt_auth],
    User,
    UserCreate,
    UserUpdate,
    UserDB
)

app.include_router(
    fastapi_users.get_auth_router(jwt_auth),
    prefix="/api/auth",
    tags=["auth"]
)
```

### Frontend Auth Hook
```javascript
const { user, isAuthenticated, login, logout } = useAuth();

if (!isAuthenticated) return <Login />;
```

---

## Testing Checklist

- [ ] Signup creates user in database
- [ ] Login returns JWT token
- [ ] Token stored in localStorage
- [ ] Protected routes redirect if not logged in
- [ ] Navbar shows correct UI based on auth
- [ ] Chat saves to user when logged in
- [ ] Logout clears token and redirects
- [ ] Guest chat works without login

---

## Deployment Notes

**Environment Variables:**
```env
SECRET_KEY=your-secret-key-here
JWT_LIFETIME=604800
```

**CORS:** Add auth endpoints to allowed origins

**HTTPS:** Required in production for secure cookies

---

## Success Checklist

✅ Users can signup and login  
✅ Session persists across refreshes  
✅ Protected pages require auth  
✅ Chat history saved per user  
✅ Dashboard shows user data  
✅ All UI matches neon theme  
✅ Guest chat works

## Response snapshot

Created implementation plan and design artifacts for User Authentication using FastAPI-Users and React Context.
Generated `plan.md`, `research.md`, `data-model.md`, `contracts/openapi.yaml`, and `quickstart.md`.
Updated agent context.

## Outcome

- ✅ Impact: Detailed plan for user auth implementation.
- 🧪 Tests: Verification steps outlined in quickstart.md.
- 📁 Files: Plan and design docs created.
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: Plan follows constitution and uses standard, secure libraries.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
