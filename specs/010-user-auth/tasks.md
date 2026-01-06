# Tasks: User Authentication System

**Feature Branch**: `010-user-auth`
**Status**: Draft

## Phase 1: Setup

**Goal**: Initialize dependencies and project structure for authentication.

- [x] T001 Install backend dependencies (fastapi-users, python-jose, passlib) in `backend/requirements.txt`
- [x] T002 Configure FastAPI-Users core setup with JWT and Bcrypt in `backend/app/core/auth.py`
- [x] T003 Create User model inheriting from SQLAlchemyBaseUserTableUUID in `backend/app/models/user.py`
- [x] T004 Create Pydantic schemas for User (Read, Create, Update) in `backend/app/schemas/user.py`
- [x] T005 [P] Implement `get_user_db` dependency in `backend/app/crud/user.py`
- [x] T006 [P] Create initial migration for users table using Alembic in `backend/alembic/versions/`

## Phase 2: Foundational

**Goal**: Establish core backend authentication infrastructure and database schema.

- [x] T007 Apply migrations to Neon Postgres database in `backend/alembic/`
- [x] T008 Register auth routers (register, login) in `backend/app/api/routes/auth.py`
- [x] T009 Include auth router in main application in `backend/app/main.py`
- [x] T010 [P] Create frontend Auth service for API calls in `docusaurus/src/services/authService.js`
- [x] T011 [P] Implement React AuthContext provider in `docusaurus/src/contexts/AuthContext.jsx`
- [x] T012 Wrap Docusaurus root with AuthProvider in `docusaurus/src/theme/Root.js`

## Phase 3: User Registration (P1)

**Goal**: Enable new visitors to sign up for an account.

- [x] T013 [US1] Create Signup page component with neon theme in `docusaurus/src/pages/signup.js`
- [x] T014 [US1] Implement form validation and API call in `docusaurus/src/pages/signup.js`
- [x] T015 [US1] Handle successful registration redirect in `docusaurus/src/pages/signup.js`
- [x] T016 [US1] Add "Sign Up" link to Navbar (conditionally rendered) in `docusaurus/src/components/UserMenu.jsx`

## Phase 4: User Login (P1)

**Goal**: Enable existing users to log in and persist their session.

- [x] T017 [US2] Create Login page component with neon theme in `docusaurus/src/pages/login.js`
- [x] T018 [US2] Implement login form submission and token storage in `docusaurus/src/pages/login.js`
- [x] T019 [US2] Update AuthContext to load user from token on boot in `docusaurus/src/contexts/AuthContext.jsx`
- [x] T020 [US2] Implement logout functionality in `docusaurus/src/services/authService.js`

## Phase 5: Protected Chat History (P1)

**Goal**: Save and retrieve chat history for logged-in users.

- [x] T021 [US4] Update Conversation model to include nullable `user_id` in `backend/app/models/sql.py`
- [x] T022 [US4] Create migration for adding `user_id` to conversations in `backend/alembic/versions/`
- [x] T023 [US4] Update ChatWidget to send Authorization header in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [x] T024 [US4] Update chat endpoint to associate message with user if authenticated in `backend/app/api/routes/chat.py`
- [x] T025 [US4] Implement `GET /chat/history` endpoint for authenticated users in `backend/app/api/routes/chat.py`
- [x] T026 [US4] Display past conversations in ChatWidget for logged-in users in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`

## Phase 6: User Profile (P2)

**Goal**: Allow users to view and manage their account information.

- [x] T027 [US3] Create `users` router for profile management in `backend/app/api/routes/users.py`
- [x] T028 [US3] Implement `GET /users/me` and `PATCH /users/me` endpoints in `backend/app/api/routes/users.py`
- [x] T029 [US3] Create Profile page component in `docusaurus/src/pages/profile.js`
- [x] T030 [US3] Implement profile editing form in `docusaurus/src/pages/profile.js`
- [x] T031 [US3] Create ProtectedRoute HOC in `docusaurus/src/components/ProtectedRoute.jsx`
- [x] T032 [US3] Protect Profile route using ProtectedRoute in `docusaurus/src/pages/profile.js`

## Phase 7: Reading Progress (P2)

**Goal**: Track which chapters a user has completed.

- [x] T033 [US5] Create UserProgress model in `backend/app/models/progress.py`
- [x] T034 [US5] Create migration for user_progress table in `backend/alembic/versions/`
- [x] T035 [US5] Implement `GET /users/me/progress` and `POST /users/me/progress` in `backend/app/api/routes/users.py`
- [x] T036 [US5] Add progress tracking logic (scroll/completion) to doc pages in `docusaurus/src/theme/DocItem/Layout/index.js`
- [x] T037 [US5] Create Dashboard page showing progress stats in `docusaurus/src/pages/dashboard.js`

## Phase 8: Bookmarks (P3)

**Goal**: Allow users to bookmark chapters.

- [x] T038 [US6] Create UserBookmark model in `backend/app/models/bookmark.py`
- [x] T039 [US6] Create migration for user_bookmarks table in `backend/alembic/versions/`
- [x] T040 [US6] Implement bookmark endpoints (GET, POST, DELETE) in `backend/app/api/routes/users.py`
- [x] T041 [US6] Add Bookmark button to doc pages in `docusaurus/src/theme/DocItem/Layout/index.js`
- [x] T042 [US6] Display bookmarks list on Dashboard in `docusaurus/src/pages/dashboard.js`

## Phase 9: Polish & Cross-Cutting

**Goal**: Final UI adjustments, security checks, and cleanup.

- [x] T043 Add "Sign Up to save history" prompt for guest users in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [x] T044 Finalize styling for Login/Signup/Dashboard to match Neon theme in `docusaurus/src/css/custom.css`
- [x] T045 Verify all secure routes return 401 for unauthenticated requests
- [x] T046 Run full manual verification checklist from quickstart.md

## Dependencies

1. **Phase 1 & 2** (Setup/Foundational) must complete before **Phase 3** (Registration).
2. **Phase 3** (Registration) must complete before **Phase 4** (Login).
3. **Phase 4** (Login) is a prerequisite for **Phase 5** (Chat), **Phase 6** (Profile), **Phase 7** (Progress), and **Phase 8** (Bookmarks).
4. **Phase 5, 6, 7, 8** can be executed in parallel after Phase 4 is complete.

## Implementation Strategy

- **MVP Scope**: Phases 1-5 (Auth + Chat History). This delivers the core value of the "Better-Auth" phase.
- **Incremental Delivery**: Deploy after Phase 5. Phases 7 & 8 can be fast-followed.
- **Parallel Execution**:
  - Frontend UI components (Login, Signup) can be built while Backend Auth setup is running.
  - Once Login is done, Chat History and Reading Progress can be developed by different streams if needed.
