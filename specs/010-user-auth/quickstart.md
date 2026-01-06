# Quickstart: User Authentication

## Prerequisites
- Backend running (`uv run main.py`)
- Frontend running (`npm start`)
- Postgres database accessible

## Setup

1. **Install Backend Dependencies**:
   ```bash
   cd backend
   pip install "fastapi-users[sqlalchemy]" "python-jose[cryptography]" "passlib[bcrypt]"
   ```

2. **Apply Migrations**:
   ```bash
   cd backend
   alembic upgrade head
   ```

## Verification Steps

### 1. Register a User
- Navigate to `http://localhost:3000/signup`.
- Enter Name: "Test User", Email: "test@example.com", Password: "Password123!".
- Submit form.
- **Expected**: Redirect to Dashboard/Login and user created in DB.

### 2. Login
- Navigate to `http://localhost:3000/login`.
- Enter credentials.
- **Expected**: Redirect to Dashboard, "Logout" button appears in Navbar.

### 3. Check Persistence
- Refresh the page.
- **Expected**: Still logged in.

### 4. Protected Route
- Logout.
- Try to access `http://localhost:3000/dashboard`.
- **Expected**: Redirect to `/login`.

### 5. Chat History
- Login.
- Open Chat Widget.
- Send a message "Hello Auth".
- Refresh page.
- Open Chat Widget.
- **Expected**: Message "Hello Auth" is visible.
