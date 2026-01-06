# Physical AI & Humanoid Robotics Textbook

Interactive educational platform with AI-powered chat, user authentication, and personalized learning for Physical AI and Humanoid Robotics.

[![Live Demo](https://img.shields.io/badge/Demo-Vercel-black?logo=vercel)](https://textbook-physical-ai-humanoid-robotics.vercel.app/)
[![Backend](https://img.shields.io/badge/API-Hugging%20Face-FFD21E?logo=huggingface)](https://abu-bakar1yousafzai-deploy-backend.hf.space/docs)

---

## 🌟 Features

- **23 Chapters** across 6 modules on Physical AI and Humanoid Robotics
- **AI Chat Assistant** with RAG (Retrieval-Augmented Generation)
- **User Authentication** - signup, login, protected routes
- **Chat History** - saved per user with thread management
- **Text Selection** - select text and ask AI about it
- **Neon Theme UI** - modern glassmorphism design
- **Fully Deployed** - Frontend on Vercel, Backend on Hugging Face
- **100% FREE** - All services on free tiers

---

## 🏗️ Architecture
```
User Browser → Docusaurus Frontend → FastAPI Backend
                                    ↓
                    ┌───────────────┼───────────────┐
                    ↓               ↓               ↓
              Gemini 2.0      Qdrant Vector   Neon Postgres
              (LLM, FREE)     (Embeddings)    (Users + Chats)
```

---

## 🚀 Tech Stack

### Frontend
- **Docusaurus 3.9** - Static site generator
- **Custom Theme** - Neon colors with glassmorphism
- **Chat Widget** - JavaScript 
- **Auth Pages** - Login, signup, dashboard, profile
- **Deployment** - Vercel with auto-deploy

### Backend
- **FastAPI** - Python async web framework
- **FastAPI-Users** - Authentication with JWT
- **Sentence Transformers** - Local embeddings (FREE)
- **Google Gemini 2.0 Flash** - LLM (FREE, 15 RPM)
- **Qdrant Cloud** - Vector database (1GB free)
- **Neon Postgres** - Database (0.5GB free)
- **Deployment** - Hugging Face Spaces (Docker, FREE)

---

## 📦 Installation

### Prerequisites
- Node.js 18+
- Python 3.11+
- UV package manager

### 1. Clone
```bash
git clone https://github.com/abubakar1yousafzai/Physical-AI-Humanoid-Robotics-TextBook.git
cd Physical-AI-Humanoid-Robotics-TextBook
```

### 2. Frontend
```bash
cd docusaurus
npm install
npm start  # http://localhost:3000
```

### 3. Backend
```bash
cd backend
uv pip install -r requirements.txt

# Setup .env
cp .env.example .env
# Add: GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, SECRET_KEY

# Migrations
uv run alembic upgrade head

# Start
uv run uvicorn app.main:app --reload  # http://localhost:8000
```

---

## 🎯 Usage

### Create Account
1. Visit https://textbook-physical-ai-humanoid-robotics.vercel.app/
2. Click **Sign Up**
3. Enter name, email, password (min 8 chars)
4. Redirected to dashboard

### Chat with AI
1. Click **💬** button (bottom-right)
2. Type question: *"What is Physical AI?"*
3. View response with textbook sources
4. History auto-saves (if logged in)

### Text Selection
1. Select any text on a chapter
2. Click **"Ask AI"** popup
3. Ask questions about selected content

---

## 🔌 API Endpoints

**Base URL:** `https://abu-bakar1yousafzai-deploy-backend.hf.space`
```bash
# Health
GET /api/health

# Auth
POST /api/auth/register
POST /api/auth/jwt/login
GET /api/users/me  # Requires: Authorization: Bearer <token>

# Chat
POST /api/chat
GET /api/chat/history  # Requires auth

# Docs
GET /docs  # Swagger UI
```

---

## 📊 Project Structure
```
├── Dockerfile                    # Hugging Face deployment
├── README.md
│
├── docusaurus/                   # Frontend
│   ├── docs/                     # 23 chapters
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatWidget/       # Chat component
│   │   │   └── UserMenu.jsx
│   │   ├── contexts/
│   │   │   └── AuthContext.jsx   # Auth state
│   │   ├── services/
│   │   │   └── authService.js    # API calls
│   │   ├── pages/
│   │   │   ├── index.js          # Landing (neon theme)
│   │   │   ├── login.js
│   │   │   ├── signup.js
│   │   │   ├── dashboard.js
│   │   │   └── profile.js
│   │   └── css/
│   │       └── custom.css        # Neon theme
│   └── docusaurus.config.js
│
└── backend/                      # Backend API
    ├── app/
    │   ├── api/routes/           # Auth, chat, users
    │   ├── core/                 # Config, auth setup
    │   ├── models/               # User, conversation
    │   ├── services/
    │   │   └── rag.py            # RAG logic
    │   └── main.py
    ├── alembic/                  # Migrations
    └── requirements.txt
```

---

## 🎓 Content Modules

1. **Introduction to Physical AI** (5 chapters)
2. **ROS 2 Fundamentals** (4 chapters)
3. **Robot Simulation with Gazebo** (3 chapters)
4. **NVIDIA Isaac Platform** (3 chapters)
5. **Humanoid Robot Development** (5 chapters)
6. **Conversational Robotics** (3 chapters)

---

## 🔑 API Keys Setup

### Google Gemini (FREE)
- Get: https://aistudio.google.com/app/apikey
- Add to `.env`: `GEMINI_API_KEY=...`

### Qdrant Cloud (FREE - 1GB)
- Sign up: https://cloud.qdrant.io
- Add: `QDRANT_URL=...` and `QDRANT_API_KEY=...`

### Neon Postgres (FREE - 0.5GB)
- Sign up: https://neon.tech
- Add: `DATABASE_URL=...`

### JWT Secret
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
# Add to .env: SECRET_KEY=...
```

---

## 🐛 Troubleshooting

### Chat Error: "Failed to connect"
**Fix:** Update `ChatWidget.jsx` API URL:
```javascript
const API_BASE_URL = window.location.hostname === 'localhost'
  ? 'http://localhost:8000'
  : 'https://abu-bakar1yousafzai-deploy-backend.hf.space';
```

### Signup Error: "Failed to fetch"
**Fix:** Update `authService.js` API URL (same as above)

### CORS Error
**Fix:** Add your frontend URL to `backend/app/main.py`:
```python
allow_origins=["https://textbook-physical-ai-humanoid-robotics.vercel.app"]
```

### Backend Down
- Check: https://abu-bakar1yousafzai-deploy-backend.hf.space/api/health
- Restart HF Space if needed

---

## 🚀 Deployment

### Frontend (Vercel)
Auto-deploys on `git push` to main.

**Manual:**
```bash
cd docusaurus
vercel --prod
```

### Backend (Hugging Face)
Already deployed at: https://abu-bakar1yousafzai-deploy-backend.hf.space

**Update:**
```bash
git clone https://huggingface.co/spaces/abu-bakar1yousafzai/deploy-backend
# Copy updated files
git push
```

---

## 📈 Performance

- **Page Load:** <2s (Vercel CDN)
- **Chat Response:** <5s
  - Embeddings: ~200ms (local)
  - Vector Search: ~100ms
  - LLM: ~3-4s
- **Auth:** <500ms

---

## 🤝 Contributing

1. Fork the repo
2. Create branch: `git checkout -b feature/name`
3. Commit: `git commit -m 'Add feature'`
4. Push: `git push origin feature/name`
5. Open Pull Request

---

## 📧 Contact

**Author:** Abu Bakar Yousafzai  
**GitHub:** [@abubakar1yousafzai](https://github.com/abubakar1yousafzai)  
**X:** [@Abu_bakar_x](https://x.com/Abu_bakar_x)  
**LinkedIn:** [abu-bakar-profile](https://www.linkedin.com/in/abu-bakar-profile/)

**Links:**
- **Live Demo:** https://textbook-physical-ai-humanoid-robotics.vercel.app/
- **Backend API:** https://abu-bakar1yousafzai-deploy-backend.hf.space/docs
- **Repository:** https://github.com/abubakar1yousafzai/Physical-AI-Humanoid-Robotics-TextBook

---

## 🙏 Acknowledgments

**Services (All FREE):**
- Vercel - Frontend hosting
- Hugging Face - Backend hosting
- Google Gemini - LLM (15 RPM)
- Qdrant Cloud - Vector DB (1GB)
- Neon - Postgres (0.5GB)

**Tech:**
- Docusaurus, FastAPI, FastAPI-Users, Sentence Transformers

---

**Built with ❤️ for the Physical AI community**

