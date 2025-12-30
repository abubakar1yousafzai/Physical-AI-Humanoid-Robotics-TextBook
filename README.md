# Physical AI & Humanoid Robotics Textbook

An interactive educational platform with AI-powered RAG chatbot for learning Physical AI and Humanoid Robotics.

[![Vercel](https://img.shields.io/badge/Deployed-Vercel-black?logo=vercel)](https://textbook-physical-ai-humanoid-robotics.vercel.app/)
[![FastAPI](https://img.shields.io/badge/Backend-FastAPI-009688?logo=fastapi)](http://localhost:8000/docs)
[![React](https://img.shields.io/badge/Frontend-React-61DAFB?logo=react)](http://localhost:3000)

---

## 🌟 Features

### 📚 Interactive Textbook
- **5 Modules** covering Physical AI fundamentals to advanced topics
- **15+ Chapters** with detailed content
- Built with **Docusaurus** for excellent reading experience
- Deployed on **Vercel** for fast global access

### 🤖 AI-Powered Chat Assistant
- **RAG (Retrieval-Augmented Generation)** system
- **Real-time responses** using Google Gemini 2.5 Flash (FREE)
- **Source citations** from textbook content
- **Conversation history** with thread persistence
- **Mobile responsive** design

### 🔍 Advanced RAG Pipeline
- **93 textbook chunks** indexed in Qdrant vector database
- **Cohere embeddings** for semantic search
- **OpenAI Agents SDK** for intelligent orchestration
- **Neon Postgres** for conversation persistence

---

## 🏗️ Architecture
```
┌─────────────────┐
│   User Browser  │
└────────┬────────┘
         │
    ┌────▼─────┐
    │ Docusaurus│ (Java Script Frontend)
    │ + Chat    │
    │  Widget   │
    └────┬─────┘
         │ HTTP/Fetch
    ┌────▼─────┐
    │  FastAPI │ (Backend API)
    │  + CORS  │
    └────┬─────┘
         │
    ┌────▼─────────────────┐
    │ OpenAI Agents SDK    │
    │ + Google Gemini 2.5  │
    └────┬─────────────────┘
         │
    ┌────▼─────┬──────────┬──────────┐
    │  Cohere  │  Qdrant  │   Neon   │
    │ Embeddings│ Vector DB│ Postgres │
    └──────────┴──────────┴──────────┘
```

---

## 🚀 Tech Stack

### Frontend
- **Framework:** Docusaurus 3.x (Java Script-based)
- **Chat Widget:** React 18+ with Hooks
- **Styling:** CSS with responsive design
- **Storage:** localStorage for thread persistence

### Backend
- **Framework:** FastAPI (Python)
- **Agent:** OpenAI Agents SDK
- **LLM:** Google Gemini 2.5 Flash (FREE tier)
- **Embeddings:** Cohere embed-english-v3.0
- **Vector DB:** Qdrant Cloud (1GB free)
- **Database:** Neon Serverless Postgres (0.5GB free)
- **Package Manager:** UV

### Integration
- **API:** RESTful with CORS
- **Real-time:** Async/await throughout
- **Error Handling:** Comprehensive try-catch
- **Rate Limiting:** 15 RPM (Gemini limit)

---

## 📦 Installation

### Prerequisites
- Node.js 18+ (for frontend)
- Python 3.10+ (for backend)
- UV package manager (for Python)

### 1. Clone Repository
```bash
git clone https://github.com/abubakar1yousafzai/Physical-AI-Humanoid-Robotics-TextBook.git
cd Physical-AI-Humanoid-Robotics-TextBook
```

### 2. Frontend Setup
```bash
cd docusaurus
npm install
npm start
# Opens http://localhost:3000
```

### 3. Backend Setup
```bash
cd backend

# Install dependencies
uv pip install fastapi uvicorn openai-agents cohere qdrant-client pydantic-settings python-dotenv httpx sqlalchemy asyncpg alembic

# Configure environment variables
cp .env.example .env
# Edit .env with your API keys:
# - GEMINI_API_KEY (from Google AI Studio)
# - COHERE_API_KEY (from Cohere)
# - QDRANT_URL & QDRANT_API_KEY (from Qdrant Cloud)
# - DATABASE_URL (from Neon)

# Run migrations
uv run alembic upgrade head

# Start server
uvicorn app.main:app --reload
# Runs on http://localhost:8000
```

---

## 🎯 Usage

### Using the Chat Widget

1. **Open the textbook** at http://localhost:3000
2. **Click the 💬 button** in the bottom-right corner
3. **Type your question** about Physical AI or robotics
4. **Press Enter** or click Send
5. **View the AI response** with source citations
6. **Expand sources** to see textbook references

### Example Questions
```
- What is Physical AI?
- Explain embodied intelligence
- What is ROS 2?
- How does bipedal locomotion work?
- Tell me about NVIDIA Isaac Sim
```

### API Endpoints

**Health Check:**
```bash
curl http://localhost:8000/api/health
```

**Chat:**
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?", "thread_id": null}'
```

**API Documentation:** http://localhost:8000/docs

---

## 📊 Project Structure
```
Physical-AI-Humanoid-Robotics-TextBook/
├── docusaurus/                 # Frontend (Textbook + Chat)
│   ├── docs/                   # Textbook content
│   ├── src/
│   │   └── components/
│   │       └── ChatWidget/     # Chat widget component
│   └── docusaurus.config.js
│
├── backend/                    # Backend API
│   ├── app/
│   │   ├── api/routes/         # API endpoints
│   │   ├── core/               # Configuration
│   │   ├── crud/               # Database operations
│   │   ├── db/                 # Database connection
│   │   ├── models/             # Pydantic & SQLAlchemy models
│   │   └── services/           # RAG logic, clients
│   ├── alembic/                # Database migrations
│   └── main.py                 # Main indexing script
│
└── specs/                      # Specifications (Spec-Kit Plus)
    ├── 001-textbook-deployment/
    ├── 002-rag-indexing/
    ├── 003-rag-backend-api/
    └── 004-chat-widget-integration/
```

---

## ✨ Features in Detail

### RAG System
- **Semantic Search:** Cohere embeddings for understanding context
- **Vector Database:** Qdrant stores 93 indexed chunks
- **Intelligent Retrieval:** Top 3 most relevant chunks per query
- **Source Attribution:** Every answer includes textbook references

### Chat Widget
- **Floating Button:** Non-intrusive, always accessible
- **Slide-in Panel:** Smooth animations, 400×600px on desktop
- **Message History:** User and assistant messages styled distinctly
- **Collapsible Sources:** Expandable citations with textbook excerpts
- **Thread Persistence:** Conversations survive page refreshes

### Backend API
- **Async Operations:** Fast, concurrent request handling
- **Error Handling:** Graceful failures with user-friendly messages
- **Rate Limiting:** Respects API quotas (15 RPM for Gemini)
- **Database Persistence:** All conversations saved in Neon Postgres

---

## 🧪 Testing

### Manual Testing
```bash
# 1. Start both servers (frontend + backend)
# 2. Open http://localhost:3000
# 3. Click chat button
# 4. Send test queries:
#    - "What is Physical AI?"
#    - "Tell me about ROS 2"
#    - "Explain embodied intelligence"
# 5. Verify responses and sources
# 6. Refresh page and check history persists
```

### API Testing
```bash
# Health check
curl http://localhost:8000/api/health

# Chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?", "thread_id": null}'
```

---

## 🎓 Educational Content

### Module 1: Foundations of Physical AI
- Chapter 1: Introduction to Physical AI
- Chapter 2: Sensors and Perception
- Chapter 3: Humanoid Robotics Overview

### Module 2: Robot Operating System (ROS 2)
- Chapter 4: ROS 2 Fundamentals
- Chapter 5: Nodes, Topics, and Services
- Chapter 6: Building ROS 2 Applications

### Module 3: Simulation and Digital Twins
- Chapter 7: Gazebo Simulation
- Chapter 8: NVIDIA Isaac Sim
- Chapter 9: Digital Twin Concepts

### Module 4: Motion and Control
- Chapter 10: Kinematics and Dynamics
- Chapter 11: Locomotion Strategies
- Chapter 12: Manipulation and Grasping

### Module 5: Advanced Topics
- Chapter 13: Learning and Adaptation
- Chapter 14: Real-World Deployment
- Chapter 15: Future of Physical AI

---

## 🔑 API Keys Required

### Google Gemini (FREE)
1. Go to [Google AI Studio](https://aistudio.google.com/app/apikey)
2. Create API key
3. Add to `.env` as `GEMINI_API_KEY`

### Cohere (FREE)
1. Sign up at [Cohere](https://cohere.com)
2. Get API key from dashboard
3. Add to `.env` as `COHERE_API_KEY`

### Qdrant Cloud (FREE)
1. Create account at [Qdrant](https://cloud.qdrant.io)
2. Create cluster
3. Add URL and API key to `.env`

### Neon Postgres (FREE)
1. Sign up at [Neon](https://neon.tech)
2. Create project
3. Copy connection string to `.env` as `DATABASE_URL`

---

## 📈 Performance

- **Response Time:** <5 seconds average
- **Embedding Generation:** ~200ms (Cohere)
- **Vector Search:** ~100ms (Qdrant)
- **LLM Generation:** ~3-4 seconds (Gemini)
- **Database Query:** ~50ms (Neon)

---

## 🐛 Troubleshooting

### CORS Error
**Issue:** `Access to fetch blocked by CORS policy`

**Solution:** Verify `backend/app/main.py` has CORS middleware:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Backend Not Responding
**Issue:** Chat shows "connection error"

**Solution:** Check backend is running:
```bash
cd backend
uvicorn app.main:app --reload
```

### Database Connection Closed
**Issue:** `connection is closed` error

**Solution:** Already fixed with `pool_pre_ping=True` in `db/session.py`

---

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

---

## 📝 License

This project is created for educational purposes.

---

## 🙏 Acknowledgments

- **Docusaurus** for the amazing documentation framework
- **FastAPI** for the high-performance backend
- **OpenAI** for the Agents SDK
- **Google** for Gemini LLM (free tier!)
- **Cohere** for embedding models
- **Qdrant** for vector database
- **Neon** for serverless Postgres

---

## 📧 Contact

**Author:** Abubakar Yousafzai  
**GitHub:** [@abubakar1yousafzai](https://github.com/abubakar1yousafzai)  
**Project:** [Physical AI Textbook](https://github.com/abubakar1yousafzai/Physical-AI-Humanoid-Robotics-TextBook)  
**Live Demo:** [https://textbook-physical-ai-humanoid-robotics.vercel.app/](https://textbook-physical-ai-humanoid-robotics.vercel.app/)

---

## ⭐ Star History

If you find this project helpful, please give it a star! ⭐

---

**Built with ❤️ for the Physical AI and Robotics community**
