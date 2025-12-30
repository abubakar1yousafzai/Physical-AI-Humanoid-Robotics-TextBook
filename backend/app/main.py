from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.core.config import settings
from app.core.errors import register_exception_handlers
from app.api.routes import health, chat

app = FastAPI(
    title=settings.PROJECT_NAME,
    openapi_url=f"{settings.API_V1_STR}/openapi.json"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register Exception Handlers
register_exception_handlers(app)

# Register Routers
app.include_router(health.router, prefix=settings.API_V1_STR, tags=["health"])
app.include_router(chat.router, prefix=settings.API_V1_STR, tags=["chat"])

@app.get("/")
async def root():
    return {"message": "Welcome to RAG Chatbot API. Visit /docs for Swagger UI."}