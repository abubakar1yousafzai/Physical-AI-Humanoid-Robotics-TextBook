from fastapi import FastAPI
from app.core.config import settings
from app.core.errors import register_exception_handlers
from app.api.routes import health

app = FastAPI(
    title=settings.PROJECT_NAME,
    openapi_url=f"{settings.API_V1_STR}/openapi.json"
)

# Register Exception Handlers
register_exception_handlers(app)

# Register Routers
app.include_router(health.router, prefix=settings.API_V1_STR, tags=["health"])

@app.get("/")
async def root():
    return {"message": "Welcome to RAG Chatbot API. Visit /docs for Swagger UI."}
