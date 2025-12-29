from typing import Optional
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    API_V1_STR: str = "/api"
    PROJECT_NAME: str = "RAG Chatbot API"
    
    # External APIs
    GEMINI_API_KEY: str
    COHERE_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: Optional[str] = None

    # Database
    DATABASE_URL: str
    
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=True,
        extra="ignore"
    )

settings = Settings()
