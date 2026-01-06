from openai import OpenAI
from app.core.config import settings

class GeminiService:
    _client = None

    @classmethod
    def get_client(cls) -> OpenAI:
        if cls._client is None:
            cls._client = OpenAI(
                api_key=settings.GEMINI_API_KEY,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
            )
        return cls._client

    @classmethod
    def get_model_name(cls) -> str:
        return "gemini-1.5-flash"

    @classmethod
    def check_health(cls) -> bool:
        """Check if Gemini client is properly configured"""
        try:
            client = cls.get_client()
            return client.api_key is not None and len(client.api_key) > 0
        except Exception:
            return False
