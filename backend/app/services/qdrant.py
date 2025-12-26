from qdrant_client import QdrantClient
from app.core.config import settings

class QdrantService:
    _instance = None
    _client = None

    @classmethod
    def get_client(cls) -> QdrantClient:
        if cls._client is None:
            cls._client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
            )
        return cls._client

    @classmethod
    def get_collection_name(cls) -> str:
        return "physical-ai-textbook" # Hardcoded based on dependencies spec

    @classmethod
    def check_health(cls) -> bool:
        """Check if Qdrant is reachable and collection exists"""
        try:
            client = cls.get_client()
            collection_name = cls.get_collection_name()
            return client.collection_exists(collection_name)
        except Exception:
            return False
