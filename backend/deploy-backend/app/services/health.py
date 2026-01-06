from app.services.qdrant import QdrantService
from app.services.gemini import GeminiService
from app.models.api import HealthResponse

async def check_services_health() -> HealthResponse:
    qdrant_healthy = QdrantService.check_health()
    gemini_healthy = GeminiService.check_health()
    
    services_status = {
        "qdrant": "connected" if qdrant_healthy else "disconnected",
        "gemini": "configured" if gemini_healthy else "misconfigured"
    }
    
    overall_status = "healthy" if all(v != "disconnected" and v != "misconfigured" for v in services_status.values()) else "unhealthy"
    
    return HealthResponse(
        status=overall_status,
        services=services_status
    )
