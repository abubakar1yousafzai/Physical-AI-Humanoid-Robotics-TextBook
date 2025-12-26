from fastapi import APIRouter, status, Response
from app.models.api import HealthResponse
from app.services.health import check_services_health

router = APIRouter()

@router.get("/health", response_model=HealthResponse)
async def health_check(response: Response):
    health_status = await check_services_health()
    if health_status.status != "healthy":
        response.status_code = status.HTTP_503_SERVICE_UNAVAILABLE
    return health_status
