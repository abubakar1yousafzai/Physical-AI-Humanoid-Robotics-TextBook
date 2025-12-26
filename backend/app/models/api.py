from pydantic import BaseModel
from typing import Dict

class HealthResponse(BaseModel):
    status: str
    services: Dict[str, str]
