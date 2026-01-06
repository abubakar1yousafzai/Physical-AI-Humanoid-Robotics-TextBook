import time
from fastapi import Request, HTTPException, status

class SimpleRateLimiter:
    def __init__(self, limit: int, window: int):
        self.limit = limit
        self.window = window
        self.requests = []

    def __call__(self, request: Request):
        now = time.time()
        # Filter out requests older than window
        self.requests = [req_time for req_time in self.requests if now - req_time < self.window]
        
        if len(self.requests) >= self.limit:
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Rate limit exceeded"
            )
        
        self.requests.append(now)

# Global instance for dependency injection
# 15 requests per 60 seconds (Gemini Limit)
rate_limiter = SimpleRateLimiter(limit=15, window=60)
