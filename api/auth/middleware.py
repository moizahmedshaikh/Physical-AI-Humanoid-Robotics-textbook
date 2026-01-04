from fastapi import Request, HTTPException, status
from datetime import datetime, timedelta
from collections import defaultdict
import time

# Simple in-memory store for rate limiting (in production, use Redis or similar)
login_attempts = defaultdict(list)

def rate_limit_check(request: Request, max_attempts: int = 5, window_minutes: int = 15):
    """
    Check if the IP address has exceeded the rate limit for login attempts.

    Args:
        request: The FastAPI request object
        max_attempts: Maximum number of attempts allowed
        window_minutes: Time window in minutes

    Returns:
        bool: True if rate limit is not exceeded, False otherwise
    """
    client_ip = request.client.host

    # Get current time
    now = datetime.utcnow()
    window_start = now - timedelta(minutes=window_minutes)

    # Remove old attempts outside the window
    login_attempts[client_ip] = [
        attempt_time for attempt_time in login_attempts[client_ip]
        if attempt_time > window_start
    ]

    # Check if we've exceeded the limit
    if len(login_attempts[client_ip]) >= max_attempts:
        return False

    return True

def record_login_attempt(request: Request):
    """
    Record a login attempt for rate limiting.

    Args:
        request: The FastAPI request object
    """
    client_ip = request.client.host
    login_attempts[client_ip].append(datetime.utcnow())

def clear_old_attempts(window_minutes: int = 15):
    """
    Clean up old attempts that are outside the time window.
    This should be called periodically.
    """
    now = datetime.utcnow()
    window_start = now - timedelta(minutes=window_minutes)

    for ip in list(login_attempts.keys()):
        login_attempts[ip] = [
            attempt_time for attempt_time in login_attempts[ip]
            if attempt_time > window_start
        ]


    