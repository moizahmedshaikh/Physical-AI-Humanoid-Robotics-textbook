from fastapi import APIRouter, HTTPException, Depends, Request, status
from pydantic import BaseModel
from typing import Optional
import uuid
from datetime import datetime, timedelta
import re
from auth.models.models import User
from auth.utils.password import hash_password, verify_password, validate_password_strength, validate_email_format
from auth.utils.jwt import create_access_token, create_refresh_token, verify_token
from auth.database import get_db, init_db
from auth.middleware import rate_limit_check, record_login_attempt

router = APIRouter()

# Pydantic models for request/response
class UserCreateRequest(BaseModel):
    email: str
    username: str
    password: str
    first_name: Optional[str] = None
    last_name: Optional[str] = None

class UserLoginRequest(BaseModel):
    email: str
    password: str

class TokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str

class UserResponse(BaseModel):
    id: str
    email: str
    username: str
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    created_at: str

# Initialize database on startup
init_db()

@router.get("/test")
def test():
    return {"msg": "ok"}

@router.post("/signup", response_model=UserResponse)
async def signup(request: UserCreateRequest):
    """Create a new user account."""
    # Validate email format
    if not validate_email_format(request.email):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid email format"
        )

    # Validate password strength
    is_valid, error_msg = validate_password_strength(request.password)
    if not is_valid:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=error_msg
        )

    # Check if user already exists
    existing_user = User.find_by_email(request.email)
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    existing_user = User.find_by_username(request.username)
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already taken"
        )

    # Hash the password
    hashed_password = hash_password(request.password)

    # Create new user
    user = User(
        email=request.email,
        username=request.username,
        password_hash=hashed_password,
        first_name=request.first_name,
        last_name=request.last_name
    )
    user.save()

    return UserResponse(
        id=user.id,
        email=user.email,
        username=user.username,
        first_name=user.first_name,
        last_name=user.last_name,
        created_at=user.created_at.isoformat()
    )

@router.post("/login", response_model=TokenResponse)
async def login(request: UserLoginRequest, fastapi_request: Request):
    """Authenticate user and return JWT tokens."""
    # Check rate limit
    if not rate_limit_check(fastapi_request):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Too many login attempts. Please try again later."
        )

    # Find user by email
    user = User.find_by_email(request.email)
    if not user or not verify_password(request.password, user.password_hash):
        # Record failed attempt for rate limiting
        record_login_attempt(fastapi_request)

        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Account is inactive"
        )

    # Create access and refresh tokens
    access_token_data = {"sub": user.id, "email": user.email}
    refresh_token_data = {"sub": user.id, "email": user.email}

    access_token = create_access_token(data=access_token_data)
    refresh_token = create_refresh_token(data=refresh_token_data)

    return TokenResponse(
        access_token=access_token,
        refresh_token=refresh_token,
        token_type="bearer"
    )

@router.post("/logout")
async def logout(request: Request):
    """Logout user (client-side token invalidation)."""
    # In a real implementation, you might want to add the token to a blacklist
    # For now, we just return a success message
    return {"message": "Successfully logged out"}

@router.get("/me", response_model=UserResponse)
async def get_current_user(request: Request):
    """Get current user information."""
    # Extract token from Authorization header
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )

    token = auth_header[7:]  # Remove "Bearer " prefix
    payload = verify_token(token)

    if not payload or "sub" not in payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )

    user_id = payload["sub"]
    user = User.find_by_id(user_id)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )

    return UserResponse(
        id=user.id,
        email=user.email,
        username=user.username,
        first_name=user.first_name,
        last_name=user.last_name,
        created_at=user.created_at.isoformat()
    )

@router.post("/refresh")
async def refresh_token(request: Request):
    """Refresh access token using refresh token."""
    # Extract refresh token from request body or header
    # For this implementation, we'll expect it in the Authorization header
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Refresh token required"
        )

    refresh_token_str = auth_header[7:]  # Remove "Bearer " prefix
    payload = verify_token(refresh_token_str)

    if not payload or "sub" not in payload or payload.get("type") != "refresh":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid refresh token"
        )

    user_id = payload["sub"]
    user = User.find_by_id(user_id)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )

    # Create new access token
    access_token_data = {"sub": user.id, "email": user.email}
    access_token = create_access_token(data=access_token_data)

    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/verify")
async def verify_authentication(request: Request):

    """Verify if the provided token is valid."""
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication token required"
        )

    token = auth_header[7:]  # Remove "Bearer " prefix
    payload = verify_token(token)

    if not payload or "sub" not in payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )

    user_id = payload["sub"]
    user = User.find_by_id(user_id)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )

    return {
        "authenticated": True,
        "user_id": user.id,
        "email": user.email,
        "username": user.username
    }


