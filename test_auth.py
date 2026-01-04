import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add the api directory to the path so we can import from api.src
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'api'))

from api.src.main import app
from api.auth.database import init_db

# Create a test client
client = TestClient(app)

def test_auth_routes_exist():
    """Test that auth routes are accessible."""
    # Test signup route exists and returns 422 for missing required fields
    response = client.post("/api/auth/signup", json={
        "email": "test@example.com",
        "username": "testuser",
        "password": "TestPass123!"
    })
    # This should succeed as the user is created (status 200) or fail with validation (422) or duplicate (400)
    assert response.status_code in [200, 400, 422]

    # Test login route exists
    response = client.post("/api/auth/login", json={
        "email": "test@example.com",
        "password": "TestPass123!"
    })
    # Should return 401 for incorrect credentials (since user may not exist if first test created it)
    assert response.status_code in [401, 400, 200]  # 200 if user was created and credentials are valid

def test_database_initialization():
    """Test that database is properly initialized."""
    # This just tests that the database initialization function runs without error
    try:
        init_db()
        assert True
    except Exception:
        assert False

def test_password_hashing():
    """Test password hashing utilities."""
    from api.auth.utils.password import hash_password, verify_password

    password = "TestPassword123!"
    hashed = hash_password(password)

    # Verify the password matches the hash
    assert verify_password(password, hashed)

    # Verify wrong password doesn't match
    assert not verify_password("WrongPassword", hashed)

def test_jwt_token_creation():
    """Test JWT token utilities."""
    from api.auth.utils.jwt import create_access_token, verify_token

    data = {"sub": "test_user", "email": "test@example.com"}
    token = create_access_token(data)

    # Verify the token can be decoded
    payload = verify_token(token)
    assert payload is not None
    assert payload["sub"] == "test_user"
    assert payload["email"] == "test@example.com"