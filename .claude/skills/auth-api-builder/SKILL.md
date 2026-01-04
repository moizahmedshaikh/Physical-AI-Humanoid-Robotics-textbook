# Skill: auth-api-builder

## Purpose
Build FastAPI authentication endpoints following RESTful conventions and security best practices. Provides standardized auth API structure.

## Capabilities
This skill can:
- Create POST /api/auth/signup endpoint
- Create POST /api/auth/login endpoint
- Create POST /api/auth/logout endpoint
- Create GET /api/auth/me endpoint
- Create GET /api/auth/verify endpoint
- Configure CORS for frontend communication
- Set up request validation with Pydantic
- Implement error handling and status codes
- Configure HTTP-only cookie responses

## How it Performs Actions
1. **Signup Endpoint**:
   - Accept name, email, password
   - Validate input (email format, password length)
   - Check email uniqueness (use sqlite-user-store skill)
   - Hash password (use password-security skill)
   - Create user in database
   - Generate JWT token (use jwt-token-manager skill)
   - Set HTTP-only cookie
   - Return success response

2. **Login Endpoint**:
   - Accept email, password
   - Get user from database
   - Verify password
   - Generate JWT token
   - Set HTTP-only cookie
   - Return success with user data

3. **Logout Endpoint**:
   - Clear auth cookie
   - Return success

4. **Verify Endpoint**:
   - Extract token from cookie
   - Validate token
   - Return user data if valid

5. **Me Endpoint**:
   - Extract user ID from token
   - Get user from database
   - Return user data

## Input (to the skill)
- Better Auth is used for session orchestration; JWT handling is custom.
- FastAPI app instance
- Better Auth configuration
- Database connection
- Skills: jwt-token-manager, password-security, sqlite-user-store

## Output (from the skill)
- Configured FastAPI router with all auth endpoints
- Request/response models (Pydantic)
- Error handling middleware
- CORS configuration

## Usage Notes
- All endpoints return JSON
- Standard HTTP status codes (200, 400, 401, 409, 500)
- Cookies: HttpOnly=true, Secure=true (production), SameSite=Lax
- Request validation with Pydantic models

## Integration
- Imported in FastAPI main application
- Used with protected-route-guard skill for authentication middleware