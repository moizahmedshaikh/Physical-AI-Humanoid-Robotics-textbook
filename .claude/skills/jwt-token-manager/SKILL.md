# Skill: jwt-token-manager

## Purpose
Generate, validate, and manage JWT tokens for user authentication sessions. Handles token creation with custom claims, expiration management, and signature verification.

## Capabilities
This skill can:
- Generate JWT tokens with user claims (user_id, email, exp, iat)
- Validate and decode JWT tokens
- Check token expiration
- Extract user information from valid tokens
- Handle token refresh logic (if needed)
- Manage token secret keys securely

## How it Performs Actions
1. **Token Generation**:
   - Accept user data (id, email)
   - Add standard claims (exp, iat, jti)
   - Sign token with secret key
   - Return encoded JWT string

2. **Token Validation**:
   - Accept JWT token string
   - Verify signature using secret key
   - Check expiration time
   - Return decoded payload or error

3. **Token Refresh**:
   - Accept valid token (or refresh token)
   - Generate new token with extended expiry
   - Return new token

## Input (to the skill)
- User data: `{id: string, email: string, name: string}`
- Token to validate: `string`
- Secret key: from environment variable
- Expiration time: 7 days (604800 seconds)

## Output (from the skill)
- Generated token: `string` (JWT format)
- Decoded payload: `{user_id, email, exp, iat}`
- Validation result: `{valid: boolean, error?: string}`

## Usage Notes
- Uses PyJWT library in Python backend
- Token secret must be strong and kept in environment variables
- Tokens are stateless (no database lookup needed)
- Expiration checked on every validation

## Integration
- Used by FastAPI auth routes (login, verify)
- Called by auth middleware for route protection
- Frontend stores token in HTTP-only cookie

## Example Usage
```python
# Generate token
user_data = {
    "id": "user123",
    "email": "user@example.com",
    "name": "John Doe"
}
token = generate_token(user_data)

# Validate token
payload = validate_token(token)

# Check if token is valid
result = validate_token(token)
if result["valid"]:
    user_info = result["payload"]
```