# ADR: Authentication System Architecture

**Date**: 2025-12-18
**Status**: Proposed
**Feature**: 001-auth-system

## Context

The project requires implementing a secure authentication system to protect documentation content while keeping the chatbot and other features public. The system must integrate with existing FastAPI backend and Docusaurus frontend.

## Decision

We will implement the authentication system using:

1. **Better Auth** as the authentication library
2. **SQLite** as the database for user storage
3. **JWT tokens** with 7-day expiry and refresh tokens
4. **HTTP-only cookies** for session management
5. **6 reusable intelligence skills**: auth-api-builder, jwt-token-manager, password-security, sqlite-user-store, protected-route-guard, and modern-auth-ui
6. **Rate limiting** for security (5 attempts per IP per 15 minutes)
7. **Password reset** functionality with email tokens

## Alternatives Considered

### Authentication Library Options:
- Custom JWT implementation: More complex, higher security risk, more maintenance
- Auth0/other cloud providers: Contradicts constitution requirement for SQLite
- Flask-Security: Doesn't match existing FastAPI stack
- **Better Auth**: Matches requirements, well-maintained, supports required features

### Database Options:
- PostgreSQL: Overkill for this project, contradicts constitution requirement
- MongoDB: Doesn't match SQLite requirement in constitution
- SQLite: Matches constitution requirement, sufficient for user base
- **SQLite**: Selected as per constitution

### Token Management:
- Session-based auth: Requires server-side storage
- JWT with long-lived tokens: Security risk
- JWT with refresh tokens: Balances security and user experience
- **JWT with refresh tokens**: Provides security with good UX

## Consequences

### Positive:
- Secure authentication with industry-standard practices
- Good user experience with refresh tokens
- Complies with project constitution
- Modular architecture using reusable skills
- Proper rate limiting for security

### Negative:
- Additional complexity with refresh token mechanism
- Requires email service for password reset
- More database tables to manage

## Technical Details

### Data Model:
- User: id, name, email, password_hash, created_at, email_verified, is_active
- Session: id, user_id, session_token, expires_at, created_at
- PasswordResetToken: id, user_id, token, expires_at, used

### API Endpoints:
- POST /api/auth/signup
- POST /api/auth/login
- POST /api/auth/logout
- GET /api/auth/me
- GET /api/auth/verify
- POST /api/auth/refresh
- POST /api/auth/forgot-password
- POST /api/auth/reset-password

## Security Considerations

- Passwords hashed with bcrypt (12 rounds)
- JWT tokens with proper expiry
- HTTP-only cookies for session storage
- Rate limiting to prevent brute force
- Input validation and sanitization
- CSRF protection enabled