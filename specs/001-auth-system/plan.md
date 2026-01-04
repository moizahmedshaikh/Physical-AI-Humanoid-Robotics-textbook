# Implementation Plan: Authentication System with Reusable Intelligence

**Branch**: `001-auth-system` | **Date**: 2025-12-18 | **Spec**: [specs/001-auth-system/spec.md](specs/001-auth-system/spec.md)
**Input**: Feature specification from `/specs/001-auth-system/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a comprehensive authentication system using reusable intelligence from .claude/skills/ including jwt-token-manager, password-security, sqlite-user-store, protected-route-guard, auth-api-builder, and modern-auth-ui. The system will protect `/docs/*` routes while keeping chatbot, home page, and login/signup pages public. The implementation will use Better Auth with SQLite backend, JWT tokens, and HTTP-only cookies following the security requirements in the constitution.

## Technical Context

**Language/Version**: Python 3.11, TypeScript/JavaScript for frontend
**Primary Dependencies**: FastAPI, Better Auth, SQLite, bcrypt, JWT, React for Docusaurus
**Storage**: SQLite database file (`auth.db`)
**Testing**: pytest for backend, React testing library for frontend
**Target Platform**: Web application (Docusaurus + FastAPI backend)
**Project Type**: Web - extends existing backend and Docusaurus frontend
**Performance Goals**: <2 second login response time, 95% success rate for auth operations
**Constraints**: Must follow security requirements (JWT expiry: 7 days, HTTP-only cookies, bcrypt hashing)
**Scale/Scope**: Individual textbook access, multiple concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Authentication System Requirements
- **Authentication Library**: Better Auth (https://www.better-auth.com/) ✅
- **Database**: SQLite (local file, NOT infrastructure DB) ✅
- **Session**: JWT tokens + HTTP-only cookies ✅
- **Backend**: FastAPI (existing backend to be extended) ✅
- **Frontend**: Docusaurus React components ✅

### User Data (Simple)
- Name (required) ✅
- Email (required, unique) ✅
- Password (required, hashed) ✅
- NO additional fields needed ✅

### Protected vs Public Content
**Protected** (Login required):
- All `/docs/*` pages (book content) ✅

**Public** (No login required):
- Home page ✅
- Login page (`/login`) ✅
- Signup page (`/signup`) ✅
- Chatbot (do NOT modify existing chatbot) ✅
- Urdu translation feature (keep public) ✅

### Security Requirements
- Passwords hashed (bcrypt/argon2) ✅
- JWT expiry: 7 days ✅
- HTTP-only cookies (not accessible via JS) ✅
- CSRF protection enabled ✅
- Secure flag in production ✅

### What NOT to Include
❌ Social login (Google, GitHub) ✅
❌ Magic links / passwordless ✅
❌ User profile editing ✅
❌ Role-based access control ✅
❌ Multi-factor authentication ✅
❌ User background questions (save for future personalization feature) ✅
❌ DO NOT modify existing chatbot ✅
❌ OAuth providers ✅

*Note: There is a conflict between the specification (FR-017 requires password reset) and the constitution (prohibits password reset). The specification takes precedence as it was created with additional clarifications.*

### Additional Requirements from Spec
- Refresh token mechanism (FR-016) ✅
- Rate limiting (FR-018) ✅
- Indefinite retention (FR-020) ✅

## Project Structure

### Documentation (this feature)

```text
specs/001-auth-system/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
api/                          # FastAPI backend
├── auth/
│   ├── routes.py            # Auth endpoints
│   ├── models.py            # User model
│   ├── config.py            # Better Auth config
│   └── middleware.py        # JWT verification and route protection
├── main.py
├── auth.db                  # SQLite file
└── requirements.txt

book_source/
├── src/
│   ├── components/
│   │   └── ProtectedRoute.tsx   # Route protection
│   ├── context/
│   │   └── AuthContext.tsx      # Auth state
│   ├── pages/
│   │   ├── login.tsx            # Login page
│   │   └── signup.tsx           # Signup page
│   └── utils/
│       └── auth.ts              # Auth helpers
└── docs/                    # Protected content

.claude/
├── skills/
│   ├── auth-api-builder/    # API endpoint creation
│   ├── jwt-token-manager/   # JWT token handling
│   ├── password-security/   # Password hashing
│   ├── sqlite-user-store/   # Database operations
│   ├── protected-route-guard/ # Route protection
│   └── modern-auth-ui/      # UI components
└── agents/
    └── auth-agent.md        # Orchestration agent
```



**Structure Decision**: Web application with separate backend (FastAPI) and frontend (Docusaurus React components) following the constitution's requirements for Better Auth, SQLite, and JWT implementation.

## Phase 0: Research & Clarification (COMPLETED)

### Research Tasks
1. **Better Auth Integration**: How to integrate Better Auth with existing FastAPI backend and SQLite
2. **JWT Token Management**: Implementation details for 7-day expiry with refresh tokens
3. **Route Protection**: How to protect `/docs/*` routes in Docusaurus while keeping other pages public

5. **Rate Limiting**: Implementation approach for 5 attempts per IP per 15 minutes
6. **Session Management**: HTTP-only cookie handling in Docusaurus environment

### Outcomes
- ✅ `research.md` with all technical decisions documented
- ✅ `data-model.md` with user and session entity definitions
- ✅ `contracts/auth-api-contract.yaml` for all authentication endpoints
- ✅ `quickstart.md` guide for setting up the authentication system

## Phase 1: Design & Architecture (COMPLETED)

### 1.1 Data Model
- **User Entity**: ID, name, email (unique), hashed_password, created_at, is_active
- **Session Entity**: session_id, user_id, jwt_token, expires_at, created_at, last_accessed

### 1.2 API Contracts
- `POST /api/auth/signup` - Create new user with name, email, password
- `POST /api/auth/login` - Authenticate user with email, password
- `POST /api/auth/logout` - Invalidate session
- `GET /api/auth/me` - Get current user info
- `GET /api/auth/verify` - Verify JWT token
- `POST /api/auth/refresh` - Refresh JWT token


### 1.3 Frontend Components
- Login page with form validation and error handling
- Signup page with form validation and error handling
- ProtectedRoute component for route protection
- AuthContext for global authentication state
- Auth utilities for token management

### 1.4 Database Schema
- Users table with proper indexing
- Sessions table for JWT tracking
- Indexes for performance optimization

## Phase 2: Implementation Plan

### Phase 2A: Backend Foundation (Days 1-2)
- Set up SQLite database with user table (sqlite-user-store skill)
- Implement password hashing (password-security skill)
- Configure JWT generation with 7-day expiry (jwt-token-manager skill)
- Create FastAPI auth endpoints (auth-api-builder skill)
- Set up Better Auth integration
- Configure environment variables
- Implement rate limiting functionality (auth-api-builder skill per clarification)

### Phase 2B: Frontend UI (Days 2-3)
- Create modern auth UI components (modern-auth-ui skill)
- Implement responsive login/signup forms with glassmorphism design
- Create AuthContext for state management
- Implement ProtectedRoute component

### Phase 2C: Route Protection (Days 3-4)
- Implement protected-route-guard to protect `/docs/*` routes
- Ensure chatbot remains public
- Test navigation and redirection logic
- Implement session persistence across page reloads

### Phase 2D: Advanced Features (Days 4-5)
- Implement refresh token mechanism (jwt-token-manager skill)
- Add rate limiting for authentication attempts (auth-api-builder skill)
- Test token refresh during active sessions
- Verify rate limiting with multiple failed login attempts

### Phase 2E: Integration & Testing (Days 5-6)
- Integrate all components and test full authentication flow
- Test mobile responsiveness
- Verify dark mode compatibility
- Performance testing
- Security audit
