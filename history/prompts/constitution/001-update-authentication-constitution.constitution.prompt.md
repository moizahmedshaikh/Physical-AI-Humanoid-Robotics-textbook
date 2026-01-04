---
id: 1
title: update-authentication-constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude
command: sp.constitution
labels: [authentication, constitution, security]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - none
---

## Prompt

update constitution for implementing user authentication in a Docusaurus-based Physical AI & Humanoid Robotics textbook project.

## Context
- Existing project: Docusaurus book with RAG chatbot and Urdu translation features
- Need to add: Better Auth authentication system
- Goal: Protect book content, require login to view `/docs` pages

## Technical Stack Requirements
- **Authentication Library**: Better Auth (https://www.better-auth.com/)
- **Database**: SQLite (local file, NOT infrastructure DB)
- **Session**: JWT tokens + HTTP-only cookies
- **Backend**: FastAPI (existing backend to be extended)
- **Frontend**: Docusaurus React components

## Authentication Requirements

### User Data (Simple)
- Name (required)
- Email (required, unique)
- Password (required, hashed)
- NO additional fields needed

### Protected vs Public Content
**Protected** (Login required):
- All `/docs/*` pages (book content)

**Public** (No login required):
- Home page
- Login page (`/login`)
- Signup page (`/signup`)
- Chatbot (do NOT modify existing chatbot)
- Urdu translation feature (keep public)

### Authentication Flow
1. User clicks "Book" in navbar
2. Frontend checks auth status (JWT presence)
3. If not authenticated → redirect to `/login`
4. User fills login/signup form (name, email, password)
5. Form submits to FastAPI auth routes
6. Better Auth:
   - Creates/authenticates user
   - Stores in SQLite (`auth.db` file)
   - Issues JWT session token
   - Sets HTTP-only cookie
7. Authenticated user can view `/docs` content
8. Logout: session invalidated, redirect to home

### Backend API Endpoints Needed
- `POST /api/auth/signup` - Create new user
- `POST /api/auth/login` - Authenticate user
- `POST /api/auth/logout` - Invalidate session
- `GET /api/auth/me` - Get current user
- `GET /api/auth/verify` - Verify JWT token

### Security Requirements
- Passwords hashed (bcrypt/argon2)
- JWT expiry: 7 days
- HTTP-only cookies (not accessible via JS)
- CSRF protection enabled
- Secure flag in production

### Environment Variables
```env
BETTER_AUTH_URL=http://localhost:3000
BETTER_AUTH_SECRET=31KgOiNyphbh5VJ5HmqZtd8xIh29psIx
DATABASE_URL=sqlite:///./auth.db
```

### File Structure
```
api/                          # FastAPI backend
├── auth/
│   ├── routes.py            # Auth endpoints
│   ├── models.py            # User model
│   ├── config.py            # Better Auth config
│   └── middleware.py        # JWT verification
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
```

## What NOT to Include
❌ Social login (Google, GitHub)
❌ Magic links / passwordless
❌ Email verification
❌ Password reset
❌ User profile editing
❌ Role-based access control
❌ Multi-factor authentication
❌ User background questions (save for future personalization feature)
❌ DO NOT modify existing chatbot
❌ OAuth providers

## Success Criteria
✅ Users can signup with name, email, password
✅ Users can login with credentials
✅ Only authenticated users access `/docs`
✅ Users can logout
✅ Sessions persist across reloads
✅ Chatbot remains public

## Implementation Approach
- Follow Spec-Kit Plus workflow: constitution → specify → plan → tasks → implement
- No vibe coding - document everything
- Better Auth documentation is the source of truth
- SQLite is just a local file, not infrastructure
- JWT tokens for session management
- Simple authentication only - no complex features

## Bonus Points
This feature earns **50 bonus points** for implementing Better Auth with user signu

## Response snapshot

Updated the project constitution to include authentication system requirements for the Physical AI & Humanoid Robotics textbook project. The changes include:

1. Updated version from 2.1.0 to 2.2.0
2. Added new section "## 14. Authentication System Requirements" with all the specified requirements
3. Updated the minimal acceptance criteria to include the authentication system
4. Updated the plan template to include authentication constitution checks
5. Updated the spec template to include authentication functional requirements
6. Updated the tasks template to include authentication-related foundational tasks
7. Created a Sync Impact Report showing the changes made

The constitution now properly documents the Better Auth implementation requirements, including user data requirements, protected vs public content, authentication flow, backend API endpoints, security requirements, environment variables, and file structure as specified in the user requirements.

## Outcome

- ✅ Impact: Updated project constitution to include authentication requirements
- 🧪 Tests: none
- 📁 Files: Updated constitution and related templates
- 🔁 Next prompts: Can proceed with specification and planning for authentication feature
- 🧠 Reflection: The constitution now properly captures the authentication requirements for the textbook project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A