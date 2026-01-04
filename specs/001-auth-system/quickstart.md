# Quickstart Guide: Authentication System

**Feature**: 001-auth-system
**Date**: 2025-12-18
**Status**: Draft

## Overview

This quickstart guide provides instructions for setting up, configuring, and using the authentication system with reusable intelligence skills. The system implements secure user authentication with Better Auth, SQLite database, JWT tokens, and protected route access.

## Prerequisites

- Python 3.11+ installed
- Node.js 16+ installed (for Docusaurus)
- Git installed
- Basic knowledge of FastAPI and React

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup (FastAPI)
```bash
cd api
pip install -r requirements.txt
```

### 3. Frontend Setup (Docusaurus)
```bash
cd book_source
npm install
```

## Configuration

### 1. Environment Variables
Create a `.env` file in the `api` directory:

```env
BETTER_AUTH_URL=http://localhost:3000
BETTER_AUTH_SECRET=31KgOiNyphbh5VJ5HmqZtd8xIh29psIx
DATABASE_URL=sqlite:///./auth.db
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your-email@gmail.com
SMTP_PASSWORD=your-app-password
```

### 2. Database Setup
The system will automatically create the SQLite database (`auth.db`) and required tables on first run.

## Running the System

### 1. Start the Backend
```bash
cd api
python -m uvicorn main:app --reload --port 8000
```

### 2. Start the Frontend
```bash
cd book_source
npm run start
```

## Key Features

### 1. User Registration
- Navigate to `/signup` to create a new account
- Provide name, email, and password
- System validates input and creates secure account

### 2. User Login
- Navigate to `/login` to authenticate
- Enter email and password
- System validates credentials and creates session

### 3. Protected Content Access
- Navigate to `/docs/*` routes to access protected content
- System automatically redirects unauthenticated users to login
- Authenticated users can access all documentation

### 4. Session Management
- Sessions automatically expire after 7 days
- Refresh tokens maintain active sessions
- Logout invalidates current session

### 5. Password Reset
- Use "Forgot Password" link on login page
- Enter registered email to receive reset token
- Use token to set new password

## API Endpoints

### Authentication Endpoints
- `POST /api/auth/signup` - Create new user account
- `POST /api/auth/login` - Authenticate user
- `POST /api/auth/logout` - End current session
- `GET /api/auth/me` - Get current user info
- `GET /api/auth/verify` - Verify authentication status
- `POST /api/auth/refresh` - Refresh JWT token
- `POST /api/auth/forgot-password` - Request password reset
- `POST /api/auth/reset-password` - Update password with token

### Protected Endpoints
- `GET /docs/*` - Access protected documentation (requires authentication)

## Integration with Skills

### 1. Using sqlite-user-store
```python
from auth.database import get_user_by_email

user = get_user_by_email("user@example.com")
```

### 2. Using password-security
```python
from auth.security import hash_password, verify_password

hashed = hash_password("user_password")
is_valid = verify_password("user_password", hashed)
```

### 3. Using jwt-token-manager
```python
from auth.tokens import create_access_token, verify_token

token = create_access_token({"user_id": user.id, "email": user.email})
payload = verify_token(token)
```

### 4. Using protected-route-guard
The route protection is automatically applied to all `/docs/*` routes via middleware.

## Frontend Components

### 1. ProtectedRoute Component
```tsx
import ProtectedRoute from './components/ProtectedRoute';

function App() {
  return (
    <ProtectedRoute>
      <DocumentationPage />
    </ProtectedRoute>
  );
}
```

### 2. AuthContext Usage
```tsx
import { useAuth } from './context/AuthContext';

function MyComponent() {
  const { user, isAuthenticated, login, logout } = useAuth();

  if (!isAuthenticated) {
    return <div>Please log in</div>;
  }

  return <div>Welcome {user?.name}!</div>;
}
```

## Testing

### 1. Backend Tests
```bash
cd api
pytest tests/
```

### 2. Frontend Tests
```bash
cd book_source
npm test
```

## Troubleshooting

### Common Issues

1. **Database Connection Errors**
   - Ensure SQLite file has proper permissions
   - Check DATABASE_URL in environment variables

2. **JWT Validation Failures**
   - Verify BETTER_AUTH_SECRET matches between services
   - Check token expiration times

3. **Route Protection Not Working**
   - Confirm middleware is properly configured
   - Verify route patterns match protected content

4. **Email Sending Failures**
   - Check SMTP configuration in environment variables
   - Verify app password for email service

## Security Best Practices

- Never expose secrets in client-side code
- Use HTTPS in production
- Implement proper rate limiting
- Regularly rotate the BETTER_AUTH_SECRET
- Monitor authentication logs for suspicious activity

## Next Steps

1. Customize the authentication UI to match your design requirements
2. Implement additional security features like 2FA if needed
3. Add user analytics and monitoring
4. Set up automated testing and deployment pipelines