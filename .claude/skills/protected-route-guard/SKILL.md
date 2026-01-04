# Skill: protected-route-guard

## Purpose
Frontend React component/HOC that protects routes by checking user authentication status. Redirects unauthenticated users to login page.

## Capabilities
This skill can:
- Check if user has valid authentication (JWT token in cookie)
- Verify token with backend API
- Redirect unauthenticated users to `/login`
- Preserve original requested URL for post-login redirect
- Display loading state during auth check
- Handle expired tokens gracefully

## How it Performs Actions
1. **Auth Check on Route Load**:
   - Check for auth token in cookies
   - If no token → redirect to login
   - If token exists → verify with backend
   - If valid → render protected content
   - If invalid → clear token, redirect to login

2. **Preserve Redirect URL**:
   - Store original URL in state/localStorage
   - After login, redirect back to original page

3. **Loading State**:
   - Show spinner/skeleton while verifying token
   - Prevent flash of unauthorized content

## Input (to the skill)
- Protected component/page to render
- Backend verify endpoint URL
- Login page path: `/login`

## Output (from the skill)
- Rendered protected content (if authenticated)
- Redirect to login (if not authenticated)
- Loading indicator during verification

## Usage Notes
- Implemented as React Higher-Order Component (HOC)
- Or as wrapper component `<ProtectedRoute>`
- Uses React Router or Docusaurus routing
- Calls backend `/api/auth/verify` endpoint

## Integration
- Wrap all `/docs` pages with this guard
- Used in Docusaurus custom pages
- Integrated with AuthContext for global state

## Example Usage
```typescript
<ProtectedRoute>
  <DocsPage />
</ProtectedRoute>
```

