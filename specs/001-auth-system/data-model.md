# Authentication System Data Model

## User Entity

### User
- **id**: string (UUID) - Unique identifier for the user
- **email**: string - User's email address (unique, required)
- **username**: string - User's chosen username (unique, required)
- **password_hash**: string - Hashed password using bcrypt or similar
- **password_salt**: string - Salt used for password hashing
- **first_name**: string - User's first name (optional)
- **last_name**: string - User's last name (optional)
- **phone**: string - User's phone number (optional)
- **avatar_url**: string - URL to user's profile picture (optional)
- **bio**: string - User's biography (optional, max 500 characters)
- **role**: string - User's role (e.g., "user", "admin", "moderator") - default: "user"
- **status**: string - Account status ("active", "inactive", "suspended", "pending_verification") - default: "pending_verification"
- **email_verified**: boolean - Whether the email has been verified - default: false
- **two_factor_enabled**: boolean - Whether 2FA is enabled - default: false
- **two_factor_secret**: string - Secret key for TOTP (optional, encrypted)
- **failed_login_attempts**: number - Count of failed login attempts - default: 0
- **locked_until**: datetime - Timestamp until which account is locked (optional)
- **last_login_at**: datetime - Timestamp of last successful login (optional)
- **created_at**: datetime - Account creation timestamp
- **updated_at**: datetime - Last update timestamp
- **deleted_at**: datetime - Soft delete timestamp (optional)

### User Preferences
- **id**: string (UUID) - Unique identifier
- **user_id**: string (UUID) - Foreign key to User
- **theme**: string - UI theme preference ("light", "dark", "system") - default: "system"
- **language**: string - Preferred language code (e.g., "en", "es") - default: "en"
- **notifications_enabled**: boolean - Whether to receive notifications - default: true
- **email_notifications**: boolean - Whether to receive email notifications - default: true
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp

## Session Management

### Session
- **id**: string (UUID) - Unique session identifier
- **user_id**: string (UUID) - Foreign key to User
- **token**: string - Session token (encrypted)
- **token_type**: string - Type of token ("session", "bearer", "refresh") - default: "session"
- **expires_at**: datetime - Session expiration timestamp
- **created_at**: datetime - Session creation timestamp
- **updated_at**: datetime - Last update timestamp
- **last_accessed_at**: datetime - Last time session was used
- **ip_address**: string - IP address of session creator
- **user_agent**: string - User agent string of session creator
- **revoked**: boolean - Whether the session has been revoked - default: false
- **revoked_at**: datetime - Timestamp when session was revoked (optional)

### Refresh Token
- **id**: string (UUID) - Unique identifier
- **user_id**: string (UUID) - Foreign key to User
- **token_hash**: string - Hash of the refresh token
- **expires_at**: datetime - Token expiration timestamp
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp
- **revoked**: boolean - Whether the token has been revoked - default: false
- **revoked_at**: datetime - Timestamp when token was revoked (optional)

## Authentication Tokens

### JWT Token
- **id**: string (UUID) - Unique identifier
- **user_id**: string (UUID) - Foreign key to User
- **token_id**: string - Unique token identifier (jti claim)
- **type**: string - Token type ("access", "id") - default: "access"
- **expires_at**: datetime - Token expiration timestamp
- **created_at**: datetime - Creation timestamp
- **revoked**: boolean - Whether the token has been revoked - default: false
- **revoked_at**: datetime - Timestamp when token was revoked (optional)

## Verification and Recovery

### Email Verification Token
- **id**: string (UUID) - Unique identifier
- **user_id**: string (UUID) - Foreign key to User
- **token**: string - Verification token
- **expires_at**: datetime - Token expiration timestamp
- **used**: boolean - Whether the token has been used - default: false
- **used_at**: datetime - Timestamp when token was used (optional)
- **created_at**: datetime - Creation timestamp

### Password Reset Token
- **id**: string (UUID) - Unique identifier
- **user_id**: string (UUID) - Foreign key to User
- **token**: string - Reset token
- **expires_at**: datetime - Token expiration timestamp
- **used**: boolean - Whether the token has been used - default: false
- **used_at**: datetime - Timestamp when token was used (optional)
- **created_at**: datetime - Creation timestamp

## Security Logs

### Login Attempt
- **id**: string (UUID) - Unique identifier
- **user_id**: string (UUID) - Foreign key to User (optional, null if user not found)
- **email**: string - Email used for login attempt
- **success**: boolean - Whether the login was successful
- **ip_address**: string - IP address of the login attempt
- **user_agent**: string - User agent string
- **created_at**: datetime - Timestamp of the attempt

### Security Event
- **id**: string (UUID) - Unique identifier
- **user_id**: string (UUID) - Foreign key to User
- **event_type**: string - Type of security event ("login", "logout", "password_change", "2fa_enabled", "2fa_disabled", "session_revoked", "account_locked")
- **description**: string - Description of the event
- **ip_address**: string - IP address associated with the event
- **user_agent**: string - User agent string
- **metadata**: JSON - Additional event-specific data
- **created_at**: datetime - Timestamp of the event

## Relationships

### User -> User Preferences (One-to-One)
- A User has zero or one User Preferences
- Foreign key: user_preferences.user_id references users.id

### User -> Sessions (One-to-Many)
- A User has zero or many Sessions
- Foreign key: sessions.user_id references users.id

### User -> Refresh Tokens (One-to-Many)
- A User has zero or many Refresh Tokens
- Foreign key: refresh_tokens.user_id references users.id

### User -> JWT Tokens (One-to-Many)
- A User has zero or many JWT Tokens
- Foreign key: jwt_tokens.user_id references users.id

### User -> Email Verification Tokens (One-to-Many)
- A User has zero or many Email Verification Tokens
- Foreign key: email_verification_tokens.user_id references users.id

### User -> Password Reset Tokens (One-to-Many)
- A User has zero or many Password Reset Tokens
- Foreign key: password_reset_tokens.user_id references users.id

### User -> Login Attempts (One-to-Many)
- A User has zero or many Login Attempts
- Foreign key: login_attempts.user_id references users.id

### User -> Security Events (One-to-Many)
- A User has zero or many Security Events
- Foreign key: security_events.user_id references users.id

## Indexes

### Users Table
- Index on email (unique)
- Index on username (unique)
- Index on created_at
- Index on updated_at
- Index on status
- Index on email_verified

### Sessions Table
- Index on user_id
- Index on token (unique, encrypted)
- Index on expires_at
- Index on last_accessed_at

### Refresh Tokens Table
- Index on token_hash (unique)
- Index on user_id
- Index on expires_at

### JWT Tokens Table
- Index on token_id (unique)
- Index on user_id
- Index on expires_at

### Verification Tokens Tables
- Index on token (unique)
- Index on user_id
- Index on expires_at
- Index on used

### Security Logs Tables
- Index on user_id
- Index on created_at
- Index on ip_address

## Constraints

### Users Table
- Email must be a valid email format
- Username must be 3-30 characters, alphanumeric and underscores/hyphens only
- Password must meet complexity requirements (min 8 chars, 1 uppercase, 1 lowercase, 1 number, 1 special char)
- Status must be one of the allowed values
- Role must be one of the allowed values
- created_at cannot be in the future
- updated_at cannot be in the future

### Sessions Table
- expires_at must be in the future
- token must be unique when not revoked

### Refresh Tokens Table
- expires_at must be in the future
- token_hash must be unique when not revoked

### Verification Tokens Tables
- expires_at must be in the future
- A token cannot be used after it expires
- A token cannot be used more than once

### Security Events Table
- event_type must be one of the allowed values
- created_at cannot be in the future