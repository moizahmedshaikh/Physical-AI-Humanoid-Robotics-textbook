# Skill: sqlite-user-store

## Purpose
Manage user data persistence in SQLite database. Handles user creation, retrieval, updates, and email uniqueness validation.

## Capabilities
This skill can:
- Create users table in SQLite if not exists
- Insert new user records (name, email, hashed_password)
- Query user by email
- Query user by ID
- Check if email already exists (uniqueness)
- Update user data (if needed)
- Handle database connections and transactions

## How it Performs Actions
1. **Initialize Database**:
   - Create SQLite connection
   - Create users table schema
   - Set up indexes on email field

2. **Create User**:
   - Accept user data (name, email, hashed_password)
   - Check email uniqueness
   - Insert into users table
   - Return user ID or error

3. **Get User**:
   - Accept email or user_id
   - Query database
   - Return user object or None

4. **Check Email Exists**:
   - Query by email
   - Return boolean

## Input (to the skill)
- Database path: `sqlite:///./auth.db`
- User data: `{name, email, hashed_password}`
- Query parameter: `email` or `user_id`

## Output (from the skill)
- User object: `{id, name, email, hashed_password, created_at}`
- Success status: `boolean`
- Error messages: `string`

## Usage Notes
- SQLite file stored in backend root: `api/auth.db`
- Email field has unique constraint
- Database is NOT infrastructure - just a local file
- Use SQLAlchemy or raw SQL with sqlite3 library
- Use check_same_thread=False for SQLite with FastAPI.

## Database Schema
```sql
CREATE TABLE users (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    email TEXT UNIQUE NOT NULL,
    hashed_password TEXT NOT NULL,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

## Example Usage
```python
# Create user
user_data = {
    "name": "John Doe",
    "email": "john@example.com",
    "hashed_password": "hashed_password_here"
}
result = create_user(user_data)

# Get user by email
user = get_user_by_email("john@example.com")

# Check if email exists
exists = check_email_exists("john@example.com")
```
