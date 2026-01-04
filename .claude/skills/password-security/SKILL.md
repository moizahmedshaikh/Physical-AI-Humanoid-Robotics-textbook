# Skill: password-security

## Purpose
Handle secure password hashing and verification using industry-standard algorithms (bcrypt or argon2). Ensures passwords are never stored in plain text.

## Capabilities
This skill can:
- Hash passwords with salt using bcrypt/argon2
- Verify passwords against stored hashes
- Validate password strength (minimum requirements)
- Generate secure password hashes
- Compare passwords securely (timing-safe)

## How it Performs Actions
1. **Password Hashing**:
   - Accept plain text password
   - Generate salt
   - Hash password with bcrypt/argon2
   - Return hashed password string

2. **Password Verification**:
   - Accept plain text password and stored hash
   - Compare using timing-safe method
   - Return boolean (valid/invalid)

3. **Password Validation**:
   - Check minimum length (8 characters)
   - Optionally check complexity rules
   - Return validation result

## Input (to the skill)
- Plain password: `string`
- Stored hash: `string` (for verification)
- Validation rules: `{min_length: 8}`

## Output (from the skill)
- Hashed password: `string`
- Verification result: `boolean`
- Validation errors: `list[string]`

## Usage Notes
- Uses bcrypt library (recommended for most cases)
- Hash complexity: 12 rounds minimum
- Never log or expose passwords
- Timing-safe comparison prevents timing attacks

## Integration
- Used in signup endpoint (hashing)
- Used in login endpoint (verification)
- Called before storing user in database

## Example Usage
```python
# Hash a password
plain_password = "mySecurePassword123"
hashed_password = hash_password(plain_password)

# Verify a password
is_valid = verify_password(plain_password, stored_hash)

# Validate password strength
errors = validate_password_strength(plain_password)
if not errors:
    print("Password is strong enough")
else:
    print(f"Password issues: {errors}")
```