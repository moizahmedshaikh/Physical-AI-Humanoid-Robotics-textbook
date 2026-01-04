from datetime import datetime
from typing import Optional, Dict, Any
import sqlite3
from auth.database import get_db, init_db

class User:
    """User model for authentication system."""

    def __init__(self, id: str = None, email: str = None, username: str = None,
                 password_hash: str = None, first_name: str = None,
                 last_name: str = None, created_at: datetime = None,
                 updated_at: datetime = None, is_active: bool = True):
        self.id = id
        self.email = email
        self.username = username
        self.password_hash = password_hash
        self.first_name = first_name
        self.last_name = last_name
        self.created_at = created_at or datetime.utcnow()
        self.updated_at = updated_at or datetime.utcnow()
        self.is_active = is_active

    def save(self):
        """Save user to database."""
        with get_db() as conn:
            cursor = conn.cursor()
            if self.id:
                # Update existing user
                cursor.execute('''
                    UPDATE users
                    SET email = ?, username = ?, password_hash = ?,
                        first_name = ?, last_name = ?, updated_at = ?, is_active = ?
                    WHERE id = ?
                ''', (self.email, self.username, self.password_hash,
                      self.first_name, self.last_name,
                      datetime.utcnow(), self.is_active, self.id))
            else:
                # Insert new user
                import uuid
                self.id = str(uuid.uuid4())
                cursor.execute('''
                    INSERT INTO users (id, email, username, password_hash,
                                      first_name, last_name, created_at, updated_at, is_active)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''', (self.id, self.email, self.username, self.password_hash,
                      self.first_name, self.last_name,
                      self.created_at, self.updated_at, self.is_active))
            conn.commit()

    @classmethod
    def find_by_email(cls, email: str) -> Optional['User']:
        """Find user by email."""
        with get_db() as conn:
            cursor = conn.cursor()
            cursor.execute('SELECT * FROM users WHERE email = ?', (email,))
            row = cursor.fetchone()
            if row:
                # Convert string dates to datetime objects if they exist
                created_at = cls._str_to_datetime(row['created_at'])
                updated_at = cls._str_to_datetime(row['updated_at'])
                return cls(
                    id=row['id'],
                    email=row['email'],
                    username=row['username'],
                    password_hash=row['password_hash'],
                    first_name=row['first_name'],
                    last_name=row['last_name'],
                    created_at=created_at,
                    updated_at=updated_at,
                    is_active=bool(row['is_active'])
                )
            return None

    @classmethod
    def find_by_id(cls, user_id: str) -> Optional['User']:
        """Find user by ID."""
        with get_db() as conn:
            cursor = conn.cursor()
            cursor.execute('SELECT * FROM users WHERE id = ?', (user_id,))
            row = cursor.fetchone()
            if row:
                # Convert string dates to datetime objects if they exist
                created_at = cls._str_to_datetime(row['created_at'])
                updated_at = cls._str_to_datetime(row['updated_at'])
                return cls(
                    id=row['id'],
                    email=row['email'],
                    username=row['username'],
                    password_hash=row['password_hash'],
                    first_name=row['first_name'],
                    last_name=row['last_name'],
                    created_at=created_at,
                    updated_at=updated_at,
                    is_active=bool(row['is_active'])
                )
            return None

    @classmethod
    def find_by_username(cls, username: str) -> Optional['User']:
        """Find user by username."""
        with get_db() as conn:
            cursor = conn.cursor()
            cursor.execute('SELECT * FROM users WHERE username = ?', (username,))
            row = cursor.fetchone()
            if row:
                # Convert string dates to datetime objects if they exist
                created_at = cls._str_to_datetime(row['created_at'])
                updated_at = cls._str_to_datetime(row['updated_at'])
                return cls(
                    id=row['id'],
                    email=row['email'],
                    username=row['username'],
                    password_hash=row['password_hash'],
                    first_name=row['first_name'],
                    last_name=row['last_name'],
                    created_at=created_at,
                    updated_at=updated_at,
                    is_active=bool(row['is_active'])
                )
            return None

    @staticmethod
    def _str_to_datetime(date_str):
        """Convert string to datetime object if it exists."""
        if date_str is None:
            return None
        if isinstance(date_str, str):
            try:
                # Try to parse ISO format datetime string
                from datetime import datetime
                return datetime.fromisoformat(date_str.replace('Z', '+00:00'))
            except ValueError:
                # If parsing fails, return as is
                return date_str
        return date_str

    def to_dict(self) -> Dict[str, Any]:
        """Convert user object to dictionary."""
        return {
            'id': self.id,
            'email': self.email,
            'username': self.username,
            'first_name': self.first_name,
            'last_name': self.last_name,
            'created_at': self.created_at.isoformat() if self.created_at else None,
            'updated_at': self.updated_at.isoformat() if self.updated_at else None,
            'is_active': self.is_active
        }