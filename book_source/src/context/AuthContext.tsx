import React, { createContext, useContext, useReducer, useEffect } from 'react';

import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
// Define the initial state for auth context
const initialState = {
  user: null,
  token: null,
  isAuthenticated: false,
  isLoading: true,
};

// Define auth reducer to handle state changes
const authReducer = (state, action) => {
  switch (action.type) {
    case 'LOGIN_START':
      return {
        ...state,
        isLoading: true,
      };
    case 'LOGIN_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token,
        isAuthenticated: true,
        isLoading: false,
      };
    case 'LOGIN_FAILURE':
      return {
        ...state,
        isLoading: false,
      };
    case 'LOGOUT':
      return {
        ...state,
        user: null,
        token: null,
        isAuthenticated: false,
        isLoading: false,
      };
    case 'SET_USER':
      return {
        ...state,
        user: action.payload,
        isAuthenticated: true,
        isLoading: false,
      };
    case 'RESTORE_TOKEN':
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token,
        isAuthenticated: !!action.payload.token,
        isLoading: false,
      };
    default:
      return state;
  }
};

// Create the AuthContext
const AuthContext = createContext();

// AuthProvider component to wrap around the app
export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);

  useEffect(() => {
    // Check if user is already logged in (from localStorage or cookie)
    const token = localStorage.getItem('auth_token');
    const user = localStorage.getItem('auth_user');

    if (token && user) {
      dispatch({
        type: 'RESTORE_TOKEN',
        payload: {
          token,
          user: JSON.parse(user),
        },
      });
    } else {
      dispatch({ type: 'RESTORE_TOKEN', payload: { token: null, user: null } });
    }
  }, []);

  // Login function
  const login = async (email, password) => {
    dispatch({ type: 'LOGIN_START' });

    try {
      const response = await fetch('http://localhost:8000/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (response.ok) {
        const { access_token, refresh_token } = data;

        // Store tokens in localStorage
        localStorage.setItem('auth_token', access_token);
        localStorage.setItem('refresh_token', refresh_token);

        // Get user info
        const userResponse = await fetch('http://localhost:8000/api/auth/me', {
          headers: {
            'Authorization': `Bearer ${access_token}`,
          },
        });

        if (userResponse.ok) {
          const user = await userResponse.json();
          localStorage.setItem('auth_user', JSON.stringify(user));

          dispatch({
            type: 'LOGIN_SUCCESS',
            payload: {
              user,
              token: access_token,
            },
          });

          return { success: true };
        } else {
          throw new Error('Failed to fetch user info');
        }
      } else {
        throw new Error(data.detail || 'Login failed');
      }
    } catch (error) {
      dispatch({ type: 'LOGIN_FAILURE' });
      return { success: false, error: error.message };
    }
  };

  // Signup function
  const signup = async (email, username, password, firstName, lastName) => {
    try {
      const response = await fetch('http://localhost:8000/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          username,
          password,
          first_name: firstName,
          last_name: lastName
        }),
      });

      const data = await response.json();

      if (response.ok) {
        return { success: true, user: data };
      } else {
        throw new Error(data.detail || 'Signup failed');
      }
    } catch (error) {
      return { success: false, error: error.message };
    }
  };

  // Logout function
  const logout = () => {
    // Remove tokens from localStorage
    localStorage.removeItem('auth_token');
    localStorage.removeItem('refresh_token');
    localStorage.removeItem('auth_user');

    dispatch({ type: 'LOGOUT' });
  };

  // Get current user
  const getCurrentUser = async () => {
    const token = localStorage.getItem('auth_token');

    if (token) {
      try {
        const response = await fetch('http://localhost:8000/api/auth/me', {
          headers: {
            'Authorization': `Bearer ${token}`,
          },
        });

        if (response.ok) {
          const user = await response.json();
          localStorage.setItem('auth_user', JSON.stringify(user));

          dispatch({
            type: 'SET_USER',
            payload: user,
          });

          return user;
        } else {
          // Token might be expired, logout user
          logout();
        }
      } catch (error) {
        console.error('Error getting current user:', error);
        logout();
      }
    }
  };

  // Value object to be provided to consumers
  const value = {
    ...state,
    login,
    signup,
    logout,
    getCurrentUser,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};