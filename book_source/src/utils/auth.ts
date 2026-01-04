// Auth utility functions for frontend

// Function to get the auth token from localStorage
export const getAuthToken = (): string | null => {
  return localStorage.getItem('auth_token');
};

// Function to set the auth token in localStorage
export const setAuthToken = (token: string): void => {
  localStorage.setItem('auth_token', token);
};

// Function to remove the auth token from localStorage
export const removeAuthToken = (): void => {
  localStorage.removeItem('auth_token');
};

// Function to get the refresh token from localStorage
export const getRefreshToken = (): string | null => {
  return localStorage.getItem('refresh_token');
};

// Function to set the refresh token in localStorage
export const setRefreshToken = (token: string): void => {
  localStorage.setItem('refresh_token', token);
};

// Function to remove the refresh token from localStorage
export const removeRefreshToken = (): void => {
  localStorage.removeItem('refresh_token');
};

// Function to get the current user from localStorage
export const getCurrentUser = (): any => {
  const userStr = localStorage.getItem('auth_user');
  return userStr ? JSON.parse(userStr) : null;
};

// Function to set the current user in localStorage
export const setCurrentUser = (user: any): void => {
  localStorage.setItem('auth_user', JSON.stringify(user));
};

// Function to remove the current user from localStorage
export const removeCurrentUser = (): void => {
  localStorage.removeItem('auth_user');
};

// Function to check if user is authenticated
export const isAuthenticated = (): boolean => {
  const token = getAuthToken();
  return token !== null && token !== undefined;
};

// Function to clear all auth data from localStorage
export const clearAuthData = (): void => {
  removeAuthToken();
  removeRefreshToken();
  removeCurrentUser();
};

// Function to make authenticated API requests
export const makeAuthenticatedRequest = async (
  url: string,
  options: RequestInit = {}
): Promise<Response> => {
  const token = getAuthToken();

  if (!token) {
    throw new Error('No authentication token available');
  }

  const authenticatedOptions: RequestInit = {
    ...options,
    headers: {
      ...options.headers,
      'Authorization': `Bearer ${token}`,
      'Content-Type': 'application/json',
    },
  };

  let response = await fetch(url, authenticatedOptions);

  // If the response is 401 (Unauthorized), try to refresh the token
  if (response.status === 401) {
    const refreshToken = getRefreshToken();

    if (refreshToken) {
      try {
        // Attempt to refresh the token
        const refreshResponse = await fetch('/api/auth/refresh', {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${refreshToken}`,
            'Content-Type': 'application/json',
          },
        });

        if (refreshResponse.ok) {
          const { access_token } = await refreshResponse.json();
          setAuthToken(access_token);

          // Retry the original request with the new token
          authenticatedOptions.headers = {
            ...authenticatedOptions.headers,
            'Authorization': `Bearer ${access_token}`,
          };

          response = await fetch(url, authenticatedOptions);
        } else {
          // Refresh failed, clear auth data and redirect to login
          clearAuthData();
          window.location.href = '/login';
        }
      } catch (error) {
        console.error('Token refresh failed:', error);
        clearAuthData();
        window.location.href = '/login';
      }
    } else {
      // No refresh token, clear auth data and redirect to login
      clearAuthData();
      window.location.href = '/login';
    }
  }

  return response;
};

// Function to verify authentication status
export const verifyAuthStatus = async (): Promise<boolean> => {
  const token = getAuthToken();

  if (!token) {
    return false;
  }

  try {
    const response = await fetch('/api/auth/verify', {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    return response.ok;
  } catch (error) {
    console.error('Error verifying auth status:', error);
    return false;
  }
};