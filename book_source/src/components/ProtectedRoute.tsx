import React from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { useAuth } from '../context/AuthContext';

interface ProtectedRouteProps {
  children: React.ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({ children }) => {
  const { isAuthenticated, isLoading } = useAuth();
  const location = useLocation();
  const history = useHistory();

  // During SSR, just render children to avoid hydration issues
  if (typeof window === 'undefined') {
    return <>{children}</>;
  }

  // If loading, show a loading indicator
  if (isLoading) {
    return (
      <div className="flex items-center justify-center min-h-screen">
        <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
      </div>
    );
  }

  // If not authenticated, redirect to login with return URL
  if (!isAuthenticated) {
    // Store the current location so user can be redirected back after login
    localStorage.setItem('returnTo', location.pathname + location.search);
    history.push('/login');
    return null; // Return null while redirecting
  }

  // If authenticated, render the protected content
  return <>{children}</>;
};

export default ProtectedRoute;