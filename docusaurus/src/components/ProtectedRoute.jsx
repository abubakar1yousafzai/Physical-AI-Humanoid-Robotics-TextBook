import React from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

export default function ProtectedRoute({ children }) {
  const { isAuthenticated, loading } = useAuth();
  const history = useHistory();

  React.useEffect(() => {
    if (!loading && !isAuthenticated) {
      history.push('/login');
    }
  }, [isAuthenticated, loading, history]);

  if (loading || !isAuthenticated) {
    return (
      <div style={{ 
        display: 'flex', 
        justifyContent: 'center', 
        alignItems: 'center', 
        height: '50vh',
        color: 'var(--neon-cyan)'
      }}>
        Loading secure session...
      </div>
    );
  }

  return <>{children}</>;
}
