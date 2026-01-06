import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/contexts/AuthContext';
import ProtectedRoute from '@site/src/components/ProtectedRoute';
import styles from './auth.module.css';
import authService from '@site/src/services/authService';

function ProfileContent() {
  const { user, login } = useAuth(); // login is essentially 'update user in context' here if we re-fetch
  const [name, setName] = useState(user?.name || '');
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');

  const handleUpdate = async (e) => {
    e.preventDefault();
    setLoading(true);
    setMessage('');
    setError('');

    try {
      const response = await fetch('http://localhost:8000/api/users/me', {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${authService.getToken()}`
        },
        body: JSON.stringify({ name }),
      });

      if (!response.ok) throw new Error('Update failed');
      
      setMessage('Profile updated successfully!');
      // Force refresh user data in context
      window.location.reload(); 
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={`glass ${styles.authCard}`}>
        <h1 className="neon-text">User Profile</h1>
        <p>Manage your personal information.</p>

        {message && <div style={{ color: 'var(--neon-cyan)', marginBottom: '20px' }}>{message}</div>}
        {error && <div className={styles.error}>{error}</div>}

        <div className={styles.formGroup}>
          <label>Email (Cannot be changed)</label>
          <input type="text" value={user?.email || ''} disabled className="glass" style={{ opacity: 0.5 }} />
        </div>

        <form onSubmit={handleUpdate}>
          <div className={styles.formGroup}>
            <label>Name</label>
            <input 
              type="text" 
              value={name} 
              onChange={(e) => setName(e.target.value)} 
              className="glass"
            />
          </div>
          <button type="submit" className="neon-btn primary" style={{ width: '100%' }} disabled={loading}>
            {loading ? 'Saving...' : 'Update Profile'}
          </button>
        </form>
      </div>
    </div>
  );
}

export default function Profile() {
  return (
    <Layout title="Profile">
      <ProtectedRoute>
        <ProfileContent />
      </ProtectedRoute>
    </Layout>
  );
}
