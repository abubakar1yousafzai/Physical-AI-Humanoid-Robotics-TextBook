import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';

export default function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  
  const { login } = useAuth();
  const history = useHistory();

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await login(email, password);
      // On success, redirect to dashboard or home
      history.push('/docs/preface');
    } catch (err) {
      setError(err.message || 'Invalid email or password');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Login" description="Login to your account">
      <div className={styles.authContainer}>
        <div className={`glass ${styles.authCard} fade-in`}>
          <h1 className="neon-text stagger-1">Welcome Back</h1>
          <p className="stagger-2">Login to access your personalized dashboard.</p>
          
          {error && <div className={styles.error}>{error}</div>}
          
          <form onSubmit={handleSubmit} className="stagger-3">
            <div className={styles.formGroup}>
              <label>Email</label>
              <input 
                type="email" 
                value={email} 
                onChange={(e) => setEmail(e.target.value)} 
                required 
                placeholder="email@example.com"
                className="glass"
              />
            </div>
            <div className={styles.formGroup}>
              <label>Password</label>
              <input 
                type="password" 
                value={password} 
                onChange={(e) => setPassword(e.target.value)} 
                required 
                placeholder="********"
                className="glass"
              />
            </div>
            <button 
              type="submit" 
              className="neon-btn secondary" 
              disabled={loading}
              style={{ width: '100%', marginTop: '20px' }}
            >
              {loading ? 'Logging in...' : 'Login'}
            </button>
          </form>
          
          <p style={{ marginTop: '20px', textAlign: 'center' }}>
            Don't have an account? <a href="/signup" style={{ color: 'var(--neon-cyan)' }}>Sign Up</a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
