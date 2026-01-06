import React from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import Link from '@docusaurus/Link';
import styles from './UserMenu.module.css';

export default function UserMenu() {
  const { user, isAuthenticated, logout } = useAuth();

  if (!isAuthenticated) {
    return (
      <div className={styles.navLinks}>
        <Link to="/login" className="neon-btn secondary" style={{ padding: '8px 20px', fontSize: '0.9rem' }}>
          Login
        </Link>
        <Link to="/signup" className="neon-btn primary" style={{ padding: '8px 20px', fontSize: '0.9rem', marginLeft: '10px' }}>
          Sign Up
        </Link>
      </div>
    );
  }

  return (
    <div className={styles.userDropdown}>
      <button className={`glass ${styles.userBtn}`}>
        <span className={styles.userName}>{user.name || 'User'}</span>
        <span className={styles.avatar}>👤</span>
      </button>
      <div className={`glass ${styles.dropdownContent}`}>
        <Link to="/dashboard">Dashboard</Link>
        <Link to="/profile">Profile</Link>
        <hr />
        <button onClick={logout} className={styles.logoutBtn}>Logout</button>
      </div>
    </div>
  );
}
