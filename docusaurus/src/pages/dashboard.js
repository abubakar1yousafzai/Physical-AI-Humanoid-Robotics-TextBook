import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/contexts/AuthContext';
import ProtectedRoute from '@site/src/components/ProtectedRoute';
import authService from '@site/src/services/authService';
import Link from '@docusaurus/Link';

function DashboardContent() {
  const { user } = useAuth();
  const [progress, setProgress] = useState([]);
  const [bookmarks, setBookmarks] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const [progressRes, bookmarksRes] = await Promise.all([
          fetch('http://localhost:8000/api/users/me/progress', {
            headers: { 'Authorization': `Bearer ${authService.getToken()}` }
          }),
          fetch('http://localhost:8000/api/users/me/bookmarks', {
            headers: { 'Authorization': `Bearer ${authService.getToken()}` }
          })
        ]);

        if (progressRes.ok) setProgress(await progressRes.json());
        if (bookmarksRes.ok) setBookmarks(await bookmarksRes.json());
      } catch (err) {
        console.error('Failed to fetch dashboard data:', err);
      } finally {
        setLoading(false);
      }
    };

    fetchData();
  }, []);

  const completedCount = progress.filter(p => p.completed).length;
  // Total chapters from constitution principle or just hardcode for now
  const totalChapters = 25; 
  const completionPercentage = Math.round((completedCount / totalChapters) * 100);

  return (
    <div className="container margin-vert--lg">
      <h1 className="neon-text">Learning Dashboard</h1>
      <p>Welcome back, <strong>{user?.name || 'Explorer'}</strong>!</p>

      <div className="row">
        <div className="col col--4">
          <div className="card glass margin-bottom--md">
            <div className="card__header">
              <h3>Course Progress</h3>
            </div>
            <div className="card__body">
              <div style={{ fontSize: '3rem', fontWeight: 'bold', color: 'var(--neon-cyan)' }}>
                {completionPercentage}%
              </div>
              <p>{completedCount} of {totalChapters} chapters completed</p>
            </div>
          </div>
        </div>

        <div className="col col--8">
          <div className="card glass margin-bottom--md">
            <div className="card__header">
              <h3>My Bookmarks</h3>
            </div>
            <div className="card__body">
              {bookmarks.length === 0 ? (
                <p>No bookmarks yet.</p>
              ) : (
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: '10px' }}>
                  {bookmarks.map(b => (
                    <Link 
                      key={b.id} 
                      to={`/docs/${b.chapter_id}`} 
                      className="neon-btn secondary"
                      style={{ padding: '5px 15px', fontSize: '0.8rem' }}
                    >
                      {b.chapter_id}
                    </Link>
                  ))}
                </div>
              )}
            </div>
          </div>

          <div className="card glass">
            <div className="card__header">
              <h3>Recently Completed</h3>
            </div>
            <div className="card__body">
              {progress.length === 0 ? (
                <p>No chapters completed yet. <Link to="/docs/preface">Start reading!</Link></p>
              ) : (
                <ul className="list-unstyled">
                  {progress.filter(p => p.completed).slice(0, 5).map(item => (
                    <li key={item.id} className="margin-bottom--sm">
                      ✅ {item.chapter_id}
                    </li>
                  ))}
                </ul>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function Dashboard() {
  return (
    <Layout title="Dashboard">
      <ProtectedRoute>
        <DashboardContent />
      </ProtectedRoute>
    </Layout>
  );
}
