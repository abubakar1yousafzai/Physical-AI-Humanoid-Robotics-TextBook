import React, { useState, useEffect, useRef } from 'react';
import './HistoryPanel.css';

const HistoryPanel = ({ 
  isOpen, 
  onClose, 
  conversations, 
  activeThreadId, 
  onSelectThread, 
  onNewChat,
  onDeleteConversation
}) => {
  const [openMenuId, setOpenMenuId] = useState(null);
  const menuRef = useRef(null);

  // Click outside handler to close menu
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (menuRef.current && !menuRef.current.contains(event.target)) {
        setOpenMenuId(null);
      }
    };

    if (openMenuId) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [openMenuId]);

  const toggleMenu = (e, id) => {
    e.stopPropagation();
    setOpenMenuId(openMenuId === id ? null : id);
  };

  const handleDelete = (e, id) => {
    e.stopPropagation();
    if (window.confirm('Are you sure you want to delete this conversation?')) {
      onDeleteConversation(id);
      setOpenMenuId(null);
    }
  };

  return (
    <div className={`history-panel ${isOpen ? 'open' : ''}`}>
      <div className="history-header">
        <h3>History</h3>
        <button className="close-history-btn" onClick={onClose} aria-label="Close history">
          ✕
        </button>
      </div>

      <div className="new-chat-container">
        <button className="new-chat-full-btn" onClick={onNewChat}>
          + New Chat
        </button>
      </div>

      <div className="history-list">
        {conversations.length === 0 ? (
          <div className="empty-history">
            <p>No past conversations.</p>
          </div>
        ) : (
          conversations.map((conv) => (
            <div 
              key={conv.id} 
              className={`history-item ${activeThreadId === conv.id ? 'active' : ''}`}
              onClick={() => onSelectThread(conv.id)}
            >
              <div className="history-item-content">
                <div className="history-item-title">{conv.title || 'New Conversation'}</div>
                <div className="history-item-meta">
                  <span className="history-date">
                    {new Date(conv.timestamp).toLocaleDateString()}
                  </span>
                  <span className="history-preview">
                    {conv.preview}
                  </span>
                </div>
              </div>
              
              <div className="history-menu-container">
                <button 
                  className="history-menu-btn" 
                  onClick={(e) => toggleMenu(e, conv.id)}
                  aria-label="Menu"
                >
                  ⋮
                </button>
                {openMenuId === conv.id && (
                  <div className="history-dropdown" ref={menuRef}>
                    <button 
                      className="history-dropdown-item delete"
                      onClick={(e) => handleDelete(e, conv.id)}
                    >
                      <svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" style={{marginRight: '6px'}}>
                        <path d="M3 6H5H21" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                        <path d="M8 6V4C8 3.46957 8.21071 3 8.58579 2.62513C8.96086 2.25026 9.46957 2.03964 10 2.03964H14C14.5304 2.03964 15.0391 2.25026 15.4142 2.62513C15.7893 3 16 3.46957 16 4V6M19 6V20C19 20.5304 18.7893 21.0391 18.4142 21.4142C18.0391 21.7893 17.5304 22 17 22H7C6.46957 22 5.96086 21.7893 5.58579 21.4142C5.21071 21.0391 5 20.5304 5 20V6H19Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                      </svg>
                      Delete
                    </button>
                  </div>
                )}
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
};

export default HistoryPanel;