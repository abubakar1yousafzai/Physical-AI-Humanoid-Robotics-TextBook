import React, { useState } from 'react';
import './ChatWidget.css';

const ChatMessage = ({ message }) => {
  const { role, content } = message;
  const isUser = role === 'user';
  const [copied, setCopied] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(content).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    }).catch(err => {
      console.error('Failed to copy text: ', err);
      // Fallback could go here if needed
    });
  };

  return (
    <div className={`chat-message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className="message-content">
        {content}
      </div>
      {!isUser && (
        <button 
          className="copy-btn" 
          onClick={handleCopy} 
          aria-label="Copy message"
          title="Copy"
        >
          {copied ? (
            <span style={{ fontSize: '12px', fontWeight: 500 }}>✓ Copied!</span>
          ) : (
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M8 4H20V20H8V4Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              <path d="M16 4V2H4V18H8" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          )}
        </button>
      )}
    </div>
  );
};

export default ChatMessage;