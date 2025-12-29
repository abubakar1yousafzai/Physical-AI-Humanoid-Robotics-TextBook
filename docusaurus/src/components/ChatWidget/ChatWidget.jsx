import React, { useState, useEffect } from 'react';
import ChatInput from './ChatInput';
import ChatMessage from './ChatMessage';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 'welcome', role: 'assistant', content: 'Welcome! How can I help you with the textbook?' }
  ]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [threadId, setThreadId] = useState(null);

  // Load thread_id from localStorage on mount
  useEffect(() => {
    const savedThreadId = localStorage.getItem('chat_thread_id');
    if (savedThreadId) {
      setThreadId(savedThreadId);
    }
  }, []);

  // Save thread_id to localStorage when it changes
  useEffect(() => {
    if (threadId) {
      localStorage.setItem('chat_thread_id', threadId);
    }
  }, [threadId]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async (text) => {
    const newUserMessage = { id: Date.now().toString(), role: 'user', content: text };
    setMessages(prev => [...prev, newUserMessage]);
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: text,
          thread_id: threadId
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response from server');
      }

      const data = await response.json();
      
      const newAssistantMessage = {
        id: Date.now().toString() + '_a',
        role: 'assistant',
        content: data.answer,
        sources: data.sources
      };

      setMessages(prev => [...prev, newAssistantMessage]);
      if (data.thread_id) {
        setThreadId(data.thread_id);
      }
    } catch (err) {
      console.error('Chat error:', err);
      setError('Sorry, I encountered an issue connecting to the assistant. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="chat-widget-container">
      {!isOpen && (
        <button className="chat-toggle-btn" onClick={toggleChat} aria-label="Open Chat">
          💬
        </button>
      )}

      {isOpen && (
        <div className="chat-panel">
          <div className="chat-header">
            <span>Assistant</span>
            <button className="close-btn" onClick={toggleChat} aria-label="Close Chat">
              ✕
            </button>
          </div>
          <div className="chat-messages">
            {messages.map(msg => (
              <ChatMessage key={msg.id} message={msg} />
            ))}
            {loading && (
              <div className="chat-message assistant-message">
                <span className="typing-indicator">Thinking...</span>
              </div>
            )}
            {error && (
              <div className="error-message">
                {error}
              </div>
            )}
          </div>
          <ChatInput onSend={sendMessage} disabled={loading} />
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
