import React, { useState, useEffect } from 'react';
import ChatInput from './ChatInput';
import ChatMessage from './ChatMessage';
import TextSelectionPopup from './TextSelectionPopup';
import HistoryPanel from './HistoryPanel';
import './ChatWidget.css';

// --- LocalStorage Helpers ---
const MAX_HISTORY = 50;
const STORAGE_KEY_HISTORY = 'chat_history';
const STORAGE_KEY_THREAD_PREFIX = 'chat_thread_';

const saveToHistory = (threadId, messages) => {
  if (!threadId || messages.length === 0) return;

  // 1. Save messages for this thread
  localStorage.setItem(STORAGE_KEY_THREAD_PREFIX + threadId, JSON.stringify(messages));

  // 2. Update conversation list
  const historyJson = localStorage.getItem(STORAGE_KEY_HISTORY);
  let history = historyJson ? JSON.parse(historyJson) : [];

  const lastMessage = messages[messages.length - 1];
  const firstUserMessage = messages.find(m => m.role === 'user');
  const title = firstUserMessage ? firstUserMessage.content.substring(0, 30) + (firstUserMessage.content.length > 30 ? '...' : '') : 'New Conversation';
  
  const metadata = {
    id: threadId,
    title: title,
    timestamp: Date.now(),
    preview: lastMessage ? lastMessage.content.substring(0, 50) + (lastMessage.content.length > 50 ? '...' : '') : ''
  };

  // Remove existing entry for this thread if exists
  history = history.filter(h => h.id !== threadId);
  
  // Add new metadata to top
  history.unshift(metadata);

  // Limit to MAX_HISTORY
  if (history.length > MAX_HISTORY) {
    const removed = history.pop();
    // Clean up messages for removed thread
    localStorage.removeItem(STORAGE_KEY_THREAD_PREFIX + removed.id);
  }

  localStorage.setItem(STORAGE_KEY_HISTORY, JSON.stringify(history));
  return history;
};

const loadConversations = () => {
  const json = localStorage.getItem(STORAGE_KEY_HISTORY);
  return json ? JSON.parse(json) : [];
};

const loadThreadMessages = (threadId) => {
  const json = localStorage.getItem(STORAGE_KEY_THREAD_PREFIX + threadId);
  return json ? JSON.parse(json) : null;
};

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  // Default welcome message shouldn't persist as "saved" until user interacts? 
  // actually, let's keep it simple. If we load a thread, we use those messages. 
  // If no thread, we show welcome.
  const [messages, setMessages] = useState([
    { id: 'welcome', role: 'assistant', content: 'Welcome! How can I help you with the textbook?' }
  ]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [threadId, setThreadId] = useState(null); // active thread id
  const [selectedText, setSelectedText] = useState('');
  
  // History State
  const [showHistory, setShowHistory] = useState(false);
  const [conversations, setConversations] = useState([]);

  // Load history list on mount
  useEffect(() => {
    setConversations(loadConversations());
    
    // Check for active thread in LS (optional, maybe we want to start fresh or restore last)
    const savedThreadId = localStorage.getItem('chat_thread_id');
    if (savedThreadId) {
      const savedMessages = loadThreadMessages(savedThreadId);
      if (savedMessages) {
        setThreadId(savedThreadId);
        setMessages(savedMessages);
      }
    }
  }, []);

  // Save active thread ID
  useEffect(() => {
    if (threadId) {
      localStorage.setItem('chat_thread_id', threadId);
    } else {
      localStorage.removeItem('chat_thread_id');
    }
  }, [threadId]);

  useEffect(() => {
    const handleAskAIEvent = (event) => {
      console.log('askAI event received:', event.detail.text);
      handleAskAI(event.detail.text);
    };

    window.addEventListener('askAI', handleAskAIEvent);

    return () => {
      window.removeEventListener('askAI', handleAskAIEvent);
    };
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const toggleHistory = () => {
    setShowHistory(!showHistory);
  };

  const handleAskAI = (text) => {
    console.log('handleAskAI called with:', text);
    setSelectedText(text);
    setIsOpen(true);
    
    // Focus input after render
    setTimeout(() => {
      const input = document.querySelector('.chat-input');
      if (input) {
        input.focus();
      }
    }, 200);
  };

  const handleSelectThread = (id) => {
    const msgs = loadThreadMessages(id);
    if (msgs) {
      setThreadId(id);
      setMessages(msgs);
      setShowHistory(false); // Close panel on mobile/desktop?
    }
  };

  const handleNewChat = () => {
    setThreadId(null);
    setMessages([{ id: 'welcome', role: 'assistant', content: 'Welcome! How can I help you with the textbook?' }]);
    setShowHistory(false);
    localStorage.removeItem('chat_thread_id');
  };

  const handleDeleteConversation = (id) => {
    // 1. Remove from localStorage
    localStorage.removeItem(STORAGE_KEY_THREAD_PREFIX + id);
    
    const historyJson = localStorage.getItem(STORAGE_KEY_HISTORY);
    if (historyJson) {
      const history = JSON.parse(historyJson);
      const updatedHistory = history.filter(item => item.id !== id);
      localStorage.setItem(STORAGE_KEY_HISTORY, JSON.stringify(updatedHistory));
      setConversations(updatedHistory);
    }

    // 2. If deleting active thread, reset to new chat
    if (id === threadId) {
      handleNewChat();
    }
  };

  const sendMessage = async (text) => {
    let currentThreadId = threadId;
    if (!currentThreadId) {
      currentThreadId = Date.now().toString(); // Generate simple ID
      setThreadId(currentThreadId);
    }

    const newUserMessage = { id: Date.now().toString(), role: 'user', content: text };
    const updatedMessages = [...messages, newUserMessage];
    
    // Capture context before clearing, explicitly null if empty
    const contextText = selectedText || null;

    setMessages(updatedMessages);
    setLoading(true);
    setError(null);
    setSelectedText('');

    // Optimistic save
    const updatedHistory = saveToHistory(currentThreadId, updatedMessages);
    if (updatedHistory) setConversations(updatedHistory);

    try {
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: text,
          thread_id: currentThreadId,
          selected_text: contextText
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

      const finalMessages = [...updatedMessages, newAssistantMessage];
      setMessages(finalMessages);
      
      // Update persistent storage with AI response
      const finalHistory = saveToHistory(currentThreadId, finalMessages);
      if (finalHistory) setConversations(finalHistory);

      if (data.thread_id && data.thread_id !== currentThreadId) {
         // If backend returns a different thread_id, update it (should align)
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
      <TextSelectionPopup />
      
      {!isOpen && (
        <button className="chat-toggle-btn" onClick={toggleChat} aria-label="Open Chat">
          💬
        </button>
      )}

      {isOpen && (
        <div className="chat-panel">
          <HistoryPanel 
            isOpen={showHistory} 
            onClose={() => setShowHistory(false)}
            conversations={conversations}
            activeThreadId={threadId}
            onSelectThread={handleSelectThread}
            onNewChat={handleNewChat}
            onDeleteConversation={handleDeleteConversation}
          />

          <div className="chat-header">
            <span>Physical AI Assistant</span>
            <div className="header-buttons">
              <button className="history-toggle-btn" onClick={toggleHistory} title="History">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="12" cy="12" r="10"/>
                  <polyline points="12 6 12 12 16 14"/>
                </svg>
              </button>
              <button className="close-btn" onClick={toggleChat} title="Close">✕</button>
            </div>
          </div>
          <div className="chat-messages">
            {selectedText && (
              <div className="selected-context">
                📝 Context: {selectedText.substring(0, 60)}...
                <button 
                  className="close-context-btn"
                  onClick={() => setSelectedText('')}
                >
                  ×
                </button>
              </div>
            )}
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
          <ChatInput 
            onSend={sendMessage} 
            disabled={loading} 
            prefilledText={selectedText}
          />
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
