import React, { useState, useEffect } from 'react';
import './ChatWidget.css';

const ChatInput = ({ onSend, disabled, prefilledText }) => {
  const [input, setInput] = useState('');

  // Update input when prefilledText changes
  useEffect(() => {
    if (prefilledText && prefilledText.trim()) {
      const formatted = `Explain this: "${prefilledText.substring(0, 100)}${prefilledText.length > 100 ? '...' : ''}"`;
      setInput(formatted);
    }
  }, [prefilledText]);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (input.trim() && !disabled) {
      onSend(input);
      setInput('');
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <form className="chat-input-area" onSubmit={handleSubmit}>
      <input
        type="text"
        className="chat-input"
        placeholder="Ask a question..."
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyDown={handleKeyDown}
        disabled={disabled}
      />
      <button type="submit" className="send-btn" disabled={!input.trim() || disabled}>
        Send
      </button>
    </form>
  );
};

export default ChatInput;