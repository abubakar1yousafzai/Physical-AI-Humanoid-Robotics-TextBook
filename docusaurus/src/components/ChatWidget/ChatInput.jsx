import React, { useState } from 'react';
import './ChatWidget.css';

const ChatInput = ({ onSend, disabled }) => {
  const [input, setInput] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (input.trim() && !disabled) {
      onSend(input);
      setInput('');
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
        disabled={disabled}
      />
      <button type="submit" className="send-btn" disabled={!input.trim() || disabled}>
        Send
      </button>
    </form>
  );
};

export default ChatInput;
