import React from 'react';
import SourceCitation from './SourceCitation';
import './ChatWidget.css';

const ChatMessage = ({ message }) => {
  const { role, content, sources } = message;
  const isUser = role === 'user';

  return (
    <div className={`chat-message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className="message-content">
        {content}
      </div>
      {!isUser && sources && sources.length > 0 && (
        <SourceCitation sources={sources} />
      )}
    </div>
  );
};

export default ChatMessage;