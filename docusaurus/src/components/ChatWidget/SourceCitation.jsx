import React, { useState } from 'react';
import './ChatWidget.css';

const SourceCitation = ({ sources }) => {
  const [isExpanded, setIsExpanded] = useState(false);

  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className="source-citation-container">
      <button 
        className="source-toggle-btn" 
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
      >
        {isExpanded ? '▼ Hide Sources' : '▶ Show Sources'}
      </button>
      
      {isExpanded && (
        <div className="source-list">
          {sources.map((source, index) => (
            <div key={index} className="source-item">
              <div className="source-title">
                {source.url ? (
                  <a href={source.url} target="_blank" rel="noopener noreferrer">
                    {source.title || 'Source'}
                  </a>
                ) : (
                  <span>{source.title || 'Source'}</span>
                )}
              </div>
              {source.snippet && (
                <div className="source-snippet">
                  {source.snippet}
                </div>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default SourceCitation;
