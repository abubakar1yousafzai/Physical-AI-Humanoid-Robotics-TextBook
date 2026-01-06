import React, { useState, useEffect, useRef } from 'react';
import './TextSelectionPopup.css';

const TextSelectionPopup = () => {
  const [selection, setSelection] = useState(null);
  const [position, setPosition] = useState({ top: 0, left: 0 });
  const popupRef = useRef(null);
  const isClickingPopup = useRef(false);

  useEffect(() => {
    const handleSelectionChange = () => {
      if (isClickingPopup.current) {
        isClickingPopup.current = false;
        return;
      }
      const currentSelection = window.getSelection();
      const text = currentSelection.toString().trim();

      if (text.length >= 10) {
        const range = currentSelection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Calculate position relative to the viewport
        // Position above the selection
        const top = rect.top - 50; 
        const left = rect.left + (rect.width / 2);

        setSelection(text);
        setPosition({ top, left });
      } else {
        setSelection(null);
      }
    };

    // Use mouseup for better stability than selectionchange for simple text selection
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange); // Handle keyboard selection

    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
    };
  }, []);

  const handleMouseDown = (e) => {
    e.preventDefault();
    e.stopPropagation();
    isClickingPopup.current = true;
    console.log('Mouse down on popup');
  };

  const handleClick = (e) => {
    e.stopPropagation(); 
    e.preventDefault();
    console.log('Ask AI clicked, selection:', selection);
    
    if (selection) {
      const event = new CustomEvent('askAI', { 
        detail: { text: selection } 
      });
      window.dispatchEvent(event);
      console.log('askAI event dispatched');
      
      setSelection(null);
      window.getSelection().removeAllRanges();
    }
  };

  if (!selection) return null;

  return (
    <div 
      ref={popupRef}
      className="text-selection-popup"
      style={{ top: `${position.top}px`, left: `${position.left}px` }}
      onMouseDown={handleMouseDown}
      onClick={handleClick}
      role="button"
      tabIndex={0}
    >
      <span className="popup-icon">✨</span> Ask AI
    </div>
  );
};

export default TextSelectionPopup;