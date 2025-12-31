---
id: 002
title: Chat Features Plan
stage: plan
date: 2025-12-30
surface: agent
model: gemini
feature: 006-chat-features
branch: 006-chat-features
user: User
command: /sp.plan
labels: ["plan", "design", "chat", "feature"]
links:
  spec: specs/006-chat-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-chat-features/plan.md
 - specs/006-chat-features/research.md
 - specs/006-chat-features/data-model.md
 - specs/006-chat-features/quickstart.md
tests:
 - null
---

## Prompt

/sp.plan 

## Technical Context

**Language:** JavaScript
**Framework:** Docusaurus
**Storage:** localStorage  
**Deployment:** Local development, then Vercel

---

## File Changes

### Files to Create
- `TextSelectionPopup.jsx` - Popup for text selection
- `HistoryPanel.jsx` - Conversation history sidebar
- `TextSelectionPopup.css` - Popup styling
- `HistoryPanel.css` - History panel styling

### Files to Modify
- `ChatWidget.jsx` - Add history state, integrate new components
- `ChatMessage.jsx` - Add copy button, remove sources
- `ChatWidget.css` - Update styles for new features

### Files to Remove/Ignore
- `SourceCitation.jsx` - No longer used

---

## State Management Updates

### New State Variables Needed
- `selectedText` - Currently selected text
- `showHistory` - History panel visibility
- `conversations` - Array of past conversations
- `copied` - Track which message was copied

### localStorage Schema Design
- `chat_history` - Array of conversation metadata
- `chat_thread_{id}` - Individual thread messages
- `current_thread_id` - Active conversation

---

## Implementation Phases

### Phase 1: Remove Source Citations
**Goal:** Simplify UI by removing source display

**Tasks:**
- Remove SourceCitation component usage from ChatMessage
- Remove sources prop from parent components
- Clean up related CSS
- Update any references

**Verification:**
- AI messages display without sources section
- No console errors
- UI looks clean

---

### Phase 2: Add Copy Feature
**Goal:** Allow users to copy AI responses

**Tasks:**
- Add copy button to ChatMessage component
- Implement clipboard API with fallback
- Add visual feedback (button text changes)
- Style copy button
- Handle errors gracefully

**Verification:**
- Copy button appears only on AI messages
- Clicking copies text to clipboard
- Button shows "Copied!" for 2 seconds
- Works on mobile devices
- Fallback works in older browsers

---

### Phase 3: Text Selection Popup
**Goal:** Show "Ask AI" button when text is selected

**Tasks:**
- Create TextSelectionPopup component
- Add selection detection event listener
- Calculate popup position dynamically
- Handle selection clear
- Integrate with ChatWidget
- Add popup styling with animation

**Verification:**
- Selecting 10+ characters shows button
- Button positioned above selection
- Clicking opens chat with pre-filled input
- Button disappears on deselection
- Works across page navigation

---

### Phase 4: History Storage Logic
**Goal:** Save conversations to localStorage

**Tasks:**
- Design localStorage data structure
- Create save/load utility functions
- Save on each message send
- Load history on component mount
- Implement 50-conversation limit
- Clean up old threads

**Verification:**
- Messages persist after page refresh
- History list updates in real-time
- Old conversations removed after limit
- Data format consistent

---

### Phase 5: History Panel UI
**Goal:** Display conversation list in sidebar

**Tasks:**
- Create HistoryPanel component
- Design conversation list item layout
- Add "New Chat" button
- Implement slide-in animation
- Add empty state message
- Style history panel
- Add close functionality

**Verification:**
- History icon toggles panel
- Panel slides smoothly from left
- Conversations listed with previews
- Timestamps formatted correctly
- Empty state shows when no history
- New Chat button accessible

---

### Phase 6: Thread Loading 
**Goal:** Load selected conversation into chat

**Tasks:**
- Implement thread loading function
- Restore messages from localStorage
- Switch active thread
- Update UI state
- Handle scroll position
- Implement "New Chat" functionality

**Verification:**
- Clicking thread loads all messages
- Messages display in correct order
- Scroll position at bottom
- Thread switches correctly
- New Chat clears current conversation
- Active thread ID updates

---

### Phase 7: Backend Integration
  
**Goal:** Send selected text as context to backend

**Tasks:**
- Update API request payload
- Add selected_text parameter
- Handle in backend RAG logic (optional)
- Test end-to-end flow
- Update error handling

**Verification:**
- Selected text sent to backend
- Backend uses context appropriately
- Response relevant to selection
- Works without selected text
- Errors handled gracefully

---

## Testing Strategy

### Unit Testing
- Test each component independently
- Verify state management
- Test localStorage operations
- Check error handling

### Integration Testing
- Text selection → chat flow
- History save/load cycle
- Thread switching
- Backend communication

### Browser Testing
- Chrome, Firefox, Safari, Edge
- Mobile browsers
- Different screen sizes

### Edge Case Testing
- Very long text selections (>500 chars)
- Empty history state
- Corrupted localStorage data
- Network failures
- Rapid user interactions
- Multiple thread switches

---

## Dependencies

### No New NPM Packages
All features use native browser APIs:
- `window.getSelection()` for text selection
- `navigator.clipboard` for copy
- `localStorage` for persistence

---

## Risk Mitigation

### Potential Issues
- Browser compatibility with clipboard API
- localStorage quota exceeded
- Race conditions in state updates
- Performance with 50 threads

### Mitigation Strategies
- Implement clipboard fallback
- Monitor localStorage usage, clean old data
- Use proper async/await patterns
- Optimize rendering with React.memo if needed

---

## Success Criteria

- All 5 user stories functional
- No regression in existing features
- Performance acceptable (<100ms interactions)
- Works in Chrome, Firefox, Safari
- Mobile responsive
- No console errors

## Response snapshot

Created implementation plan, research, data model, and quickstart guide for chat features. Validated against Constitution. All artifacts aligned with user requirements and existing project structure.

## Outcome

- ✅ Impact: Established technical design for chat history, text selection, and copy features.
- 🧪 Tests: Defined manual test cases in quickstart.md.
- 📁 Files: Plan, Research, Data Model, Quickstart.
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: Plan aligns with user input and constitution.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
