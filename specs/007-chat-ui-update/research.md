# Research: Chat Widget UI Updates

## Decisions & Rationale

### 1. Icon Strategy
- **Decision**: Use inline SVGs.
- **Rationale**: 
  - Zero external dependencies.
  - Complete control over styling (`currentColor`, size).
  - No network requests for icon fonts/images.
- **Alternatives Considered**: 
  - `react-icons`: Adds dependency/bundle size.
  - Font Awesome: Overkill for 4 icons.

### 2. Delete Confirmation
- **Decision**: Use `window.confirm()` for MVP.
- **Rationale**: 
  - Simple, native, accessible.
  - Avoids building a custom modal component for a single action.
  - Meets "Medium risk" assessment without over-engineering.
- **Alternatives Considered**: 
  - Custom Modal: Better UX but higher complexity/code volume.
  - Toast with Undo: Best UX but complex state management (out of scope).

### 3. Menu Implementation
- **Decision**: Custom CSS dropdown within `HistoryPanel`.
- **Rationale**: 
  - Lightweight.
  - Can be styled to match existing design system.
  - "Click outside" handler needed for UX (close menu when clicking away).

### 4. Active Conversation Deletion
- **Decision**: Reset to "New Chat" state immediately.
- **Rationale**: 
  - Clear feedback to user.
  - Prevents "zombie" state where user is viewing a deleted thread.
  - Consistent with "New Chat" button behavior.

## Unknowns Resolved
- **Assistant Avatar**: It is currently rendered in `ChatMessage.jsx`. Removing it is a simple JSX deletion.
- **LocalStorage Structure**: Confirmed keys are `chat_history` (list) and `chat_thread_{id}` (messages). Both must be cleaned on delete.
