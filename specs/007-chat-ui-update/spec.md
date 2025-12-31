# Feature Specification: Chat Widget UI Updates

**Feature Branch**: `007-chat-ui-update`
**Created**: 2025-12-31
**Status**: Draft
**Input**: Chat Widget UI Updates: Icon Redesign and History Delete Feature Target audience: Frontend developers working with React, Docusaurus, familiar with component-based architecture and localStorage operations. Focus: Replace emoji icons with consistent SVG document-style icons Reorganize chat header button layout (close left, history right) Add conversation deletion capability via three-dot menu in history panel Success criteria: All emoji icons (🕐, 📋) replaced with SVG icons Copy button displays overlapping-rectangles icon History button displays file-document icon Chat header shows close button on left, history button on right Assistant avatar removed from message bubbles Users can delete conversations via three-dot menu in history Deleted conversations removed from both UI and localStorage Deleting active conversation automatically starts new chat All existing functionality (copy, history toggle, selection) works correctly Constraints: Files to modify: ChatWidget.jsx, ChatMessage.jsx, HistoryPanel.jsx, ChatWidget.css Use inline SVG (no new dependencies unless already present) Maintain localStorage compatibility with existing data structure Preserve mobile responsiveness No breaking changes to existing features Not building: New icon library or design system Advanced delete features (bulk delete, undo, trash bin) Export/backup conversations before deletion Search or filter in history panel Keyboard shortcuts for delete Animation effects for deletion

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Icon & Layout Refresh (Priority: P1)

As a user, I want the chat interface to look cleaner and more consistent with modern UI standards by replacing emojis with SVG icons and improving button layout.

**Why this priority**: Improves the visual quality and professionalism of the interface, which is the primary touchpoint for users.

**Independent Test**: Can be tested visually by opening the chat widget and verifying icon appearances and positions without needing full functional interaction.

**Acceptance Scenarios**:

1. **Given** the chat widget is open, **When** I look at the header, **Then** the close button is on the left and the history button (document icon) is on the right.
2. **Given** the chat widget is open, **When** I look at the copy button on a message, **Then** it displays an SVG "overlapping rectangles" icon instead of a clipboard emoji.
3. **Given** I am viewing a message bubble, **Then** no assistant avatar is visible.

---

### User Story 2 - Delete Conversation (Priority: P1)

As a user, I want to delete old conversations from my history so I can keep my workspace organized.

**Why this priority**: Essential for user control over their data and managing a cluttered history list.

**Independent Test**: Can be tested by creating a conversation, opening history, deleting it, and verifying it disappears from the list and storage.

**Acceptance Scenarios**:

1. **Given** the history panel is open with at least one conversation, **When** I click the three-dot menu on a conversation item, **Then** a delete option appears.
2. **Given** the delete option is visible, **When** I click "Delete", **Then** the conversation is removed from the history list and local storage immediately.
3. **Given** I delete the currently active conversation, **Then** the chat resets to a "New Chat" state automatically.

---

### Edge Cases

- What happens when the user tries to delete the last remaining conversation? (Should show empty state)
- What happens if the delete action fails (e.g., storage error)? (Should handle gracefully, though unlikely with localStorage)
- How does the three-dot menu behave on mobile screens? (Should remain accessible and touch-friendly)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST replace the history button emoji (🕐) with a "file-document" style SVG icon.
- **FR-002**: System MUST replace the copy button emoji (📋) with an "overlapping-rectangles" style SVG icon.
- **FR-003**: System MUST position the chat close button on the left side of the header.
- **FR-004**: System MUST position the history button on the right side of the header.
- **FR-005**: System MUST remove the assistant avatar/icon from message bubbles.
- **FR-006**: System MUST provide a "three-dot" menu (kebab menu) for each item in the history list.
- **FR-007**: The three-dot menu MUST contain a "Delete" option.
- **FR-008**: Clicking "Delete" MUST remove the specific conversation from `localStorage`.
- **FR-009**: Clicking "Delete" MUST remove the conversation from the UI list immediately.
- **FR-010**: If the active conversation is deleted, the system MUST automatically initiate a new chat session.

### Key Entities

- **Conversation Item**: Represents a saved chat thread in the history list, now including a menu trigger.
- **SVG Icon**: Inline SVG code used for UI elements (history, copy, close, menu, trash).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of emoji icons (history, copy) in the chat widget are replaced with SVG icons.
- **SC-002**: Users can delete any single conversation from the history panel with 2 clicks (Menu -> Delete).
- **SC-003**: Deleting the active conversation results in a reset chat state 100% of the time.
- **SC-004**: The chat header layout strictly follows the "Close Left, History Right" arrangement.