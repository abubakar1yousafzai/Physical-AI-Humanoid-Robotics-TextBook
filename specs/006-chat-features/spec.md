# Feature Specification: ChatBot Features

**Feature Branch**: `006-chat-features`  
**Created**: 2025-12-30  
**Status**: Draft  
**Input**: User description: "Add three enhancements to chat widget: text selection queries, conversation history, and message copying. Remove source citations."

## Overview
Add three enhancements to chat widget: text selection queries, conversation history, and message copying. Remove source citations.

### Out of Scope
- Real-time presence/typing indicators.
- Multi-user conversations.
- AI personalization beyond basic conversation history.

## Clarifications

### Session 2025-12-30
- Q: What specific scenarios are explicitly out-of-scope for this chat widget enhancement? → A: Real-time presence/typing indicators, multi-user conversations, or AI personalization beyond basic conversation history.
- Q: What specific data points (e.g., timestamps, user ID) should be stored for each message in a conversation? → A: message_id, role, content, timestamp, thread_id
- Q: How should the UI behave if there is no conversation history available? → A: Show empty state message
- Q: What are the security and privacy requirements for storing conversation history locally? → A: localStorage, no encryption needed
- Q: What is the expected behavior when the external AI service is unavailable or returns an error? → A: Show error, allow retry

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Text Selection → Ask AI (Priority: P1)

Given I am reading the textbook When I highlight text (10+ characters) Then "✨ Ask AI" button appears above selection And clicking it opens chat with context

**Why this priority**: This is a core new interaction for leveraging AI directly from content, significantly improving user experience.

**Independent Test**: Can be fully tested by selecting text on any page and verifying the button appears and opens the chat with the selected text.

**Acceptance Scenarios**:

1.  **Given** I am reading the textbook, **When** I highlight text longer than 10 characters, **Then** a "✨ Ask AI" button appears above the selection.
2.  **Given** a "✨ Ask AI" button is visible above selected text, **When** I click the button, **Then** the chat widget opens with the selected text pre-filled in the input.

---

### User Story 2 - Conversation History (Priority: P1)

Given I have past conversations When I click history icon (🕐) Then history panel shows with conversation list And clicking a thread loads that conversation

**Why this priority**: Essential for maintaining context and productivity across multiple interactions, enabling users to revisit previous queries.

**Independent Test**: Can be fully tested by generating multiple conversations, then navigating to the history panel, selecting a conversation, and verifying its content loads correctly.

**Acceptance Scenarios**:

1.  **Given** I have previous chat conversations, **When** I click the history icon (🕐), **Then** a history panel slides in from the left showing a list of past conversations.
2.  **Given** the history panel is open and lists conversations, **When** I click on a conversation thread, **Then** that conversation's messages are loaded and displayed in the chat widget.

---

### User Story 3 - New Chat (Priority: P2)

Given I am in a conversation When I click "New Chat" button Then current chat clears and new thread starts

**Why this priority**: Provides a clear way to start fresh, preventing confusion from old context and improving chat manageability.

**Independent Test**: Can be fully tested by having an active conversation, clicking "New Chat", and verifying the chat interface clears and is ready for a new input.

**Acceptance Scenarios**:

1.  **Given** I am in an active chat conversation, **When** I click the "New Chat" button, **Then** the current chat messages are cleared, and a new, empty chat thread is initiated.

---

### User Story 4 - Copy Message (Priority: P1)

Given AI has responded When I click copy button (📋) on message Then text copies to clipboard And button shows "✓ Copied!" for 2 seconds

**Why this priority**: Improves usability by allowing quick extraction of AI-generated content for external use.

**Independent Test**: Can be fully tested by receiving an AI response, clicking the copy button, and verifying the text is in the clipboard and the visual feedback is correct.

**Acceptance Scenarios**:

1.  **Given** the AI has provided a response, **When** I click the copy button (📋) associated with the AI message, **Then** the message text is copied to my clipboard.
2.  **Given** I have clicked the copy button, **Then** the button temporarily changes to display "✓ Copied!" for 2 seconds before reverting to its original state.

---

### User Story 5 - Clean UI (Priority: P2)

Given AI responds with answer When message displays Then NO sources section shown

**Why this priority**: Streamlines the UI by removing potentially distracting elements not needed for the core interaction.

**Independent Test**: Can be fully tested by observing AI responses and ensuring no source citations are visible in any message display.

**Acceptance Scenarios**:

1.  **Given** the AI provides a response that would typically include source citations, **When** the message is displayed in the chat widget, **Then** no "sources" section or citation links are visible within the message content.

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST detect text selections of 10 or more characters within the textbook content.
-   **FR-002**: The system MUST display an "Ask AI" button above any detected text selection that meets the character threshold.
-   **FR-003**: When the "Ask AI" button is clicked, the chat input field MUST be pre-filled with the selected text.
-   **FR-004**: The system MUST store chat conversations locally (e.g., in `localStorage`), with a limit of 50 conversations.
-   **FR-005**: The history panel MUST slide in from the left, with a width of 300px.
-   **FR-006**: The copy button on AI messages MUST copy the exact text content of the AI's response to the user's clipboard.
-   **FR-007**: The system MUST prevent the display of source citations within AI responses in the chat UI.
-   **FR-008**: The system MUST display an "empty state" message in the history panel when no conversation history is available.
-   **FR-009**: The system MUST display an error message and provide an option to retry when the external AI service is unavailable or returns an error.

### Non-Functional Requirements

#### Security & Privacy
-   **NFR-001**: Conversation history data stored in `localStorage` MUST NOT be encrypted, as per clarification.

### Key Entities

-   **Conversation**: Represents a complete chat exchange between the user and the AI. Each conversation has a `thread_id` (unique identifier).
-   **Message**: Represents a single turn in a conversation. Each message has a `message_id` (unique identifier), `role` (user/AI), `content` (text), and `timestamp`).

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: The "Ask AI" button appears 100% of the time when text selections meet the specified criteria across supported browsers (Chrome, Firefox, Safari).
-   **SC-002**: All past conversations (up to the 50-conversation limit) are accurately displayed and loadable from the history panel.
-   **SC-003**: The copy message functionality successfully copies AI responses to the clipboard in Chrome, Firefox, and Safari, and provides visual feedback as specified.
-   **SC-004**: Source citations are never visible in any AI message displayed in the chat widget.
-   **SC-005**: Clicking "New Chat" consistently creates a fresh, empty conversation thread without retaining previous context.